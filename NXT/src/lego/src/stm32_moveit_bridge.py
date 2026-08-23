#!/usr/bin/env python3
import math
import time
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Int32MultiArray, Bool
from sensor_msgs.msg import JointState 

import json
import os
from ament_index_python.packages import get_package_share_directory

def rad_to_cdeg(rad: float) -> int:
    # radians -> centi-degrees
    return int(round(rad * 180.0 / math.pi * 100.0))


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class Stm32MoveItBridge(Node):
    """
    MoveIt FollowJointTrajectory:
      /BRAZO_controller/follow_joint_trajectory -> STM32A/STM32B cmd_deg
      /PINZA_controller/follow_joint_trajectory -> /stm32A/gripper_cmd Bool

    STM32A expects: [J1, J2, J3] in cdeg
    STM32B expects: [Y, Z, X] in cdeg

    Mapping MoveIt -> STM32B:
      Joint4 -> X -> STM32B data[2]
      Joint5 -> Y -> STM32B data[0]
      Joint6 -> Z -> STM32B data[1]
    """

    def __init__(self):
        super().__init__('stm32_moveit_bridge')
        #self.gear_ratios = [1.0, 14.0, 1.0, 1.0, 1.0, 1.0]
        ########################################################
        # --- CARGAR RELACIÓN DE ENGRANAJES DESDE JSON ---
        ########################################################
        
        try:
            # Opción A: Buscando a través del directorio share del paquete ROS2 (Recomendado)
            # package_path = get_package_share_directory('lego')
            # json_path = os.path.join(package_path, 'config', 'gearRatio.json')

            # Opción B: Leyendo directamente desde tu ruta dentro de /root/ros_ws/src/
            json_path = '/root/ros_ws/src/lego/config/gearRatio.json'

            with open(json_path, 'r') as f:
                data = json.load(f)
                
            # Extraemos los valores en orden estricto de joint1 a joint6
            joints_cfg = data.get("joints", {})
            self.gear_ratios = [
                float(joints_cfg.get(f"joint{i}", {}).get("gear_ratio", 1.0)) 
                for i in range(1, 7)
            ]
            
            self.get_logger().info(f"Relaciones de engranajes cargadas desde JSON: {self.gear_ratios}")

        except Exception as e:
            self.get_logger().warn(f"No se pudo cargar gearRatio.json ({e}). Usando valores por defecto.")
            self.gear_ratios = [5.0, 14.01, 10.02, 7.5, 7.5, 7.0] # Fallback de seguridad
        ########################################################             
        self.cb_group = ReentrantCallbackGroup()

        # Publishers hacia STM32
        self.pub_cmd_a = self.create_publisher(Int32MultiArray, '/stm32A/cmd_deg', 10)
        self.pub_cmd_b = self.create_publisher(Int32MultiArray, '/stm32B/cmd_deg', 10)

        # La pinza está actuada SOLO por STM32A
        self.pub_gripper = self.create_publisher(Bool, '/stm32A/gripper_cmd', 10)

        # Action servers esperados por MoveIt según tu controllers.yaml
        self.arm_action_name = '/BRAZO_controller/follow_joint_trajectory'
        self.gripper_action_name = '/PINZA_controller/follow_joint_trajectory'

        self.pub_referencia = self.create_publisher(JointState, '/referencia_moveit', 10)

        self._as_arm = ActionServer(
            self,
            FollowJointTrajectory,
            self.arm_action_name,
            self.execute_cb,
            callback_group=self.cb_group
        )

        self._as_gripper = ActionServer(
            self,
            FollowJointTrajectory,
            self.gripper_action_name,
            self.execute_gripper_cb,
            callback_group=self.cb_group
        )

        # STM32A joints
        self.joints_a = ['Joint1', 'Joint2', 'Joint3']

        # MoveIt joints for wrist
        self.jointX = 'Joint4'  # -> STM32B data[2] X
        self.jointY = 'Joint5'  # -> STM32B data[0] Y
        self.jointZ = 'Joint6'  # -> STM32B data[1] Z

        # Joints de pinza en MoveIt
        self.gripper_joints = ['Joint7', 'Joint8']

        # Sign corrections
        self.signX = +1
        self.signY = +1
        self.signZ = +1

        # Umbral de cierre de pinza.
        # OJO: si tu pinza en URDF se mueve entre 0.0 y 0.04 rad/m,
        # 0.5 no sirve. Por eso dejo un valor chico configurable.
        
        self.declare_parameter("gripper_close_threshold", 0.005)
        self.declare_parameter('gripper_invert', False)
        self.gripper_close_threshold = float(
            self.get_parameter('gripper_close_threshold').value
        )
        self.gripper_invert = bool(
            self.get_parameter('gripper_invert').value
        )

        # Sending rate
        self.rate_hz = 100.0
        self.dt = 1.0 / self.rate_hz

        # Últimos comandos conocidos.
        # Si están en None, todavía no publica nada.
        self.lastA_cmd: Optional[List[int]] = None
        self.lastB_cmd: Optional[List[int]] = None

        # Lock porque el timer y el action callback pueden correr en paralelo
        self.cmd_lock = threading.Lock()

        # Timer permanente de publicación.
        # Este timer mantiene vivo el stream hacia las STM32.
        self.hold_timer = self.create_timer(
            self.dt,
            self.publish_last_cmds,
            callback_group=self.cb_group
        )

        self.get_logger().info(
            f'Bridge listo: {self.arm_action_name} -> /stm32A/cmd_deg + /stm32B/cmd_deg @ {self.rate_hz:.1f} Hz'
        )
        self.get_logger().info(
            f'Bridge listo: {self.gripper_action_name} -> /stm32A/gripper_cmd Bool'
        )
        self.get_logger().info(
            f'STM32A: {self.joints_a} -> [J1,J2,J3]'
        )
        self.get_logger().info(
            f'STM32B mapping: X={self.jointX}(sign {self.signX}) -> data[2], '
            f'Y={self.jointY}(sign {self.signY}) -> data[0], '
            f'Z={self.jointZ}(sign {self.signZ}) -> data[1]'
        )
        self.get_logger().info(
            f'Pinza: joints={self.gripper_joints}, threshold={self.gripper_close_threshold}, invert={self.gripper_invert}'
        )

    def enviar_pinza(self, cerrar: bool):
        msg = Bool()
        msg.data = bool(cerrar)
        self.pub_gripper.publish(msg)

        self.get_logger().info(
            f"Pinza -> {'CERRAR' if msg.data else 'ABRIR'} por STM32A"
        )

    def publish_last_cmds(self):
        """
        Publica continuamente el último comando conocido.

        Esto evita que la STM32 entre en SAFE_HOLD
        cuando una trayectoria terminó correctamente.
        """
        with self.cmd_lock:
            lastA = None if self.lastA_cmd is None else list(self.lastA_cmd)
            lastB = None if self.lastB_cmd is None else list(self.lastB_cmd)

        if lastA is not None:
            msgA = Int32MultiArray()
            msgA.data = lastA
            self.pub_cmd_a.publish(msgA)

        if lastB is not None:
            msgB = Int32MultiArray()
            msgB.data = lastB
            self.pub_cmd_b.publish(msgB)

    def update_last_cmds(self, a, x, y, z):
        """
        Actualiza el último comando conocido.
        El timer se encarga de publicarlo.
        """
        lastA_cmd = [
            rad_to_cdeg(a[0]),
            rad_to_cdeg(a[1]),
            rad_to_cdeg(a[2]),
        ]

        X_cdeg = self.signX * rad_to_cdeg(x)
        Y_cdeg = self.signY * rad_to_cdeg(y)
        Z_cdeg = self.signZ * rad_to_cdeg(z)

        # STM32B expects [Y, Z, X]
        lastB_cmd = [Y_cdeg, Z_cdeg, X_cdeg]

        with self.cmd_lock:
            self.lastA_cmd = lastA_cmd
            self.lastB_cmd = lastB_cmd

    def make_result(self, error_code=0, error_string=''):
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_string
        return result

    def execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)

        # --- Validate joints present ---
        idx_a: List[int] = []

        for j in self.joints_a:
            if j not in names:
                msg = f'Falta {j} en trajectory. joint_names={names}'
                self.get_logger().error(msg)
                goal_handle.abort()
                return self.make_result(
                    FollowJointTrajectory.Result.INVALID_JOINTS,
                    msg
                )

            idx_a.append(names.index(j))

        for j in [self.jointX, self.jointY, self.jointZ]:
            if j not in names:
                msg = f'Falta {j} en trajectory. joint_names={names}'
                self.get_logger().error(msg)
                goal_handle.abort()
                return self.make_result(
                    FollowJointTrajectory.Result.INVALID_JOINTS,
                    msg
                )

        idxX = names.index(self.jointX)
        idxY = names.index(self.jointY)
        idxZ = names.index(self.jointZ)

        if len(traj.points) == 0:
            msg = 'Trajectory vacía'
            self.get_logger().error(msg)
            goal_handle.abort()
            return self.make_result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                msg
            )

        # --- Build arrays ---
        t_pts: List[float] = []
        qA_pts: List[List[float]] = []
        qX_pts: List[float] = []
        qY_pts: List[float] = []
        qZ_pts: List[float] = []

        for p in traj.points:
            t = p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec

            if len(p.positions) != len(names):
                msg = 'Point con positions incompatible con joint_names'
                self.get_logger().error(msg)
                goal_handle.abort()
                return self.make_result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    msg
                )

            t_pts.append(float(t))
            qA_pts.append([float(p.positions[i]) for i in idx_a])
            qX_pts.append(float(p.positions[idxX]))
            qY_pts.append(float(p.positions[idxY]))
            qZ_pts.append(float(p.positions[idxZ]))

        # Ensure increasing time_from_start
        if any(t_pts[i + 1] < t_pts[i] for i in range(len(t_pts) - 1)):
            msg = 'time_from_start no es monótono creciente'
            self.get_logger().error(msg)
            goal_handle.abort()
            return self.make_result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                msg
            )

        def interp(a, b, u):
            return a + u * (b - a)

        t_end = t_pts[-1]
        t0 = time.monotonic()
        k = 0

        # Inicializar con el primer punto.
        self.update_last_cmds(
            qA_pts[0],
            qX_pts[0],
            qY_pts[0],
            qZ_pts[0]
        )

        self.get_logger().info(
            f'Recibida trayectoria brazo con {len(traj.points)} puntos, duración {t_end:.3f} s'
        )

        # --- Execution loop ---
        '''
        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Goal cancelado. Mantengo último comando publicado.')
                goal_handle.canceled()
                return self.make_result(
                    FollowJointTrajectory.Result.SUCCESSFUL,
                    'Goal cancelado; bridge mantiene último setpoint.'
                )

            t_now = time.monotonic() - t0

            # Finish:
            # Actualizo el último punto y salgo.
            # El timer sigue publicando ese último punto después del SUCCEEDED.
            if t_now >= t_end:
                self.update_last_cmds(
                    qA_pts[-1],
                    qX_pts[-1],
                    qY_pts[-1],
                    qZ_pts[-1]
                )
                break

            # Advance segment index
            while (k + 1) < len(t_pts) and t_pts[k + 1] <= t_now:
                k += 1

            # Interpolate in segment k..k+1
            if (k + 1) >= len(t_pts):
                a = qA_pts[-1]
                x = qX_pts[-1]
                y = qY_pts[-1]
                z = qZ_pts[-1]
            else:
                ta, tb = t_pts[k], t_pts[k + 1]

                if tb <= ta:
                    u = 0.0
                else:
                    u = clamp((t_now - ta) / (tb - ta), 0.0, 1.0)

                a_k = qA_pts[k]
                a_k1 = qA_pts[k + 1]

                x_k = qX_pts[k]
                x_k1 = qX_pts[k + 1]

                y_k = qY_pts[k]
                y_k1 = qY_pts[k + 1]

                z_k = qZ_pts[k]
                z_k1 = qZ_pts[k + 1]

                a = [interp(a_k[i], a_k1[i], u) for i in range(3)]
                x = interp(x_k, x_k1, u)
                y = interp(y_k, y_k1, u)
                z = interp(z_k, z_k1, u)

            # Actualizo últimos comandos.
            # El timer paralelo los publica a 100 Hz.
            self.update_last_cmds(a, x, y, z)

            time.sleep(self.dt)
        '''
        # --- Execution loop ---
        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Goal cancelado. Mantengo último comando publicado.')
                goal_handle.canceled()
                return self.make_result(
                    FollowJointTrajectory.Result.SUCCESSFUL,
                    'Goal cancelado; bridge mantiene último setpoint.'
                )

            t_now = time.monotonic() - t0

            # --- LLEGADA AL DESTINO (Finish) ---
            if t_now >= t_end:
                self.update_last_cmds(
                    qA_pts[-1], qX_pts[-1], qY_pts[-1], qZ_pts[-1]
                )
                
                # Publicar el último punto a Foxglove en Ticks
                msg_ref = JointState()
                msg_ref.header.stamp = self.get_clock().now().to_msg()
                msg_ref.name = self.joints_a + [self.jointX, self.jointY, self.jointZ]
                
                posiciones_joint_rad = qA_pts[-1] + [qX_pts[-1], qY_pts[-1], qZ_pts[-1]]
                posiciones_motor_ticks = []
                for pos_rad, gr in zip(posiciones_joint_rad, self.gear_ratios):
                    ticks = pos_rad * (gr * 720.0) / (2.0 * math.pi)
                    posiciones_motor_ticks.append(ticks)
                    
                msg_ref.position = posiciones_motor_ticks
                self.pub_referencia.publish(msg_ref)
                break

            # Advance segment index
            while (k + 1) < len(t_pts) and t_pts[k + 1] <= t_now:
                k += 1

            # Interpolate in segment k..k+1
            if (k + 1) >= len(t_pts):
                a = qA_pts[-1]
                x = qX_pts[-1]
                y = qY_pts[-1]
                z = qZ_pts[-1]
            else:
                ta, tb = t_pts[k], t_pts[k + 1]
                if tb <= ta:
                    u = 0.0
                else:
                    u = clamp((t_now - ta) / (tb - ta), 0.0, 1.0)

                a_k, a_k1 = qA_pts[k], qA_pts[k + 1]
                x_k, x_k1 = qX_pts[k], qX_pts[k + 1]
                y_k, y_k1 = qY_pts[k], qY_pts[k + 1]
                z_k, z_k1 = qZ_pts[k], qZ_pts[k + 1]

                a = [interp(a_k[i], a_k1[i], u) for i in range(3)]
                x = interp(x_k, x_k1, u)
                y = interp(y_k, y_k1, u)
                z = interp(z_k, z_k1, u)

            # Actualizo últimos comandos hacia la STM32
            self.update_last_cmds(a, x, y, z)

            # --- INTERPOLACIÓN FOXGLOVE (Durante el movimiento) ---
            msg_ref = JointState()
            msg_ref.header.stamp = self.get_clock().now().to_msg()
            msg_ref.name = self.joints_a + [self.jointX, self.jointY, self.jointZ]
            
            posiciones_joint_rad = a + [x, y, z]
            posiciones_motor_ticks = []
            
            # Convertimos Radianes a Ticks con tus engranajes
            for pos_rad, gr in zip(posiciones_joint_rad, self.gear_ratios):
                ticks = pos_rad * (gr * 720.0) / (2.0 * math.pi)
                posiciones_motor_ticks.append(ticks)
                
            msg_ref.position = posiciones_motor_ticks
            self.pub_referencia.publish(msg_ref)
            # --------------------------------------------------------

            time.sleep(self.dt)
        goal_handle.succeed()

        self.get_logger().info(
            'Trayectoria brazo finalizada. El bridge queda publicando el último setpoint.'
        )

        return self.make_result(
            FollowJointTrajectory.Result.SUCCESSFUL,
            'Trajectory executed; holding last setpoint.'
        )

    def execute_gripper_cb(self, goal_handle):
        traj = goal_handle.request.trajectory

        if not traj.points:
            self.get_logger().error("PINZA: trayectoria vacía")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        point = traj.points[-1]

        try:
            idx7 = traj.joint_names.index("Joint7")
        except ValueError:
            self.get_logger().error(
                f"PINZA: no encontré Joint7 en joint_names={traj.joint_names}"
            )
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        joint7_pos = float(point.positions[idx7])

        threshold = float(self.gripper_close_threshold)

        # Lógica:
        # Joint7 > threshold  -> cerrar
        # Joint7 <= threshold -> abrir
        cerrar = joint7_pos > threshold

        if self.gripper_invert:
            cerrar = not cerrar

        self.enviar_pinza(cerrar)

        self.get_logger().info(
            f"PINZA Joint7={joint7_pos:.5f}, threshold={threshold:.5f} "
            f"-> {'CERRAR' if cerrar else 'ABRIR'} / Bool={cerrar}"
        )

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

def main():
    rclpy.init()

    node = Stm32MoveItBridge()

    # Usamos MultiThreadedExecutor porque el ActionServer duerme en execute_cb.
    # El timer de hold necesita seguir corriendo en paralelo.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



'''
#!/usr/bin/env python3
import math
import time
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Int32MultiArray


def rad_to_cdeg(rad: float) -> int:
    # radians -> centi-degrees
    return int(round(rad * 180.0 / math.pi * 100.0))


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class Stm32MoveItBridge(Node):
    """
    MoveIt FollowJointTrajectory (BRAZO_controller) -> STM32A/STM32B cmd_deg

    STM32A expects: [J1, J2, J3] in cdeg

    STM32B expects: [Y, Z, X] in cdeg
      cmd_cdeg[0] = Y
      cmd_cdeg[1] = Z
      cmd_cdeg[2] = X

    Mapping MoveIt -> STM32B:
      Joint4 -> X
      Joint5 -> Y
      Joint6 -> Z

    Importante:
      El ActionServer ejecuta trayectorias finitas.
      El timer publica SIEMPRE el último comando conocido a 100 Hz.
      Así, cuando la trayectoria termina, la STM32 sigue recibiendo comandos.
    """

    def __init__(self):
        super().__init__('stm32_moveit_bridge')

        self.cb_group = ReentrantCallbackGroup()

        # Publishers
        self.pub_cmd_a = self.create_publisher(Int32MultiArray, '/stm32A/cmd_deg', 10)
        self.pub_cmd_b = self.create_publisher(Int32MultiArray, '/stm32B/cmd_deg', 10)

        # Action server name
        self.action_name = '/BRAZO_controller/follow_joint_trajectory'

        self._as = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            self.execute_cb,
            callback_group=self.cb_group
        )

        # STM32A joints
        self.joints_a = ['Joint1', 'Joint2', 'Joint3']

        # MoveIt joints for wrist
        self.jointX = 'Joint4'  # -> STM32B data[2] X
        self.jointY = 'Joint5'  # -> STM32B data[0] Y
        self.jointZ = 'Joint6'  # -> STM32B data[1] Z

        # Sign corrections
        self.signX = +1
        self.signY = +1
        self.signZ = +1

        # Sending rate
        self.rate_hz = 100.0
        self.dt = 1.0 / self.rate_hz

        # Últimos comandos conocidos.
        # Si están en None, todavía no publica nada.
        self.lastA_cmd: Optional[List[int]] = None
        self.lastB_cmd: Optional[List[int]] = None

        # Lock porque el timer y el action callback pueden correr en paralelo
        self.cmd_lock = threading.Lock()

        # Timer permanente de publicación.
        # Este timer mantiene vivo el stream hacia las STM32.
        self.hold_timer = self.create_timer(
            self.dt,
            self.publish_last_cmds,
            callback_group=self.cb_group
        )

        self.get_logger().info(
            f'Bridge listo: {self.action_name} -> /stm32A/cmd_deg + /stm32B/cmd_deg @ {self.rate_hz:.1f} Hz'
        )
        self.get_logger().info(
            f'STM32A: {self.joints_a} -> [J1,J2,J3]'
        )
        self.get_logger().info(
            f'STM32B mapping: X={self.jointX}(sign {self.signX}) -> data[2], '
            f'Y={self.jointY}(sign {self.signY}) -> data[0], '
            f'Z={self.jointZ}(sign {self.signZ}) -> data[1]'
        )

    def publish_last_cmds(self):
        """
        Publica continuamente el último comando conocido.

        Esto es lo que evita que la STM32 entre en SAFE_HOLD
        cuando una trayectoria terminó correctamente.
        """
        with self.cmd_lock:
            lastA = None if self.lastA_cmd is None else list(self.lastA_cmd)
            lastB = None if self.lastB_cmd is None else list(self.lastB_cmd)

        if lastA is not None:
            msgA = Int32MultiArray()
            msgA.data = lastA
            self.pub_cmd_a.publish(msgA)

        if lastB is not None:
            msgB = Int32MultiArray()
            msgB.data = lastB
            self.pub_cmd_b.publish(msgB)

    def update_last_cmds(self, a, x, y, z):
        """
        Actualiza el último comando conocido.
        El timer se encarga de publicarlo.
        """
        lastA_cmd = [
            rad_to_cdeg(a[0]),
            rad_to_cdeg(a[1]),
            rad_to_cdeg(a[2]),
        ]

        X_cdeg = self.signX * rad_to_cdeg(x)
        Y_cdeg = self.signY * rad_to_cdeg(y)
        Z_cdeg = self.signZ * rad_to_cdeg(z)

        # STM32B expects [Y, Z, X]
        lastB_cmd = [Y_cdeg, Z_cdeg, X_cdeg]

        with self.cmd_lock:
            self.lastA_cmd = lastA_cmd
            self.lastB_cmd = lastB_cmd

    def make_result(self, error_code=0, error_string=''):
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_string
        return result

    def execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)

        # --- Validate joints present ---
        idx_a: List[int] = []

        for j in self.joints_a:
            if j not in names:
                msg = f'Falta {j} en trajectory. joint_names={names}'
                self.get_logger().error(msg)
                goal_handle.abort()
                return self.make_result(
                    FollowJointTrajectory.Result.INVALID_JOINTS,
                    msg
                )

            idx_a.append(names.index(j))

        for j in [self.jointX, self.jointY, self.jointZ]:
            if j not in names:
                msg = f'Falta {j} en trajectory. joint_names={names}'
                self.get_logger().error(msg)
                goal_handle.abort()
                return self.make_result(
                    FollowJointTrajectory.Result.INVALID_JOINTS,
                    msg
                )

        idxX = names.index(self.jointX)
        idxY = names.index(self.jointY)
        idxZ = names.index(self.jointZ)

        if len(traj.points) == 0:
            msg = 'Trajectory vacía'
            self.get_logger().error(msg)
            goal_handle.abort()
            return self.make_result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                msg
            )

        # --- Build arrays ---
        t_pts: List[float] = []
        qA_pts: List[List[float]] = []
        qX_pts: List[float] = []
        qY_pts: List[float] = []
        qZ_pts: List[float] = []

        for p in traj.points:
            t = p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec

            if len(p.positions) != len(names):
                msg = 'Point con positions incompatible con joint_names'
                self.get_logger().error(msg)
                goal_handle.abort()
                return self.make_result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    msg
                )

            t_pts.append(float(t))
            qA_pts.append([float(p.positions[i]) for i in idx_a])
            qX_pts.append(float(p.positions[idxX]))
            qY_pts.append(float(p.positions[idxY]))
            qZ_pts.append(float(p.positions[idxZ]))

        # Ensure increasing time_from_start
        if any(t_pts[i + 1] < t_pts[i] for i in range(len(t_pts) - 1)):
            msg = 'time_from_start no es monótono creciente'
            self.get_logger().error(msg)
            goal_handle.abort()
            return self.make_result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                msg
            )

        def interp(a, b, u):
            return a + u * (b - a)

        t_end = t_pts[-1]
        t0 = time.monotonic()
        k = 0

        # Inicializar con el primer punto.
        self.update_last_cmds(
            qA_pts[0],
            qX_pts[0],
            qY_pts[0],
            qZ_pts[0]
        )

        self.get_logger().info(
            f'Recibida trayectoria con {len(traj.points)} puntos, duración {t_end:.3f} s'
        )

        # --- Execution loop ---
        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Goal cancelado. Mantengo último comando publicado.')
                goal_handle.canceled()
                return self.make_result(
                    FollowJointTrajectory.Result.SUCCESSFUL,
                    'Goal cancelado; bridge mantiene último setpoint.'
                )

            t_now = time.monotonic() - t0

            # Finish:
            # Actualizo el último punto y salgo.
            # El timer sigue publicando ese último punto después del SUCCEEDED.
            if t_now >= t_end:
                self.update_last_cmds(
                    qA_pts[-1],
                    qX_pts[-1],
                    qY_pts[-1],
                    qZ_pts[-1]
                )
                break

            # Advance segment index
            while (k + 1) < len(t_pts) and t_pts[k + 1] <= t_now:
                k += 1

            # Interpolate in segment k..k+1
            if (k + 1) >= len(t_pts):
                a = qA_pts[-1]
                x = qX_pts[-1]
                y = qY_pts[-1]
                z = qZ_pts[-1]
            else:
                ta, tb = t_pts[k], t_pts[k + 1]

                if tb <= ta:
                    u = 0.0
                else:
                    u = clamp((t_now - ta) / (tb - ta), 0.0, 1.0)

                a_k = qA_pts[k]
                a_k1 = qA_pts[k + 1]

                x_k = qX_pts[k]
                x_k1 = qX_pts[k + 1]

                y_k = qY_pts[k]
                y_k1 = qY_pts[k + 1]

                z_k = qZ_pts[k]
                z_k1 = qZ_pts[k + 1]

                a = [interp(a_k[i], a_k1[i], u) for i in range(3)]
                x = interp(x_k, x_k1, u)
                y = interp(y_k, y_k1, u)
                z = interp(z_k, z_k1, u)

            # Actualizo últimos comandos.
            # El timer paralelo los publica a 100 Hz.
            self.update_last_cmds(a, x, y, z)

            time.sleep(self.dt)

        goal_handle.succeed()

        self.get_logger().info(
            'Trayectoria finalizada. El bridge queda publicando el último setpoint.'
        )

        return self.make_result(
            FollowJointTrajectory.Result.SUCCESSFUL,
            'Trajectory executed; holding last setpoint.'
        )


def main():
    rclpy.init()

    node = Stm32MoveItBridge()

    # Importante:
    # Usamos MultiThreadedExecutor porque el ActionServer duerme en execute_cb.
    # El timer de hold necesita seguir corriendo en paralelo.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()












#!/usr/bin/env python3
import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Int32MultiArray


def rad_to_cdeg(rad: float) -> int:
    # radians -> centi-degrees
    return int(round(rad * 180.0 / math.pi * 100.0))


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class Stm32MoveItBridge(Node):
    """
    MoveIt FollowJointTrajectory (BRAZO_controller) -> STM32A/STM32B cmd_deg

    STM32A expects: [J1, J2, J3] in cdeg
    STM32B expects: [Y, Z, X]  in cdeg  (según tu firmware: cmd_cdeg[0]=Y, [1]=Z, [2]=X)
    Mapping MoveIt->STM32B:
      Joint4 -> X (M6)
      Joint5 -> Y
      Joint6 -> Z
    """

    def __init__(self):
        super().__init__('stm32_moveit_bridge')

        # Publishers
        self.pub_cmd_a = self.create_publisher(Int32MultiArray, '/stm32A/cmd_deg', 10)
        self.pub_cmd_b = self.create_publisher(Int32MultiArray, '/stm32B/cmd_deg', 10)

        # Action server name (MoveIt talks to BRAZO_controller)
        self.action_name = '/BRAZO_controller/follow_joint_trajectory'
        self._as = ActionServer(self, FollowJointTrajectory, self.action_name, self.execute_cb)

        # STM32A joints
        self.joints_a = ['Joint1', 'Joint2', 'Joint3']

        # MoveIt joints for wrist
        self.jointX = 'Joint4'  # -> STM32B data[2] (X / M6)
        self.jointY = 'Joint5'  # -> STM32B data[0] (Y)
        self.jointZ = 'Joint6'  # -> STM32B data[1] (Z)

        # Sign corrections (ajustá acá si un eje queda invertido)
        self.signX = +1
        self.signY = +1
        self.signZ = +1

        # Sending rate
        self.rate_hz = 100.0
        self.dt = 1.0 / self.rate_hz

        self.get_logger().info(
            f'Bridge listo: {self.action_name} -> /stm32A/cmd_deg + /stm32B/cmd_deg @ {self.rate_hz:.1f} Hz'
        )
        self.get_logger().info(
            f'STM32A: {self.joints_a} -> [J1,J2,J3]'
        )
        self.get_logger().info(
            f'STM32B mapping: X={self.jointX}(sign {self.signX}) -> data[2], '
            f'Y={self.jointY}(sign {self.signY}) -> data[0], '
            f'Z={self.jointZ}(sign {self.signZ}) -> data[1]'
        )

    def execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)

        # --- Validate joints present ---
        idx_a: List[int] = []
        for j in self.joints_a:
            if j not in names:
                self.get_logger().error(f'Falta {j} en trajectory. joint_names={names}')
                goal_handle.abort()
                return FollowJointTrajectory.Result()
            idx_a.append(names.index(j))

        for j in [self.jointX, self.jointY, self.jointZ]:
            if j not in names:
                self.get_logger().error(f'Falta {j} en trajectory. joint_names={names}')
                goal_handle.abort()
                return FollowJointTrajectory.Result()

        idxX = names.index(self.jointX)
        idxY = names.index(self.jointY)
        idxZ = names.index(self.jointZ)

        if len(traj.points) == 0:
            self.get_logger().error("Trajectory vacía")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # --- Build arrays ---
        t_pts: List[float] = []
        qA_pts: List[List[float]] = []
        qX_pts: List[float] = []
        qY_pts: List[float] = []
        qZ_pts: List[float] = []

        for p in traj.points:
            t = p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec

            if len(p.positions) != len(names):
                self.get_logger().error("Point con positions incompatible con joint_names")
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            t_pts.append(float(t))
            qA_pts.append([float(p.positions[i]) for i in idx_a])
            qX_pts.append(float(p.positions[idxX]))
            qY_pts.append(float(p.positions[idxY]))
            qZ_pts.append(float(p.positions[idxZ]))

        # Ensure increasing time_from_start
        if any(t_pts[i + 1] < t_pts[i] for i in range(len(t_pts) - 1)):
            self.get_logger().error("time_from_start no es monótono creciente")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # --- Helpers ---
        def interp(a, b, u):
            return a + u * (b - a)

        msgA = Int32MultiArray()
        msgB = Int32MultiArray()

        t_end = t_pts[-1]
        t0 = time.time()
        k = 0

        # init last cmds (first point)
        a0 = qA_pts[0]
        x0 = qX_pts[0]
        y0 = qY_pts[0]
        z0 = qZ_pts[0]

        lastA_cmd = [rad_to_cdeg(a0[0]), rad_to_cdeg(a0[1]), rad_to_cdeg(a0[2])]

        X_cdeg = self.signX * rad_to_cdeg(x0)
        Y_cdeg = self.signY * rad_to_cdeg(y0)
        Z_cdeg = self.signZ * rad_to_cdeg(z0)

        # STM32B expects [Y, Z, X]
        lastB_cmd = [Y_cdeg, Z_cdeg, X_cdeg]

        # --- Control loop ---
        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Goal cancelado")
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            t_now = time.time() - t0

            # Finish: publish last point once
            if t_now >= t_end:
                a = qA_pts[-1]
                x = qX_pts[-1]
                y = qY_pts[-1]
                z = qZ_pts[-1]

                lastA_cmd = [rad_to_cdeg(a[0]), rad_to_cdeg(a[1]), rad_to_cdeg(a[2])]

                X_cdeg = self.signX * rad_to_cdeg(x)
                Y_cdeg = self.signY * rad_to_cdeg(y)
                Z_cdeg = self.signZ * rad_to_cdeg(z)

                lastB_cmd = [Y_cdeg, Z_cdeg, X_cdeg]

                msgA.data = lastA_cmd
                msgB.data = lastB_cmd
                self.pub_cmd_a.publish(msgA)
                self.pub_cmd_b.publish(msgB)
                break

            # Advance segment index
            while (k + 1) < len(t_pts) and t_pts[k + 1] <= t_now:
                k += 1

            # Interpolate in segment k..k+1
            if (k + 1) >= len(t_pts):
                a = qA_pts[-1]
                x = qX_pts[-1]
                y = qY_pts[-1]
                z = qZ_pts[-1]
            else:
                ta, tb = t_pts[k], t_pts[k + 1]
                if tb <= ta:
                    u = 0.0
                else:
                    u = clamp((t_now - ta) / (tb - ta), 0.0, 1.0)

                a_k, a_k1 = qA_pts[k], qA_pts[k + 1]
                x_k, x_k1 = qX_pts[k], qX_pts[k + 1]
                y_k, y_k1 = qY_pts[k], qY_pts[k + 1]
                z_k, z_k1 = qZ_pts[k], qZ_pts[k + 1]

                a = [interp(a_k[i], a_k1[i], u) for i in range(3)]
                x = interp(x_k, x_k1, u)
                y = interp(y_k, y_k1, u)
                z = interp(z_k, z_k1, u)

            # Convert to commands
            lastA_cmd = [rad_to_cdeg(a[0]), rad_to_cdeg(a[1]), rad_to_cdeg(a[2])]

            X_cdeg = self.signX * rad_to_cdeg(x)
            Y_cdeg = self.signY * rad_to_cdeg(y)
            Z_cdeg = self.signZ * rad_to_cdeg(z)

            lastB_cmd = [Y_cdeg, Z_cdeg, X_cdeg]

            msgA.data = lastA_cmd
            msgB.data = lastB_cmd
            self.pub_cmd_a.publish(msgA)
            self.pub_cmd_b.publish(msgB)

            time.sleep(self.dt)

        # Republish last setpoint a few times (more robust)
        for _ in range(3):
            msgA.data = lastA_cmd
            msgB.data = lastB_cmd
            self.pub_cmd_a.publish(msgA)
            self.pub_cmd_b.publish(msgB)
            time.sleep(0.01)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()


def main():
    rclpy.init()
    node = Stm32MoveItBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Int32MultiArray


def rad_to_cdeg(rad: float) -> int:
    return int(round(rad * 180.0 / math.pi * 100.0))


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class Stm32BrazoBridge(Node):
    def __init__(self):
        super().__init__('stm32_brazo_bridge')

        self.pub_cmd_a = self.create_publisher(Int32MultiArray, '/stm32A/cmd_deg', 10)
        self.pub_cmd_b = self.create_publisher(Int32MultiArray, '/stm32B/cmd_deg', 10)

        self.action_name = '/BRAZO_controller/follow_joint_trajectory'
        self._as = ActionServer(self, FollowJointTrajectory, self.action_name, self.execute_cb)

        # STM32A
        self.joints_a = ['Joint1', 'Joint2', 'Joint3']

        # MoveIt joints (muñeca)
        # Joint4 NO se controla (servo 6)
        self.joint4 = 'Joint4'
        self.jointY = 'Joint5'  # <-- si ves que está cruzado, cambiá a 'Joint6'
        self.jointZ = 'Joint6'  # <-- si ves que está cruzado, cambiá a 'Joint5'

        # Signos (solo corregimos Y invertido aquí)
        self.signY = -1
        self.signZ = +1

        self.rate_hz = 100.0
        self.dt = 1.0 / self.rate_hz

        self.get_logger().info(
            f'Bridge: {self.action_name} -> /stm32A/cmd_deg + /stm32B/cmd_deg @ {self.rate_hz:.1f} Hz'
        )
        self.get_logger().info(
            f'STM32B mapping: Y={self.jointY} (sign {self.signY}), Z={self.jointZ} (sign {self.signZ}), Joint4 ignored'
        )

    def execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)

        # indices A
        idx_a = []
        for j in self.joints_a:
            if j not in names:
                self.get_logger().error(f'Falta {j} en trajectory. joint_names={names}')
                goal_handle.abort()
                return FollowJointTrajectory.Result()
            idx_a.append(names.index(j))

        # indices B (Y/Z y opcional Joint4)
        for j in [self.jointY, self.jointZ]:
            if j not in names:
                self.get_logger().error(f'Falta {j} en trajectory. joint_names={names}')
                goal_handle.abort()
                return FollowJointTrajectory.Result()

        idxY = names.index(self.jointY)
        idxZ = names.index(self.jointZ)

        if len(traj.points) == 0:
            self.get_logger().error("Trajectory vacía")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # build arrays
        t_pts: List[float] = []
        qA_pts: List[List[float]] = []
        qY_pts: List[float] = []
        qZ_pts: List[float] = []

        for p in traj.points:
            t = p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec
            if len(p.positions) != len(names):
                self.get_logger().error("Point con positions incompatible con joint_names")
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            t_pts.append(float(t))
            qA_pts.append([float(p.positions[i]) for i in idx_a])
            qY_pts.append(float(p.positions[idxY]))
            qZ_pts.append(float(p.positions[idxZ]))

        if any(t_pts[i + 1] < t_pts[i] for i in range(len(t_pts) - 1)):
            self.get_logger().error("time_from_start no es monótono creciente")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        t_end = t_pts[-1]
        t0 = time.time()
        k = 0

        msgA = Int32MultiArray()
        msgB = Int32MultiArray()

        def interp(a, b, u):
            return a + u * (b - a)

        # loop
        lastA_cmd = [rad_to_cdeg(qA_pts[0][0]), rad_to_cdeg(qA_pts[0][1]), rad_to_cdeg(qA_pts[0][2])]
        lastB_cmd = [0, 0, 0]

        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Goal cancelado")
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            t_now = time.time() - t0

            if t_now >= t_end:
                # last point
                a = qA_pts[-1]
                y = qY_pts[-1]
                z = qZ_pts[-1]

                lastA_cmd = [rad_to_cdeg(a[0]), rad_to_cdeg(a[1]), rad_to_cdeg(a[2])]

                Y_cdeg = self.signY * rad_to_cdeg(y)
                Z_cdeg = self.signZ * rad_to_cdeg(z)
                lastB_cmd = [Y_cdeg, Z_cdeg, 0]  # Joint4 ignored

                msgA.data = lastA_cmd
                msgB.data = lastB_cmd
                self.pub_cmd_a.publish(msgA)
                self.pub_cmd_b.publish(msgB)
                break

            while (k + 1) < len(t_pts) and t_pts[k + 1] <= t_now:
                k += 1

            if (k + 1) >= len(t_pts):
                a = qA_pts[-1]
                y = qY_pts[-1]
                z = qZ_pts[-1]
            else:
                ta, tb = t_pts[k], t_pts[k + 1]
                if tb <= ta:
                    u = 0.0
                else:
                    u = clamp((t_now - ta) / (tb - ta), 0.0, 1.0)

                a0, a1 = qA_pts[k], qA_pts[k + 1]
                y0, y1 = qY_pts[k], qY_pts[k + 1]
                z0, z1 = qZ_pts[k], qZ_pts[k + 1]
                x0, x1 = qX_pts[k], qX_pts[k + 1]

                a = [interp(a0[i], a1[i], u) for i in range(3)]
                y = interp(y0, y1, u)
                z = interp(z0, z1, u)
                x = interp(x0, x1, u)

            lastA_cmd = [rad_to_cdeg(a[0]), rad_to_cdeg(a[1]), rad_to_cdeg(a[2])]

            Y_cdeg = self.signY * rad_to_cdeg(y)
            Z_cdeg = self.signZ * rad_to_cdeg(z)
            Z_cdeg = self.signZ * rad_to_cdeg(x)
            lastB_cmd = [Y_cdeg, Z_cdeg, 0]

            msgA.data = lastA_cmd
            msgB.data = lastB_cmd
            self.pub_cmd_a.publish(msgA)
            self.pub_cmd_b.publish(msgB)

            time.sleep(self.dt)

        # republish last
        for _ in range(3):
            msgA.data = lastA_cmd
            msgB.data = lastB_cmd
            self.pub_cmd_a.publish(msgA)
            self.pub_cmd_b.publish(msgB)
            time.sleep(0.01)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()


def main():
    rclpy.init()
    node = Stm32BrazoBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''