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


def rad_to_cdeg(rad: float) -> int:
    # radians -> centi-degrees
    return int(round(rad * 180.0 / math.pi * 100.0))


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class Stm32MoveItBridge(Node):
    def __init__(self):
        super().__init__('stm32_moveit_bridge')

        self.cb_group = ReentrantCallbackGroup()

        # Publishers hacia STM32
        self.pub_cmd_a = self.create_publisher(Int32MultiArray, '/stm32A/cmd_deg', 10)
        self.pub_cmd_b = self.create_publisher(Int32MultiArray, '/stm32B/cmd_deg', 10)
        self.pub_gripper = self.create_publisher(Bool, '/stm32A/gripper_cmd', 10)

        # --- NUEVO: Suscriptores de Encoders (Telemetría real) ---
        self.current_enc_a: Optional[List[int]] = None
        self.current_enc_b: Optional[List[int]] = None

        self.sub_enc_a = self.create_subscription(
            Int32MultiArray, '/stm32A/encoders', self.enc_a_cb, 10, callback_group=self.cb_group
        )
        self.sub_enc_b = self.create_subscription(
            Int32MultiArray, '/stm32B/encoders', self.enc_b_cb, 10, callback_group=self.cb_group
        )

        # Action servers
        #self.arm_action_name = '/BRAZO_controller/follow_joint_trajectory'
        self.arm_action_name = '/arm_controller/follow_joint_trajectory'
        self.gripper_action_name = '/PINZA_controller/follow_joint_trajectory'

        self._as_arm = ActionServer(
            self, FollowJointTrajectory, self.arm_action_name,
            self.execute_cb, callback_group=self.cb_group
        )

        self._as_gripper = ActionServer(
            self, FollowJointTrajectory, self.gripper_action_name,
            self.execute_gripper_cb, callback_group=self.cb_group
        )

        # Configuración de Juntas
        self.joints_a = ['Joint1', 'Joint2', 'Joint3']
        self.jointX = 'Joint4'  # -> STM32B data[2] X
        self.jointY = 'Joint5'  # -> STM32B data[0] Y
        self.jointZ = 'Joint6'  # -> STM32B data[1] Z
        self.gripper_joints = ['Joint7', 'Joint8']

        self.signX = +1
        self.signY = +1
        self.signZ = +1

        # Parámetros tolerancias
        self.declare_parameter("gripper_close_threshold", 0.005)
        self.declare_parameter('gripper_invert', False)
        self.declare_parameter('tolerance_cdeg', 5) # 5 cdeg de tolerancia para dar por terminada la trayectoria
        
        self.gripper_close_threshold = float(self.get_parameter('gripper_close_threshold').value)
        self.gripper_invert = bool(self.get_parameter('gripper_invert').value)
        self.tolerance_cdeg = int(self.get_parameter('tolerance_cdeg').value)

        self.rate_hz = 100.0
        self.dt = 1.0 / self.rate_hz

        self.lastA_cmd: Optional[List[int]] = None
        self.lastB_cmd: Optional[List[int]] = None
        self.cmd_lock = threading.Lock()

        # Timer de publicación constante
        self.hold_timer = self.create_timer(
            self.dt, self.publish_last_cmds, callback_group=self.cb_group
        )

        self.get_logger().info('Bridge LAZO CERRADO listo. Esperando acciones...')

    # --- Callbacks de Encoders ---
    def enc_a_cb(self, msg):
        self.current_enc_a = list(msg.data)

    def enc_b_cb(self, msg):
        self.current_enc_b = list(msg.data)

    def enviar_pinza(self, cerrar: bool):
        msg = Bool()
        msg.data = bool(cerrar)
        self.pub_gripper.publish(msg)

    def publish_last_cmds(self):
        with self.cmd_lock:
            lastA = None if self.lastA_cmd is None else list(self.lastA_cmd)
            lastB = None if self.lastB_cmd is None else list(self.lastB_cmd)

        if lastA is not None:
            msgA = Int32MultiArray(data=lastA)
            self.pub_cmd_a.publish(msgA)

        if lastB is not None:
            msgB = Int32MultiArray(data=lastB)
            self.pub_cmd_b.publish(msgB)

    def update_last_cmds(self, a, x, y, z):
        lastA_cmd = [rad_to_cdeg(a[0]), rad_to_cdeg(a[1]), rad_to_cdeg(a[2])]
        lastB_cmd = [
            self.signY * rad_to_cdeg(y),
            self.signZ * rad_to_cdeg(z),
            self.signX * rad_to_cdeg(x)
        ]
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

        # (Validaciones de joints se mantienen igual...)
        idx_a, idxX, idxY, idxZ = [], None, None, None
        try:
            for j in self.joints_a: idx_a.append(names.index(j))
            idxX = names.index(self.jointX)
            idxY = names.index(self.jointY)
            idxZ = names.index(self.jointZ)
        except ValueError as e:
            self.get_logger().error(f'Faltan articulaciones: {str(e)}')
            goal_handle.abort()
            return self.make_result(FollowJointTrajectory.Result.INVALID_JOINTS)

        t_pts, qA_pts, qX_pts, qY_pts, qZ_pts = [], [], [], [], []
        for p in traj.points:
            t = p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec
            t_pts.append(float(t))
            qA_pts.append([float(p.positions[i]) for i in idx_a])
            qX_pts.append(float(p.positions[idxX]))
            qY_pts.append(float(p.positions[idxY]))
            qZ_pts.append(float(p.positions[idxZ]))

        def interp(a, b, u): return a + u * (b - a)

        t_end = t_pts[-1]
        t0 = time.monotonic()
        k = 0

        self.update_last_cmds(qA_pts[0], qX_pts[0], qY_pts[0], qZ_pts[0])
        self.get_logger().info(f'Ejecutando trayectoria: {t_end:.3f} s')

        # --- BUCLE PRINCIPAL DE INTERPOLACIÓN ---
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self.make_result(FollowJointTrajectory.Result.SUCCESSFUL, 'Cancelado')

            t_now = time.monotonic() - t0

            # --- NUEVA LÓGICA: LAZO CERRADO FÍSICO ---
            if t_now >= t_end:
                self.update_last_cmds(qA_pts[-1], qX_pts[-1], qY_pts[-1], qZ_pts[-1])
                
                self.get_logger().info("Tiempo finalizado. Esperando confirmación de encoders...")
                
                # Targets físicos en centi-grados
                target_A = [rad_to_cdeg(qA_pts[-1][0]), rad_to_cdeg(qA_pts[-1][1]), rad_to_cdeg(qA_pts[-1][2])]
                target_B = [
                    self.signY * rad_to_cdeg(qY_pts[-1]),
                    self.signZ * rad_to_cdeg(qZ_pts[-1]),
                    self.signX * rad_to_cdeg(qX_pts[-1])
                ]
                
                wait_start = time.monotonic()
                timeout_espera_segundos = 5.0 # Corta la espera si el brazo se trabó físicamente
                
                while True:
                    if self.current_enc_a is None or self.current_enc_b is None:
                        time.sleep(0.05)
                        continue
                        
                    # Calcular error máximo
                    try:
                        err_a = max(abs(target_A[i] - self.current_enc_a[i]) for i in range(3))
                        err_b = max(abs(target_B[i] - self.current_enc_b[i]) for i in range(3))
                    except IndexError:
                        break # Prevención de crash si falla un cable
                        
                    if err_a <= self.tolerance_cdeg and err_b <= self.tolerance_cdeg:
                        self.get_logger().info("¡Llegada física confirmada por encoders!")
                        break
                        
                    if time.monotonic() - wait_start > timeout_espera_segundos:
                        self.get_logger().warn(f"Timeout físico. El brazo no pudo llegar a la cota exacta. Err A:{err_a} Err B:{err_b}")
                        break
                        
                    time.sleep(0.05)
                break

            while (k + 1) < len(t_pts) and t_pts[k + 1] <= t_now: k += 1

            if (k + 1) >= len(t_pts):
                a, x, y, z = qA_pts[-1], qX_pts[-1], qY_pts[-1], qZ_pts[-1]
            else:
                ta, tb = t_pts[k], t_pts[k + 1]
                u = 0.0 if tb <= ta else clamp((t_now - ta) / (tb - ta), 0.0, 1.0)
                
                a = [interp(qA_pts[k][i], qA_pts[k + 1][i], u) for i in range(3)]
                x = interp(qX_pts[k], qX_pts[k + 1], u)
                y = interp(qY_pts[k], qY_pts[k + 1], u)
                z = interp(qZ_pts[k], qZ_pts[k + 1], u)

            self.update_last_cmds(a, x, y, z)
            time.sleep(self.dt)

        goal_handle.succeed()
        return self.make_result(FollowJointTrajectory.Result.SUCCESSFUL, 'Finalizado Lazo Cerrado')

    def execute_gripper_cb(self, goal_handle):
        traj = goal_handle.request.trajectory

        # --- VALIDACIONES DE SEGURIDAD (Salida inmediata sin sleep) ---
        if not traj.points:
            self.get_logger().error("PINZA: trayectoria vacía")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        try:
            idx7 = traj.joint_names.index("Joint7")
        except ValueError:
            self.get_logger().error(
                f"PINZA: no encontré Joint7 en joint_names={traj.joint_names}"
            )
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # --- PROCESAMIENTO DEL COMANDO ---
        point = traj.points[-1]
        joint7_pos = float(point.positions[idx7])
        threshold = float(self.gripper_close_threshold)

        # Lógica de conversión binaria
        cerrar = joint7_pos > threshold
        if self.gripper_invert:
            cerrar = not cerrar

        # Publica el booleano hacia la STM32 (micro-ROS)
        self.enviar_pinza(cerrar)

        self.get_logger().info(
            f"PINZA Joint7={joint7_pos:.5f}, threshold={threshold:.5f} "
            f"-> {'CERRAR' if cerrar else 'ABRIR'} / Bool={cerrar}"
        )

        # --- ESPERA FÍSICA Y CIERRE DEL GOAL ---
        # 1. Simula el tiempo que le toma a los engranajes de LEGO abrir/cerrar
        time.sleep(2.0) 

        # 2. Le avisa a MoveIt que la acción concluyó correctamente
        goal_handle.succeed()
        
        # 3. Retorna el resultado esperado por el Action Server
        return FollowJointTrajectory.Result()

    #def execute_gripper_cb(self, goal_handle):
    #    traj = goal_handle.request.trajectory
    #    if not traj.points:
    #        goal_handle.abort()
    #        return FollowJointTrajectory.Result()

    #    point = traj.points[-1]
    #    idx7 = traj.joint_names.index("Joint7")
    #    joint7_pos = float(point.positions[idx7])

    #    cerrar = joint7_pos > self.gripper_close_threshold
    #    if self.gripper_invert: cerrar = not cerrar

    #    self.enviar_pinza(cerrar)
    #    time.sleep(1.0) # Espera 1 segundo físico para asegurar que agarró la pieza
        
    #    goal_handle.succeed()
    #    return FollowJointTrajectory.Result()


def main():
    rclpy.init()
    node = Stm32MoveItBridge()
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