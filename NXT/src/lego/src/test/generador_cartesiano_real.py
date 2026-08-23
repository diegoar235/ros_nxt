#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import pinocchio as pin
import numpy as np


class GeneradorCartesianoRealZ(Node):
    def __init__(self):
        super().__init__('generador_cartesiano_real_z')

        # Rutas y configuración del modelo Pinocchio
        self.urdf_path = '/root/ros_ws/src/lego/urdf/Ensamblaje2.urdf'
        self.tip_link = 'Pinza'

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(self.tip_link)

        self.joint_order = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        # Parámetros cinemáticos y de trayectoria
        self.dt = 0.02              # Paso de tiempo coincidente con tu bucle original
        self.duracion = 4.0         # Duración total del movimiento
        self.distancia_z = 0.20     # Desplazamiento cartesiano objetivo en metros
        self.damping = 1e-3         # Factor para evitar singularidades
        self.ganancia_pos = 1.0     # Ganancia proporcional cartesiana
        self.ganancia_ori = 2.0

        # Posición inicial por defecto del manipulador (en radianes)
        # TIP: Idealmente podés inicializarla leyendo una pose estática del archivo YAML
        self.q_inicial = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Instanciar el cliente de acciones que dialogará con tu bridge real
        self.arm_action_name = '/BRAZO_controller/follow_joint_trajectory'
        self.traj_client = ActionClient(self, FollowJointTrajectory, self.arm_action_name)

        # Iniciar proceso de cálculo y envío
        self.ejecutar_trayectoria_cartesiana()

    def Z_parametrizado_vel_max(self, x0, xf, vmax):
        D = xf - x0
        if abs(D) < 1e-12:
            return np.array([x0, 0.0, 0.0, 0.0, 0.0, 0.0]), 0.0

        if vmax <= 0:
            raise ValueError("vmax debe ser mayor que cero")

        T = 1.875 * abs(D) / vmax
        a0 = x0
        a1 = a2 = 0.0
        a3 = 10.0 * D / T**3
        a4 = -15.0 * D / T**4
        a5 = 6.0 * D / T**5

        return np.array([a0, a1, a2, a3, a4, a5]), T

    def evaluar_polinomio(self, coef, t):
        return sum(coef[i] * t**i for i in range(len(coef)))

    def evaluar_derivada_polinomio(self, coef, t):
        return sum(i * coef[i] * t**(i - 1) for i in range(1, len(coef)))

    def calcular_puntos_offline(self):
        """
        Calcula de antemano toda la cinemática inversa cartesiana paso a paso
        evitando depender de la red en tiempo de ejecución.
        """
        puntos_articulares = []
        q_actual = self.q_inicial.copy()
        
        # Inicializar perfiles de quínticos
        coef, T_total = self.Z_parametrizado_vel_max(0.0, self.distancia_z, 0.1)
        
        # Calcular los pasos necesarios para cubrir la duración completa
        N_pasos = int(round(self.duracion / self.dt))
        t = 0.0

        # Obtener pose de partida cartesiana basándonos en q_inicial
        q_full = np.zeros(self.model.nq)
        q_full[0:6] = q_actual
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        oMf_inicio = pin.SE3(self.data.oMf[self.frame_id])

        for _ in range(N_pasos):
            Zpos = self.evaluar_polinomio(coef, t)
            Zvel = self.evaluar_derivada_polinomio(coef, t)

            # Re-evaluar el estado actual de cinemática directa
            q_full[0:6] = q_actual
            pin.forwardKinematics(self.model, self.data, q_full)
            pin.computeJointJacobians(self.model, self.data, q_full)
            pin.updateFramePlacements(self.model, self.data)
            oMf_actual = self.data.oMf[self.frame_id]

            # Definir la Pose deseada de la rampa cartesiana
            oMf_deseada = pin.SE3(oMf_inicio)
            oMf_deseada.translation[2] = oMf_inicio.translation[2] + Zpos

            # Vector de error (Twist de control)
            err_pos = oMf_deseada.translation - oMf_actual.translation
            R_error = oMf_deseada.rotation @ oMf_actual.rotation.T
            err_ori = pin.log3(R_error)
            twist_error = np.concatenate([err_pos, err_ori])

            # Jacobiano y velocidades deseadas (Feedforward)
            J = pin.getFrameJacobian(self.model, self.data, self.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_arm = J[:, 0:6]

            K = np.diag([self.ganancia_pos, self.ganancia_pos, self.ganancia_pos,
                         self.ganancia_ori, self.ganancia_ori, self.ganancia_ori])

            twist_ref = np.array([0.0, 0.0, Zvel, 0.0, 0.0, 0.0])
            twist_cmd = twist_ref + K @ twist_error

            # Pseudoinversa amortiguada por mínimos cuadrados (Damped Least Squares)
            A = J_arm @ J_arm.T + self.damping * np.eye(6)
            dq_arm = J_arm.T @ np.linalg.solve(A, twist_cmd)

            # Integración del paso numérico
            q_actual += dq_arm * self.dt
            puntos_articulares.append(q_actual.copy())

            t += self.dt

        return puntos_articulares

    def ejecutar_trayectoria_cartesiana(self):
        self.get_logger().info('Calculando puntos en espacio cartesiano...')
        lista_q = self.calcular_puntos_offline()

        self.get_logger().info(f'Esperando al servidor de acción {self.arm_action_name}...')
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('El bridge stm32_moveit_bridge no está activo.')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_order

        for i, q_arm in enumerate(lista_q):
            punto = JointTrajectoryPoint()
            punto.positions = [float(x) for x in q_arm]
            punto.velocities = [0.0] * 6

            # Estructurar la estampa de tiempo acumulativa
            t_acumulado = i * self.dt
            sec = int(t_acumulado)
            nanosec = int((t_acumulado - sec) * 1e9)
            punto.time_from_start = Duration(sec=sec, nanosec=nanosec)

            goal_msg.trajectory.points.append(punto)

        self.get_logger().info(f'Enviando {len(goal_msg.trajectory.points)} puntos cartesianos al brazo real.')
        future = self.traj_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('El bridge de las STM32 rechazó la trayectoria cartesiana.')
            return
        self.get_logger().info('Trayectoria aceptada por las STM32, ejecutando interpolación local...')


def main(args=None):
    rclpy.init(args=args)
    node = GeneradorCartesianoRealZ()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()