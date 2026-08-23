#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import pinocchio as pin
import numpy as np


class GeneradorIdaYVuelta(Node):
    def __init__(self):
        super().__init__('generador_ida_y_vuelta_cartesiano')

        # Rutas y configuración del modelo Pinocchio
        self.urdf_path = '/root/ros_ws/src/lego/urdf/Ensamblaje2.urdf'
        self.tip_link = 'Pinza'

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(self.tip_link)

        self.joint_order = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        # Parámetros de control
        self.dt = 0.02
        self.distancia_z = 0.20     # Distancia de ida (positivo o negativo)
        self.vmax = 0.1             # Velocidad máxima en m/s
        self.damping = 1e-3
        self.ganancia_pos = 1.0
        self.ganancia_ori = 2.0

        # Postura física inicial de la cual parte el robot (Home)
        self.q_inicial = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.arm_action_name = '/BRAZO_controller/follow_joint_trajectory'
        self.traj_client = ActionClient(self, FollowJointTrajectory, self.arm_action_name)

        # Iniciar cálculo y ejecución
        self.ejecutar_ciclo_completo()

    def Z_parametrizado_vel_max(self, x0, xf, vmax):
        """Genera los coeficientes del quíntico y el tiempo total T"""
        D = xf - x0
        if abs(D) < 1e-12:
            return np.array([x0, 0.0, 0.0, 0.0, 0.0, 0.0]), 0.0

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

    def paso_cinematico(self, q_actual, oMf_inicio, Zpos, Zvel):
        """Calcula 1 diferencial (dt) de cinemática inversa y devuelve el nuevo 'q'"""
        q_full = np.zeros(self.model.nq)
        q_full[0:6] = q_actual
        
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.computeJointJacobians(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        
        oMf_actual = self.data.oMf[self.frame_id]

        # Pose Deseada (El origen absoluto + el corrimiento actual en Z)
        oMf_deseada = pin.SE3(oMf_inicio)
        oMf_deseada.translation[2] = oMf_inicio.translation[2] + Zpos

        # Errores
        err_pos = oMf_deseada.translation - oMf_actual.translation
        R_error = oMf_deseada.rotation @ oMf_actual.rotation.T
        err_ori = pin.log3(R_error)
        twist_error = np.concatenate([err_pos, err_ori])

        # Jacobiano
        J = pin.getFrameJacobian(self.model, self.data, self.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_arm = J[:, 0:6]

        K = np.diag([self.ganancia_pos, self.ganancia_pos, self.ganancia_pos,
                     self.ganancia_ori, self.ganancia_ori, self.ganancia_ori])

        twist_ref = np.array([0.0, 0.0, Zvel, 0.0, 0.0, 0.0])
        twist_cmd = twist_ref + K @ twist_error

        # Resolver IK con Damped Least Squares
        A = J_arm @ J_arm.T + self.damping * np.eye(6)
        dq_arm = J_arm.T @ np.linalg.solve(A, twist_cmd)

        # Integrar
        # 1. Armamos el vector completo de velocidades (v)
        dq_full = np.zeros(self.model.nv)
        dq_full[0:6] = dq_arm

        # 2. Reconstruimos el vector completo de posición (q)
        q_full_actual = np.zeros(self.model.nq)
        q_full_actual[0:6] = q_actual

        # 3. Integración topológica segura con Pinocchio
        q_full_next = pin.integrate(self.model, q_full_actual, dq_full * self.dt)

        # 4. Devolvemos únicamente el segmento del brazo (primeros 6 elementos)
        return q_full_next[0:6]

    def calcular_puntos_offline(self):
        puntos_articulares = []
        q_actual = self.q_inicial.copy()

        # 1. Fijamos la Pose Absoluta Inicial (El cero cartesiano relativo)
        q_full = np.zeros(self.model.nq)
        q_full[0:6] = q_actual
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        oMf_inicio = pin.SE3(self.data.oMf[self.frame_id])

        # --- FASE 1: IDA (Desde 0 hasta distancia_z) ---
        coef_ida, T_ida = self.Z_parametrizado_vel_max(0.0, self.distancia_z, self.vmax)
        N_ida = int(round(T_ida / self.dt))
        t = 0.0
        
        for _ in range(N_ida):
            Zpos = self.evaluar_polinomio(coef_ida, t)
            Zvel = self.evaluar_derivada_polinomio(coef_ida, t)
            q_actual = self.paso_cinematico(q_actual, oMf_inicio, Zpos, Zvel)
            puntos_articulares.append(q_actual.copy())
            t += self.dt

        # --- PAUSA DE 1 SEGUNDO (Mantiene la pose) ---
        N_pausa = int(1.0 / self.dt)
        for _ in range(N_pausa):
            puntos_articulares.append(q_actual.copy())

        # --- FASE 2: VUELTA (Desde distancia_z hasta 0) ---
        coef_vuelta, T_vuelta = self.Z_parametrizado_vel_max(self.distancia_z, 0.0, self.vmax)
        N_vuelta = int(round(T_vuelta / self.dt))
        t = 0.0

        for _ in range(N_vuelta):
            Zpos = self.evaluar_polinomio(coef_vuelta, t)
            Zvel = self.evaluar_derivada_polinomio(coef_vuelta, t)
            q_actual = self.paso_cinematico(q_actual, oMf_inicio, Zpos, Zvel)
            puntos_articulares.append(q_actual.copy())
            t += self.dt

        return puntos_articulares

    def ejecutar_ciclo_completo(self):
        self.get_logger().info('Calculando trayectoria Cartesiana: Ida -> Pausa -> Vuelta...')
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

        self.get_logger().info(f'Enviando {len(goal_msg.trajectory.points)} puntos. ¡A observar al robot!')
        future = self.traj_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('El bridge rechazó la trayectoria.')
            return
        self.get_logger().info('¡Trayectoria aceptada! Ejecutando interpolación en STM32...')

def main(args=None):
    rclpy.init(args=args)
    node = GeneradorIdaYVuelta()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()