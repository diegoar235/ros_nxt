#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import pinocchio as pin


class CartesianZKeepOrientation(Node):

    def __init__(self):
        super().__init__('cartesian_z_keep_orientation')

        # =========================================================
        # Parámetros ROS
        # =========================================================
        self.declare_parameter(
            'urdf_path',
            '/root/ros_ws/src/lego/urdf/Ensamblaje.urdf'
        )

        self.declare_parameter('tip_link', 'Pinza')
        self.declare_parameter('controller_topic', '/arm_controller/joint_trajectory')

        # Periodo de control
        self.declare_parameter('dt', 0.05)

        # Trayectoria cartesiana en Z
        self.declare_parameter('amplitude', 0.1)      # 2 cm
        self.declare_parameter('frequency', 0.1)      # Hz
        self.declare_parameter('axis_sign', 1.0)

        # Control cartesiano
        self.declare_parameter('kp_position', 2.0)
        self.declare_parameter('kp_orientation', 1.5)

        # Pseudoinversa amortiguada
        self.declare_parameter('damping', 0.08)

        # Límite de velocidad articular
        self.declare_parameter('qdot_max', 0.6)

        # Pesos para posición y orientación
        self.declare_parameter('position_weight', 1.0)
        self.declare_parameter('orientation_weight', 3.0)

        # =========================================================
        # Leer parámetros
        # =========================================================
        self.urdf_path = self.get_parameter('urdf_path').value
        self.tip_link = self.get_parameter('tip_link').value
        self.controller_topic = self.get_parameter('controller_topic').value

        self.dt = float(self.get_parameter('dt').value)
        self.A = float(self.get_parameter('amplitude').value)
        self.f = float(self.get_parameter('frequency').value)
        self.axis_sign = float(self.get_parameter('axis_sign').value)

        self.kp_position = float(self.get_parameter('kp_position').value)
        self.kp_orientation = float(self.get_parameter('kp_orientation').value)

        self.damping = float(self.get_parameter('damping').value)
        self.qdot_max = float(self.get_parameter('qdot_max').value)

        self.position_weight = float(self.get_parameter('position_weight').value)
        self.orientation_weight = float(self.get_parameter('orientation_weight').value)

        # Movimiento en Z global
        self.axis_vector = self.axis_sign * np.array([0.0, 0.0, 1.0])

        self.get_logger().info('Control cartesiano en Z manteniendo orientación inicial')
        self.get_logger().info(f'axis_vector = {self.axis_vector}')
        self.get_logger().info(f'amplitude = {self.A}')
        self.get_logger().info(f'frequency = {self.f}')
        self.get_logger().info(f'dt = {self.dt}')
        self.get_logger().info(f'kp_position = {self.kp_position}')
        self.get_logger().info(f'kp_orientation = {self.kp_orientation}')
        self.get_logger().info(f'damping = {self.damping}')
        self.get_logger().info(f'qdot_max = {self.qdot_max}')
        self.get_logger().info(f'position_weight = {self.position_weight}')
        self.get_logger().info(f'orientation_weight = {self.orientation_weight}')

        # =========================================================
        # Modelo Pinocchio
        # =========================================================
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        if not self.model.existFrame(self.tip_link):
            self.get_logger().error(f'No existe el frame "{self.tip_link}"')
            self.print_available_frames()
            raise RuntimeError(f'Frame {self.tip_link} no encontrado')

        self.frame_id = self.model.getFrameId(self.tip_link)

        self.get_logger().info(f'Modelo cargado desde: {self.urdf_path}')
        self.get_logger().info(f'nq = {self.model.nq}, nv = {self.model.nv}')
        self.get_logger().info(f'TCP = {self.tip_link}, frame_id = {self.frame_id}')

        # =========================================================
        # Juntas del brazo
        # =========================================================
        self.arm_joint_names = [
            'Joint1',
            'Joint2',
            'Joint3',
            'Joint4',
            'Joint5',
            'Joint6',
        ]

        # Leemos solo las 6 juntas del brazo.
        # Joint7 y Joint8 son dedos, y no afectan al frame Pinza.
        self.feedback_joint_names = [
            'Joint1',
            'Joint2',
            'Joint3',
            'Joint4',
            'Joint5',
            'Joint6',
        ]

        self.q_current = np.zeros(self.model.nq)
        self.joint_state_received = False

        # Referencias iniciales
        self.t0 = None
        self.p_start = None
        self.R_start = None

        # =========================================================
        # ROS
        # =========================================================
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            self.controller_topic,
            10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Nodo iniciado.')

    # =============================================================
    # Mostrar frames disponibles
    # =============================================================
    def print_available_frames(self):
        for frame in self.model.frames:
            self.get_logger().info(f'Frame: {frame.name}')

    # =============================================================
    # Leer joint states
    # =============================================================
    def joint_state_callback(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))

        q = np.zeros(self.model.nq)

        for i, joint_name in enumerate(self.feedback_joint_names):
            if joint_name in name_to_pos:
                q[i] = name_to_pos[joint_name]
            else:
                self.get_logger().warn(
                    f'{joint_name} no aparece en /joint_states',
                    throttle_duration_sec=2.0
                )

        # Los dedos quedan en cero. No afectan a Pinza.
        if self.model.nq >= 8:
            q[6] = 0.0
            q[7] = 0.0

        self.q_current = q
        self.joint_state_received = True

    # =============================================================
    # FK y Jacobiano completo 6D
    # =============================================================
    def get_tcp_pose_and_jacobian(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        T = self.data.oMf[self.frame_id]

        p = T.translation.copy()
        R = T.rotation.copy()

        J_full = pin.getFrameJacobian(
            self.model,
            self.data,
            self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )

        # Jacobiano del brazo: Joint1...Joint6
        # Filas: vx, vy, vz, wx, wy, wz
        # Columnas: Joint1...Joint6
        J_arm = J_full[:, 0:6]

        return p, R, J_arm

    # =============================================================
    # Pseudoinversa amortiguada ponderada
    # =============================================================
    def damped_pseudoinverse(self, J):
        m, n = J.shape

        return J.T @ np.linalg.inv(
            J @ J.T + self.damping**2 * np.eye(m)
        )

    # =============================================================
    # Error de orientación
    # =============================================================
    def orientation_error(self, R_des, R_current):
        """
        Devuelve el error angular como vector 3D.

        R_error representa la rotación que lleva desde la orientación actual
        hacia la orientación deseada.
        """
        R_error = R_des @ R_current.T
        error_ori = pin.log3(R_error)

        return error_ori

    # =============================================================
    # Publicar comando de posición articular
    # =============================================================
    def publish_joint_position(self, q_cmd):
        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = q_cmd[0:6].tolist()

        total_ns = int(self.dt * 1e9)
        point.time_from_start.sec = total_ns // 1_000_000_000
        point.time_from_start.nanosec = total_ns % 1_000_000_000

        msg.points.append(point)
        self.traj_pub.publish(msg)

    # =============================================================
    # Trayectoria senoidal en Z
    # =============================================================
    def compute_z_reference(self, t):
        z_des = self.A * math.sin(2.0 * math.pi * self.f * t)

        vz_des = (
            self.A
            * 2.0
            * math.pi
            * self.f
            * math.cos(2.0 * math.pi * self.f * t)
        )

        return z_des, vz_des

    # =============================================================
    # Diagnóstico simple del Jacobiano
    # =============================================================
    def analyze_jacobian(self, J):
        try:
            S = np.linalg.svd(J, compute_uv=False)
            sigma_min = S[-1]
            sigma_max = S[0]

            if sigma_min < 1e-10:
                cond = float('inf')
            else:
                cond = sigma_max / sigma_min

            rank = np.linalg.matrix_rank(J)

            return rank, sigma_min, cond

        except Exception:
            return -1, 0.0, float('inf')

    # =============================================================
    # Loop principal de control
    # =============================================================
    def control_loop(self):

        if not self.joint_state_received:
            self.get_logger().warn(
                'Esperando /joint_states...',
                throttle_duration_sec=2.0
            )
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        q = self.q_current.copy()

        p_current, R_current, J_arm = self.get_tcp_pose_and_jacobian(q)

        # ---------------------------------------------------------
        # Inicialización: guardar pose inicial
        # ---------------------------------------------------------
        if self.t0 is None:
            self.t0 = now
            self.p_start = p_current.copy()
            self.R_start = R_current.copy()

            self.get_logger().info(
                f'Pose inicial guardada. p_start = '
                f'[{self.p_start[0]:.4f}, {self.p_start[1]:.4f}, {self.p_start[2]:.4f}]'
            )

            self.get_logger().info('R_start guardada. La orientación deseada será constante.')
            return

        t = now - self.t0

        # ---------------------------------------------------------
        # Referencia cartesiana: solo cambia Z
        # ---------------------------------------------------------
        z_des, vz_des = self.compute_z_reference(t)

        p_des = self.p_start + z_des * self.axis_vector
        R_des = self.R_start

        v_ff = vz_des * self.axis_vector

        # ---------------------------------------------------------
        # Error de posición
        # ---------------------------------------------------------
        error_pos = p_des - p_current

        v_cmd = self.kp_position * error_pos + v_ff

        # ---------------------------------------------------------
        # Error de orientación
        # ---------------------------------------------------------
        error_ori = self.orientation_error(R_des, R_current)

        w_cmd = self.kp_orientation * error_ori

        # ---------------------------------------------------------
        # Comando cartesiano 6D
        # ---------------------------------------------------------
        xdot = np.concatenate((v_cmd, w_cmd))

        # ---------------------------------------------------------
        # Ponderación: ayuda a que no se pierda orientación
        # ---------------------------------------------------------
        W = np.diag([
            self.position_weight,
            self.position_weight,
            self.position_weight,
            self.orientation_weight,
            self.orientation_weight,
            self.orientation_weight,
        ])

        J_weighted = W @ J_arm
        xdot_weighted = W @ xdot

        # ---------------------------------------------------------
        # qdot = J# xdot
        # ---------------------------------------------------------
        J_pinv = self.damped_pseudoinverse(J_weighted)

        qdot_arm = J_pinv @ xdot_weighted

        qdot_arm = np.clip(qdot_arm, -self.qdot_max, self.qdot_max)

        # ---------------------------------------------------------
        # Integración con feedback real
        # ---------------------------------------------------------
        q_cmd = q.copy()
        q_cmd[0:6] = q[0:6] + qdot_arm * self.dt

        # Dedos quietos
        if self.model.nq >= 8:
            q_cmd[6] = 0.0
            q_cmd[7] = 0.0

        # ---------------------------------------------------------
        # Diagnóstico
        # ---------------------------------------------------------
        rank, sigma_min, cond = self.analyze_jacobian(J_arm)

        self.get_logger().info(
            f'p_current = [{p_current[0]:.4f}, {p_current[1]:.4f}, {p_current[2]:.4f}] | '
            f'p_des = [{p_des[0]:.4f}, {p_des[1]:.4f}, {p_des[2]:.4f}] | '
            f'err_pos = [{error_pos[0]:.4f}, {error_pos[1]:.4f}, {error_pos[2]:.4f}]',
            throttle_duration_sec=1.0
        )

        self.get_logger().info(
            f'err_ori = [{error_ori[0]:.4f}, {error_ori[1]:.4f}, {error_ori[2]:.4f}] | '
            f'v_cmd = [{v_cmd[0]:.4f}, {v_cmd[1]:.4f}, {v_cmd[2]:.4f}] | '
            f'w_cmd = [{w_cmd[0]:.4f}, {w_cmd[1]:.4f}, {w_cmd[2]:.4f}]',
            throttle_duration_sec=1.0
        )

        self.get_logger().info(
            f'J rank={rank} | sigma_min={sigma_min:.6f} | cond={cond:.2f} | '
            f'qdot = {[round(float(x), 4) for x in qdot_arm]}',
            throttle_duration_sec=1.0
        )

        # ---------------------------------------------------------
        # Publicar
        # ---------------------------------------------------------
        self.publish_joint_position(q_cmd)


def main(args=None):
    rclpy.init(args=args)

    node = CartesianZKeepOrientation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()