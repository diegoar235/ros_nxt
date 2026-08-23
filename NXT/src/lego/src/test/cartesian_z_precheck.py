#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import pinocchio as pin


class CartesianZKeepOrientationPrecheck(Node):

    def __init__(self):
        super().__init__('cartesian_z_keep_orientation_precheck')

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
        # OJO: 0.02 m = 2 cm
        self.declare_parameter('amplitude', 0.02)
        self.declare_parameter('frequency', 0.1)
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
        # Parámetros de precheck
        # =========================================================
        self.declare_parameter('enable_precheck', True)
        self.declare_parameter('precheck_duration', 10.0)

        # Criterios de singularidad
        self.declare_parameter('sigma_min_warn', 0.05)
        self.declare_parameter('sigma_min_stop', 0.01)
        self.declare_parameter('cond_warn', 100.0)
        self.declare_parameter('cond_stop', 300.0)

        # Errores máximos aceptables durante la simulación interna
        self.declare_parameter('max_position_error_allowed', 0.03)
        self.declare_parameter('max_orientation_error_allowed', 0.25)

        # Si True, ante WARNING permite moverse.
        # Si False, cualquier WARNING bloquea el movimiento.
        self.declare_parameter('allow_warnings', True)

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

        self.enable_precheck = bool(self.get_parameter('enable_precheck').value)
        self.precheck_duration = float(self.get_parameter('precheck_duration').value)

        self.sigma_min_warn = float(self.get_parameter('sigma_min_warn').value)
        self.sigma_min_stop = float(self.get_parameter('sigma_min_stop').value)
        self.cond_warn = float(self.get_parameter('cond_warn').value)
        self.cond_stop = float(self.get_parameter('cond_stop').value)

        self.max_position_error_allowed = float(
            self.get_parameter('max_position_error_allowed').value
        )
        self.max_orientation_error_allowed = float(
            self.get_parameter('max_orientation_error_allowed').value
        )

        self.allow_warnings = bool(self.get_parameter('allow_warnings').value)

        # Movimiento en Z global
        self.axis_vector = self.axis_sign * np.array([0.0, 0.0, 1.0])

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

        # Estado del precheck
        self.precheck_done = False
        self.motion_enabled = False

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

        self.print_startup_info()

    # =============================================================
    # Info inicial
    # =============================================================
    def print_startup_info(self):
        self.get_logger().info('Nodo iniciado: control cartesiano Z con orientación fija + precheck')
        self.get_logger().info(f'axis_vector = {self.axis_vector}')
        self.get_logger().info(f'amplitude = {self.A:.4f} m')
        self.get_logger().info(f'frequency = {self.f:.4f} Hz')
        self.get_logger().info(f'dt = {self.dt:.4f} s')
        self.get_logger().info(f'kp_position = {self.kp_position}')
        self.get_logger().info(f'kp_orientation = {self.kp_orientation}')
        self.get_logger().info(f'damping = {self.damping}')
        self.get_logger().info(f'qdot_max = {self.qdot_max} rad/s')
        self.get_logger().info(f'position_weight = {self.position_weight}')
        self.get_logger().info(f'orientation_weight = {self.orientation_weight}')
        self.get_logger().info(f'enable_precheck = {self.enable_precheck}')
        self.get_logger().info(f'precheck_duration = {self.precheck_duration} s')
        self.get_logger().info(f'sigma_min_warn = {self.sigma_min_warn}')
        self.get_logger().info(f'sigma_min_stop = {self.sigma_min_stop}')
        self.get_logger().info(f'cond_warn = {self.cond_warn}')
        self.get_logger().info(f'cond_stop = {self.cond_stop}')

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

        # Si el modelo tiene dedos u otras juntas, las dejamos en cero.
        # En tu caso Joint7 y Joint8 no afectan el frame Pinza.
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
    # Pseudoinversa amortiguada
    # =============================================================
    def damped_pseudoinverse(self, J):
        m, _ = J.shape

        return J.T @ np.linalg.inv(
            J @ J.T + self.damping**2 * np.eye(m)
        )

    # =============================================================
    # Error de orientación
    # =============================================================
    def orientation_error(self, R_des, R_current):
        """
        Devuelve error angular como vector 3D.

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
    # Referencia cartesiana general
    # =============================================================
    def desired_pose_and_twist(self, t):
        """
        Por ahora genera una trayectoria senoidal en Z global,
        manteniendo orientación inicial.

        Más adelante, para cambiar a X, Y, rotación u órbita,
        conviene modificar solo esta función.
        """
        z_des, vz_des = self.compute_z_reference(t)

        p_des = self.p_start + z_des * self.axis_vector
        R_des = self.R_start

        v_ff = vz_des * self.axis_vector
        w_ff = np.zeros(3)

        return p_des, R_des, v_ff, w_ff

    # =============================================================
    # Diagnóstico del Jacobiano
    # =============================================================
    def analyze_jacobian(self, J):
        try:
            S = np.linalg.svd(J, compute_uv=False)

            sigma_min = float(S[-1])
            sigma_max = float(S[0])

            if sigma_min < 1e-10:
                cond = float('inf')
            else:
                cond = float(sigma_max / sigma_min)

            rank = int(np.linalg.matrix_rank(J, tol=1e-6))

            if rank < 6:
                status = 'SINGULAR'
            elif sigma_min < self.sigma_min_stop:
                status = 'DANGER'
            elif sigma_min < self.sigma_min_warn:
                status = 'WARNING'
            elif cond > self.cond_stop:
                status = 'WARNING'
            elif cond > self.cond_warn:
                status = 'WARNING'
            else:
                status = 'OK'

            return {
                'rank': rank,
                'sigma_min': sigma_min,
                'sigma_max': sigma_max,
                'cond': cond,
                'status': status,
                'singular_values': S,
            }

        except Exception as e:
            return {
                'rank': -1,
                'sigma_min': 0.0,
                'sigma_max': 0.0,
                'cond': float('inf'),
                'status': f'ERROR: {e}',
                'singular_values': np.array([]),
            }
    # =============================================================
    # Escalado de velocidad
    # =============================================================
    def compute_velocity_scaling(self, qdot_arm):
        """
        Calcula cuánto habría que escalar qdot para cumplir qdot_max.

        scale = 1.0 significa que no hay saturación.
        scale < 1.0 significa que la trayectoria pide más velocidad
        que la permitida.
        """
        max_abs = float(np.max(np.abs(qdot_arm)))

        if max_abs <= self.qdot_max:
            return 1.0, False, max_abs

        scale = self.qdot_max / max_abs
        return scale, True, max_abs

    def limit_joint_velocity_by_scaling(self, qdot_arm):
        """
        Limita qdot escalando todo el vector.
        Esto conserva mejor la dirección del movimiento articular
        que hacer np.clip junta por junta.
        """
        scale, saturated, max_abs = self.compute_velocity_scaling(qdot_arm)

        if not saturated:
            return qdot_arm, scale, saturated, max_abs

        return qdot_arm * scale, scale, saturated, max_abs

    # =============================================================
    # Cálculo qdot desde el control cartesiano
    # =============================================================
    def compute_qdot_command(self, q, t):
        p_current, R_current, J_arm = self.get_tcp_pose_and_jacobian(q)

        p_des, R_des, v_ff, w_ff = self.desired_pose_and_twist(t)

        # Error posición
        error_pos = p_des - p_current
        v_cmd = self.kp_position * error_pos + v_ff

        # Error orientación
        error_ori = self.orientation_error(R_des, R_current)
        w_cmd = self.kp_orientation * error_ori + w_ff

        # Comando cartesiano 6D
        xdot = np.concatenate((v_cmd, w_cmd))

        # Pesos
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

        J_pinv = self.damped_pseudoinverse(J_weighted)
        qdot_arm = J_pinv @ xdot_weighted

        info = {
            'p_current': p_current,
            'R_current': R_current,
            'p_des': p_des,
            'R_des': R_des,
            'error_pos': error_pos,
            'error_ori': error_ori,
            'v_cmd': v_cmd,
            'w_cmd': w_cmd,
            'xdot': xdot,
            'J_arm': J_arm,
            'J_weighted': J_weighted,
            'qdot_arm': qdot_arm,
        }

        return qdot_arm, info

    # =============================================================
    # Precheck de trayectoria
    # =============================================================
    def precheck_trajectory(self, q_start):
        q_sim = q_start.copy()

        max_qdot_required = 0.0
        min_sigma_seen = float('inf')
        max_cond_seen = 0.0
        max_pos_error_seen = 0.0
        max_ori_error_seen = 0.0
        min_velocity_scale = 1.0

        problems = []
        warnings = []

        steps = int(self.precheck_duration / self.dt)

        self.get_logger().info(
            f'Precheck: simulando {steps} pasos, duración={self.precheck_duration:.2f}s'
        )

        for k in range(steps):
            t = k * self.dt

            qdot_arm, info = self.compute_qdot_command(q_sim, t)

            J_arm = info['J_arm']
            error_pos = info['error_pos']
            error_ori = info['error_ori']

            # ---------------------------------------------
            # Singularidad / condicionamiento
            # ---------------------------------------------
            jac_info = self.analyze_jacobian(J_arm)

            min_sigma_seen = min(min_sigma_seen, jac_info['sigma_min'])
            max_cond_seen = max(max_cond_seen, jac_info['cond'])

            if jac_info['status'] in ['SINGULAR', 'DANGER'] or 'ERROR' in jac_info['status']:
                problems.append(
                    f"t={t:.2f}s | Jacobiano {jac_info['status']} | "
                    f"rank={jac_info['rank']} | "
                    f"sigma_min={jac_info['sigma_min']:.6f} | "
                    f"cond={jac_info['cond']:.2f}"
                )

            elif jac_info['status'] == 'WARNING':
                warnings.append(
                    f"t={t:.2f}s | Jacobiano WARNING | "
                    f"sigma_min={jac_info['sigma_min']:.6f} | "
                    f"cond={jac_info['cond']:.2f}"
                )

            # ---------------------------------------------
            # Velocidad articular requerida
            # ---------------------------------------------
            qdot_required = float(np.max(np.abs(qdot_arm)))
            max_qdot_required = max(max_qdot_required, qdot_required)

            velocity_scale, saturated, _ = self.compute_velocity_scaling(qdot_arm)
            min_velocity_scale = min(min_velocity_scale, velocity_scale)

            if saturated:
                problems.append(
                    f"t={t:.2f}s | velocidad excedida | "
                    f"qdot_required={qdot_required:.3f} rad/s | "
                    f"qdot_max={self.qdot_max:.3f} rad/s | "
                    f"scale={velocity_scale:.3f}"
                )

            # ---------------------------------------------
            # Errores cartesianos
            # ---------------------------------------------
            pos_error_norm = float(np.linalg.norm(error_pos))
            ori_error_norm = float(np.linalg.norm(error_ori))

            max_pos_error_seen = max(max_pos_error_seen, pos_error_norm)
            max_ori_error_seen = max(max_ori_error_seen, ori_error_norm)

            if pos_error_norm > self.max_position_error_allowed:
                warnings.append(
                    f"t={t:.2f}s | error posición alto | "
                    f"err_pos={pos_error_norm:.4f} m"
                )

            if ori_error_norm > self.max_orientation_error_allowed:
                warnings.append(
                    f"t={t:.2f}s | error orientación alto | "
                    f"err_ori={ori_error_norm:.4f} rad"
                )

            # ---------------------------------------------
            # Integración simulada
            # ---------------------------------------------
            q_sim[0:6] = q_sim[0:6] + qdot_arm * self.dt

            if self.model.nq >= 8:
                q_sim[6] = 0.0
                q_sim[7] = 0.0

        has_problems = len(problems) > 0
        has_warnings = len(warnings) > 0

        if has_problems:
            ok = False
        elif has_warnings and not self.allow_warnings:
            ok = False
        else:
            ok = True

        return {
            'ok': ok,
            'problems': problems,
            'warnings': warnings,
            'max_qdot_required': max_qdot_required,
            'min_sigma_seen': min_sigma_seen,
            'max_cond_seen': max_cond_seen,
            'max_pos_error_seen': max_pos_error_seen,
            'max_ori_error_seen': max_ori_error_seen,
            'min_velocity_scale': min_velocity_scale,
        }

    # =============================================================
    # Ejecutar precheck
    # =============================================================
    def run_precheck_or_enable_motion(self, q):
        if not self.enable_precheck:
            self.get_logger().warn('Precheck deshabilitado. Habilito movimiento directamente.')
            self.motion_enabled = True
            self.precheck_done = True
            return

        self.get_logger().info('Ejecutando precheck cinemático de trayectoria...')

        result = self.precheck_trajectory(q)

        self.get_logger().info(
            'Resumen precheck: '
            f"ok={result['ok']} | "
            f"max_qdot_required={result['max_qdot_required']:.3f} rad/s | "
            f"min_sigma={result['min_sigma_seen']:.6f} | "
            f"max_cond={result['max_cond_seen']:.2f} | "
            f"max_err_pos={result['max_pos_error_seen']:.4f} m | "
            f"max_err_ori={result['max_ori_error_seen']:.4f} rad | "
            f"min_velocity_scale={result['min_velocity_scale']:.3f}"
        )

        if len(result['warnings']) > 0:
            self.get_logger().warn(
                f'Precheck generó {len(result["warnings"])} warnings. Muestro los primeros 10:'
            )
            for warning in result['warnings'][:10]:
                self.get_logger().warn(warning)

        if len(result['problems']) > 0:
            self.get_logger().error(
                f'Precheck encontró {len(result["problems"])} problemas. Muestro los primeros 10:'
            )
            for problem in result['problems'][:10]:
                self.get_logger().error(problem)

        if result['ok']:
            self.get_logger().info('Precheck OK. Habilito movimiento.')
            self.motion_enabled = True
        else:
            self.get_logger().error('Precheck FALLÓ. No ejecuto la trayectoria.')
            self.motion_enabled = False

        self.precheck_done = True

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

            self.get_logger().info(
                'R_start guardada. La orientación deseada será constante.'
            )

            self.run_precheck_or_enable_motion(q)
            return

        # Si el precheck falló, no ejecutar movimiento.
        if self.precheck_done and not self.motion_enabled:
            self.publish_joint_position(q)
            return

        t = now - self.t0

        # ---------------------------------------------------------
        # Diagnóstico online del Jacobiano
        # ---------------------------------------------------------
        jac_info = self.analyze_jacobian(J_arm)

        self.get_logger().info(
            f"J status={jac_info['status']} | "
            f"rank={jac_info['rank']} | "
            f"sigma_min={jac_info['sigma_min']:.6f} | "
            f"cond={jac_info['cond']:.2f}",
            throttle_duration_sec=1.0
        )

        if jac_info['status'] in ['SINGULAR', 'DANGER'] or 'ERROR' in jac_info['status']:
            self.get_logger().error(
                'Movimiento detenido: Jacobiano singular o peligroso.',
                throttle_duration_sec=1.0
            )
            self.publish_joint_position(q)
            return

        # ---------------------------------------------------------
        # Calcular comando cartesiano y qdot
        # ---------------------------------------------------------
        qdot_arm, info = self.compute_qdot_command(q, t)

        error_pos = info['error_pos']
        error_ori = info['error_ori']

        # ---------------------------------------------------------
        # Limitar velocidad articular por escalado
        # ---------------------------------------------------------
        qdot_limited, velocity_scale, saturated, qdot_required = (
            self.limit_joint_velocity_by_scaling(qdot_arm)
        )

        if saturated:
            self.get_logger().warn(
                f'Velocidad articular limitada | '
                f'qdot_required={qdot_required:.3f} rad/s | '
                f'qdot_max={self.qdot_max:.3f} rad/s | '
                f'scale={velocity_scale:.3f}',
                throttle_duration_sec=1.0
            )

        # ---------------------------------------------------------
        # Integrar posición articular
        # ---------------------------------------------------------
        q_cmd = q.copy()
        q_cmd[0:6] = q[0:6] + qdot_limited * self.dt

        if self.model.nq >= 8:
            q_cmd[6] = 0.0
            q_cmd[7] = 0.0

        # ---------------------------------------------------------
        # Publicar comando
        # ---------------------------------------------------------
        self.publish_joint_position(q_cmd)

        # ---------------------------------------------------------
        # Debug
        # ---------------------------------------------------------
        self.get_logger().info(
            f't={t:.2f}s | '
            f'err_pos={np.linalg.norm(error_pos):.5f} m | '
            f'err_ori={np.linalg.norm(error_ori):.5f} rad | '
            f'qdot_max_now={np.max(np.abs(qdot_limited)):.3f} rad/s',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)

    node = CartesianZKeepOrientationPrecheck()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()