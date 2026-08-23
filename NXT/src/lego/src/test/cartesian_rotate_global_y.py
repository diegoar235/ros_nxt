#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import pinocchio as pin


class OrbitAroundExternalAxis(Node):

    def __init__(self):
        super().__init__('orbit_around_external_axis')

        # ============================================================
        # PARÁMETROS CONFIGURADOS DENTRO DEL SCRIPT
        # ============================================================

        # Ruta absoluta al URDF
        self.urdf_path = "/root/ros_ws/src/lego/urdf/Ensamblaje.urdf"

        # Frame TCP / end-effector
        self.tip_frame = "Pinza"

        # ------------------------------------------------------------
        # Juntas que usa Pinocchio
        # ------------------------------------------------------------
        # Tu URDF tiene 8 juntas móviles:
        # Joint1 ... Joint8
        #
        # Pinocchio espera que q tenga tamaño 8.
        # ------------------------------------------------------------

        self.model_joint_names = [
            "Joint1",
            "Joint2",
            "Joint3",
            "Joint4",
            "Joint5",
            "Joint6",
            "Joint7",
            "Joint8"
        ]

        # ------------------------------------------------------------
        # Juntas que recibe el controlador
        # ------------------------------------------------------------
        # El arm_controller probablemente solo controla Joint1...Joint6.
        # Por eso publicamos solo esas 6.
        # ------------------------------------------------------------

        self.command_joint_names = [
            "Joint1",
            "Joint2",
            "Joint3",
            "Joint4",
            "Joint5",
            "Joint6"
        ]

        # Tópicos
        self.joint_states_topic = "/joint_states"
        self.controller_topic = "/arm_controller/joint_trajectory"

        # Frecuencia del lazo
        self.dt = 0.01  # 100 Hz

        # Tiempo que se le da al controlador para alcanzar cada punto
        self.command_time_from_start = 0.02  # segundos

        # Damping del Jacobiano amortiguado
        self.damping = 0.05

        # ------------------------------------------------------------
        # Límite de velocidad articular [rad/s]
        # ------------------------------------------------------------
        # Joint7 y Joint8 quedan bloqueadas con 0.0
        # ------------------------------------------------------------

        self.qdot_max = np.array([
            0.6,   # Joint1
            0.6,   # Joint2
            0.6,   # Joint3
            0.6,   # Joint4
            0.6,   # Joint5
            0.6,   # Joint6
            0.0,   # Joint7 bloqueada
            0.0    # Joint8 bloqueada
        ])

        # ============================================================
        # EJE EXTERNO DE ROTACIÓN
        # ============================================================
        # p_eje: un punto por donde pasa el eje externo
        # u_eje: dirección del eje externo
        #
        # Este ejemplo:
        # eje global Y que pasa por el punto [0.30, 0.00, 0.35]
        # ============================================================

        self.p_eje = np.array([0.30, 0.00, 0.35])
        self.u_eje = np.array([0.0, 1.0, 0.0])

        # Velocidad angular de órbita [rad/s]
        self.omega = 0.05

        # True:
        #   el TCP orbita alrededor del eje y la herramienta rota también.
        #
        # False:
        #   el TCP orbita pero la herramienta intenta mantener orientación.
        self.rotar_herramienta = False

        # ============================================================
        # VARIABLES INTERNAS
        # ============================================================

        self.q_actual = None
        self.joint_state_ok = False

        self.model = None
        self.data = None
        self.frame_id = None

        # ============================================================
        # CARGAR MODELO PINOCCHIO
        # ============================================================

        self.load_robot_model_from_urdf()

        # ============================================================
        # ROS2 SUBSCRIBER / PUBLISHER / TIMER
        # ============================================================

        self.joint_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            10
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            self.controller_topic,
            10
        )

        self.timer = self.create_timer(
            self.dt,
            self.control_loop
        )

        self.get_logger().info("Nodo orbit_around_external_axis iniciado.")
        self.get_logger().info(f"URDF: {self.urdf_path}")
        self.get_logger().info(f"TCP frame: {self.tip_frame}")
        self.get_logger().info(f"Eje externo p_eje: {self.p_eje}")
        self.get_logger().info(f"Eje externo u_eje: {self.u_eje}")
        self.get_logger().info(f"omega: {self.omega} rad/s")
        self.get_logger().info(f"rotar_herramienta: {self.rotar_herramienta}")

    # ================================================================
    # CARGAR ROBOT DESDE URDF
    # ================================================================

    def load_robot_model_from_urdf(self):
        self.get_logger().info("Cargando modelo Pinocchio desde URDF...")

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        self.get_logger().info(
            f"Modelo cargado. nq={self.model.nq}, nv={self.model.nv}"
        )

        self.frame_id = self.model.getFrameId(self.tip_frame)

        if self.frame_id >= len(self.model.frames):
            raise RuntimeError(
                f"No existe el frame '{self.tip_frame}' en el URDF."
            )

        self.get_logger().info(
            f"Frame TCP encontrado: {self.tip_frame}, id={self.frame_id}"
        )

        self.get_logger().info("Juntas del modelo Pinocchio:")
        for name in self.model.names:
            self.get_logger().info(f"  {name}")

        self.get_logger().info("Frames disponibles:")
        for frame in self.model.frames:
            self.get_logger().info(f"  {frame.name}")

        if self.model.nq != len(self.model_joint_names):
            self.get_logger().warn(
                f"Atención: model.nq={self.model.nq}, "
                f"pero model_joint_names tiene {len(self.model_joint_names)} elementos."
            )
            self.get_logger().warn(
                "Si esto no coincide, revisá las juntas móviles del URDF."
            )

    # ================================================================
    # CALLBACK DE JOINT STATES
    # ================================================================

    def joint_state_callback(self, msg):
        """
        Arma q_actual usando TODAS las juntas que espera Pinocchio.
        Si Joint7 o Joint8 no aparecen en /joint_states, las fija en 0.0.
        """

        q = []

        for joint_name in self.model_joint_names:

            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                q.append(msg.position[idx])

            else:
                # Joint7 y Joint8 pueden no estar publicadas por /joint_states.
                # Las fijamos en 0.0 para que Pinocchio reciba q de tamaño 8.
                if joint_name in ["Joint7", "Joint8"]:
                    q.append(0.0)

                else:
                    self.get_logger().warn(
                        f"No encuentro la junta '{joint_name}' en {self.joint_states_topic}"
                    )
                    return

        self.q_actual = np.array(q, dtype=float)
        self.joint_state_ok = True

    # ================================================================
    # TWIST PARA ROTAR ALREDEDOR DE UN EJE EXTERNO
    # ================================================================

    def twist_rotacion_alrededor_de_eje(
        self,
        p_tcp,
        p_eje,
        u_eje,
        omega,
        rotar_herramienta=True
    ):
        """
        Calcula xdot = [vx, vy, vz, wx, wy, wz]
        para que el TCP rote alrededor de un eje externo.

        p_tcp:
            posición actual del TCP en coordenadas globales.

        p_eje:
            punto cualquiera perteneciente al eje externo.

        u_eje:
            dirección del eje externo.

        omega:
            velocidad angular de giro alrededor del eje [rad/s].

        rotar_herramienta:
            True  -> el TCP orbita y la herramienta rota.
            False -> el TCP orbita pero intenta mantener orientación.
        """

        p_tcp = np.array(p_tcp, dtype=float)
        p_eje = np.array(p_eje, dtype=float)
        u_eje = np.array(u_eje, dtype=float)

        norm_u = np.linalg.norm(u_eje)

        if norm_u < 1e-9:
            raise ValueError("u_eje no puede ser un vector cero.")

        # Dirección unitaria del eje
        u_eje = u_eje / norm_u

        # Vector desde un punto del eje hasta el TCP
        a = p_tcp - p_eje

        # Componente paralela al eje
        a_paralelo = np.dot(a, u_eje) * u_eje

        # Vector radial desde el eje hasta el TCP
        r = a - a_paralelo

        # Velocidad angular de la órbita
        w_orbita = omega * u_eje

        # Velocidad lineal tangencial:
        #
        # v = w x r
        #
        # Esta es la parte clave para que el TCP orbite alrededor
        # del eje externo.
        v_des = np.cross(w_orbita, r)

        if rotar_herramienta:
            # La herramienta acompaña la rotación alrededor del eje
            w_des = w_orbita
        else:
            # La herramienta intenta mantener orientación
            w_des = np.zeros(3)

        xdot = np.hstack((v_des, w_des))

        return xdot, r

    # ================================================================
    # PSEUDOINVERSA AMORTIGUADA
    # ================================================================

    def damped_pseudo_inverse(self, J, damping):
        """
        Calcula:

            J_dls = J.T (J J.T + lambda² I)^-1

        Es más estable que np.linalg.pinv cerca de singularidades.
        """

        m = J.shape[0]

        return J.T @ np.linalg.inv(
            J @ J.T + damping**2 * np.eye(m)
        )

    # ================================================================
    # ANÁLISIS SIMPLE DE SINGULARIDAD
    # ================================================================

    def analyze_jacobian(self, J):
        try:
            S = np.linalg.svd(J, compute_uv=False)

            sigma_min = S[-1]
            sigma_max = S[0]

            if sigma_min < 1e-12:
                cond = float("inf")
            else:
                cond = sigma_max / sigma_min

            rank = np.linalg.matrix_rank(J)

            return rank, sigma_min, cond

        except Exception:
            return -1, 0.0, float("inf")

    # ================================================================
    # PUBLICAR TRAYECTORIA
    # ================================================================

    def publish_joint_trajectory(self, q_cmd):
        """
        Publica solamente Joint1...Joint6 al controlador.
        q_cmd viene de Pinocchio y tiene 8 valores.
        """

        msg = JointTrajectory()
        msg.joint_names = self.command_joint_names

        point = JointTrajectoryPoint()

        # Publicamos solo las primeras 6 juntas.
        # Joint7 y Joint8 no se mandan al arm_controller.
        point.positions = q_cmd[:6].tolist()

        sec = int(self.command_time_from_start)
        nanosec = int(
            (self.command_time_from_start - sec) * 1e9
        )

        point.time_from_start = Duration(
            sec=sec,
            nanosec=nanosec
        )

        msg.points.append(point)

        self.traj_pub.publish(msg)

    # ================================================================
    # LOOP PRINCIPAL
    # ================================================================

    def control_loop(self):

        if not self.joint_state_ok:
            return

        q = self.q_actual.copy()

        # ------------------------------------------------------------
        # Verificación de dimensión
        # ------------------------------------------------------------

        if q.shape[0] != self.model.nq:
            self.get_logger().error(
                f"Dimensión incorrecta de q: model.nq={self.model.nq}, "
                f"pero q tiene {q.shape[0]}"
            )
            return

        # ------------------------------------------------------------
        # 1) Cinemática directa
        # ------------------------------------------------------------

        pin.forwardKinematics(
            self.model,
            self.data,
            q
        )

        pin.updateFramePlacements(
            self.model,
            self.data
        )

        T_tcp = self.data.oMf[self.frame_id]
        p_tcp = T_tcp.translation

        # ------------------------------------------------------------
        # 2) Calcular twist deseado
        # ------------------------------------------------------------

        xdot, r = self.twist_rotacion_alrededor_de_eje(
            p_tcp=p_tcp,
            p_eje=self.p_eje,
            u_eje=self.u_eje,
            omega=self.omega,
            rotar_herramienta=self.rotar_herramienta
        )

        # ------------------------------------------------------------
        # 3) Calcular Jacobiano del TCP
        # ------------------------------------------------------------

        J = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )

        # ------------------------------------------------------------
        # 4) Analizar singularidad
        # ------------------------------------------------------------

        rank, sigma_min, cond = self.analyze_jacobian(J)

        if sigma_min < 1e-3 or cond > 500:
            self.get_logger().warn(
                f"Cerca de singularidad: "
                f"rank={rank}, "
                f"sigma_min={sigma_min:.3e}, "
                f"cond={cond:.2f}"
            )

        # ------------------------------------------------------------
        # 5) Resolver velocidades articulares
        # ------------------------------------------------------------

        J_dls = self.damped_pseudo_inverse(
            J,
            self.damping
        )

        qdot = J_dls @ xdot

        # ------------------------------------------------------------
        # 6) Limitar velocidades articulares
        # ------------------------------------------------------------

        qdot = np.clip(
            qdot,
            -self.qdot_max,
            self.qdot_max
        )

        # ------------------------------------------------------------
        # 7) Integrar configuración
        # ------------------------------------------------------------

        q_next = pin.integrate(
            self.model,
            q,
            qdot * self.dt
        )

        # Mantener explícitamente Joint7 y Joint8 iguales.
        # Esto evita que la pseudoinversa intente mover los dedos.
        q_next[6] = q[6]
        q_next[7] = q[7]

        # ------------------------------------------------------------
        # 8) Publicar comando
        # ------------------------------------------------------------

        self.publish_joint_trajectory(q_next)

        # ------------------------------------------------------------
        # Debug
        # ------------------------------------------------------------

        self.get_logger().info(
            f"p_tcp=[{p_tcp[0]:.3f}, {p_tcp[1]:.3f}, {p_tcp[2]:.3f}] "
            f"| r_norm={np.linalg.norm(r):.3f} "
            f"| qdot_norm={np.linalg.norm(qdot):.3f} "
            f"| sigma_min={sigma_min:.2e} "
            f"| cond={cond:.1f}",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)

    node = OrbitAroundExternalAxis()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()