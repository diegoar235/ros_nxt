#!/usr/bin/env python3

#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF


class CartesianLineController(Node):

    def __init__(self):
        super().__init__('cartesian_line_controller')

        self.robot = URDF.from_xml_file('/root/ros_ws/src/lego/urdf/Ensamblaje2.urdf')

        self.joint_chain = [
            'Joint1',
            'Joint2',
            'Joint3',
            'Joint4',
            'Joint5',
            'Joint6'
        ]

        self.dt = 0.05
        self.t = 0.0

        self.Kp_pos = 2.0
        self.Kp_ori = 1.0

        self.damping = 0.1
        self.max_qdot = 1 

        self.T_inicio = None
        self.juntas_actual = {}

        self.position_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('CartesianLineController iniciado')

    def joint_state_callback(self, msg):
        self.juntas_actual = dict(zip(msg.name, msg.position))

    def rotation_matrix_from_rpy(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)

        cp = math.cos(pitch)
        sp = math.sin(pitch)

        cy = math.cos(yaw)
        sy = math.sin(yaw)

        Rx = np.array([
            [1, 0, 0],
            [0, cr, -sr],
            [0, sr, cr]
        ])

        Ry = np.array([
            [cp, 0, sp],
            [0, 1, 0],
            [-sp, 0, cp]
        ])

        Rz = np.array([
            [cy, -sy, 0],
            [sy, cy, 0],
            [0, 0, 1]
        ])

        return Rz @ Ry @ Rx

    def transform_from_origin(self, xyz, rpy):
        T = np.eye(4)

        R = self.rotation_matrix_from_rpy(
            rpy[0],
            rpy[1],
            rpy[2]
        )

        T[0:3, 0:3] = R
        T[0:3, 3] = np.array(xyz, dtype=float)

        return T

    def rotation_about_axis(self, axis, angle):
        axis = np.array(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)

        x, y, z = axis

        c = math.cos(angle)
        s = math.sin(angle)
        C = 1.0 - c

        R = np.array([
            [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
            [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
            [z*x*C - y*s,   z*y*C + x*s, c + z*z*C]
        ])

        T = np.eye(4)
        T[0:3, 0:3] = R

        return T

    def forward_kinematics(self, q_dict):
        T = np.eye(4)

        for joint_name in self.joint_chain:
            joint = self.robot.joint_map[joint_name]

            T_origin = self.transform_from_origin(
                joint.origin.xyz,
                joint.origin.rpy
            )

            q = q_dict[joint_name]

            T_joint = self.rotation_about_axis(
                joint.axis,
                q
            )

            T = T @ T_origin @ T_joint

        return T

    def compute_jacobian(self, q_dict):
        T_ee = self.forward_kinematics(q_dict)
        p_ee = T_ee[0:3, 3]

        n = len(self.joint_chain)
        J = np.zeros((6, n))

        T = np.eye(4)

        for i, joint_name in enumerate(self.joint_chain):
            joint = self.robot.joint_map[joint_name]

            T_origin = self.transform_from_origin(
                joint.origin.xyz,
                joint.origin.rpy
            )

            T_joint_frame = T @ T_origin

            p_i = T_joint_frame[0:3, 3]
            R_i = T_joint_frame[0:3, 0:3]

            axis_local = np.array(joint.axis, dtype=float)
            z_i = R_i @ axis_local
            z_i = z_i / np.linalg.norm(z_i)

            J[0:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:6, i] = z_i

            q = q_dict[joint_name]

            T_joint_motion = self.rotation_about_axis(
                joint.axis,
                q
            )

            T = T_joint_frame @ T_joint_motion

        return J

    def damped_pseudoinverse(self, J, damping=0.1):
        m, n = J.shape
        I = np.eye(m)

        return J.T @ np.linalg.inv(
            J @ J.T + damping**2 * I
        )

    def rotation_matrix_to_rotvec(self, R):
        cos_angle = (np.trace(R) - 1.0) / 2.0
        cos_angle = np.clip(cos_angle, -1.0, 1.0)

        angle = np.arccos(cos_angle)

        if abs(angle) < 1e-6:
            return np.zeros(3)

        sin_angle = np.sin(angle)

        if abs(sin_angle) < 1e-6:
            return np.zeros(3)

        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2.0 * sin_angle)

        return angle * axis

    def calcular_twist_cmd(self, T_actual, T_deseada):
        p_actual = T_actual[0:3, 3]
        p_deseada = T_deseada[0:3, 3]

        R_actual = T_actual[0:3, 0:3]
        R_deseada = T_deseada[0:3, 0:3]

        error_pos = p_deseada - p_actual

        R_error = R_deseada @ R_actual.T
        error_ori = self.rotation_matrix_to_rotvec(R_error)

        v_cmd = self.Kp_pos * error_pos
        w_cmd = self.Kp_ori * error_ori

        return np.concatenate((v_cmd, w_cmd))

    def control_loop(self):

        if not all(j in self.juntas_actual for j in self.joint_chain):
            return

        self.t += self.dt

        q_actual = np.array([
            self.juntas_actual[j]
            for j in self.joint_chain
        ])

        T_actual = self.forward_kinematics(self.juntas_actual)

        if self.T_inicio is None:
            self.T_inicio = T_actual.copy()
            self.get_logger().info('Pose inicial guardada')
            return

        A = 0.2
        frecuencia = 0.05

        y = A * math.sin(2.0 * math.pi * frecuencia * self.t)

        T_delta = np.eye(4)
        T_delta[1, 3] = y

        # Movimiento sobre eje Y LOCAL de la pose inicial
        T_deseada = self.T_inicio @ T_delta

        twist_cmd = self.calcular_twist_cmd(
            T_actual,
            T_deseada
        )

        J = self.compute_jacobian(self.juntas_actual)
        J_pinv = self.damped_pseudoinverse(
            J,
            damping=self.damping
        )

        q_dot = J_pinv @ twist_cmd
        q_dot = np.clip(q_dot, -self.max_qdot, self.max_qdot)

        q_next = q_actual + q_dot * self.dt

        if np.any(np.isnan(q_next)):
            self.get_logger().warn('q_next contiene NaN. No publico comando.')
            return

        msg = JointTrajectory()
        msg.joint_names = self.joint_chain

        point = JointTrajectoryPoint()
        point.positions = q_next.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)

        msg.points.append(point)

        self.position_pub.publish(msg)

        self.get_logger().info(
            f'y_ref={y:.4f}, q_next={np.round(q_next, 3)}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = CartesianLineController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

