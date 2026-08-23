#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64


class Joint2Debug(Node):
    def __init__(self):
        super().__init__("joint2_debug")

        self.joint_name = "Joint2"

        self.pub_real = self.create_publisher(
            Float64,
            "/debug/Joint2_real",
            10
        )

        self.pub_ref = self.create_publisher(
            Float64,
            "/debug/Joint2_ref",
            10
        )

        self.sub_joint_states = self.create_subscription(
            JointState,
            "/joint_states",
            self.cb_joint_states,
            10
        )

        self.sub_traj = self.create_subscription(
            JointTrajectory,
            "/BRAZO_controller/joint_trajectory",
            self.cb_traj,
            10
        )

        self.get_logger().info("Graficador Joint2 iniciado.")
        self.get_logger().info("Publicando /debug/Joint2_real y /debug/Joint2_ref")

    def cb_joint_states(self, msg):
        if self.joint_name not in msg.name:
            return

        idx = msg.name.index(self.joint_name)

        y = Float64()
        y.data = float(msg.position[idx])

        self.pub_real.publish(y)

    def cb_traj(self, msg):
        if self.joint_name not in msg.joint_names:
            return

        idx = msg.joint_names.index(self.joint_name)

        # Tomo todos los puntos de la trayectoria y publico el último.
        # Para rqt_plot esto muestra la referencia objetivo actual.
        if not msg.points:
            return

        p = msg.points[-1]

        if idx >= len(p.positions):
            return

        y = Float64()
        y.data = float(p.positions[idx])

        self.pub_ref.publish(y)


def main():
    rclpy.init()
    node = Joint2Debug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()