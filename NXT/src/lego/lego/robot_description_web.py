#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess


class RobotDescriptionWebPublisher(Node):
    def __init__(self):
        super().__init__("robot_description_web_publisher")
        self.pub = self.create_publisher(String, "/robot_description_web", 10)
        self.timer = self.create_timer(2.0, self.publish_description)

    def publish_description(self):
        result = subprocess.run(
            ["ros2", "param", "get", "/robot_state_publisher", "robot_description"],
            capture_output=True,
            text=True
        )

        urdf = result.stdout
        
        if "String value is:" not in urdf:
            self.get_logger().error("No se pudo obtener robot_description")
            self.get_logger().error(f"STDOUT: {result.stdout}")
            self.get_logger().error(f"STDERR: {result.stderr}")
            return

        urdf = urdf.replace("String value is:", "").strip()
        urdf_web = urdf.replace("package://lego/", "http://localhost:8000/")
        self.get_logger().info(f"URDF length: {len(urdf)}")
        self.get_logger().info(f"Contiene package://lego/: {'package://lego/' in urdf}")
        self.get_logger().info(f"Contiene http://localhost:8000/: {'http://localhost:8000/' in urdf_web}")
        msg = String()
        msg.data = urdf_web
        self.pub.publish(msg)

        self.get_logger().info("Publicado /robot_description_web")


def main():
    rclpy.init()
    node = RobotDescriptionWebPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()