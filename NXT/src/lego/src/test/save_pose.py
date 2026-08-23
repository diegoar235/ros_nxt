#!/usr/bin/env python3

import yaml
import rclpy
import threading
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PoseRecorder(Node):
    def __init__(self):
        super().__init__("pose_recorder")

        self.joint_order = [
            "Joint1",
            "Joint2",
            "Joint3",
            "Joint4",
            "Joint5",
            "Joint6",
        ]

        self.yaml_path = (
            Path.home() / "ros_ws" / "src" / "lego" / "config" / "poses.yaml"
        )

        self.last_msg = None
        self.running = True

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.cb_joint_states,
            10
        )

        self.get_logger().info("Grabador de poses iniciado.")
        self.get_logger().info("Mové el robot con MoveIt y escribí el nombre de la pose.")
        self.get_logger().info("Escribí 'q' para salir.")

        self.thread = threading.Thread(target=self.console_loop, daemon=True)
        self.thread.start()

    def cb_joint_states(self, msg):
        self.last_msg = msg

    def console_loop(self):
        while self.running and rclpy.ok():
            pose_name = input("\nNombre de pose a guardar: ").strip()

            if pose_name in ["q", "quit", "exit"]:
                self.running = False
                rclpy.shutdown()
                return

            if pose_name == "":
                continue

            self.save_current_pose(pose_name)

    def save_current_pose(self, pose_name):
        if self.last_msg is None:
            self.get_logger().warn("Todavía no llegó ningún /joint_states.")
            return

        pos_dict = dict(zip(self.last_msg.name, self.last_msg.position))

        missing = [j for j in self.joint_order if j not in pos_dict]
        if missing:
            self.get_logger().warn(f"Faltan joints: {missing}")
            self.get_logger().warn(f"Joints recibidos: {self.last_msg.name}")
            return

        q = [float(pos_dict[j]) for j in self.joint_order]

        if self.yaml_path.exists():
            with open(self.yaml_path, "r") as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        if "poses" not in data:
            data["poses"] = {}

        data["poses"][pose_name] = {
            "joints": q
        }

        self.yaml_path.parent.mkdir(parents=True, exist_ok=True)

        with open(self.yaml_path, "w") as f:
            yaml.dump(data, f, sort_keys=False)

        self.get_logger().info(f"Pose guardada: {pose_name}")
        self.get_logger().info(f"q = {q}")


def main():
    rclpy.init()
    node = PoseRecorder()
    rclpy.spin(node)


if __name__ == "__main__":
    main()