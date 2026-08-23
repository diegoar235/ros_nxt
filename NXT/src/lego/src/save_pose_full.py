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

        self.arm_joint_order = [
            "Joint1",
            "Joint2",
            "Joint3",
            "Joint4",
            "Joint5",
            "Joint6",
        ]

        self.gripper_joint_order = [
            "Joint7",
            "Joint8",
        ]

        self.yaml_path = (
            #Path.home() / "ros_ws" / "src" / "lego" / "poses" / "poses.yaml"
            Path.home() / "ros_ws" / "src" / "lego" / "poses" / "poses_.yaml"
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
        self.get_logger().info("Comandos:")
        self.get_logger().info("  b nombre_pose  -> guardar brazo")
        self.get_logger().info("  g nombre_pose  -> guardar pinza")
        self.get_logger().info("  q              -> salir")

        self.thread = threading.Thread(target=self.console_loop, daemon=True)
        self.thread.start()

    def cb_joint_states(self, msg):
        self.last_msg = msg

    def console_loop(self):
        while self.running and rclpy.ok():
            line = input("\nComando: ").strip()

            if line in ["q", "quit", "exit"]:
                self.running = False
                rclpy.shutdown()
                return

            if line == "":
                continue

            parts = line.split(maxsplit=1)

            if len(parts) != 2:
                self.get_logger().warn("Usá: b nombre_pose  o  g nombre_pose")
                continue

            tipo = parts[0].lower()
            pose_name = parts[1].strip()

            if tipo == "b":
                self.save_current_pose(pose_name, mode="arm")
            elif tipo == "g":
                self.save_current_pose(pose_name, mode="gripper")
            else:
                self.get_logger().warn("Tipo inválido. Usá 'b' para brazo o 'g' para pinza.")

    def load_yaml_file(self):
        if self.yaml_path.exists():
            with open(self.yaml_path, "r") as f:
                return yaml.safe_load(f) or {}
        return {}

    def save_yaml_file(self, data):
        self.yaml_path.parent.mkdir(parents=True, exist_ok=True)

        with open(self.yaml_path, "w") as f:
            yaml.dump(data, f, sort_keys=False)

    def save_current_pose(self, pose_name, mode):
        if self.last_msg is None:
            self.get_logger().warn("Todavía no llegó ningún /joint_states.")
            return

        pos_dict = dict(zip(self.last_msg.name, self.last_msg.position))

        if mode == "arm":
            joint_order = self.arm_joint_order
            key = "joints"
        elif mode == "gripper":
            joint_order = self.gripper_joint_order
            key = "gripper"
        else:
            self.get_logger().error(f"Modo inválido: {mode}")
            return

        missing = [j for j in joint_order if j not in pos_dict]
        if missing:
            self.get_logger().warn(f"Faltan joints: {missing}")
            self.get_logger().warn(f"Joints recibidos: {self.last_msg.name}")
            return

        q = [float(pos_dict[j]) for j in joint_order]

        data = self.load_yaml_file()

        if "poses" not in data:
            data["poses"] = {}

        data["poses"][pose_name] = {
            key: q
        }

        self.save_yaml_file(data)

        self.get_logger().info(f"Pose guardada: {pose_name}")
        self.get_logger().info(f"{key} = {q}")


def main():
    rclpy.init()
    node = PoseRecorder()
    rclpy.spin(node)


if __name__ == "__main__":
    main()