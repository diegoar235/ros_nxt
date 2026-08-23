#!/usr/bin/env python3

import time
import yaml
import rclpy
import numpy as np
import pinocchio as pin
from pathlib import Path

from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64, Bool

class PlaySequenceV2(Node):
    def __init__(self):
        super().__init__("play_sequence_v2")

        self.arm_joint_order = [
            "Joint1", "Joint2", "Joint3",
            "Joint4", "Joint5", "Joint6",
        ]
        self.gripper_joint_order = ["Joint7", "Joint8"]

        # --- RUTAS DE ARCHIVOS ---
        # Cambiado para apuntar de forma dedicada al nuevo archivo de poses aislado
        self.yaml_path = Path.home() / "ros_ws" / "src" / "lego" / "poses" / "poses1.yaml"
        urdf_path = str(Path.home() / "ros_ws" / "src" / "lego" / "urdf" / "Ensamblaje2.urdf")

        # --- CARGAR PINOCCHIO ---
        try:
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            self.frame_id = self.model.getFrameId(self.model.frames[-1].name)
        except Exception as e:
            self.get_logger().error(f"Error cargando URDF para Pinocchio: {e}")

        # --- CONFIGURACIÓN DE ACCIONES ---
        self.arm_action_name = "/BRAZO_controller/follow_joint_trajectory"
        self.gripper_action_name = "/PINZA_controller/follow_joint_trajectory"

        self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action_name)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, self.gripper_action_name)

        # --- CONFIGURACIÓN DE TÓPICOS DIRECTOS ---
        self.joint2_pub = self.create_publisher(Float64, "/joint2_position_controller/commands", 10)
        
        # Publicador directo para controlar la bomba de succión sin despertar controladores de trayectoria
        self.pump_pub = self.create_publisher(Bool, "/pump_cmd", 10)

        # --- LÍMITES DINÁMICOS ---
        self.max_joint_vel = 0.5       # rad/s
        self.max_cartesian_vel = 0.05  # m/s

        self.get_logger().info("Esperando a los servidores de acciones y red DDS...")
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        
        # Retraso crítico para permitir el descubrimiento DDS del tópico de la pinza
        time.sleep(0.5)
        self.get_logger().info("Nodo play_sequence_v2 listo y conectado.")

    def load_yaml(self):
        if not self.yaml_path.exists():
            self.get_logger().error(f"No se encontró el archivo YAML en: {self.yaml_path}")
            return None
        with open(self.yaml_path, "r") as f:
            return yaml.safe_load(f)

    def calculate_joint_duration(self, q_start, q_end):
        diffs = np.abs(np.array(q_end) - np.array(q_start))
        max_diff = np.max(diffs)
        duration = max_diff / self.max_joint_vel
        return max(duration, 1.5)

    def build_arm_segment_goal(self, q_start, q_end):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joint_order

        duration = self.calculate_joint_duration(q_start, q_end)

        p_start = JointTrajectoryPoint()
        p_start.positions = [float(x) for x in q_start]
        p_start.time_from_start.sec = 0
        p_start.time_from_start.nanosec = 0

        p_end = JointTrajectoryPoint()
        p_end.positions = [float(x) for x in q_end]
        p_end.time_from_start.sec = int(duration)
        p_end.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        goal_msg.trajectory.points = [p_start, p_end]
        return goal_msg

    def build_gripper_goal(self, positions, duration=1.0):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joint_order

        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in positions]
        p.time_from_start.sec = int(duration)
        p.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        goal_msg.trajectory.points = [p]
        return goal_msg

    def send_goal_async_checked(self, client, name, goal_msg):
        self.get_logger().info(f"Enviando meta al servidor: {name}")
        return client.send_goal_async(goal_msg)

    def wait_result(self, future_handle, name):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future_handle.done():
                break
        
        goal_handle = future_handle.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Meta rechazada por el servidor: {name}")
            return

        result_future = goal_handle.get_result_async()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_future.done():
                break
        self.get_logger().info(f"Trayectoria completada en el servidor: {name}")

    def update_joint2_ref_during_segment(self, q_start, q_end):
        msg = Float64()
        msg.data = float(q_end[1])
        self.joint2_pub.publish(msg)

    def run(self):
        data = self.load_yaml()
        if not data or "poses" not in data:
            self.get_logger().error("Estructura de archivo YAML inválida o vacía.")
            return

        poses = data["poses"]
        self.get_logger().info(f"Se cargaron {len(poses)} pasos desde el archivo YAML.")

        for i in range(len(poses)):
                    pose_b = poses[i]
                    pose_a = poses[i-1] if i > 0 else None

                    self.get_logger().info(f"--- Procesando Paso {pose_b.get('step', i+1)} ---")

                    # 1. ACCIÓN AISLADA DE LA PINZA
                    if "pump" in pose_b:
                        pump_msg = Bool()
                        pump_msg.data = bool(pose_b["pump"])
                        self.pump_pub.publish(pump_msg)
                        self.get_logger().info(f"Pinza: {'Cerrada (Succionando)' if pump_msg.data else 'Abierta (Apagada)'}")

                    # Buscamos si hay un comando cartesiano en este paso
                    cartesian_key = next((k for k in pose_b.keys() if k.startswith("comando_")), None)

                    # 2. MOVIMIENTO ARTICULAR (Si hay 'joints')
                    if "joints" in pose_b:
                        if pose_a and "joints" in pose_a:
                            q_start = pose_a["joints"]
                            q_end = pose_b["joints"]
                            
                            arm_goal = self.build_arm_segment_goal(q_start, q_end)
                            arm_handle = self.send_goal_async_checked(self.arm_client, self.arm_action_name, arm_goal)
                            self.update_joint2_ref_during_segment(q_start, q_end)
                            self.wait_result(arm_handle, self.arm_action_name)
                    
                    # 3. MOVIMIENTO CARTESIANO (Si hay 'comando_...')
                    elif cartesian_key:
                        # AQUÍ VA TU CÓDIGO ORIGINAL DEL PARSER CARTESIANO
                        # Ese que extrae los CM, revisa si dice "subir" o "bajar", 
                        # y llama a build_arm_segment_goal_cartesiano(...)
                        
                        # (Mantén la lógica que ya te funcionaba para esta parte)
                        self.get_logger().info(f"Ejecutando movimiento cartesiano: {cartesian_key}")
                        # arm_goal = self.build_arm_segment_goal_cartesiano(...)
                        # arm_handle = self.send_goal_async_checked(...)
                        # self.wait_result(...)

                    # 4. SI NO HAY NI JOINTS NI CARTESIANO
                    else:
                        self.get_logger().info("Paso sin movimiento de brazo. Manteniendo posición actual.")

                    # TIEMPO DE ESPERA ENTRE PASOS
                    step_duration = pose_b.get("duration", 2.0)
                    time.sleep(step_duration)

        self.get_logger().info("¡Secuencia completa ejecutada con éxito!")

def main():
    rclpy.init()
    node = PlaySequenceV2()

    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"Excepción crítica durante la ejecución: {str(e)}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()