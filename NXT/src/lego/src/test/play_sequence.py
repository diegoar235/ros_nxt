#!/usr/bin/env python3

'''
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
from std_msgs.msg import Float64

class PlaySequence(Node):
    def __init__(self):
        super().__init__("play_sequence")

        self.arm_joint_order = [
            "Joint1", "Joint2", "Joint3",
            "Joint4", "Joint5", "Joint6",
        ]
        self.gripper_joint_order = ["Joint7", "Joint8"]

        # --- RUTAS DE ARCHIVOS ---
        self.yaml_path = Path.home() / "ros_ws" / "src" / "lego" / "poses" / "poses_.yaml"
        urdf_path = str(Path.home() / "ros_ws" / "src" / "lego" / "urdf" / "Ensamblaje2.urdf")

        # --- CARGAR PINOCCHIO ---
        try:
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            self.frame_id = self.model.getFrameId(self.model.frames[-1].name)
        except Exception as e:
            self.get_logger().error(f"Error cargando URDF para Pinocchio: {e}")

        # Poses que van en línea recta (entre poses pregrabadas)
        self.poses_destino_cartesianas = ["retroceso1"]

        self.max_joint_vel = 0.8
        self.max_cartesian_vel = 0.05
        self.dt = 0.05

        self.arm_action_name = "/BRAZO_controller/follow_joint_trajectory"
        self.gripper_action_name = "/PINZA_controller/follow_joint_trajectory"

        self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action_name)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, self.gripper_action_name)

        self.current_joint2_ref = 0.0
        self.pub_joint2_ref = self.create_publisher(Float64, "/debug/Joint2_ref", 10)
        self.timer_joint2_ref = self.create_timer(0.05, self.publish_joint2_ref)

    def load_yaml(self):
        if not self.yaml_path.exists():
            raise FileNotFoundError(f"No existe: {self.yaml_path}")
        with open(self.yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}
        return data

    def set_time(self, point, t):
        point.time_from_start.sec = int(t)
        point.time_from_start.nanosec = int((t - int(t)) * 1e9)

    def publish_joint2_ref(self):
        msg = Float64()
        msg.data = float(self.current_joint2_ref)
        self.pub_joint2_ref.publish(msg)

    # ==========================================
    # 1. GENERADOR ARTICULAR 
    # ==========================================
    def build_arm_segment_goal(self, q0_arm, q1_arm):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_order
        q0, q1 = np.array(q0_arm, dtype=float), np.array(q1_arm, dtype=float)

        T = max(np.max(np.abs(q1 - q0)) / self.max_joint_vel, 1.0)
        steps = int(T / self.dt)
        t_acumulado = 0.0

        for k in range(1, steps + 1):
            s = k / float(steps)
            q_actual = q0 + s * (q1 - q0)
            p = JointTrajectoryPoint(positions=q_actual.tolist(), velocities=[0.0]*6)
            t_acumulado += self.dt
            self.set_time(p, t_acumulado)
            goal.trajectory.points.append(p)

        return goal

    # ==========================================
    # 2. GENERADOR CARTESIANO (Poses absolutas)
    # ==========================================
    def build_arm_segment_goal_cartesiano(self, q0_arm, q1_arm):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_order

        q0_full, q1_full = np.zeros(self.model.nq), np.zeros(self.model.nq)
        q0_full[:6] = [float(x) for x in q0_arm]
        q1_full[:6] = [float(x) for x in q1_arm]

        pin.forwardKinematics(self.model, self.data, q0_full)
        pin.updateFramePlacements(self.model, self.data)
        pose_ini = self.data.oMf[self.frame_id].copy()

        pin.forwardKinematics(self.model, self.data, q1_full)
        pin.updateFramePlacements(self.model, self.data)
        pose_fin = self.data.oMf[self.frame_id].copy()

        distancia = np.linalg.norm(pose_fin.translation - pose_ini.translation)
        T = max(distancia / self.max_cartesian_vel, 1.5)
        steps = int(T / self.dt)

        q_actual_full = q0_full.copy()
        t_acumulado = 0.0

        for k in range(1, steps + 1):
            s = k / float(steps)
            pos_deseada = pose_ini.translation + s * (pose_fin.translation - pose_ini.translation)
            rot_deseada = pin.Quaternion(pose_ini.rotation).slerp(s, pin.Quaternion(pose_fin.rotation)).matrix()
            pose_deseada = pin.SE3(rot_deseada, pos_deseada)

            pin.forwardKinematics(self.model, self.data, q_actual_full)
            pin.updateFramePlacements(self.model, self.data)
            
            error_twist = pin.log6(self.data.oMf[self.frame_id].inverse() * pose_deseada).vector
            J = pin.computeFrameJacobian(self.model, self.data, q_actual_full, self.frame_id, pin.ReferenceFrame.LOCAL)
            
            J_inv = J.T @ np.linalg.inv(J @ J.T + 1e-4 * np.eye(6))
            q_actual_full = pin.integrate(self.model, q_actual_full, (J_inv @ error_twist) * 1.0)
            
            p = JointTrajectoryPoint(positions=q_actual_full[:6].tolist(), velocities=[0.0]*6)
            t_acumulado += self.dt
            self.set_time(p, t_acumulado)
            goal.trajectory.points.append(p)

        return goal

    # ==========================================
    # 3. GENERADOR DINÁMICO RELATIVO (El nuevo Comando Z)
    # ==========================================
    def build_relative_z_goal(self, q0_arm, dist_z):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_order

        q_full = np.zeros(self.model.nq)
        q_full[:6] = [float(x) for x in q0_arm]

        # Obtener pose actual exacta
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        pose_ini = pin.SE3(self.data.oMf[self.frame_id].copy())

        # Parámetros del Polinomio Quintico
        vmax = 0.1 
        D = dist_z
        T = max(1.875 * abs(D) / vmax, 1.0) # Tiempo dinámico
        steps = int(T / self.dt)

        a3 = 10.0 * D / T**3
        a4 = -15.0 * D / T**4
        a5 = 6.0 * D / T**5

        q_actual_full = q_full.copy()
        t_acumulado = 0.0

        for k in range(1, steps + 1):
            t = k * self.dt
            
            # Polinomio para Z
            Zpos = a3*t**3 + a4*t**4 + a5*t**5
            
            # Construimos la meta en 3D (X e Y se quedan fijos, Z baja)
            pose_deseada = pin.SE3(pose_ini.rotation, pose_ini.translation)
            pose_deseada.translation[2] += Zpos
            
            pin.forwardKinematics(self.model, self.data, q_actual_full)
            pin.updateFramePlacements(self.model, self.data)
            
            error_twist = pin.log6(self.data.oMf[self.frame_id].inverse() * pose_deseada).vector
            J = pin.computeFrameJacobian(self.model, self.data, q_actual_full, self.frame_id, pin.ReferenceFrame.LOCAL)
            
            J_inv = J.T @ np.linalg.inv(J @ J.T + 1e-3 * np.eye(6))
            q_actual_full = pin.integrate(self.model, q_actual_full, (J_inv @ error_twist) * 1.0)
            
            p = JointTrajectoryPoint(positions=q_actual_full[:6].tolist(), velocities=[0.0]*6)
            t_acumulado += self.dt
            self.set_time(p, t_acumulado)
            goal.trajectory.points.append(p)

        return goal, q_actual_full[:6].tolist()

    def build_gripper_goal(self, q, duration=1.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joint_order
        p = JointTrajectoryPoint(positions=[float(x) for x in q], velocities=[0.0]*2)
        self.set_time(p, duration)
        goal.trajectory.points.append(p)
        return goal

    def send_goal_async_checked(self, client, action_name, goal):
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"Goal rechazado: {action_name}")
        return handle

    def wait_result(self, handle, action_name):
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().result

    def run(self):
        data = self.load_yaml()

        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("No encontré BRAZO_controller.")
            return

        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("No encontré PINZA_controller.")
            return

        sequence = data["sequence"]

        # --- MEMORIA DEL ROBOT: Empezamos en la primera pose ---
        current_arm_joints = data["poses"][sequence[0]]["joints"]

        for i in range(len(sequence) - 1):
            pose_a_name = sequence[i]
            pose_b_name = sequence[i + 1]

            # ¡AQUÍ ESTABA EL FANTASMA! Ya no se busca en data["poses"] acá arriba.

            self.get_logger().info(f"--- Ejecutando tramo: {pose_a_name} -> {pose_b_name} ---")

            # ==========================================
            # Detección de Comandos Especiales (Dinámico)
            # ==========================================
            if pose_b_name.startswith("comando_bajar_Z_"):
                dist_cm = float(pose_b_name.replace("comando_bajar_Z_", "").replace("cm", ""))
                dist_m = -(dist_cm / 100.0)

                self.get_logger().info(f"⚙️ COMANDO DINÁMICO: Bajando {dist_cm} cm en Z...")
                
                arm_goal, new_arm_joints = self.build_relative_z_goal(current_arm_joints, dist_m)
                
                arm_handle = self.send_goal_async_checked(self.arm_client, self.arm_action_name, arm_goal)
                self.wait_result(arm_handle, self.arm_action_name)

                # Actualizamos la memoria
                current_arm_joints = new_arm_joints
                
                # Cierra la pinza
                self.get_logger().info("Pinza cerrando para agarrar objeto...")
                grip_handle = self.send_goal_async_checked(self.gripper_client, self.gripper_action_name, self.build_gripper_goal([0.0, 0.0], 1.0))
                self.wait_result(grip_handle, self.gripper_action_name)

            # ==========================================
            # Poses Absolutas Clásicas (Del YAML)
            # ==========================================
            elif pose_b_name in data["poses"]:
                pose_b = data["poses"][pose_b_name]

                if "joints" in pose_b:
                    target_arm_joints = pose_b["joints"]
                    
                    if pose_b_name in self.poses_destino_cartesianas:
                        self.get_logger().info("Estrategia: Control Diferencial CARTESIANO (Línea recta).")
                        arm_goal = self.build_arm_segment_goal_cartesiano(current_arm_joints, target_arm_joints)
                    else:
                        self.get_logger().info("Estrategia: Interpolación ARTICULAR (Curva rápida).")
                        arm_goal = self.build_arm_segment_goal(current_arm_joints, target_arm_joints)

                    arm_handle = self.send_goal_async_checked(self.arm_client, self.arm_action_name, arm_goal)
                    self.wait_result(arm_handle, self.arm_action_name)
                    
                    current_arm_joints = target_arm_joints

                if "gripper" in pose_b:
                    self.get_logger().info("Accionando pinza...")
                    grip_handle = self.send_goal_async_checked(self.gripper_client, self.gripper_action_name, self.build_gripper_goal(pose_b["gripper"], 1.0))
                    self.wait_result(grip_handle, self.gripper_action_name)

            else:
                self.get_logger().error(f"La pose/comando '{pose_b_name}' no es válido.")

def main():
    rclpy.init()
    node = PlaySequence()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"Error crítico: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
'''
'''
import yaml
import rclpy
from pathlib import Path

from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class PlaySequence(Node):
    def __init__(self):
        super().__init__("play_sequence")

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

        #self.action_name = "/arm_controller/follow_joint_trajectory"
        self.action_name = "/BRAZO_controller/follow_joint_trajectory"

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_name
        )

    def load_yaml(self):
        if not self.yaml_path.exists():
            raise FileNotFoundError(f"No existe: {self.yaml_path}")

        with open(self.yaml_path, "r") as f:
            data = yaml.safe_load(f)

        if "poses" not in data:
            raise RuntimeError("El YAML no tiene sección 'poses'.")

        if "sequence" not in data:
            raise RuntimeError("El YAML no tiene sección 'sequence'.")

        return data
  
    def build_goal(self, data, segment_time=3.0):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = self.joint_order

        t = 0.0

        for pose_name in data["sequence"]:
            if pose_name not in data["poses"]:
                raise RuntimeError(f"La pose '{pose_name}' no existe en poses.")

            q = data["poses"][pose_name]["joints"]

            if len(q) != len(self.joint_order):
                raise RuntimeError(
                    f"La pose '{pose_name}' tiene {len(q)} joints, "
                    f"pero se esperaban {len(self.joint_order)}."
                )

            t += segment_time

            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in q]
            p.velocities = [0.0] * len(self.joint_order)
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)

            goal.trajectory.points.append(p)

        return goal
 
    def build_goal(self, data, segment_time=1.0, dt=0.05):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_order

        sequence = data["sequence"]

        if len(sequence) < 2:
            raise RuntimeError("La secuencia debe tener al menos dos poses.")

        t = 0.0

        for i in range(len(sequence) - 1):
            pose_a = sequence[i]
            pose_b = sequence[i + 1]

            if pose_a not in data["poses"]:
                raise RuntimeError(f"La pose '{pose_a}' no existe.")

            if pose_b not in data["poses"]:
                raise RuntimeError(f"La pose '{pose_b}' no existe.")

            q0 = data["poses"][pose_a]["joints"]
            q1 = data["poses"][pose_b]["joints"]

            q0 = [float(x) for x in q0]
            q1 = [float(x) for x in q1]

            steps = int(segment_time / dt)

            for k in range(steps):
                s = k / float(steps)

                q = [
                    q0[j] + s * (q1[j] - q0[j])
                    for j in range(len(self.joint_order))
                ]

                p = JointTrajectoryPoint()
                p.positions = q
                p.velocities = [0.0] * len(self.joint_order)

                t += dt
                p.time_from_start.sec = int(t)
                p.time_from_start.nanosec = int((t - int(t)) * 1e9)

                goal.trajectory.points.append(p)

        return goal
    def run(self):
        data = self.load_yaml()

        self.get_logger().info(f"Esperando action server: {self.action_name}")

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("No encontré el action server.")
            return

        goal = self.build_goal(data, segment_time=2.0)

        self.get_logger().info("Enviando secuencia...")
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("La trayectoria fue rechazada.")
            return

        self.get_logger().info("Trayectoria aceptada. Ejecutando...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        self.get_logger().info(f"Resultado: error_code={result.error_code}")


def main():
    rclpy.init()
    node = PlaySequence()

    try:
        node.run()
    except Exception as e:
        node.get_logger().error(str(e))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    '''