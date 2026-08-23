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
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import math
class PlaySequence(Node):
    def __init__(self):
        super().__init__("play_sequence")

        self.arm_joint_order = [
            "Joint1", "Joint2", "Joint3",
            "Joint4", "Joint5", "Joint6",
        ]
        self.gripper_joint_order = ["Joint7", "Joint8"]

        # --- RUTAS DE ARCHIVOS cargado desde comandos---
        self.declare_parameter('archivo_yaml', 'poses.yaml')
        nombre_archivo = self.get_parameter('archivo_yaml').get_parameter_value().string_value
        self.yaml_path = Path.home() / "ros_ws" / "src" / "lego" / "poses" / nombre_archivo
        self.get_logger().info(f"Cargando secuencia desde: {self.yaml_path}")
        #self.yaml_path = Path.home() / "ros_ws" / "src" / "lego" / "poses" / "poses2.yaml"
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

        self.max_joint_vel = 1.5
        self.max_cartesian_vel = 0.05
        self.dt = 0.05

        self.arm_action_name = "/BRAZO_controller/follow_joint_trajectory"
        self.gripper_action_name = "/PINZA_controller/follow_joint_trajectory"
        self.gripper_pub = self.create_publisher(Bool, "/stm32A/gripper_cmd", 10)

        self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action_name)
        #self.gripper_client = ActionClient(self, FollowJointTrajectory, self.gripper_action_name)

        self.current_joint2_ref = 0.0
        self.pub_joint2_ref = self.create_publisher(Float64, "/debug/Joint2_ref", 10)
        self.timer_joint2_ref = self.create_timer(0.05, self.publish_joint2_ref)
        self.pump_pub = self.create_publisher(Bool, "/pump_cmd", 10)

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

    def validate_trajectory_limits(self, goal): #verifica que las trayectorioas articulares no sean mas alla de los limites
        """
        Verifica que todos los puntos de la trayectoria articular no excedan
        los límites definidos en el URDF cargado por Pinocchio.
        Retorna True si la trayectoria es segura, False si excede los límites.
        """
        # Extraemos los límites inferiores y superiores del modelo de Pinocchio.
        # Solo tomamos los primeros 6 correspondientes a las articulaciones del brazo.
        lower_limits = self.model.lowerPositionLimit[:6]
        upper_limits = self.model.upperPositionLimit[:6]

        for p_idx, point in enumerate(goal.trajectory.points):
            positions = point.positions
            
            for j_idx in range(6):
                q_val = positions[j_idx]
                q_min = lower_limits[j_idx]
                q_max = upper_limits[j_idx]
                
                # Agregamos una pequeña tolerancia (1e-4 rad) para evitar falsos positivos
                # causados por imprecisiones numéricas de los cálculos de coma flotante.
                tol = 1e-4 
                if q_val < (q_min - tol) or q_val > (q_max + tol):
                    self.get_logger().error(
                        f"¡ALERTA DE LÍMITE! Articulación {self.arm_joint_order[j_idx]} "
                        f"(índice {j_idx}) fuera de rango en el punto {p_idx} de la trayectoria.\n"
                        f" -> Valor calculado: {q_val:.4f}\n"
                        f" -> Rango permitido: [{q_min:.4f} , {q_max:.4f}]"
                    )
                    return False
                    
        return True

    # ==========================================
    # 0. GENERADOR ARTICULAR 
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
    # 2. GENERADOR DINÁMICO RELATIVO (Comando X)
    # ==========================================
    def build_relative_x_goal(self, q0_arm, dist_x):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_order

        q_full = np.zeros(self.model.nq)
        q_full[:6] = [float(x) for x in q0_arm]

        # Obtener pose actual exacta
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        pose_ini = pin.SE3(self.data.oMf[self.frame_id].copy())

        # Parámetros del Polinomio Quíntico
        vmax = 0.1 
        D = dist_x
        T = max(1.875 * abs(D) / vmax, 1.0) # Tiempo dinámico
        steps = int(T / self.dt)

        a3 = 10.0 * D / T**3
        a4 = -15.0 * D / T**4
        a5 = 6.0 * D / T**5

        q_actual_full = q_full.copy()
        t_acumulado = 0.0

        for k in range(1, steps + 1):
            t = k * self.dt
            
            # Polinomio para X
            Xpos = a3*t**3 + a4*t**4 + a5*t**5
            
            # Construimos la meta en 3D (Y y Z se quedan fijos, X se desplaza)
            pose_deseada = pin.SE3(pose_ini.rotation, pose_ini.translation)
            pose_deseada.translation[0] += Xpos
            
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

    # ==========================================
    # 3. GENERADOR DINÁMICO RELATIVO (Comando Z)
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

    # ==========================================
    # 4. GENERADOR DINÁMICO RELATIVO (Comando Y)
    # ==========================================
    def build_relative_y_goal(self, q0_arm, dist_y):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_order

        q_full = np.zeros(self.model.nq)
        q_full[:6] = [float(x) for x in q0_arm]

        # Obtener pose actual exacta
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        pose_ini = pin.SE3(self.data.oMf[self.frame_id].copy())

        # Parámetros del Polinomio Quíntico
        vmax = 0.1 
        D = dist_y
        T = max(1.875 * abs(D) / vmax, 1.0) # Tiempo dinámico
        steps = int(T / self.dt)

        a3 = 10.0 * D / T**3
        a4 = -15.0 * D / T**4
        a5 = 6.0 * D / T**5

        q_actual_full = q_full.copy()
        t_acumulado = 0.0

        for k in range(1, steps + 1):
            t = k * self.dt
            
            # Polinomio para Y
            Ypos = a3*t**3 + a4*t**4 + a5*t**5
            
            # Construimos la meta en 3D (X e Z se quedan fijos, Y se desplaza)
            pose_deseada = pin.SE3(pose_ini.rotation, pose_ini.translation)
            pose_deseada.translation[1] += Ypos
            
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

    # ==========================================
    # 5. GENERADOR DINÁMICO RELATIVO ROTACION EN Y
    # ==========================================
    def build_pure_rotation_y_goal(self, q0_arm, angle_y_rad):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_order

        q_full = np.zeros(self.model.nq)
        q_full[:6] = [float(x) for x in q0_arm]

        # Obtener pose actual exacta
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        pose_ini = pin.SE3(self.data.oMf[self.frame_id].copy())

        # Parámetros del Polinomio Quíntico (Solo rotación)
        omega_max = 0.5 # Velocidad angular máxima (rad/s)
        
        # Tiempo dinámico basado solo en el giro
        T = max(1.875 * abs(angle_y_rad) / omega_max, 1.0) if angle_y_rad != 0 else 1.0
        steps = int(T / self.dt)

        # Coeficientes para la ROTACIÓN en Y
        a3_r = 10.0 * angle_y_rad / T**3
        a4_r = -15.0 * angle_y_rad / T**4
        a5_r = 6.0 * angle_y_rad / T**5

        q_actual_full = q_full.copy()
        t_acumulado = 0.0

        for k in range(1, steps + 1):
            t = k * self.dt
            
            # Polinomio para el ángulo
            Yrot = a3_r*t**3 + a4_r*t**4 + a5_r*t**5
            
            # La traslación se copia de pose_ini y NO se toca (se mantiene fija)
            pose_deseada = pin.SE3(pose_ini.rotation, pose_ini.translation)
            
            # Creamos la matriz de rotación pura en Y
            rotacion_en_y = pin.AngleAxis(Yrot, np.array([0.0, 1.0, 0.0])).matrix()
            
            # APLICAMOS LA ROTACIÓN
            # Al multiplicar por la derecha, rotamos sobre el eje Y LOCAL (de la pinza)
            # Si quisieras rotar sobre el eje Y GLOBAL (de la base), sería: rotacion_en_y @ pose_ini.rotation
            pose_deseada.rotation = pose_ini.rotation @ rotacion_en_y
            
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
    
    def accionar_pinza(self, cerrar: bool):
            msg = Bool()
            msg.data = cerrar
            
            accion = "CERRAR" if cerrar else "ABRIR"
            self.get_logger().info(f"Enviando comando DIRECTO a STM32: {accion} pinza...")
            
            self.gripper_pub.publish(msg)
            
            # Aquí sí corresponde el sleep, ya que al usar un tópico (y no una Acción), 
            # debemos darle el tiempo físico al actuador para que responda.
            time.sleep(1.5)
    
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

        sequence = data["sequence"]

        # --- MEMORIA DEL ROBOT: Empezamos en la primera pose ---
        current_arm_joints = data["poses"][sequence[0]]["joints"]

        for i in range(len(sequence) - 1):
            pose_a_name = sequence[i]
            pose_b_name = sequence[i + 1]

            self.get_logger().info(f"--- Ejecutando tramo: {pose_a_name} -> {pose_b_name} ---")

            # ==========================================
            # Detección de Comandos Especiales (Dinámico)
            # ==========================================
            
            # --- Movimiento Dinámico en Z ---
            if pose_b_name.startswith("comando_bajar_Z_") or pose_b_name.startswith("comando_subir_Z_"):
                
                # 1. Identificar si es subida o bajada y extraer el número
                if pose_b_name.startswith("comando_bajar_Z_"):
                    dist_cm = float(pose_b_name.replace("comando_bajar_Z_", "").replace("cm", ""))
                    dist_m = -(dist_cm / 100.0) # NEGATIVO para bajar
                    accion_txt = "Bajando"
                else:
                    dist_cm = float(pose_b_name.replace("comando_subir_Z_", "").replace("cm", ""))
                    dist_m = (dist_cm / 100.0)  # POSITIVO para subir
                    accion_txt = "Subiendo"

                self.get_logger().info(f"COMANDO DINÁMICO: {accion_txt} {dist_cm} cm en Z...")
                
                # 2. Ejecutar el movimiento matemático
                arm_goal, new_arm_joints = self.build_relative_z_goal(current_arm_joints, dist_m)
                # --- NUEVA VERIFICACIÓN DE SEGURIDAD ---
                if not self.validate_trajectory_limits(arm_goal):
                    self.get_logger().fatal(f"Secuencia abortada: El comando {pose_b_name} excede los límites físicos del robot.")
                    return # Detenemos la ejecución de todo el script
                    # ---------------------------------------
                arm_handle = self.send_goal_async_checked(self.arm_client, self.arm_action_name, arm_goal)
                self.wait_result(arm_handle, self.arm_action_name)
                
                # 3. Actualizar la memoria
                current_arm_joints = new_arm_joints

            # --- Movimiento Dinámico en Y ---
            elif pose_b_name.startswith("comando_mover_Y_positivo_") or pose_b_name.startswith("comando_mover_Y_negativo_"):
                
                # 1. Identificar la dirección y extraer el número
                if pose_b_name.startswith("comando_mover_Y_positivo_"):
                    dist_cm = float(pose_b_name.replace("comando_mover_Y_positivo_", "").replace("cm", ""))
                    dist_m = (dist_cm / 100.0) # POSITIVO
                    accion_txt = "Moviendo (Y+)"
                else:
                    dist_cm = float(pose_b_name.replace("comando_mover_Y_negativo_", "").replace("cm", ""))
                    dist_m = -(dist_cm / 100.0)  # NEGATIVO
                    accion_txt = "Moviendo (Y-)"

                self.get_logger().info(f"COMANDO DINÁMICO: {accion_txt} {dist_cm} cm en Y...")
                
                # 2. Ejecutar el movimiento matemático
                arm_goal, new_arm_joints = self.build_relative_y_goal(current_arm_joints, dist_m)
                # --- NUEVA VERIFICACIÓN DE SEGURIDAD ---
                if not self.validate_trajectory_limits(arm_goal):
                    self.get_logger().fatal(f"Secuencia abortada: El comando {pose_b_name} excede los límites físicos del robot.")
                    return # Detenemos la ejecución de todo el script
                # ---------------------------------------
                arm_handle = self.send_goal_async_checked(self.arm_client, self.arm_action_name, arm_goal)
                self.wait_result(arm_handle, self.arm_action_name)
                
                # 3. Actualizar la memoria
                current_arm_joints = new_arm_joints

            # --- Movimiento Dinámico en X ---
            elif pose_b_name.startswith("comando_mover_X_positivo_") or pose_b_name.startswith("comando_mover_X_negativo_"):
                
                # 1. Identificar la dirección y extraer la distancia
                if pose_b_name.startswith("comando_mover_X_positivo_"):
                    dist_cm = float(pose_b_name.replace("comando_mover_X_positivo_", "").replace("cm", ""))
                    dist_m = (dist_cm / 100.0) # POSITIVO (hacia adelante)
                    accion_txt = "Moviendo (X+)"
                else:
                    dist_cm = float(pose_b_name.replace("comando_mover_X_negativo_", "").replace("cm", ""))
                    dist_m = -(dist_cm / 100.0)  # NEGATIVO (hacia atrás)
                    accion_txt = "Moviendo (X-)"

                self.get_logger().info(f"COMANDO DINÁMICO: {accion_txt} {dist_cm} cm en X...")
                
                # 2. Ejecutar el movimiento matemático
                arm_goal, new_arm_joints = self.build_relative_x_goal(current_arm_joints, dist_m)
                
                # --- VERIFICACIÓN DE SEGURIDAD ---
                if not self.validate_trajectory_limits(arm_goal):
                    self.get_logger().fatal(f"Secuencia abortada: El comando {pose_b_name} excede los límites físicos del robot.")
                    return # Detenemos la ejecución de todo el script
                # ---------------------------------------
                
                arm_handle = self.send_goal_async_checked(self.arm_client, self.arm_action_name, arm_goal)
                self.wait_result(arm_handle, self.arm_action_name)
                
                # 3. Actualizar la memoria
                current_arm_joints = new_arm_joints

            # --- Movimiento Dinámico de Rotación Pura en Y ---
            elif pose_b_name.startswith("comando_rotar_Y_"):
                
                # 1. Extraer el número (en grados)
                grados_str = pose_b_name.replace("comando_rotar_Y_", "")
                grados = float(grados_str)
                
                # 2. Convertir de grados a radianes (Pinocchio y ROS2 usan radianes)
                radianes = math.radians(grados)
                
                self.get_logger().info(f"COMANDO DINÁMICO: Rotando {grados} grados en el eje Y local...")
                
                # 3. Ejecutar el movimiento matemático
                arm_goal, new_arm_joints = self.build_pure_rotation_y_goal(current_arm_joints, radianes)
                # --- NUEVA VERIFICACIÓN DE SEGURIDAD ---
                if not self.validate_trajectory_limits(arm_goal):
                    self.get_logger().fatal(f"Secuencia abortada: El comando {pose_b_name} excede los límites físicos del robot.")
                    return # Detenemos la ejecución de todo el script
                # ---------------------------------------
                arm_handle = self.send_goal_async_checked(self.arm_client, self.arm_action_name, arm_goal)
                self.wait_result(arm_handle, self.arm_action_name)
                
                # 4. Actualizar la memoria
                current_arm_joints = new_arm_joints

            # ==========================================
            # Poses Absolutas Clásicas (Del YAML) y pinza
            # ==========================================
            elif pose_b_name in data["poses"]:
                pose_b = data["poses"][pose_b_name]

                # 1. Si la pose tiene coordenadas articulares (mueve el brazo)
                if "joints" in pose_b:
                    target_arm_joints = pose_b["joints"]
                    
                    self.get_logger().info("Estrategia: Interpolación ARTICULAR (Curva rápida).")
                    arm_goal = self.build_arm_segment_goal(current_arm_joints, target_arm_joints)

                    # --- ESTO DEBE ESTAR ADENTRO DEL 'if' ---
                    if not self.validate_trajectory_limits(arm_goal):
                        self.get_logger().fatal(f"Secuencia abortada: El comando {pose_b_name} excede los límites físicos del robot.")
                        return 
                    
                    # --- ESTO TAMBIÉN DEBE ESTAR ADENTRO DEL 'if' ---
                    arm_handle = self.send_goal_async_checked(self.arm_client, self.arm_action_name, arm_goal)
                    self.wait_result(arm_handle, self.arm_action_name)
                    
                    current_arm_joints = target_arm_joints

                # 2. Si la pose tiene comandos de pinza (este bloque va a la MISMA altura que el 'if "joints"...')
                if "cerrar_pinza" in pose_b:
                    self.accionar_pinza(cerrar=True)
                    time.sleep(3)
                    
                elif "abrir_pinza" in pose_b:
                    self.accionar_pinza(cerrar=False)
                    time.sleep(3)
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
