#!/usr/bin/env python3
import os
import glob
import torch
import torch.nn as nn
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pinocchio as pin
import numpy as np
class ActorNetwork(nn.Module):
    def __init__(self, input_dim=30, output_dim=8):
        super(ActorNetwork, self).__init__()
        self.fc1 = nn.Linear(input_dim, 256)
        self.elu1 = nn.ELU()
        self.fc2 = nn.Linear(256, 128)
        self.elu2 = nn.ELU()
        self.fc3 = nn.Linear(128, 64)
        self.elu3 = nn.ELU()
        self.mu = nn.Linear(64, output_dim)

    def forward(self, x):
        x = self.elu1(self.fc1(x))
        x = self.elu2(self.fc2(x))
        x = self.elu3(self.fc3(x))
        return self.mu(x)

class LegoRLBridge(Node):
    def __init__(self):
        super().__init__('lego_rl_bridge')
        self.policy = None
        self.urdf_path = "/root/ros_ws/src/lego/urdf/Ensamblaje2.urdf"        
        self.pin_model = pin.buildModelFromUrdf(self.urdf_path)
        self.pin_data = self.pin_model.createData()
        
        self.joint_signs = np.array([-1.0, -1.0, 1.0, 1.0, 1.0, 1.0])
        self.ee_frame_name = "Pinza" 
        self.ee_frame_id = self.pin_model.getFrameId(self.ee_frame_name)
        # Ruta del modelo persistente en el workspace compartido
        self.model_path = "src/lego/config/modelo_lego.pth"
            
        self.get_logger().info(f"🧠 Cargando política RL desde: {self.model_path}")

        
        try:
            checkpoint = torch.load(self.model_path, map_location='cpu', weights_only=False)
            self.policy = ActorNetwork(input_dim=30, output_dim=8)
            
            if isinstance(checkpoint, dict):
                state_dict = checkpoint.get('model', checkpoint)
            else:
                state_dict = checkpoint

            self.policy.load_state_dict(state_dict, strict=False)
            self.policy.eval()
            
            # --- BÚSQUEDA DINÁMICA DEL NORMALIZADOR ---
            mean_key = next((k for k in state_dict.keys() if 'running_mean' in k and 'std' not in k.split('.')[-1]), None)
            var_key = next((k for k in state_dict.keys() if 'running_var' in k), None)

            if mean_key and var_key:
                self.obs_mean = state_dict[mean_key].cpu()
                self.obs_var = state_dict[var_key].cpu()
                self.get_logger().info(f"✅ ¡Normalizador detectado usando claves: {mean_key} / {var_key}!")
            else:
                self.get_logger().warn("⚠️ No se encontraron las medias/varianzas en el .pth. Usando normalización neutra (sin cambios).")
                self.obs_mean = torch.zeros(30)
                self.obs_var = torch.ones(30)
            # ------------------------------------------

            self.get_logger().info("✅ ¡Modelo cargado y mapeado correctamente!")
        except Exception as e:
            self.get_logger().error(f"❌ Error crítico al cargar los pesos: {e}")
            self.policy = None

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        self.latest_joint_states = None
        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        
        # Definir posición del objetivo (Objeto LEGO en la mesa frente al robot)
#        self.target_position = [0.0, 0.0, 0.415]
        
        self.target_position = [0.0, 0.0, -0.0415]
# 2. ENVIAR POSTURA INICIAL AUTOMÁTICA (CEROS)
        angles_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Las 6 articulaciones del brazo en 0.0
        
        self.get_logger().info("🏠 Enviando robot a la postura inicial de entrenamiento (Ceros)...")
        self.enviar_home(angles_home)

        # 3. Esperar 2 segundos simulados a que el robot llegue a la pose de inicio
        import time
        time.sleep(2.0) 

        # Timer de control a 50 Hz (20 ms)
        self.timer = self.create_timer(0.02, self.control_loop)


        self.get_logger().info("🤖 ¡Cerebro de RL activado y tomando el control!")

    def enviar_home(self, angles):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names[:6]  # Asegúrate de usar los nombres de las 6 juntas del brazo
        
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start.sec = 2  # Se toma 2 segundos para llegar suavemente
        point.time_from_start.nanosec = 0
        
        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)

    def joint_state_callback(self, msg):
        self.latest_joint_states = msg

    def control_loop(self):
        if self.latest_joint_states is None or self.policy is None:
            return

        try:
            # 1. Leer estado FÍSICO actual de Gazebo
            q, dq = [], []
            for name in self.joint_names:
                if name in self.latest_joint_states.name:
                    idx = self.latest_joint_states.name.index(name)
                    q.append(self.latest_joint_states.position[idx])
                    dq.append(self.latest_joint_states.velocity[idx])
                else:
                    q.append(0.0)
                    dq.append(0.0)

            if not hasattr(self, 'cmd_q'):
                self.cmd_q = np.array(q[:6])
                self.cmd_dq = np.zeros(6)

            # 2. Cinemática Directa con Pinocchio (Realidad de Gazebo)
            q_pin = np.array(self.cmd_q.tolist() + [0.0, 0.0])
            pin.forwardKinematics(self.pin_model, self.pin_data, q_pin)
            pin.updateFramePlacements(self.pin_model, self.pin_data)
            
            ee_pose = self.pin_data.oMf[self.ee_frame_id]
            raw_ee_pos = ee_pose.translation
            raw_ee_rot = ee_pose.rotation
            
            # ---------------------------------------------------------
            # [TRADUCTOR 1] - ESPACIO CARTESIANO (Rotar 180° en Z)
            # Engañamos a la IA para que crea que el frente es +X
            # ---------------------------------------------------------
            import math
            R_180_Z = np.array([
                [math.cos(math.pi), -math.sin(math.pi), 0],
                [math.sin(math.pi),  math.cos(math.pi), 0],
                [0,                  0,                 1]
            ])
            ee_pos = (R_180_Z @ raw_ee_pos).tolist()
            ee_rot_rotada = R_180_Z @ raw_ee_rot
            
            quat_obj = pin.Quaternion(ee_rot_rotada)
            ee_quat = [quat_obj.x, quat_obj.y, quat_obj.z, quat_obj.w]

            # 3. Objetivo Inyectado (En el mundo feliz de la IA: +X)
            obj_pos = [0.20, 0.0, 0.0415] 
            obj_quat = [0.0, 0.0, 0.0, 1.0]

            # ---------------------------------------------------------
            # [TRADUCTOR 2A] - MOTORES (Gazebo -> IA)
            # ---------------------------------------------------------
            q_isaac = self.cmd_q * self.joint_signs
            dq_isaac = self.cmd_dq * self.joint_signs
            
            dof_pos_isaac = q_isaac.tolist() + [0.0, 0.0]  
            dof_vel_isaac = dq_isaac.tolist() + [0.0, 0.0]

            # 4. Ensamblar y Normalizar
            obs_list = dof_pos_isaac + dof_vel_isaac + ee_pos + ee_quat + obj_pos + obj_quat
            
            obs_tensor_raw = torch.tensor([obs_list], dtype=torch.float32)
            obs_norm = (obs_tensor_raw - self.obs_mean.float()) / torch.sqrt(self.obs_var.float() + 1e-5)
            obs_norm = torch.clamp(obs_norm, -20.0, 20.0)

            # 5. Distancia al objetivo en el mundo de la IA
            distancia = np.linalg.norm(np.array(ee_pos) - np.array(obj_pos))
            
            if distancia < 0.03:
                self.get_logger().info(f"🎯 ¡OBJETIVO ALCANZADO! Distancia: {distancia:.3f}m. Congelando red.")
                return 
            
            # 6. Inferencia IA
            with torch.no_grad():
                action = self.policy(obs_norm.float())

            actions_np = action.numpy().flatten()
            self.get_logger().info(f"✈️ Viajando... Dist: {distancia:.3f}m | Z_virtual: {ee_pos[2]:.3f} | Acción IA: {actions_np[:3]}")

            # ---------------------------------------------------------
            # [TRADUCTOR 2B] - MOTORES (IA -> Gazebo)
            # ---------------------------------------------------------
            escala_accion = 0.05
            if distancia < 0.10: # Amortiguamiento adaptativo
                escala_accion = 0.01

            delta_q_isaac = actions_np[:6] * escala_accion
            delta_q_gazebo = delta_q_isaac * self.joint_signs
            
            # 7. Integrador y Topes Virtuales Estrictos
            self.cmd_q = self.cmd_q + delta_q_gazebo
            self.cmd_dq = delta_q_gazebo / 0.02
            
            # Límites de tu URDF para proteger los motores (Ajustar si difieren)
            lim_inf = np.array([-3.14, -1.56, -1.56, -3.14, -1.56, -3.14]) 
            lim_sup = np.array([ 3.14,  1.56,  1.56,  3.14,  1.56,  3.14])
            self.cmd_q = np.clip(self.cmd_q, lim_inf, lim_sup)

            # 8. Enviar Comando a Gazebo
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names[:6]
            point = JointTrajectoryPoint()
            point.positions = self.cmd_q.tolist()
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 40000000 
            traj_msg.points.append(point)
            
            self.trajectory_pub.publish(traj_msg)

        except Exception as e:
            self.get_logger().warn(f"⚠️ Error en bucle: {e}")            

def main(args=None):
    rclpy.init(args=args)
    bridge = LegoRLBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            bridge.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()