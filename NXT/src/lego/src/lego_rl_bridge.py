#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
import torch
import torch.nn as nn
import numpy as np
import pinocchio as pin  # <-- Añadido Pinocchio

# 1. Arquitectura de la red neuronal (Actor)
class ActorNetwork(nn.Module):
    def __init__(self, input_dim=19, output_dim=8):
        super(ActorNetwork, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_dim, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, output_dim)
        )

    def forward(self, x):
        return self.fc(x)

class LegoInferenceNode(Node):
    def __init__(self):
        super().__init__('lego_rl_inference_node')
        
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = ActorNetwork().to(self.device)
        
        # 2. Cargar Checkpoint
        checkpoint_path = '/root/ros_ws/src/lego/config/LegoTask.pth'
        
        try:
            checkpoint = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
            if 'model' in checkpoint:
                self.model.load_state_dict(checkpoint['model'], strict=False)
            else:
                self.model.load_state_dict(checkpoint, strict=False)
            self.get_logger().info("¡Cerebro .pth cargado exitosamente!")
        except Exception as e:
            self.get_logger().error(f"Error al cargar el checkpoint: {e}")

        self.model.eval()

        # 3. Variables de control RL
        self.current_dof_pos = np.zeros(8)
        self.current_dof_vel = np.zeros(8)
        self.target_pos = np.array([0.4, 0.0, 0.2], dtype=np.float32)

        # ---------------------------------------------------------
        # 4. CONFIGURACIÓN DE PINOCCHIO PARA VALIDACIÓN CARTESIANA
        # ---------------------------------------------------------
        # ¡IMPORTANTE! Actualiza esta ruta al archivo real de tu URDF
        urdf_path = '/root/ros_ws/src/lego/urdf/lego_manipulator.urdf' 
        
        try:
            self.pin_model = pin.buildModelFromUrdf(urdf_path)
            self.pin_data = self.pin_model.createData()
            
            # ¡IMPORTANTE! Nombre exacto del eslabón final en tu URDF
            self.ee_frame_name = 'Mano' 
            
            if self.pin_model.existFrame(self.ee_frame_name):
                self.ee_frame_id = self.pin_model.getFrameId(self.ee_frame_name)
                self.get_logger().info(f"Pinocchio listo para rastrear el frame: {self.ee_frame_name}")
            else:
                self.get_logger().error(f"El frame {self.ee_frame_name} no existe en el URDF.")
        except Exception as e:
            self.get_logger().error(f"Error cargando Pinocchio: {e}")

        # Tolerancia en metros (ej. 3 centímetros)
        self.tolerance = 0.03
        # ---------------------------------------------------------

        # 5. Suscripciones y Publicadores
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.create_subscription(Float32MultiArray, '/lego_target', self.target_callback, 10)
        
        # Bucle de control a 50 Hz (0.02s) para que la red neuronal fluya como en Isaac Gym
        self.timer = self.create_timer(0.02, self.control_loop)

    def joint_callback(self, msg: JointState):
        valid_joints = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        for i, name in enumerate(msg.name):
            if name in valid_joints:
                idx = valid_joints.index(name)
                self.current_dof_pos[idx] = msg.position[i]
                self.current_dof_vel[idx] = msg.velocity[i] if len(msg.velocity) > i else 0.0

    def target_callback(self, msg: Float32MultiArray):
        if len(msg.data) == 3:
            self.target_pos = np.array(msg.data, dtype=np.float32)
            self.get_logger().info(f"Nuevo objetivo -> X: {self.target_pos[0]:.2f}, Y: {self.target_pos[1]:.2f}, Z: {self.target_pos[2]:.2f}")

    def control_loop(self):
        # ---------------------------------------------------------
        # A. VALIDACIÓN DE LLEGADA CON PINOCCHIO ANTES DE MOVER
        # ---------------------------------------------------------
        # Extraemos las posiciones articulares. Ojo: Pinocchio asume que le pasas 
        # un array con la misma cantidad de articulaciones móviles que tiene el URDF.
        # Si tu URDF tiene 6 joints móviles, le pasamos los 6 primeros.
        q = self.current_dof_pos[:self.pin_model.nq] 
        
        if len(q) == self.pin_model.nq:
            # Cálculamos la posición del extremo
            pin.forwardKinematics(self.pin_model, self.pin_data, q)
            pin.updateFramePlacements(self.pin_model, self.pin_data)
            current_ee_pos = self.pin_data.oMf[self.ee_frame_id].translation
            
            # Calculamos la distancia (Error)
            error_dist = np.linalg.norm(current_ee_pos - self.target_pos)
            
            if error_dist < self.tolerance:
                self.get_logger().info(f"✅ ¡OBJETIVO ALCANZADO! Error: {error_dist:.3f} m", throttle_duration_sec=0.1)
            else:
                self.get_logger().info(f"Distancia restante: {error_dist:.3f} m", throttle_duration_sec=0.1)
        
        # ---------------------------------------------------------
        # B. INFERENCIA DE LA RED NEURONAL (RL)
        # ---------------------------------------------------------
        obs = np.concatenate([
            self.current_dof_pos,
            self.current_dof_vel,
            self.target_pos
        ])
        
        obs_tensor = torch.tensor(obs, dtype=torch.float32).unsqueeze(0).to(self.device)

        with torch.no_grad():
            actions = self.model(obs_tensor)
            
        actions_np = actions.cpu().numpy().squeeze()
        target_angles = actions_np * 3.14

        # ---------------------------------------------------------
        # C. PUBLICACIÓN A GAZEBO
        # ---------------------------------------------------------
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'] 
        
        point = JointTrajectoryPoint()
        point.positions = target_angles[:6].tolist()
        point.velocities = [0.0] * 6 
        
        # Al ser un bucle rápido a 50Hz, le decimos que llegue a ese punto en 20ms
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 20000000 # 20 ms
                                        
        traj_msg.points.append(point)
        self.publisher.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LegoInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()