#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory 
from trajectory_msgs.msg import JointTrajectoryPoint

import pinocchio as pin
import numpy as np
from pathlib import Path
import sys
import time
import threading

class CerebroPickAndPlace(Node):
    def __init__(self):
        super().__init__('cerebro_pick_and_place_node')
        
        # --- 1. CONFIGURACIÓN DE ROS 2 ---
        self.action_client = ActionClient(self, FollowJointTrajectory, '/BRAZO_controller/follow_joint_trajectory')
        self.gripper_pub = self.create_publisher(Bool, '/gripper_command', 10)
        self.subscription = self.create_subscription(Point, '/target_position', self.vision_callback, 10)
        
        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        
        # --- 2. CONFIGURACIÓN DE PINOCCHIO ---
        self.urdf_path = str(Path.home() / "ros_ws" / "src" / "lego" / "urdf" / "Ensamblaje2.urdf")
        self.ee_frame_name = 'Pinza'
        
        try:
            self.model = pin.buildModelFromUrdf(self.urdf_path)
            self.data = self.model.createData()
            self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)
            if self.ee_frame_id >= len(self.model.frames):
                raise ValueError("Eslabón no encontrado")
            self.get_logger().info('Modelo Pinocchio cargado. Listo para calcular trayectorias híbridas.')
        except Exception as e:
            self.get_logger().error(f'Error de URDF: {e}')
            sys.exit(1)

        # --- 3. ESTADOS Y MEMORIA DEL ROBOT ---
        self.sistema_ocupado = False
        self.en_home = False
        
        # Guardamos la 'posicion1' del YAML en formato 8-Joints para Pinocchio
        self.q_home_8 = np.array([-0.785398163, 1.91986, -1.91986, 0.0, 3.14159, 0.0, 0.0, 0.0])
        # Esta variable recordará dónde está parado el robot para usarlo de "Semilla" en los cálculos
        self.q_memoria = self.q_home_8.copy() 
        
        # Alturas clave (en metros)
        self.Z_SEGURO = 0.15 # 15 cm por encima de la pieza
        self.Z_AGARRE = 0.02 # 2 cm (altura física del bloque LEGO sobre la mesa)

        # Iniciar yendo a Home
        self.timer = self.create_timer(1.0, self.arrancar_en_home)

    def arrancar_en_home(self):
        self.timer.cancel()
        self.sistema_ocupado = True
        
        self.get_logger().info('--- SECUENCIA INICIAL ---')
        self.get_logger().info('1. Abriendo Pinza...')
        self.controlar_pinza(False) # Falso = Abrir
        time.sleep(1.0)
        
        self.get_logger().info('2. Viajando a Pose de Espera (Home)...')
        self.enviar_trayectoria_articular(self.q_home_8[:6].tolist(), tiempo_sec=4.0)
        time.sleep(4.5)
        
        self.q_memoria = self.q_home_8.copy()
        self.en_home = True
        self.sistema_ocupado = False
        self.get_logger().info('¡Robot listo! Esperando detectar pieza ArUco en la mesa...')

    def vision_callback(self, msg):
        """Escucha a la cámara, pero la ignora si el brazo ya está moviéndose."""
        if not self.en_home or self.sistema_ocupado:
            return
            
        self.sistema_ocupado = True
        target_x = msg.x
        target_y = msg.y
        self.get_logger().info(f'¡Pieza fijada en X={target_x:.3f}m, Y={target_y:.3f}m! Iniciando cacería...')
        
        # Disparamos la secuencia en un hilo paralelo para poder usar time.sleep sin romper ROS2
        threading.Thread(target=self.rutina_pick_and_place, args=(target_x, target_y)).start()

    def rutina_pick_and_place(self, x, y):
        """La Máquina de Estados del Movimiento Híbrido."""
        
        # FASE 1: Aproximación Articular Rápida (Ir justo encima de la pieza)
        self.get_logger().info('FASE 1: Viajando por el aire (Over-Target)...')
        q_encima = self.calcular_ik_se3(x, y, self.Z_SEGURO, self.q_memoria)
        if q_encima is not None:
            self.enviar_trayectoria_articular(q_encima[:6].tolist(), tiempo_sec=3.0)
            self.q_memoria = q_encima.copy()
            time.sleep(3.5) # Esperar a que el robot físico llegue
        else:
            self.abortar('Fuera de alcance en Fase 1')
            return

        # FASE 2: Descenso Cartesiano Recto
        self.get_logger().info('FASE 2: Descenso cartesiano en línea recta...')
        trayectoria_bajada = self.generar_linea_recta_cartesiana(x, y, self.Z_SEGURO, self.Z_AGARRE, 3.0)
        self.action_client.send_goal_async(trayectoria_bajada)
        time.sleep(3.5)

        # FASE 3: Agarre
        self.get_logger().info('FASE 3: Cerrando Pinza (Grasp)...')
        self.controlar_pinza(True) # True = Cerrar
        time.sleep(1.5)

        # FASE 4: Retirada Cartesiana Recta
        self.get_logger().info('FASE 4: Ascenso cartesiano con la pieza...')
        trayectoria_subida = self.generar_linea_recta_cartesiana(x, y, self.Z_AGARRE, self.Z_SEGURO, 3.0)
        self.action_client.send_goal_async(trayectoria_subida)
        time.sleep(3.5)
        
        # FASE 5: Volver a Home con la pieza
        self.get_logger().info('FASE 5: Retorno articular a Home...')
        self.enviar_trayectoria_articular(self.q_home_8[:6].tolist(), tiempo_sec=3.0)
        self.q_memoria = self.q_home_8.copy()
        time.sleep(3.5)
        
        self.get_logger().info('¡PICK & PLACE COMPLETADO CON ÉXITO! Listo para la siguiente pieza.')
        self.sistema_ocupado = False

    def abortar(self, motivo):
        self.get_logger().error(f'Secuencia abortada: {motivo}')
        self.sistema_ocupado = False

    def controlar_pinza(self, cerrar: bool):
        msg = Bool()
        msg.data = cerrar
        self.gripper_pub.publish(msg)

    # ==============================================================
    # MOTORES MATEMÁTICOS (PINOCCHIO)
    # ==============================================================

    def calcular_ik_se3(self, x, y, z, q_semilla):
        """Cinemática Inversa que fuerza la Posición y la Orientación vertical."""
        
        # Queremos que la pinza mire hacia el piso. Dependiendo de cómo armaste tu URDF, 
        # suele ser una rotación de 90 grados (1.57 rad) en Pitch (Eje Y).
        R_mirar_abajo = pin.utils.rpyToMatrix(0.0, 1.5707963, 0.0) 
        M_deseada = pin.SE3(R_mirar_abajo, np.array([x, y, z]))
        
        q = q_semilla.copy()
        eps = 1e-4
        damp = 1e-6

        for i in range(1000):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            
            # Twist de error en el marco LOCAL del end effector
            M_actual = self.data.oMf[self.ee_frame_id]
            err = pin.log6(M_actual.actInv(M_deseada)).vector
            
            if np.linalg.norm(err) < eps:
                return q

            # Jacobiano en el marco LOCAL
            J_full = pin.computeFrameJacobian(self.model, self.data, q, self.ee_frame_id, pin.ReferenceFrame.LOCAL)
            
            # Aislar los 6 motores del brazo (congelar los dedos de la pinza)
            J_arm = J_full[:, :6]
            v_arm = J_arm.T @ np.linalg.solve(J_arm @ J_arm.T + damp * np.eye(6), err)
            
            v_full = np.zeros(self.model.nv)
            v_full[:6] = v_arm
            q = pin.integrate(self.model, q, v_full * 0.1)

        return None # No convergió

    def generar_linea_recta_cartesiana(self, x, y, z_inicio, z_fin, tiempo_total):
        """Genera una trayectoria densa de Action Server bajando/subiendo milímetro a milímetro."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        pasos = 15
        alturas_Z = np.linspace(z_inicio, z_fin, pasos)
        dt = tiempo_total / pasos
        
        tiempo_acumulado = 0.0
        
        for z in alturas_Z:
            # Calculamos la cinemática inversa usando el paso anterior como semilla
            q_paso = self.calcular_ik_se3(x, y, z, self.q_memoria)
            
            if q_paso is not None:
                self.q_memoria = q_paso.copy() # Actualizar memoria
                
                point = JointTrajectoryPoint()
                point.positions = q_paso[:6].tolist()
                
                tiempo_acumulado += dt
                point.time_from_start.sec = int(tiempo_acumulado)
                point.time_from_start.nanosec = int((tiempo_acumulado - int(tiempo_acumulado)) * 1e9)
                
                goal_msg.trajectory.points.append(point)
            else:
                self.get_logger().warning(f'Advertencia: No se pudo resolver IK en Z={z:.3f}')
                
        return goal_msg

    def enviar_trayectoria_articular(self, posiciones_rad, tiempo_sec):
        """Envía un solo punto al Action Server (Ideal para movimientos rápidos Fase 1 y 5)."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = posiciones_rad
        point.time_from_start.sec = int(tiempo_sec)
        point.time_from_start.nanosec = int((tiempo_sec - int(tiempo_sec)) * 1e9)
        
        goal_msg.trajectory.points = [point]
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    nodo = CerebroPickAndPlace()
    try:
        # MultiThreadedExecutor permite procesar callbacks mientras otro hilo hace time.sleep
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(nodo)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()