#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import cv2.aruco as aruco
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class VisionArucoNode(Node):
    def __init__(self):
        super().__init__('vision_aruco_node')
        
        # 1. Crear el publicador de la posición del objeto (en metros)
        self.publisher_ = self.create_publisher(Point, '/target_position', 10)
        
        # 2. Cargar la Matriz de Homografía de forma dinámica
        try:
            package_share_dir = get_package_share_directory('lego')
            ruta_matriz = os.path.join(package_share_dir, 'homografia.npy')
            self.H = np.load(ruta_matriz)
            self.get_logger().info(f'Matriz cargada dinámicamente desde: {ruta_matriz}')
        except Exception as e:
            self.get_logger().error(f'Error al cargar homografia.npy: {str(e)}')
            raise

        # 3. Inicializar la cámara
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara en el índice 2.')
            raise
            
        # 4. Configurar el detector ArUco (SINTAXIS PARA OPENCV < 4.7.0)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # 5. Crear un timer para capturar y procesar video a 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nodo de visión ArUco iniciado. Buscando pieza LEGO...')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Fallo al capturar el frame de la cámara.')
            return

        # Detectar los marcadores (SINTAXIS PARA OPENCV < 4.7.0)
        corners, ids, rejected = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            c = corners[0][0]
            
            centro_u = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4.0
            centro_v = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4.0
            
            cv2.circle(frame, (int(centro_u), int(centro_v)), 5, (0, 0, 255), -1)

            # --- CONVERSIÓN HOMOGRÁFICA ---
            punto_pixeles = np.array([centro_u, centro_v, 1.0])
            punto_mundo_homogeneo = np.dot(self.H, punto_pixeles)
            
            x_metro = punto_mundo_homogeneo[0] / punto_mundo_homogeneo[2]
            y_metro = punto_mundo_homogeneo[1] / punto_mundo_homogeneo[2]

            self.get_logger().info(f'Pieza detectada en: X={x_metro:.3f}m, Y={y_metro:.3f}m')

            # --- PUBLICAR EN ROS 2 ---
            msg = Point()
            msg.x = float(x_metro)
            msg.y = float(y_metro)
            msg.z = 0.0 
            
            self.publisher_.publish(msg)

        cv2.imshow('Vision ArUco - Manipulador LEGO', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Deteniendo nodo de visión ArUco de forma segura...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()