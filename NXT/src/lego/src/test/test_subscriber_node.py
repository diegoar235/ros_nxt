#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class TestVisionSubscriber(Node):
    def __init__(self):
        super().__init__('test_vision_subscriber')
        
        # Suscriptor al tópico donde la cámara publica las coordenadas
        self.subscription = self.create_subscription(
            Point,
            '/target_position',
            self.listener_callback,
            10) # 10 es el tamaño de la cola de mensajes
            
        self.get_logger().info('Oído robótico encendido. Esperando coordenadas de la cámara...')

    def listener_callback(self, msg):
        # Esta función se ejecuta automáticamente CADA VEZ que la cámara ve la pieza
        x = msg.x
        y = msg.y
        
        # Imprimimos la confirmación en verde
        self.get_logger().info(f'¡Coordenada recibida! Listo para mandar el brazo a: X={x:.3f}m, Y={y:.3f}m')

def main(args=None):
    rclpy.init(args=args)
    nodo_prueba = TestVisionSubscriber()
    
    try:
        # spin() mantiene el nodo vivo escuchando mensajes infinitamente
        rclpy.spin(nodo_prueba)
    except KeyboardInterrupt:
        nodo_prueba.get_logger().info('Apagando nodo de prueba...')
    finally:
        nodo_prueba.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()