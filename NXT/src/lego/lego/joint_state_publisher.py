#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        # Asumimos 6 joints con nombres como en tu URDF
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Configuración A y B (en radianes)
        self.qA = np.array([0.0, -0.5, 0.3, 0.0, 0.5, 0.0])
        self.qB = np.array([0.5,  0.0, -0.3, 0.3, 0.0, 0.3])

        self.n_steps = 100
        self.step = 0
        self.forward = True

    def timer_callback(self):
        # Interpolación lineal
        t = self.step / self.n_steps
        q = (1 - t) * self.qA + t * self.qB

        # Crear mensaje
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q.tolist()

        self.publisher.publish(msg)

        # Avance / retroceso
        if self.forward:
            self.step += 1
            if self.step >= self.n_steps:
                self.forward = False
        else:
            self.step -= 1
            if self.step <= 0:
                self.forward = True

def main():
    rclpy.init()
    node = JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()