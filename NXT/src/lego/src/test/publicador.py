#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class GeneradorTrayectoria(Node):
    def __init__(self):
        super().__init__('generador_trayectoria')

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/BRAZO_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Esperando ActionServer...')
        self.client.wait_for_server()
        self.get_logger().info('ActionServer conectado.')

        self.enviar_trayectoria()

    def enviar_trayectoria(self):
        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory.joint_names = [
            'Joint1',
            'Joint2',
            'Joint3',
            'Joint4',
            'Joint5',
            'Joint6'
        ]

        duracion = 5.0
        dt = 0.1
        n_puntos = int(duracion / dt) + 1

        for i in range(n_puntos):
            t = i * dt

            # Ejemplo: posiciones articulares deseadas en radianes
            q1 = 0.3 * math.sin(2.0 * math.pi * t / duracion)
            q2 = 0.0
            q3 = 0.0
            q4 = 0.0
            q5 = 0.0
            q6 = 0.0

            punto = JointTrajectoryPoint()
            punto.positions = [q1, q2, q3, q4, q5, q6]

            punto.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t - int(t)) * 1e9)
            )

            goal_msg.trajectory.points.append(punto)

        self.get_logger().info(
            f'Enviando trayectoria con {len(goal_msg.trajectory.points)} puntos'
        )

        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Trayectoria rechazada')
            return

        self.get_logger().info('Trayectoria aceptada')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info('Trayectoria finalizada')


def main():
    rclpy.init()
    node = GeneradorTrayectoria()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()