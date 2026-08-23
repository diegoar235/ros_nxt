#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from moveit_py.robot_model import RobotModel
from moveit_py.planning import PlanningComponent
from geometry_msgs.msg import PoseStamped

class MoveItPyExample(Node):
    def __init__(self):
        super().__init__('moveit_py_example_node')

        # Inicializa el modelo del robot
        self.robot_model = RobotModel()
        self.robot_model.load_robot_model("robot_description")

        # Crea el componente de planificación para el grupo de planificación (ej: "panda_arm")
        self.planning_component = PlanningComponent("panda_arm", self.robot_model)

        # Espera que estén disponibles los nodos de MoveIt
        self.get_logger().info("Esperando a MoveIt...")

        # Define la pose objetivo
        target_pose = PoseStamped()
        target_pose.header.frame_id = "panda_link0"
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.4
        target_pose.pose.orientation.w = 1.0

        # Asigna objetivo de pose
        self.planning_component.set_goal_state(pose_stamped_msg=target_pose)

        # Planea la trayectoria
        plan_solution = self.planning_component.plan()

        if plan_solution:
            self.get_logger().info("Plan encontrado. Ejecutando...")
            self.planning_component.execute()
        else:
            self.get_logger().warn("No se encontró plan.")

def main(args=None):
    rclpy.init(args=args)
    node = MoveItPyExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
