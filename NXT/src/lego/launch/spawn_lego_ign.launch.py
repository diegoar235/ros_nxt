from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=[
                '-name', 'lego_robot',
                '-file', '/root/ros_ws/src/lego/urdf/Ensamblaje.urdf',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )
    ])

