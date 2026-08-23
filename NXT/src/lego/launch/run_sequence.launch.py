from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    bridge = Node(
        package="lego",
        executable="stm32_moveit_bridge.py",
        name="stm32_moveit_bridge",
        output="screen",
        parameters=[
            {"gripper_close_threshold": 0.005},
            {"gripper_invert": False},
        ],
    )

    play_sequence = Node(
        package="lego",
        executable="play_sequence_full.py",
        name="play_sequence_full",
        output="screen",
    )

    joint2_debug = Node(
        package="lego",
        executable="plot_joint2_debug.py",
        name="joint2_debug",
        output="screen",
    )

    return LaunchDescription([
        bridge,
        joint2_debug,
        play_sequence,
    ])