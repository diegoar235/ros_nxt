from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_lego = FindPackageShare('lego')
    pkg_ctrl = FindPackageShare('lego_cartesian_control')

    urdf_file = PathJoinSubstitution([
        pkg_lego,
        'urdf',
        'Ensamblaje.urdf'
    ])

    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    controllers_file = PathJoinSubstitution([
        pkg_ctrl,
        'config',
        'controllers.yaml'
    ])

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'lego_robot'],
        output='screen'
    )



    # spawners
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster'],
        output='screen'
    )

    arm_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'arm_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_entity,
        joint_state_broadcaster,
        arm_controller
    ])