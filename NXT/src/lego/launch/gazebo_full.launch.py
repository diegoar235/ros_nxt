from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    urdf_file = PathJoinSubstitution([
        FindPackageShare('lego'),
        'urdf',
        'Ensamblaje2.urdf'
    ])

    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # -----------------------------
    # Gazebo
    # -----------------------------
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', '/opt/ros/humble/lib/libgazebo_ros_init.so',
            '-s', '/opt/ros/humble/lib/libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # -----------------------------
    # Robot State Publisher
    # -----------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # -----------------------------
    # Spawn robot en Gazebo
    # -----------------------------
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'lego_robot',
            '-z', '0.2'
        ]
    )

    # -----------------------------
    # Spawner joint_state_broadcaster
    # -----------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    # -----------------------------
    # Spawner arm_controller
    # -----------------------------
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    # -----------------------------
    # Pose inicial del brazo
    # Equivalente a:
    #
    # ros2 topic pub --once /arm_controller/joint_trajectory ...
    #
    # Joint1 = 0
    # Joint2 = 0
    # Joint3 = 0
    # Joint4 = 0
    # Joint5 = pi/2
    # Joint6 = 0
    # -----------------------------
    set_initial_arm_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/arm_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory',
            """{
joint_names: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
points: [
  {
    positions: [0.0, 0.0, 0.0, 0.0, 0, 0.0],
    time_from_start: {sec: 3, nanosec: 0}
  }
]
}"""
        ],
        output='screen'
    )

    return LaunchDescription([

        # -----------------------------
        # Variables de entorno Gazebo
        # -----------------------------
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', '/root/ros_ws/src'),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', '/opt/ros/humble/lib'),

        # -----------------------------
        # Lanzar Gazebo
        # -----------------------------
        gzserver,
        gzclient,

        # -----------------------------
        # Publicar robot_description
        # -----------------------------
        robot_state_publisher,

        # -----------------------------
        # Spawn del robot
        # Espero 5 s para que Gazebo esté listo
        # -----------------------------
        TimerAction(
            period=5.0,
            actions=[
                spawn_robot
            ]
        ),

        # -----------------------------
        # Activar controladores
        # Espero 8 s para que el robot ya exista en Gazebo
        # -----------------------------
        TimerAction(
            period=8.0,
            actions=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
            ]
        ),

        # -----------------------------
        # Mandar el robot a la pose inicial
        # Espero 11 s para que arm_controller ya esté activo
        # -----------------------------
        TimerAction(
            period=11.0,
            actions=[
                set_initial_arm_pose
            ]
        ),
    ])