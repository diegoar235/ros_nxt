from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1) Include del launch principal de MoveIt
    moveit_pkg = "Moveit"  # <-- cambiá esto
    moveit_share = get_package_share_directory(moveit_pkg)

    # Ejemplo típico: demo.launch.py o move_group.launch.py
    moveit_launch = os.path.join(moveit_share, "launch", "demo.launch.py")  # <-- ajustá

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
    )

    # 2) Tu bridge (ActionServer)
    bridge = Node(
        package="lego",          # <-- el paquete donde lo pusiste
        executable="stm32_moveit_bridge.py",  # <-- si es C++; si es Python ver opción 2

        output="screen",
        parameters=[
            # opcional: parámetros si los hacés configurables
        ],
    )

    return LaunchDescription([
        moveit,
        bridge
    ])

