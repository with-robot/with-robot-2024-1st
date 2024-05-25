import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name = "gmapping"

    unity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("jetauto_description"),
                "launch",
                "unity.launch.py",
            )
        ),
    )
    grid_map_node = Node(
        package=package_name,
        executable="create_map",
        output="screen",
        parameters=[],
    )

    return LaunchDescription([unity_launch, grid_map_node])
