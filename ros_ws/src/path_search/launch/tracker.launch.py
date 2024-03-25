import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name = "path_search"

    unity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("jetauto_description"),
                "launch",
                "unity.launch.py",
            )
        ),
    )

    node_web_control = Node(
        package=package_name,
        executable="track",
        output="screen",
        parameters=[],
    )

    return LaunchDescription(
        [
            unity_launch,
            node_web_control,
        ]
    )
