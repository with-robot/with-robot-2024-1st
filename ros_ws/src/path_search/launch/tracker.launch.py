import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name1 = "path_search"
    package_name2 = "object_detection"

    unity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("jetauto_description"),
                "launch",
                "unity.launch.py",
            )
        ),
    )

    path_find = Node(
        package=package_name1,
        executable="track",
        output="screen",
        parameters=[],
    )

    node_object_detection = Node(
        package=package_name2,
        executable="object_detection",
        output="screen",
        parameters=[],
    )

    return LaunchDescription([unity_launch, path_find, node_object_detection])
