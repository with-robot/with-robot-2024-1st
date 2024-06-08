import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name = "jetauto_control"

    unity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("jetauto_description"),
                "launch",
                "unity.launch.py",
            )
        ),
    )

    node_object_detection = Node(
        package=package_name,
        executable="object_detection",
        output="screen",
        parameters=[],
    )

    node_object_detection_viz = Node(
        package=package_name,
        executable="object_detection_viz",
        output="screen",
        parameters=[],
    )

    node_lidar_driving = Node(
        package=package_name,
        executable="lidar_driving",
        output="screen",
        parameters=[],
    )

    return LaunchDescription(
        [
            unity_launch,
            node_object_detection,
            node_object_detection_viz,
            node_lidar_driving,
        ]
    )