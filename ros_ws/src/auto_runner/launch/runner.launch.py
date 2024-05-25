import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name1 = "auto_runner"
    package_name2 = "object_detection"
    package_name3 = "gmapping"


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
        executable="runner",
        output="screen",
        parameters=[],
    )

    node_object_detection = Node(
        package=package_name2,
        executable="object_detection",
        output="screen",
        parameters=[],
    )

    create_map = Node(
        package=package_name3,
        executable="create_map",
        output="screen",
        parameters=[],
    )

    # return LaunchDescription([unity_launch, path_find, node_object_detection, map_make])
    return LaunchDescription([unity_launch, path_find, create_map])
