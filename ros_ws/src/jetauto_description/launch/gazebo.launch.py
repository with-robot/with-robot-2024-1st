import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_name = "jetauto_description"
    use_sim_time = True
    odom_frame = "odom"
    base_frame = "base_footprint"
    lidar_view = "false"
    depth_camera_name = "camera"

    pkg_share = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_share, "urdf", "jetauto.xacro")
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            "odom_frame": odom_frame,
            "base_frame": base_frame,
            "lidar_view": lidar_view,
            "depth_camera_name": depth_camera_name,
        },
    )

    node_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description.toxml(),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    node_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    node_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "jetauto"],
        output="screen",
    )

    node_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                ),
            ]
        ),
        launch_arguments={"verbose": "true", "s": "libgazebo_ros_factory.so"}.items(),
    )

    rviz_file = os.path.join(pkg_share, "rviz", "jetauto.rviz")
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=node_spawn_entity,
                    on_exit=[node_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=node_joint_state_broadcaster,
                    on_exit=[node_rviz],
                )
            ),
            node_gazebo,
            node_rsp,
            node_spawn_entity,
        ]
    )
