import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
    params = {
        "robot_description": robot_description.toxml(),
        "use_sim_time": use_sim_time,
    }
    node_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    rviz_file = os.path.join(pkg_share, "rviz", "jetauto.rviz")
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
    )

    node_joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        arguments=[],
    )

    return LaunchDescription(
        [
            node_rsp,
            node_rviz,
            node_joint_state_publisher,
        ]
    )
