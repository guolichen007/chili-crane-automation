"""Explicit candidate view; does not start drivers, publish TF or activate extrinsics."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("chili_crane_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("site_config", default_value=share + "/config/sites/crane_01"),
        DeclareLaunchArgument("candidate"),
        Node(package="chili_crane_slam", executable="installation_preview_node.py", namespace="crane_01",
             parameters=[{"site_config": LaunchConfiguration("site_config"), "candidate": LaunchConfiguration("candidate")}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", share + "/rviz/extrinsic_candidate.rviz"])])
