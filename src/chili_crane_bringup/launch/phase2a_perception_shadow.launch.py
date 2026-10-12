"""Perception evidence only. No hardware, clock, safety, executor or task startup."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("chili_crane_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=share + "/config/perception/phase2a_grab.template.yaml"),
        Node(package="chili_crane_perception", executable="grab_perception_node.py", namespace="crane_01",
             parameters=[{"config_file": LaunchConfiguration("config_file")}]),
        Node(package="chili_crane_perception", executable="grab_clearance_node.py", namespace="crane_01",
             parameters=[{"config_file": LaunchConfiguration("config_file")}]),
    ])
