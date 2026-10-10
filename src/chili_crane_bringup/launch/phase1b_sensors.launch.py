"""Dual LiDAR timing-only and one optional auxiliary camera; no output/control nodes."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def camera(context):
    site = Path(LaunchConfiguration("config_root").perform(context)) / "sites/crane_01"
    cfg_path = site / "sensors/hik_01.yaml"
    cfg = yaml.safe_load(cfg_path.read_text())
    if LaunchConfiguration("start_camera").perform(context).lower() != "true":
        return []
    if cfg.get("driver_enabled") is not True or cfg.get("camera_control_authority") is not False:
        raise ValueError("explicit camera identity/configuration required")
    return [Node(package="chili_crane_hardware", executable="hik_camera_node.py", namespace="crane_01",
                 parameters=[{"camera_config": str(cfg_path), "use_sim_time": False}], output="screen")]


def generate_launch_description():
    share = get_package_share_directory("chili_crane_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("config_root", default_value=share + "/config"),
        DeclareLaunchArgument("start_vendor", default_value="false"),
        DeclareLaunchArgument("vendor_config", default_value=""),
        DeclareLaunchArgument("start_camera", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share + "/launch/phase1_lidar.launch.py"),
            launch_arguments={"config_root": LaunchConfiguration("config_root"),
                "start_vendor": LaunchConfiguration("start_vendor"), "vendor_config": LaunchConfiguration("vendor_config"),
                "timing_only": "true"}.items()),
        OpaqueFunction(function=camera),
    ])
