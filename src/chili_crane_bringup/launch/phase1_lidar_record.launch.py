"""Bag recording of explicit Phase1A topics; caller supplies a new artifact directory."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOPICS = ["/crane_01/lidar/er1_204/vendor_points", "/crane_01/lidar/er1_205/vendor_points",
    "/crane_01/lidar/er1_204/points", "/crane_01/lidar/er1_205/points", "/crane_01/lidar/merged_points",
    "/crane_01/lidar/dual_lidar/diagnostics", "/crane_01/lidar/dual_lidar/readiness",
    "/crane_01/hardware/trolley_state", "/crane_01/hardware/adam6052/raw_di", "/crane_01/hardware/adam6251/raw_di",
    "/crane_01/system/readiness", "/crane_01/perception/grab_state", "/crane_01/perception/pit_surface",
    "/crane_01/system/run_manifest", "/tf", "/tf_static"]


def compose(context):
    artifact = Path(LaunchConfiguration("artifact_dir").perform(context)).resolve()
    if not artifact.is_dir() or (artifact / "rosbag2").exists() or not (artifact / "run_manifest.yaml").is_file():
        raise ValueError("prepared new manifest/artifact directory required")
    return [Node(package="chili_crane_slam", executable="run_manifest_node.py", namespace="crane_01",
                 parameters=[{"manifest_file": str(artifact / "run_manifest.yaml")}]),
            ExecuteProcess(cmd=["ros2", "bag", "record", "--output", str(artifact / "rosbag2")] + TOPICS)]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("artifact_dir"), OpaqueFunction(function=compose)])
