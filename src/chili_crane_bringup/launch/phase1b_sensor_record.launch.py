"""Record explicit time-only topics; sensors must already be separately running."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def compose(context):
    artifact = Path(LaunchConfiguration("artifact_dir").perform(context)).resolve()
    manifest = artifact / "run_manifest.yaml"
    if not artifact.is_dir() or (artifact / "rosbag2").exists() or not manifest.is_file():
        raise ValueError("prepared new artifact directory required")
    data = yaml.safe_load(manifest.read_text())
    topics = data.get("required_topics")
    if data.get("phase") != "1B" or not topics or any("merged" in t or not t.startswith("/crane_01/") for t in topics):
        raise ValueError("Phase1B time-only manifest required")
    return [Node(package="chili_crane_slam", executable="run_manifest_node.py", namespace="crane_01",
                 parameters=[{"manifest_file": str(manifest)}]),
            ExecuteProcess(cmd=["ros2", "bag", "record", "--output", str(artifact / "rosbag2")] + topics)]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("artifact_dir"), OpaqueFunction(function=compose)])
