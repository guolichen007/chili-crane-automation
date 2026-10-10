"""Phase 1A site composition: physical reads, isolated simulation, no output writer."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def compose(context):
    root = Path(LaunchConfiguration("config_root").perform(context))
    site = root / "sites" / "crane_01"
    mode = LaunchConfiguration("profile").perform(context)
    if mode not in {"algorithm_dev", "shadow_control", "production"}:
        raise ValueError("unknown Phase1A profile")
    policy = yaml.safe_load((site / "runtime" / (mode + ".yaml")).read_text())
    if policy.get("physical_output_enabled") is not False or policy.get("automatic_control_enabled") is not False:
        raise ValueError("Phase1A requires physical and automatic outputs disabled")
    if mode == "production" and any(policy.get(k) is not False for k in (
            "simulated_evidence_allowed", "synthetic_evidence_allowed", "replay_evidence_allowed")):
        raise ValueError("production evidence policy must be physical only")
    ns = "crane_01"
    nodes = []
    for device in ("adam6052", "adam6251"):
        nodes.append(Node(package="chili_crane_hardware", executable=device + "_node.py",
            namespace=ns, parameters=[{"device_config": str(site / "hardware" / (device + ".yaml"))}]))
    nodes.append(Node(package="chili_crane_hardware", executable="pull_wire_node.py", namespace=ns,
        parameters=[{"device_config": str(site / "hardware/pull_wire_y.development.yaml"),
                     "mapping_config": str(site / "hardware/io_mapping.yaml")}]))
    nodes.append(Node(package="chili_crane_hardware", executable="io_normalizer_node.py", namespace=ns,
        parameters=[{"mapping_config": str(site / "hardware/io_mapping.yaml")}]))
    nodes.append(Node(package="chili_crane_control", executable="fail_safe_safety_supervisor.py", namespace=ns))
    nodes.append(Node(package="chili_crane_control", executable="action_executor.py", namespace=ns,
        parameters=[{"capability_config": str(site / "runtime" / (mode + ".yaml"))}]))
    if mode != "production":
        nodes.append(Node(package="chili_crane_hardware", executable="development_sources.py", namespace=ns,
            parameters=[{"simulation_config": str(site / "hardware/simulated_observations.yaml")}]))
        nodes.append(Node(package="chili_crane_hardware", executable="mock_hardware_adapter.py",
            namespace=ns + "/mock_sink", remappings=[("control/actuation_request", "/crane_01/control/actuation_request")]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory("chili_crane_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("config_root", default_value=share + "/config"),
        DeclareLaunchArgument("profile", default_value="algorithm_dev"),
        OpaqueFunction(function=compose),
    ])
