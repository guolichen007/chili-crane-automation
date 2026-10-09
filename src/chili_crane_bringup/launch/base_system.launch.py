"""Read-only composition; physical pulses exist only in an explicit CLI."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def compose(context):
    root = Path(LaunchConfiguration("config_root").perform(context))
    namespace = LaunchConfiguration("crane_id").perform(context)
    sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    if LaunchConfiguration("physical_output_enabled").perform(context).lower() != "false":
        raise ValueError("launch is read-only; use the finite-pulse bench CLI")
    mode = LaunchConfiguration("runtime_mode").perform(context)
    if mode not in {"mapping", "localization", "replay"}:
        raise ValueError("unsupported runtime mode")
    profile = yaml.safe_load((root / "profiles" / (mode + ".yaml")).read_text(encoding="utf-8"))
    if profile["safety"]["default_allow_auto_task"] is not False:
        raise ValueError("automatic control must remain disabled")
    common = {"use_sim_time": sim_time}
    nodes = [Node(
        package="chili_crane_control", executable="fail_safe_safety_supervisor.py",
        namespace=namespace, name="safety_supervisor", parameters=[common], output="screen")]
    if LaunchConfiguration("entry_mode").perform(context) == "bench":
        for device in ("adam6052", "adam6251"):
            nodes.append(Node(
                package="chili_crane_hardware", executable=device + "_node.py",
                namespace=namespace, parameters=[common, {
                    "device_config": str(root / "hardware" / (device + ".template.yaml")),
                }], output="screen"))
        nodes.append(Node(
            package="chili_crane_hardware", executable="io_normalizer_node.py",
            namespace=namespace, parameters=[common, {
                "mapping_config": str(root / "hardware" / "io_mapping.template.yaml"),
            }], output="screen"))
        nodes.append(Node(
            package="chili_crane_hardware", executable="pull_wire_node.py",
            namespace=namespace, parameters=[common, {
                "device_config": str(root / "hardware" / "pull_wire.template.yaml"),
                "mapping_config": str(root / "hardware" / "io_mapping.template.yaml"),
            }], output="screen"))
    else:
        nodes.append(Node(
            package="chili_crane_hardware", executable="mock_hardware_adapter.py",
            namespace=namespace, parameters=[common, {
                "crane_base_frame": namespace + "/base",
            }], output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("crane_id", default_value="crane_01"),
        DeclareLaunchArgument("config_root"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("physical_output_enabled", default_value="false"),
        DeclareLaunchArgument("runtime_mode", default_value="localization"),
        DeclareLaunchArgument("entry_mode", default_value="mock"),
        OpaqueFunction(function=compose),
    ])
