"""Explicit auxiliary image preview; SDK identity/config required, never writes PTP."""
from pathlib import Path
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def compose(context):
    root = Path(LaunchConfiguration("config_root").perform(context))
    cfg = yaml.safe_load((root / "sites/crane_01/sensors/hik_01.yaml").read_text())
    if cfg.get("expected_serial") in {None, "", "NOT_CONFIGURED"}:
        raise ValueError("CAMERA_SDK_SERIAL_REQUIRED")
    preview = yaml.safe_load((root / "runtime/camera_installation_preview.yaml").read_text())
    cfg.update({k: v for k, v in preview.items() if k not in {"profile", "note"}})
    cfg["driver_enabled"] = True
    path = Path(tempfile.mkdtemp(prefix="chili-camera-preview-")) / "camera.yaml"
    path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
    return [Node(package="chili_crane_hardware", executable="hik_camera_node.py", namespace="crane_01",
        parameters=[{"camera_config": str(path)}]),
        Node(package="rqt_image_view", executable="rqt_image_view", arguments=["/crane_01/camera/hik_01/image_raw"])]


def generate_launch_description():
    share = get_package_share_directory("chili_crane_bringup")
    return LaunchDescription([DeclareLaunchArgument("config_root", default_value=share + "/config"),
                              OpaqueFunction(function=compose)])
