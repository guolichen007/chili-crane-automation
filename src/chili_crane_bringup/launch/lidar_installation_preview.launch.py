"""Explicit sensor-only installation profile, no control nodes or production TF."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from chili_crane_slam.installation_profile import prepare
from chili_crane_hardware.er1_transport import driver_transport
from chili_crane_slam.timebase import ClockContract


def compose(context):
    root = LaunchConfiguration("config_root").perform(context)
    profile = LaunchConfiguration("profile").perform(context)
    configured = prepare(root, profile)
    site = configured / "sites/crane_01"
    # Same fixed SDK config contract; no decoder or time-core modifications.
    lidars = []
    for sensor in ("204", "205"):
        cfg = yaml.safe_load((site / ("sensors/er1_" + sensor + ".yaml")).read_text())
        ClockContract.from_config(cfg)
        lidars.append({"driver": {**driver_transport(cfg), "lidar_type": "RSE1",
            "msop_port": cfg["msop_port"], "difop_port": cfg["difop_port"], "use_lidar_clock": True,
            "ts_first_point": True, "dense_points": True}, "ros": {
            "ros_frame_id": cfg["frame_id"], "ros_send_point_cloud_topic": cfg["raw_topic"], "ros_queue_length": 5}})
    vendor = configured.parent / "vendor.yaml"
    vendor.write_text(yaml.safe_dump({"common": {"msg_source": 1, "send_packet_ros": False,
        "send_point_cloud_ros": True}, "lidar": lidars}), encoding="utf-8")
    share = get_package_share_directory("chili_crane_bringup")
    view = LaunchConfiguration("view").perform(context)
    if view not in {"204_only", "205_only", "rough"}:
        raise ValueError("EXPLICIT_INSTALLATION_VIEW_REQUIRED")
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share + "/launch/phase1b_sensors.launch.py"),
            launch_arguments={"config_root": str(configured), "start_vendor": "true", "vendor_config": str(vendor),
                              "start_clock_observer": "true", "start_camera": "false"}.items()),
        Node(package="chili_crane_slam", executable="installation_preview_node.py", namespace="crane_01",
             parameters=[{"site_config": str(site)}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", share + "/rviz/lidar_installation_" + view + ".rviz"]),
    ]


def generate_launch_description():
    share = get_package_share_directory("chili_crane_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("config_root", default_value=share + "/config"),
        DeclareLaunchArgument("profile", default_value="installation_preview"),
        DeclareLaunchArgument("view", default_value="204_only"),
        OpaqueFunction(function=compose)])
