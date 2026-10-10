"""Normalization/health always; vendor process only with explicitly validated config."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from chili_crane_slam.timebase import ClockContract
from chili_crane_hardware.er1_transport import driver_transport


def compose(context):
    site = Path(LaunchConfiguration("config_root").perform(context)) / "sites/crane_01"
    nodes = [Node(package="chili_crane_slam", executable="dual_lidar_node.py", namespace="crane_01",
                  parameters=[{"site_config": str(site), "use_sim_time": False,
                  "force_timing_only": LaunchConfiguration("timing_only").perform(context).lower() == "true"}], output="screen")]
    if LaunchConfiguration("start_vendor").perform(context).lower() == "true":
        path = Path(LaunchConfiguration("vendor_config").perform(context))
        vendor = yaml.safe_load(path.read_text())
        if len(vendor.get("lidar", [])) != 2:
            raise ValueError("exactly two official lidar configurations required")
        ports = set()
        clocks = []
        for sensor, driver in zip(("er1_204", "er1_205"), vendor["lidar"]):
            cfg = yaml.safe_load((site / "sensors" / (sensor + ".yaml")).read_text())
            clocks.append(ClockContract.from_config(cfg))
            if any(driver["driver"].get(k) != v for k, v in driver_transport(cfg).items()):
                raise ValueError("vendor/site transport contract mismatch")
            if (cfg.get("driver_enabled") is not True or cfg.get("port_roles") != "VALID"
                    or cfg.get("clock_mode") not in {"SENSOR_PTP", "HOST_DERIVED"}
                    or driver["driver"].get("lidar_type") != "RSE1"
                    or driver["driver"].get("use_lidar_clock") != (cfg["clock_mode"] == "SENSOR_PTP")
                    or driver["ros"].get("ros_send_point_cloud_topic") != cfg["raw_topic"]
                    or driver["ros"].get("ros_frame_id") != cfg["frame_id"]):
                raise ValueError("vendor/site clock/frame/port contract mismatch")
            for role in ("msop_port", "difop_port"):
                port = cfg.get(role)
                if type(port) is not int or not 1 <= port <= 65535 or port in ports or driver["driver"].get(role) != port:
                    raise ValueError("vendor port not explicitly verified")
                ports.add(port)
        if not clocks[0].compatible(clocks[1]):
            raise ValueError("MIXED_CLOCK_MODE_OR_DOMAIN")
        nodes.append(Node(package="rslidar_sdk", executable="rslidar_sdk_node", namespace="crane_01/vendor",
                          parameters=[{"config_path": str(path)}], output="screen"))
    return nodes


def generate_launch_description():
    share = get_package_share_directory("chili_crane_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("config_root", default_value=share + "/config"),
        DeclareLaunchArgument("start_vendor", default_value="false"),
        DeclareLaunchArgument("vendor_config", default_value=""),
        DeclareLaunchArgument("timing_only", default_value="true"),
        OpaqueFunction(function=compose),
    ])
