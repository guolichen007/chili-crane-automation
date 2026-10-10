"""ROS2 adapter: raw normalization, health, consume-once pairs, calibrated products."""
from pathlib import Path
import time
import json
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from chili_crane_control.qos import state_qos
from chili_crane_control.evidence_policy import SourceType
from chili_crane_msgs.msg import DualLidarReadiness
from .pointcloud import normalize_cloud, crop, validate_points
from .dual_lidar import Cloud, DualLidarHealth, DualLidarSynchronizer, Extrinsic, DualLidarMerger
from .rotation import quaternion


def cloud_message(cloud):
    validate_points(cloud.points)
    msg = PointCloud2()
    msg.header.stamp = Time(nanoseconds=int(cloud.stamp * 1e9)).to_msg()
    msg.header.frame_id = cloud.frame_id
    msg.height, msg.width = 1, len(cloud.points)
    msg.fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
        for name, offset, datatype in (("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7),
                                     ("ring", 16, 4), ("sensor_id", 18, 2), ("timestamp", 24, 8))]
    msg.point_step, msg.row_step = 32, 32 * msg.width
    msg.is_bigendian, msg.is_dense = False, True
    msg.data = cloud.points.tobytes()
    return msg


class DualLidarNode(Node):
    def __init__(self):
        super().__init__("dual_lidar_pipeline")
        self.declare_parameter("site_config", "")
        site = Path(self.get_parameter("site_config").value)
        self._cfg = yaml.safe_load((site / "sensors/dual_lidar.yaml").read_text())
        if (self._cfg.get("consume_once") is not True or self._cfg.get("allow_old_frame_reuse") is not False
                or self._cfg.get("single_lidar_fallback") is not False):
            raise ValueError("unsafe dual-lidar contract")
        self._source = getattr(SourceType, self._cfg.get("source_type", "UNKNOWN"))
        tolerance = self._cfg.get("point_timestamp_header_tolerance_sec")
        if tolerance is not None and (type(tolerance) not in (float, int)
                or not math.isfinite(tolerance) or tolerance <= 0):
            raise ValueError("invalid point timestamp tolerance")
        if self.get_parameter("use_sim_time").value:
            self._source = SourceType.REPLAY
        self._sync = self._merger = None
        # 1s is only the offline diagnostic display threshold; no ready uses this fallback.
        self._health = DualLidarHealth(self._cfg.get("stale_timeout_sec") or 1.0)
        try:
            if self._cfg.get("config_state") != "VALID" or self._cfg.get("clocks_synchronized") is not True:
                raise ValueError("pair timing/clock evidence NOT_CONFIGURED")
            self._sync = DualLidarSynchronizer(self._cfg["maximum_pair_delta_sec"],
                self._cfg["stale_timeout_sec"], self._cfg["maximum_queue_size"])
        except (TypeError, ValueError, KeyError) as exc:
            self.get_logger().warning(str(exc))
        try:
            a = Extrinsic.from_config(yaml.safe_load((site / "calibration/er1_204_to_base.yaml").read_text()))
            b = Extrinsic.from_config(yaml.safe_load((site / "calibration/er1_205_to_base.yaml").read_text()))
            if a.target_frame != self._cfg["output_frame"]:
                raise ValueError("configured output frame differs from extrinsic")
            self._merger = DualLidarMerger(a, b)
        except (TypeError, ValueError, KeyError) as exc:
            self.get_logger().warning(str(exc))
        if self._merger is not None:
            self._tf = StaticTransformBroadcaster(self)
            transforms = []
            for calibration in (self._merger.a, self._merger.b):
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = calibration.target_frame
                transform.child_frame_id = calibration.source_frame
                transform.transform.translation.x = float(calibration.translation[0])
                transform.transform.translation.y = float(calibration.translation[1])
                transform.transform.translation.z = float(calibration.translation[2])
                q = quaternion(calibration.rotation)
                transform.transform.rotation.x, transform.transform.rotation.y = float(q[0]), float(q[1])
                transform.transform.rotation.z, transform.transform.rotation.w = float(q[2]), float(q[3])
                transforms.append(transform)
            self._tf.sendTransform(transforms)
        self._frames = {}
        self._frame_valid = {"204": False, "205": False}
        self._raw_pubs, self._cloud_subscriptions = {}, []
        for sensor in ("204", "205"):
            cfg = yaml.safe_load((site / "sensors" / ("er1_" + sensor + ".yaml")).read_text())
            self._frames[sensor] = cfg["frame_id"]
            self._raw_pubs[sensor] = self.create_publisher(PointCloud2, cfg["normalized_topic"], qos_profile_sensor_data)
            self._cloud_subscriptions.append(self.create_subscription(PointCloud2, cfg["raw_topic"],
                lambda msg, sensor=sensor: self._receive(sensor, msg), qos_profile_sensor_data))
        self._merged = self.create_publisher(PointCloud2, "lidar/merged_points", qos_profile_sensor_data)
        self._products = {name: self.create_publisher(PointCloud2, name, qos_profile_sensor_data)
                          for name in self._cfg["products"]}
        self._diagnostics = self.create_publisher(String, "lidar/dual_lidar/diagnostics", state_qos())
        self._ready = self.create_publisher(DualLidarReadiness, "lidar/dual_lidar/readiness", state_qos())
        self._error = "WAITING_FOR_TWO_CLOUDS"
        self._last_merged = None
        self._last_source_now = 0
        self._clock_fault = False
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(.1, self._publish_health, clock=self._steady)

    def _now(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self._last_source_now:
            self._clock_fault = True
            self._error = "SOURCE_CLOCK_REGRESSION_RESTART_REQUIRED"
            self._last_merged = None
        self._last_source_now = now
        return now, time.monotonic()

    def _receive(self, sensor, msg):
        now, monotonic = self._now()
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if (self._clock_fault or not math.isfinite(stamp) or stamp <= 0 or stamp > now
                    or msg.header.frame_id != self._frames[sensor]):
                raise ValueError("cloud stamp/frame invalid")
            points = normalize_cloud(msg.data, [(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
                msg.point_step, msg.row_step, msg.width, msg.height, msg.is_bigendian,
                0 if sensor == "204" else 1)
            tolerance = self._cfg.get("point_timestamp_header_tolerance_sec")
            if tolerance is not None and (not isinstance(tolerance, (float, int)) or tolerance <= 0
                    or (abs(points["timestamp"] - stamp) > tolerance).any()):
                raise ValueError("point timestamp clock/units mismatch")
            cloud = Cloud(stamp, monotonic, msg.header.frame_id, points, int(self._source))
            self._frame_valid[sensor] = True
            self._health.accepted(sensor, cloud)
            self._raw_pubs[sensor].publish(cloud_message(cloud))
            if self._sync is not None:
                pairs = self._sync.push(sensor, cloud, now, monotonic)
                for pair in pairs:
                    if self._merger is None:
                        self._error = "EXTRINSIC_NOT_CONFIGURED"
                        continue
                    merged = self._merger.merge(pair)
                    self._merged.publish(cloud_message(merged))
                    self._last_merged = pair
                    self._error = "PAIRED_CALIBRATED"
                    for name, product in self._cfg["products"].items():
                        if product.get("config_state") == "VALID":
                            cropped = crop(merged.points, product.get("roi_bounds_m"))
                            if len(cropped):
                                self._products[name].publish(cloud_message(Cloud(
                                    merged.stamp, merged.received_monotonic, merged.frame_id,
                                    cropped, merged.source_type)))
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self._health.invalid[sensor] += 1
            self._frame_valid[sensor] = False
            self._last_merged = None
            self._error = str(exc)

    def _publish_health(self):
        now, monotonic = self._now()
        a, b = [self._health.status(sensor, now, monotonic) for sensor in ("204", "205")]
        if self._sync:
            self._sync.expire(now, monotonic)
        pair = self._last_merged
        pairing = bool(pair and self._sync and all(0 <= now - c.stamp <= self._sync.stale
            and 0 <= monotonic - c.received_monotonic <= self._sync.stale for c in pair))
        message = DualLidarReadiness()
        message.header.stamp = self.get_clock().now().to_msg()
        message.reason = self._error
        for label, status in (("a", a), ("b", b)):
            setattr(message, label + "_online", status["online"])
            setattr(message, label + "_fresh", status["fresh"])
            sensor = "204" if label == "a" else "205"
            setattr(message, label + "_frame_valid", status["online"] and self._frame_valid[sensor])
            setattr(message, label + "_calibration_valid", self._merger is not None)
        message.pairing_valid = pairing
        message.extrinsic_valid = self._merger is not None
        message.coverage_valid = self._cfg.get("coverage_verified") is True
        message.dual_lidar_ready = (not self._clock_fault and message.a_frame_valid and message.b_frame_valid
            and a["fresh"] and b["fresh"] and pairing
            and message.extrinsic_valid and message.coverage_valid
            and self._cfg.get("point_timestamp_header_tolerance_sec") is not None)
        message.validity = message.VALID if message.dual_lidar_ready else message.DEGRADED
        message.evidence.source_type = int(self._source)
        message.evidence.validity = message.validity
        message.evidence.reason = message.reason
        if pair:
            message.evidence.measurement_stamp = Time(nanoseconds=int(min(c.stamp for c in pair) * 1e9)).to_msg()
            message.evidence.receive_stamp = message.header.stamp
            message.evidence.source_counter = self._sync.paired_count
            message.evidence.evidence_age_sec = max(now - c.stamp for c in pair)
        self._ready.publish(message)
        diagnostics = {"204": a, "205": b, "invalid": self._health.invalid,
            "pair_delta_ms": self._sync.pair_delta * 1000 if self._sync and self._sync.pair_delta is not None else None,
            "paired_count": self._sync.paired_count if self._sync else 0,
            "dropped_204": self._sync.dropped["204"] if self._sync else 0,
            "dropped_205": self._sync.dropped["205"] if self._sync else 0,
            "queue_depth": {k: len(v) for k, v in self._sync.queues.items()} if self._sync else {"204": 0, "205": 0},
            "frame_valid": message.a_frame_valid and message.b_frame_valid,
            "calibration_valid": message.extrinsic_valid, "pairing_valid": pairing,
            "coverage_valid": message.coverage_valid, "dual_lidar_ready": message.dual_lidar_ready,
            "reused_old_frame": False, "source_type": self._source.name, "reason": self._error}
        self._diagnostics.publish(String(data=json.dumps(diagnostics, allow_nan=False)))


def main():
    rclpy.init()
    node = DualLidarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
