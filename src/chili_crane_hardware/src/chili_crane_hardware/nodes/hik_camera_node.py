"""One explicitly bound auxiliary camera; no SafetyPermit or actuation publishers."""
import json
from pathlib import Path
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from chili_crane_msgs.msg import CameraTimingState
from chili_crane_hardware.camera_time import CameraTimeModel
from chili_crane_hardware.hik_mvs import MvsCamera


class HikCameraNode(Node):
    def __init__(self):
        super().__init__("hik_camera_evidence")
        self.declare_parameter("camera_config", "")
        self.cfg = yaml.safe_load(Path(self.get_parameter("camera_config").value).read_text())
        if self.cfg.get("driver_enabled") is not True or self.cfg.get("camera_control_authority") is not False:
            raise ValueError("camera disabled or unsafe authority")
        self.model = CameraTimeModel(self.cfg)
        self.camera = MvsCamera(self.cfg)
        try:
            self.camera.start()
        except Exception:
            self.camera.close()
            raise
        prefix = "camera/" + self.cfg["sensor_id"]
        self.image = self.create_publisher(Image, prefix + "/image_raw", qos_profile_sensor_data)
        self.info = self.create_publisher(CameraInfo, prefix + "/camera_info", qos_profile_sensor_data)
        self.timing = self.create_publisher(CameraTimingState, prefix + "/timing", 10)
        self.diagnostics = self.create_publisher(String, prefix + "/diagnostics", 10)
        self.ptp = "NOT_CONFIGURED"
        self.last_probe = self.last_publish = 0
        fps = self.cfg.get("maximum_publish_fps")
        if not isinstance(fps, (float, int)) or not 0 < fps <= 5:
            self.camera.close()
            raise ValueError("maximum_publish_fps must be 0..5 for this bench profile")
        self.period = 1 / fps
        self.timer = self.create_timer(.01, self.receive)

    def receive(self):
        now_mono = time.monotonic()
        try:
            if now_mono - self.last_probe > 1:
                value = self.camera.feature("GevIEEE1588Status", "enum")["current_value"]
                self.ptp = str(value) if value is not None else "NOT_CONFIGURED"
                self.last_probe = now_mono
            frame = self.camera.frame(10)
            if frame is None:
                return
            metadata, raw, encoding = frame
            receive = self.get_clock().now().nanoseconds / 1e9
            data = self.model.observe(metadata, receive, self.ptp, now_mono)
            data.update(width=metadata["width"], height=metadata["height"], pixel_format=encoding,
                        intrinsic_state=self.cfg["intrinsic_state"], shutter_type="ROLLING",
                        timing_model="FRAME_TIMESTAMP_PLUS_ROLLING_SHUTTER")
            if now_mono - self.last_publish < self.period:
                return
            self.last_publish = now_mono
            header = Image().header
            header.stamp = Time(nanoseconds=int(data["stamp"] * 1e9)).to_msg()
            header.frame_id = self.cfg["frame_id"]
            image = Image(header=header, height=metadata["height"], width=metadata["width"],
                          encoding=encoding, is_bigendian=0, step=metadata["width"], data=raw)
            self.image.publish(image)
            # ROS CameraInfo K[0]=0 explicitly means uncalibrated; never identity calibration.
            self.info.publish(CameraInfo(header=header, height=image.height, width=image.width))
            message = CameraTimingState()
            message.header = header
            message.sensor_id = self.cfg["sensor_id"]
            message.validity = getattr(message, data["validity"])
            message.reason = data["reason"]
            message.ros_receive_stamp = Time(nanoseconds=int(receive * 1e9)).to_msg()
            fields = ("timestamp_mode", "device_timestamp_raw", "host_timestamp_raw", "device_timestamp_sec",
                "host_timestamp_sec", "device_time_valid", "host_time_valid", "device_host_offset_sec",
                "host_receive_latency_sec", "frame_number", "frame_gaps", "lost_packets", "frame_rate_hz",
                "exposure_time_raw", "exposure_unit", "width", "height", "pixel_format",
                "timestamp_regression_count", "timestamp_jump_count", "ptp_status", "high_precision_time_valid",
                "camera_control_authority")
            for name in fields:
                if data[name] is not None:
                    setattr(message, name, data[name])
            message.evidence.source_type = message.evidence.SOURCE_PHYSICAL
            message.evidence.validity, message.evidence.reason = message.validity, message.reason
            message.evidence.measurement_stamp, message.evidence.receive_stamp = header.stamp, message.ros_receive_stamp
            message.evidence.source_counter = message.frame_number
            self.timing.publish(message)
            self.diagnostics.publish(String(data=json.dumps(data, allow_nan=False)))
        except (RuntimeError, ValueError, KeyError) as exc:
            self.diagnostics.publish(String(data=json.dumps({"state": "DEGRADED", "reason": str(exc),
                "camera_control_authority": False})))

    def destroy_node(self):
        self.camera.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = HikCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
