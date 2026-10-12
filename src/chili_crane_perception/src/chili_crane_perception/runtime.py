"""ROS2 shadow only: bounded common-frame input, typed evidence, NO control topics."""
import copy
import json
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud2
from chili_crane_msgs.msg import (GrabState, GrabGeometrySafetyState, GrabClearanceState,
                                 EvidenceMetadata, GrabIoState, SuspensionPrior as PriorMessage)
from chili_crane_slam.pointcloud import CANONICAL, validate_points
from chili_crane_slam.rotation import quaternion
from .geometry import positive, covariance, PitBoundaryModel
from .pipeline import GrabPerceptionPipeline, LatestFrameSlot
from .tracker import SuspensionPrior

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
SOURCE_NAMES = {1: "PHYSICAL", 2: "SIMULATED", 3: "REPLAY", 4: "SYNTHETIC"}


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def polygon(points):
    from geometry_msgs.msg import Polygon
    result = Polygon()
    result.points = [Point32(x=float(p[0]), y=float(p[1]), z=0.0) for p in points]
    return result


def configure(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if any(data.get(k) is not False for k in ("physical_output_enabled", "automatic_control_enabled", "camera_control_authority")):
        raise ValueError("SHADOW_CONTROL_FLAGS_MUST_BE_FALSE")
    return data


class GrabPerceptionNode(Node):
    def __init__(self):
        super().__init__("grab_perception_node")
        self.declare_parameter("config_file", "")
        self.config, self.pipeline, self.error = {}, None, "NOT_CONFIGURED"
        try:
            self.config = configure(self.get_parameter("config_file").value)
            if (self.config.get("config_state") != "VALID" or self.config.get("source_type") not in SOURCE_NAMES.values()
                    or self.config.get("common_frame") in (None, "", "NOT_CONFIGURED")
                    or self.config.get("extrinsic_calibration_id") in (None, "", "NOT_CONFIGURED")
                    or self.config.get("time_domain") not in ("HOST_UTC","ROS_TIME")
                    or self.config.get("source_type") == "PHYSICAL" and self.config.get("time_domain") != "HOST_UTC"
                    or self.config.get("source_type") == "PHYSICAL" and self.get_parameter("use_sim_time").value
                    or self.config.get("config_version") in (None, "", "NOT_CONFIGURED")):
                raise ValueError("SOURCE_FRAME_CALIBRATION_NOT_CONFIGURED")
            covariance(self.config["extrinsic_covariance"], 6)
            self.pipeline = GrabPerceptionPipeline(self.config)
        except (OSError, KeyError, ValueError, TypeError) as exc:
            self.error = str(exc)
        self.slot = LatestFrameSlot()
        self.meta, self.io, self.prior = None, None, None
        self.last_counter, self.last_arrival = 0, None
        self.inputs = ReentrantCallbackGroup()
        self.worker = MutuallyExclusiveCallbackGroup()
        self.cloud_sub = self.create_subscription(PointCloud2, "perception/grab_input",
            lambda m: self.slot.put((m, time.monotonic())), QOS, callback_group=self.inputs)
        self.meta_sub = self.create_subscription(EvidenceMetadata, "perception/grab_input_evidence",
            lambda m: setattr(self, "meta", (m, time.monotonic())), QOS, callback_group=self.inputs)
        self.io_sub = self.create_subscription(GrabIoState, "hardware/grab_io_state",
            lambda m: setattr(self, "io", (m, time.monotonic())), QOS, callback_group=self.inputs)
        self.prior_sub = self.create_subscription(PriorMessage, "perception/suspension_prior",
            lambda m: setattr(self, "prior", (m, time.monotonic())), QOS, callback_group=self.inputs)
        self.state_pub = self.create_publisher(GrabState, "perception/grab_state", QOS)
        self.geometry_pub = self.create_publisher(GrabGeometrySafetyState, "perception/grab_geometry_safety", QOS)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.05, self.process, clock=self.steady, callback_group=self.worker)

    def invalid(self, reason):
        if self.pipeline is not None:
            self.pipeline.tracker.missing(self.pipeline.tracker.stamp or 0, force_lost=True)
        state, geometry = GrabState(), GrabGeometrySafetyState()
        for message in (state, geometry):
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.config.get("common_frame", "NOT_CONFIGURED")
            message.validity = message.NOT_CONFIGURED
            message.reason = reason
            message.evidence_age_sec = 1e9
        state.tracking_state = GrabState.TRACK_LOST
        state.source = GrabState.SOURCE_NONE
        geometry.tracking_state = "LOST"
        geometry.valid_for_control = False
        geometry.dropped_frames = self.slot.dropped
        self.state_pub.publish(state)
        self.geometry_pub.publish(geometry)

    def process(self):
        if self.pipeline is None:
            self.invalid(self.error)
            return
        item = self.slot.take()
        ttl = self.pipeline.policy["evidence_timeout_sec"]
        if item is None:
            if self.last_arrival is None or time.monotonic() - self.last_arrival > ttl:
                self.invalid("INPUT_TIMEOUT")
            return
        cloud, arrival = item
        now = time.monotonic()
        try:
            meta_tuple, io_tuple, prior_tuple = self.meta, self.io, self.prior
            if meta_tuple is None or io_tuple is None:
                raise ValueError("EVIDENCE_OR_OPENING_MISSING")
            meta, meta_arrival = meta_tuple
            io, io_arrival = io_tuple
            frame_stamp = seconds(cloud.header.stamp)
            source_now = time.time() if self.config["time_domain"] == "HOST_UTC" else self.get_clock().now().nanoseconds/1e9
            if not 0 <= source_now-frame_stamp <= ttl:
                raise ValueError("SOURCE_TIME_DOMAIN_FRESHNESS_GATE")
            if seconds(meta.measurement_stamp) != frame_stamp:
                if seconds(meta.measurement_stamp) < frame_stamp and now-arrival <= ttl:
                    self.slot.restore_if_empty(item)
                return  # Two topics may arrive out of order; never pair different frames.
            if (cloud.header.frame_id != self.config["common_frame"] or meta.header.frame_id != cloud.header.frame_id
                    or meta.validity != meta.VALID or meta.source_counter <= self.last_counter
                    or meta.calibration_id != self.config["extrinsic_calibration_id"]
                    or meta.config_version != self.config["config_version"]
                    or SOURCE_NAMES.get(meta.source_type) != self.config["source_type"]
                    or seconds(meta.measurement_stamp) != frame_stamp
                    or not 0 <= meta.evidence_age_sec <= ttl
                    or max(now-arrival, now-meta_arrival, now-io_arrival, io.evidence_age_sec) > ttl
                    or io.validity != io.VALID or io.fault):
                raise ValueError("FRAME_SOURCE_FRESHNESS_OR_OPENING_GATE")
            expected_fields = dict(zip(CANONICAL.names, (0,4,8,12,16,18,24)))
            datatypes = dict(zip(CANONICAL.names,(7,7,7,7,4,2,8)))
            if (cloud.point_step != 32 or cloud.is_bigendian
                    or not 0 < cloud.width * cloud.height <= self.pipeline.policy["maximum_input_points"]
                    or len(cloud.data) != cloud.row_step * cloud.height or cloud.row_step < cloud.width*32
                    or len(cloud.fields) != len(expected_fields)
                    or any(not any(f.name == key and f.offset == offset and f.count == 1
                                   and f.datatype == datatypes[key] for f in cloud.fields)
                           for key, offset in expected_fields.items())):
                raise ValueError("CANONICAL_XYZIRT_REQUIRED")
            points = np.ndarray((cloud.height, cloud.width), dtype=CANONICAL, buffer=cloud.data,
                                strides=(cloud.row_step, 32)).reshape(-1).copy()
            validate_points(points)
            if not points["timestamp"].min() <= frame_stamp <= points["timestamp"].max():
                raise ValueError("MID_SCAN_DOMAIN_MISMATCH")
            opening = ("CONFLICT" if io.open_limit and io.closed_limit else
                       "OPEN" if io.open_limit else "CLOSED" if io.closed_limit else "TRANSITION")
            prior = None
            if prior_tuple is not None:
                p, received = prior_tuple
                if (p.validity == p.VALID and p.header.frame_id == cloud.header.frame_id
                        and p.evidence.source_type == meta.source_type and 0 <= p.evidence_age_sec <= ttl
                        and now-received <= ttl):
                    prior = SuspensionPrior(np.array([p.point.x, p.point.y, p.point.z]),
                        seconds(p.evidence.measurement_stamp), p.evidence.calibration_id)
            result = self.pipeline.process(points, frame_stamp, opening, self.config["source_type"],
                                           self.config["extrinsic_covariance"], prior)
            self.last_counter, self.last_arrival = meta.source_counter, arrival
            age = max(time.monotonic() - arrival,
                      time.monotonic() - meta_arrival + meta.evidence_age_sec,
                      (time.time() if self.config["time_domain"] == "HOST_UTC" else self.get_clock().now().nanoseconds/1e9)-frame_stamp)
            if age > ttl:
                raise ValueError("PROCESSING_DEADLINE_EXCEEDED")
            state, geometry = GrabState(), GrabGeometrySafetyState()
            for message in (state, geometry):
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = cloud.header.frame_id
                message.validity = message.DEGRADED
                message.reason = "SHADOW_ONLY_NOT_VALID_FOR_CONTROL"
                message.evidence = copy.deepcopy(meta)
                message.evidence_age_sec = age
                message.evidence.evidence_age_sec = age
                message.evidence.validity = message.evidence.DEGRADED
                message.evidence.reason = "SHADOW_ONLY"
            tracker = self.pipeline.tracker
            state.tracking_state = getattr(GrabState, "TRACK_" + result["tracking"])
            state.source = getattr(GrabState, "SOURCE_" + result["source"])
            state.opening_state = getattr(GrabState, "OPENING_" + opening)
            state.motion_stable = result["swing"].get("motion_stable", False)
            geometry.tracking_state, geometry.observation_source = result["tracking"], result["source"]
            geometry.opening_state, geometry.source_type = opening, self.config["source_type"]
            geometry.measurement_accepted = result["accepted"]
            geometry.observation_status = ("ACCEPTED" if result["accepted"] else "DEGRADED"
                if result["reasons"] and set(result["reasons"]) <= {"WEAK_OBSERVABILITY","INSUFFICIENT_SPATIAL_SUPPORT"}
                else "REJECTED")
            geometry.rejection_reasons = result["reasons"]
            geometry.stage_runtime_json = json.dumps(result["stage_seconds"],allow_nan=False)
            geometry.reason = "SHADOW_ONLY|" + ("MEASUREMENT_ACCEPTED" if result["accepted"] else "|".join(result["reasons"]))
            geometry.valid_for_control = False
            geometry.total_runtime_sec = result["total_seconds"]
            geometry.dropped_frames = self.slot.dropped
            geometry.frame_span_sec = float(np.ptp(points["timestamp"]))
            geometry.motion_smear_translation_margin_m = result.get("motion_smear_translation_margin_m",0.)
            geometry.motion_smear_rotation_margin_rad = result.get("motion_smear_rotation_margin_rad",0.)
            env = result["envelope"]
            if tracker.pose is not None:
                state.pose.pose.position.x, state.pose.pose.position.y, state.pose.pose.position.z = map(float, tracker.pose[:3,3])
                q = quaternion(tracker.pose[:3,:3])
                state.pose.pose.orientation.x, state.pose.pose.orientation.y, state.pose.pose.orientation.z, state.pose.pose.orientation.w = q
                # ROS orientation covariance is fixed-axis/world; internal is right/body.
                rotate = np.eye(6)
                rotate[3:,3:] = tracker.pose[:3,:3]
                state.pose.covariance = (rotate @ tracker.covariance[:6,:6] @ rotate.T).reshape(-1).tolist()
                for field, values in (("linear_velocity", tracker.velocity), ("angular_velocity", tracker.pose[:3,:3] @ tracker.omega)):
                    getattr(geometry, field).x, getattr(geometry, field).y, getattr(geometry, field).z = map(float, values)
            if env is not None:
                state.bottom_z_m, state.top_z_m = env.bottom_nominal, env.top_upper_bound
                state.footprint_map = polygon(env.nominal_footprint)
                geometry.nominal_footprint, geometry.conservative_footprint = polygon(env.nominal_footprint), polygon(env.conservative_footprint)
                for field, value in (("bottom_nominal_m",env.bottom_nominal),("bottom_lower_bound_m",env.bottom_lower_bound),
                    ("bottom_upper_bound_m",env.bottom_upper_bound),("top_upper_bound_m",env.top_upper_bound),
                    ("pose_uncertainty_margin_m",env.pose_margin),("geometry_uncertainty_margin_m",env.geometry_margin),
                    ("extrinsic_uncertainty_margin_m",env.extrinsic_margin),("motion_smear_margin_m",env.smear_margin)):
                    setattr(geometry, field, float(value))
                geometry.swing_valid = result["swing"]["valid"]
                for field, value in result["swing"].items():
                    if field != "valid":
                        setattr(geometry, field, value)
            observed = result["observation"]
            if observed:
                geometry.sensor_support = list(observed.sensors)
                geometry.residual_p50_m, geometry.residual_p95_m = observed.p50, observed.p95
                geometry.inlier_ratio = observed.inlier_ratio
                geometry.information_eigenvalues = observed.eigenvalues.tolist()
                state.confidence = geometry.confidence = observed.inlier_ratio
            self.state_pub.publish(state)
            self.geometry_pub.publish(geometry)
        except (ValueError, KeyError, TypeError, RuntimeError, np.linalg.LinAlgError) as exc:
            self.invalid(str(exc))


class GrabClearanceNode(Node):
    def __init__(self):
        super().__init__("grab_clearance_node")
        self.declare_parameter("config_file", "")
        self.config, self.pit, self.error = {}, None, "PIT_NOT_CONFIGURED"
        try:
            self.config = configure(self.get_parameter("config_file").value)
            semantic = yaml.safe_load(Path(self.config["semantic_map_file"]).read_text(encoding="utf-8"))
            if semantic.get("config_state") != "VALID":
                raise ValueError("SEMANTIC_MAP_NOT_CONFIGURED")
            pit = next(p for p in semantic["pits"] if p["pit_id"] == self.config["pit_id"])
            self.pit = PitBoundaryModel(dict(pit, config_state=semantic["config_state"]), semantic["frame_id"])
        except (ValueError, KeyError, OSError, TypeError, StopIteration) as exc:
            self.error = str(exc)
        self.latest = None
        self.sub = self.create_subscription(GrabGeometrySafetyState, "perception/grab_geometry_safety",
            lambda m: setattr(self, "latest", (m, time.monotonic())), QOS)
        self.pub = self.create_publisher(GrabClearanceState, "perception/grab_clearance", QOS)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.05, self.publish, clock=self.steady)

    def publish(self):
        result = GrabClearanceState()
        result.header.stamp = self.get_clock().now().to_msg()
        result.validity, result.reason = result.NOT_CONFIGURED, self.error
        result.valid_for_control = False
        try:
            if self.pit is None or self.latest is None:
                raise ValueError(self.error)
            message, received = self.latest
            ttl = positive(self.config["policy"]["evidence_timeout_sec"], "evidence_timeout_sec")
            if (message.header.frame_id != self.pit.frame_id or message.tracking_state != "TRACKED"
                    or message.observation_source not in ("MEASURED","FILTERED") or not message.measurement_accepted
                    or message.validity not in (message.VALID,message.DEGRADED)
                    or message.evidence.source_type not in SOURCE_NAMES
                    or message.evidence.calibration_id in ("","NOT_CONFIGURED")
                    or message.evidence.config_version in ("","NOT_CONFIGURED")
                    or not 0 <= message.evidence_age_sec + time.monotonic() - received <= ttl):
                raise ValueError("STALE_OR_PREDICTED_OR_FRAME_MISMATCH")
            from .geometry import GrabEnvelope
            env = GrabEnvelope(np.array([[p.x,p.y] for p in message.nominal_footprint.points]),
                np.array([[p.x,p.y] for p in message.conservative_footprint.points]),
                message.bottom_nominal_m,message.bottom_lower_bound_m,message.bottom_upper_bound_m,
                message.top_upper_bound_m,message.pose_uncertainty_margin_m,
                message.geometry_uncertainty_margin_m,message.extrinsic_uncertainty_margin_m,message.motion_smear_margin_m)
            for field, value in self.pit.clearance(env).items():
                setattr(result, field, value)
            result.header.frame_id = self.pit.frame_id
            result.validity, result.reason = result.DEGRADED, "SHADOW_GEOMETRY_ONLY"
            result.grab_bottom_nominal_m, result.grab_bottom_lower_bound_m = env.bottom_nominal, env.bottom_lower_bound
            result.grab_top_upper_bound_m = env.top_upper_bound
            result.pose_uncertainty_margin_m, result.geometry_uncertainty_margin_m = env.pose_margin, env.geometry_margin
            result.motion_smear_margin_m = env.smear_margin
            result.confidence = message.confidence
            result.evidence = copy.deepcopy(message.evidence)
            result.evidence_age_sec = message.evidence_age_sec + time.monotonic() - received
        except (ValueError, TypeError, KeyError) as exc:
            result.reason = str(exc)
            result.wall_collision_risk = True
            result.evidence_age_sec = 1e9
        self.pub.publish(result)


def run(kind):
    rclpy.init()
    node = kind()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def perception_main():
    run(GrabPerceptionNode)


def clearance_main():
    run(GrabClearanceNode)
