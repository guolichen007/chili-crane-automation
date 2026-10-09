"""Hardware-neutral Phase 0.5 contracts. No policy in this file grants motion."""
import math
from dataclasses import dataclass
from enum import IntEnum, Enum


class SystemMode(IntEnum):
    BOOT = 0
    SELF_CHECK = 1
    SAFE_IDLE = 2
    AUTO_PENDING = 3
    AUTO_READY = 4
    AUTO_ACTIVE = 5
    REMOTE = 6
    PROTECTIVE_STOP = 7
    FAULT_LATCHED = 8
    EMERGENCY_STOP = 9
    MAINTENANCE = 10


class DriveProfile(str, Enum):
    NOT_CONFIGURED = "NOT_CONFIGURED"
    VARIABLE_SPEED = "VARIABLE_SPEED"
    FIXED_SLOW = "FIXED_SLOW"


@dataclass(frozen=True)
class Evidence:
    measurement_stamp: float = 0.0
    receive_stamp: float = 0.0
    source_counter: int = 0
    calibration_id: str = "NOT_CONFIGURED"
    config_version: str = "NOT_CONFIGURED"

    def fresh(self, now, max_age):
        numbers = (now, max_age, self.measurement_stamp, self.receive_stamp)
        return (all(type(v) in (int, float) and math.isfinite(v) for v in numbers)
                and max_age > 0 and 0 < self.measurement_stamp <= self.receive_stamp <= now
                and now - self.measurement_stamp <= max_age
                and type(self.source_counter) is int and self.source_counter > 0
                and isinstance(self.calibration_id, str)
                and self.calibration_id not in ("", "NOT_CONFIGURED")
                and isinstance(self.config_version, str)
                and self.config_version not in ("", "NOT_CONFIGURED"))


@dataclass(frozen=True)
class AxisCapability:
    profile: DriveProfile = DriveProfile.NOT_CONFIGURED
    verified: bool = False
    speed_command_supported: bool = False
    stop_distance_verified: bool = False
    reaction_time_verified: bool = False
    fixed_speed_verified_safe: bool = False
    pulse_jog_allowed: bool = False
    minimum_on_time: float | None = None
    minimum_off_time: float | None = None

    def ready(self):
        if self.verified is not True:
            return False
        if self.profile == DriveProfile.VARIABLE_SPEED:
            return self.speed_command_supported is True
        if self.profile == DriveProfile.FIXED_SLOW:
            return all(v is True for v in (
                self.stop_distance_verified, self.reaction_time_verified, self.fixed_speed_verified_safe))
        return False

    def pulse_jog_ready(self):
        return (self.ready() and self.pulse_jog_allowed is True
                and all(type(v) in (int, float) and math.isfinite(v) and v > 0
                        for v in (self.minimum_on_time, self.minimum_off_time)))


@dataclass(frozen=True)
class StopDistance:
    sensor_latency: float | None = None
    processing_latency: float | None = None
    communication_latency: float | None = None
    output_release_latency: float | None = None
    mechanical_coast_distance: float | None = None
    safety_margin: float | None = None

    def distance(self, speed):
        values = tuple(self.__dict__.values())
        if (type(speed) not in (int, float) or not math.isfinite(speed)
                or any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in values)):
            return None
        return abs(speed) * sum(values[:4]) + values[4] + values[5]


@dataclass(frozen=True)
class LidarReady:
    online: bool = False
    fresh: bool = False
    frame_valid: bool = False
    calibration_valid: bool = False

    def ready(self):
        return all(v is True for v in self.__dict__.values())


def dual_lidar_ready(a, b, pairing_valid=False, extrinsic_valid=False, coverage_valid=False):
    return a.ready() and b.ready() and all(
        v is True for v in (pairing_valid, extrinsic_valid, coverage_valid))


@dataclass(frozen=True)
class SystemReadiness:
    configuration_ready: bool = False
    hardware_ready: bool = False
    dual_lidar_ready: bool = False
    localization_ready: bool = False
    grab_tracking_ready: bool = False
    pit_perception_ready: bool = False
    control_ready: bool = False
    safety_ready: bool = False
    x_auto_ready: bool = False
    y_auto_ready: bool = False
    z_raise_ready: bool = False
    z_lower_ready: bool = False
    grab_ready: bool = False

    @property
    def system_ready(self):
        return all(v is True for v in self.__dict__.values())


def evaluate_readiness(configuration_ready=False, hardware_ready=False, dual_lidar=False,
                       localization_ready=False, grab_tracking_ready=False, pit_perception_ready=False,
                       control_ready=False, safety_ready=False, x_verified=False, grab_verified=False,
                       y=AxisCapability(), z=AxisCapability(), grab_bottom_valid=False,
                       target_valid=False, wall_clearance_valid=False,
                       raise_clearance_valid=False, jam_free_verified=False):
    base = all(v is True for v in (configuration_ready, hardware_ready, control_ready, safety_ready))
    y_ready = base and localization_ready is True and y.ready()
    z_base = (base and grab_tracking_ready is True and z.ready()
              and z.stop_distance_verified is True and jam_free_verified is True)
    # No blind raise. Tracking and a distinct upward-clearance/jam assessment are required.
    z_raise = z_base and raise_clearance_valid is True
    z_lower = z_base and all(v is True for v in (
        dual_lidar, grab_bottom_valid, pit_perception_ready, target_valid, wall_clearance_valid))
    return SystemReadiness(
        configuration_ready is True, hardware_ready is True, dual_lidar is True,
        localization_ready is True, grab_tracking_ready is True, pit_perception_ready is True,
        control_ready is True, safety_ready is True,
        base and localization_ready is True and x_verified is True, y_ready,
        z_raise, z_lower, base and grab_verified is True)


def grab_feedback_ready(tracking_state, source, fresh):
    # Prediction is only evidence for hold/stop; never permission to continue lowering.
    return fresh is True and tracking_state == "TRACKED" and source in {"MEASURED", "FILTERED"}


class Severity(IntEnum):
    INFO = 0
    WARNING = 1
    ERROR = 2
    CRITICAL = 3


class EventDomain(IntEnum):
    CYCLE_FAILURE = 0
    PROTECTIVE_STOP = 1
    RECOVERABLE_FAULT = 2
    LATCHED_FAULT = 3
    EMERGENCY_STOP = 4


@dataclass(frozen=True)
class SafetyEvent:
    severity: Severity
    domain: EventDomain
    code: str
    source: str
    reason: str
    active: bool = True
    latched: bool = False
    recoverable: bool = False
    run_id: str = ""
    task_id: str = ""
    cycle_id: str = ""
    command_id: str = ""
    first_seen: float = 0.0
    last_seen: float = 0.0


@dataclass(frozen=True)
class UnloadSafeZone:
    x_min: float | None = None
    x_max: float | None = None
    y_min: float | None = None
    y_max: float | None = None
    z_min: float | None = None
    z_max: float | None = None
    frame_id: str = "NOT_CONFIGURED"

    def contains(self, envelope, flow_projection_verified=False):
        if self.frame_id == "NOT_CONFIGURED" or flow_projection_verified is not True:
            return False
        bounds = tuple(self.__dict__.values())[:6]
        if len(envelope) != 6 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in bounds + tuple(envelope)):
            return False
        return all(low <= high and low <= e_low <= e_high <= high
                   for low, high, e_low, e_high in zip(
                       bounds[::2], bounds[1::2], envelope[::2], envelope[1::2]))

def direction_permitted(negative_limit, positive_limit, direction, safety_ok=False):
    if safety_ok is not True or type(direction) is not int or direction not in (-1, 1):
        return False
    if type(negative_limit) is not bool or type(positive_limit) is not bool:
        return False
    if negative_limit and positive_limit:
        return False
    return not (negative_limit if direction < 0 else positive_limit)
