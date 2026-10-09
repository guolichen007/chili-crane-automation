"""24DI semantic mapping, including remote-control observations, never output."""
import math
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from chili_crane_control.qos import state_qos
from chili_crane_control.mode_policy import evaluate_mode
from chili_crane_hardware.mapping import RawInput, normalize, derive_logical_inputs
from chili_crane_msgs.msg import (
    DigitalInputState, ControlBoardState, RemoteControlState, GrabIoState, HoistState,
)


class IoNormalizerNode(Node):
    def __init__(self):
        super().__init__("io_normalizer")
        self.declare_parameter("mapping_config", "")
        path = self.get_parameter("mapping_config").value
        config = yaml.safe_load(Path(path).read_text(encoding="utf-8")) if path else {}
        self._mapping = derive_logical_inputs(config)
        self._stale = float(config.get("stale_timeout_sec", 1.0))
        if not math.isfinite(self._stale) or not 0 < self._stale <= 10:
            raise ValueError("invalid stale_timeout_sec")
        self._samples = {}
        self._subscriptions_by_device = [
            self.create_subscription(
                DigitalInputState, "hardware/" + device + "/raw_di",
                lambda msg, device=device: self._receive(device, msg), state_qos())
            for device in ("adam6052", "adam6251")
        ]
        self._types = {
            "control_board_state": ControlBoardState, "remote_control_state": RemoteControlState,
            "grab_io_state": GrabIoState, "hoist_state": HoistState,
        }
        self._publishers_by_topic = {
            topic: self.create_publisher(kind, "hardware/" + topic, state_qos())
            for topic, kind in self._types.items()
        }
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(0.1, self._publish, clock=self._steady_clock)

    def _receive(self, device, message):
        if message.device_id != device:
            self._samples.pop(device, None)
            return
        self._samples[device] = RawInput(
            tuple(message.raw_di), time.monotonic(), message.evidence_age_sec,
            message.validity == DigitalInputState.VALID and message.communication_ok)

    def _publish(self):
        now = time.monotonic()
        values = normalize(self._mapping, self._samples, now, self._stale)

        def get(name):
            return values.get(name)

        def known(*names):
            return all(get(name) is not None for name in names)

        def value(name):
            return get(name) is True

        def base(kind, required):
            message = kind()
            message.header.stamp = self.get_clock().now().to_msg()
            configured = known(*required)
            message.validity = kind.VALID if configured else kind.NOT_CONFIGURED
            message.reason = "mapped_input_evidence" if configured else "mapping_missing_or_input_stale"
            referenced = [self._mapping.get(name, {}).get("device") for name in required]
            ages = [
                self._samples[device].evidence_age_sec + now - self._samples[device].received_monotonic
                for device in referenced if device in self._samples
            ]
            message.evidence_age_sec = max(ages) if configured and ages else 1.0e9
            return message

        board = base(ControlBoardState, ("mode_auto", "safety_ok", "power_ok", "e_stop", "overload"))
        board.safety_ok_known = known("safety_ok")
        board.safety_ok = value("safety_ok")
        board.e_stop_known = known("e_stop")
        board.e_stop_active = value("e_stop")
        board.power_ok = value("power_ok")
        board.auto_mode = value("mode_auto")
        board.manual_mode = get("mode_auto") is False
        board.io_heartbeat_ok = known("mode_auto", "safety_ok")
        board.fault = value("overload")
        self._publishers_by_topic["control_board_state"].publish(board)

        remote_names = (
            "x_positive", "x_negative", "y_left", "y_right", "z_up", "z_down",
            "g_open", "g_close", "stop",
        )
        required = ("remote_receiver_ready", "mode_auto", "safety_ok") + tuple(
            "remote_" + name for name in remote_names)
        remote = base(RemoteControlState, required)
        remote.receiver_ready = value("remote_receiver_ready")
        remote.mode_auto = value("mode_auto")
        remote.safety_ok = value("safety_ok")
        remote.e_stop_known = known("e_stop")
        remote.e_stop_active = value("e_stop")
        for name in remote_names:
            setattr(remote, "stop_requested" if name == "stop" else name, value("remote_" + name))
        remote.conflict = any(
            value("remote_" + left) and value("remote_" + right)
            for left, right in (("x_positive", "x_negative"), ("y_left", "y_right"),
                                ("z_up", "z_down"), ("g_open", "g_close")))
        active = any(value("remote_" + name) for name in remote_names)
        decision = evaluate_mode(
            get("mode_auto"), get("safety_ok"),
            active if known(*required) and remote.receiver_ready else None,
            known(*required), stopped_verified=False, outputs_off_verified=False)
        remote.control_mode = decision.mode
        remote.release_automatic_outputs = decision.release_automatic_outputs
        remote.discard_pending_commands = decision.discard_pending_commands
        if remote.conflict:
            remote.validity = RemoteControlState.DEGRADED
            remote.reason = "opposing_remote_inputs"
        self._publishers_by_topic["remote_control_state"].publish(remote)

        grab = base(GrabIoState, ("g_open_limit", "g_close_limit", "g_fault", "mode_auto"))
        if grab.validity == GrabIoState.VALID:
            grab.validity = GrabIoState.DEGRADED
            grab.reason = "limits_available_but_running_feedback_not_configured"
        grab.open_limit = value("g_open_limit")
        grab.closed_limit = value("g_close_limit")
        grab.fault = value("g_fault")
        grab.auto_mode = value("mode_auto")
        grab.manual_mode = get("mode_auto") is False
        if grab.open_limit and grab.closed_limit:
            grab.validity = GrabIoState.DEGRADED
            grab.reason = "grab_limit_conflict"
        self._publishers_by_topic["grab_io_state"].publish(grab)

        hoist = base(HoistState, ("z_top_limit", "z_bottom_limit", "z_fault", "z_brake"))
        if hoist.validity == HoistState.VALID:
            hoist.validity = HoistState.DEGRADED
            hoist.reason = "limits_available_but_motion_feedback_not_configured"
        hoist.upper_limit = value("z_top_limit")
        hoist.lower_limit = value("z_bottom_limit")
        hoist.fault = value("z_fault")
        hoist.brake_engaged = value("z_brake")
        if hoist.upper_limit and hoist.lower_limit:
            hoist.validity = HoistState.DEGRADED
            hoist.reason = "hoist_limit_conflict"
        hoist.motion = HoistState.MOTION_UNKNOWN
        self._publishers_by_topic["hoist_state"].publish(hoist)


def main():
    rclpy.init()
    node = IoNormalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
