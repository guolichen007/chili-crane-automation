"""Fault scenarios use synthetic calibration only, never production values."""
import math
import unittest
from dataclasses import replace
from chili_crane_control.contracts import (
    SystemMode, Evidence, AxisCapability, DriveProfile, StopDistance, LidarReady,
    dual_lidar_ready, evaluate_readiness, grab_feedback_ready, EventDomain, UnloadSafeZone,
    direction_permitted,
)
from chili_crane_control.execution import (
    ControlAuthority, AuthorizedAction, PermitEvidence, ActuationRequest,
    VariableSpeedStrategy, FixedSlowStrategy, TrolleyExecutor, HoistExecutor,
    BridgeServoExecutor, GrabExecutor,
)
from chili_crane_control.cycle import TaskCycles, CycleState
from chili_crane_control.configuration import configuration_hash, capability_from_config, manifest_ready
from chili_crane_hardware.actuation import request_to_do
from chili_crane_hardware.bridge_servo import UnconfiguredBridgeServo

VARIABLE = AxisCapability(DriveProfile.VARIABLE_SPEED, True, True, True, True)
FIXED = AxisCapability(DriveProfile.FIXED_SLOW, True, False, True, True, True)
STOP = StopDistance(0.02, 0.03, 0.02, 0.01, 0.05, 0.02)


def active_authority():
    authority = ControlAuthority("synthetic-session")
    authority.transition(SystemMode.AUTO_READY, stopped=True, outputs_off=True,
                         safety_fresh=True, perception_ready=True, localization_ready=True)
    assert authority.activate_new_task(True)
    return authority


def command(authority):
    return AuthorizedAction("cmd", "intent", "permit", 1, authority.session_id,
                            authority.command_epoch, 1, 10, 20, True, "Y", 1, 4.0)


def permit(action):
    return PermitEvidence(action.permit_id, action.permit_generation, action.intent_id,
                          action.session_id, action.command_epoch, 10, 20, True, True)


def readiness(**overrides):
    values = dict(configuration_ready=True, hardware_ready=True, dual_lidar=True,
                  localization_ready=True, grab_tracking_ready=True, pit_perception_ready=True,
                  control_ready=True, safety_ready=True, x_verified=True, grab_verified=True,
                  y=FIXED, z=FIXED)
    values.update(overrides)
    return evaluate_readiness(**values)


class ArchitectureScenarios(unittest.TestCase):
    def test_permit_expiry(self):
        authority = active_authority()
        action = command(authority)
        self.assertFalse(authority.admit(action, 21, permit(action), True)[0])

    def test_auto_to_remote(self):
        authority = active_authority()
        previous = authority.command_epoch
        action = command(authority)
        authority.transition(SystemMode.REMOTE)
        self.assertGreater(authority.command_epoch, previous)
        self.assertTrue(authority.cycle_aborted)
        self.assertFalse(authority.admit(action, 11, permit(action), True)[0])

    def test_remote_to_auto(self):
        authority = active_authority()
        authority.transition(SystemMode.REMOTE)
        self.assertEqual(SystemMode.AUTO_PENDING, authority.transition(SystemMode.AUTO_READY))
        self.assertFalse(authority.activate_new_task(True))
        authority.transition(SystemMode.AUTO_READY, stopped=True, outputs_off=True,
                             safety_fresh=True, perception_ready=True, localization_ready=True)
        self.assertFalse(authority.activate_new_task(False))
        self.assertTrue(authority.activate_new_task(True))

    def test_old_command_after_epoch_change(self):
        authority = active_authority()
        action = command(authority)
        authority.transition(SystemMode.REMOTE)
        authority.transition(SystemMode.AUTO_READY, stopped=True, outputs_off=True,
                             safety_fresh=True, perception_ready=True, localization_ready=True)
        authority.activate_new_task(True)
        self.assertFalse(authority.admit(action, 11, permit(action), True)[0])

    def test_duplicate_command(self):
        authority = active_authority()
        action = command(authority)
        self.assertTrue(authority.admit(action, 11, permit(action), True)[0])
        self.assertFalse(authority.admit(action, 11, permit(action), True)[0])

    def test_node_restart(self):
        authority = active_authority()
        action = command(authority)
        restarted = active_authority()
        restarted.session_id = "restarted-session"
        self.assertFalse(restarted.admit(action, 11, permit(action), True)[0])

    def test_y_position_stale(self):
        result = FixedSlowStrategy(FIXED, STOP, 0.05, 0.2).step(
            1, 4, speed=0.2, now=11, feedback_fresh=False, permitted=True)
        self.assertFalse(result.enable)

    def test_dual_lidar_stale(self):
        ready = LidarReady(True, True, True, True)
        self.assertFalse(dual_lidar_ready(ready, replace(ready, fresh=False), True, True, True))

    def test_single_lidar_loss(self):
        self.assertFalse(dual_lidar_ready(LidarReady(True, True, True, True),
                                         LidarReady(), True, True, True))

    def test_grab_tracking_loss(self):
        for state, source in (("LOST", "MEASURED"), ("TRACKED", "PREDICTED"),
                              ("LOST_HOLD", "PREDICTED")):
            self.assertFalse(grab_feedback_ready(state, source, True))
        self.assertTrue(grab_feedback_ready("TRACKED", "FILTERED", True))

    def test_z_fixed_speed_not_verified(self):
        result = readiness(z=replace(FIXED, stop_distance_verified=False))
        self.assertTrue(result.y_auto_ready)
        self.assertFalse(result.z_lower_ready)
        self.assertFalse(result.system_ready)

    def test_limit_conflict(self):
        for direction in (-1, 1):
            self.assertFalse(direction_permitted(True, True, direction, True))
        self.assertFalse(readiness(hardware_ready=False).z_raise_ready)

    def test_safety_ok_loss(self):
        self.assertFalse(readiness(safety_ready=False).system_ready)

    def test_cycle_empty_grab_retry(self):
        task = TaskCycles("run", "task")
        task.accept_scan(1, True)
        old_cycle = task.cycle_id
        task.fail_cycle("EMPTY_GRAB", retry_authorized=True)
        self.assertEqual(EventDomain.CYCLE_FAILURE, task.events[-1].domain)
        self.assertFalse(task.events[-1].latched)
        self.assertNotEqual(old_cycle, task.cycle_id)
        self.assertFalse(task.accept_scan(1, True))
        self.assertTrue(task.accept_scan(2, True))

    def test_protective_stop(self):
        authority = active_authority()
        action = command(authority)
        authority.transition(SystemMode.PROTECTIVE_STOP)
        self.assertFalse(authority.admit(action, 11, permit(action), True)[0])

    def test_stop_dominance(self):
        for mode in SystemMode:
            authority = ControlAuthority()
            authority.mode = mode
            for executor in (TrolleyExecutor, HoistExecutor, GrabExecutor, BridgeServoExecutor):
                result = executor(authority).execute(AuthorizedAction(direction=0), 99)
                self.assertFalse(result.enable)
                self.assertEqual("STOP_DOMINANCE", result.reason)
                self.assertFalse(any(request_to_do(result)))

    def test_permit_binding(self):
        action = command(active_authority())
        for bad in (replace(permit(action), permit_id="other"),
                    replace(permit(action), permit_generation=2),
                    replace(permit(action), permit_generation=True),
                    replace(permit(action), evaluated_intent_id="other"),
                    replace(permit(action), direction_allowed=False)):
            self.assertFalse(bad.authorizes(action, 11))

    def test_variable_speed_profile(self):
        strategy = VariableSpeedStrategy(VARIABLE, ((0.05, 0.02, "JOG"),
                                                   (0.5, 0.1, "SLOW"), (2, 0.2, "FAST")))
        for position, phase in ((0, "FAST"), (3, "SLOW"), (3.8, "JOG"), (4, "STOP")):
            self.assertEqual(phase, strategy.step(position, 4, feedback_fresh=True, permitted=True).phase)
        self.assertFalse(VariableSpeedStrategy(VARIABLE).step(
            0, 4, feedback_fresh=True, permitted=True).enable)

    def test_fixed_slow_early_stop_and_settle(self):
        strategy = FixedSlowStrategy(FIXED, STOP, 0.05, 0.2)
        moving = strategy.step(1, 4, speed=0.2, now=10, feedback_fresh=True, permitted=True)
        self.assertTrue(moving.enable)
        self.assertFalse(moving.speed_command_valid)
        stopped = strategy.step(3.95, 4, speed=0.2, now=11, feedback_fresh=True, permitted=True)
        self.assertFalse(stopped.enable)
        self.assertEqual("SETTLING", stopped.phase)
        settled = strategy.step(3.8, 4, speed=0, now=12, feedback_fresh=True, permitted=True)
        self.assertEqual("REPLAN_REQUIRED", settled.phase)

    def test_stop_model_and_pulse_jog_unknown(self):
        self.assertIsNone(StopDistance().distance(0.2))
        self.assertIsNone(STOP.distance(math.nan))
        self.assertAlmostEqual(0.086, STOP.distance(0.2))
        self.assertFalse(FIXED.pulse_jog_ready())
        self.assertFalse(replace(FIXED, pulse_jog_allowed=True).pulse_jog_ready())

    def test_independent_axis_profiles(self):
        config = {"y": {"drive_profile": "FIXED_SLOW"}, "z": {"drive_profile": "VARIABLE_SPEED"}}
        self.assertNotEqual(capability_from_config(config, "Y").profile,
                            capability_from_config(config, "Z").profile)
        self.assertFalse(capability_from_config(config, "Z").ready())

    def test_evidence_heartbeat_not_measurement(self):
        evidence = Evidence(10, 10.1, 1, "cal", "cfg")
        self.assertTrue(evidence.fresh(10.2, 1))
        self.assertFalse(replace(evidence, receive_stamp=12).fresh(12, 1))
        self.assertFalse(replace(evidence, calibration_id=None).fresh(10.2, 1))

    def test_repeated_cycle_rescan_and_safe_return(self):
        task = TaskCycles("run", "task")
        task.accept_scan(1, True)
        while task.state != CycleState.COMPLETE:
            self.assertTrue(task.advance(True))
        task.next_cycle(True)
        self.assertTrue(task.needs_new_scan)
        self.assertFalse(task.advance(True))
        self.assertFalse(task.accept_scan(1, True))
        task.accept_scan(2, True)
        while task.state != CycleState.COMPLETE:
            self.assertTrue(task.advance(True))
        task.next_cycle(False)
        self.assertFalse(task.returned_safe(False))
        self.assertTrue(task.returned_safe(True))
        self.assertFalse(task.next_cycle(True))

    def test_unload_zone_and_do_mapping(self):
        zone = UnloadSafeZone(0, 5, 0, 5, 1, 3, "map")
        self.assertFalse(zone.contains((1, 2, 1, 2, 1.5, 2)))
        self.assertTrue(zone.contains((1, 2, 1, 2, 1.5, 2), True))
        self.assertFalse(zone.contains((1, 6, 1, 2, 1.5, 2), True))
        expected = {("Y", -1): 0, ("Y", 1): 1, ("Z", 1): 2,
                    ("Z", -1): 3, ("G", 1): 4, ("G", -1): 5}
        for (axis, direction), channel in expected.items():
            outputs = request_to_do(ActuationRequest(axis, direction, True))
            self.assertEqual([channel], [i for i, on in enumerate(outputs) if on])
        with self.assertRaises(ValueError):
            request_to_do(ActuationRequest("Y", 1, True, True, 0.2))

    def test_provenance_and_unknown_servo(self):
        self.assertEqual(configuration_hash({"a": 1, "b": 2}), configuration_hash({"b": 2, "a": 1}))
        self.assertFalse(manifest_ready({}))
        servo = UnconfiguredBridgeServo()
        self.assertFalse(servo.ready())
        self.assertFalse(servo.stop()["physical_stop_verified"])
        with self.assertRaises(RuntimeError):
            servo.move_to(1, "cmd")

    def test_emergency_latch_requires_explicit_reset(self):
        authority = ControlAuthority()
        authority.transition(SystemMode.EMERGENCY_STOP)
        self.assertEqual(SystemMode.EMERGENCY_STOP, authority.transition(SystemMode.REMOTE))
        authority.transition(SystemMode.SAFE_IDLE, reset_authorized=True,
                             stopped=True, outputs_off=True, safety_fresh=True)
        self.assertEqual(SystemMode.SAFE_IDLE, authority.mode)

    def test_executor_never_reverses_authorized_direction(self):
        authority = active_authority()
        action = replace(command(authority), target=0)
        executor = TrolleyExecutor(authority, VariableSpeedStrategy(VARIABLE, ((0.05, 0.02, "JOG"),)))
        result = executor.execute(action, 11, readiness=readiness(), permit=permit(action),
                                  position=1, feedback_fresh=True)
        self.assertFalse(result.enable)
        self.assertEqual("AUTHORIZED_DIRECTION_MISMATCH", result.reason)
        action = command(authority)
        short_permit = replace(permit(action), expire_stamp=12)
        authority.last_sequence = 0
        result = executor.execute(action, 11, readiness=readiness(), permit=short_permit,
                                  position=1, feedback_fresh=True)
        self.assertTrue(result.enable)
        self.assertEqual(12, result.expire_stamp)
