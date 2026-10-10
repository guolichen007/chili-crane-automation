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
from chili_crane_hardware.lease import ActuationLeaseGuard
from chili_crane_hardware.mapping import derive_logical_inputs, RawInput, normalize
from chili_crane_control.execution import direction_permission, PERMISSION_FIELDS

VARIABLE = AxisCapability(DriveProfile.VARIABLE_SPEED, True, True, True, True)
FIXED = AxisCapability(DriveProfile.FIXED_SLOW, True, False, True, True, True)
STOP = StopDistance(0.02, 0.03, 0.02, 0.01, 0.05, 0.02)


def active_authority():
    authority = ControlAuthority("synthetic-session")
    ready_authority(authority)
    assert authority.activate_new_task(True)
    return authority


def ready_authority(authority):
    if authority.mode == SystemMode.BOOT:
        authority.transition(SystemMode.SELF_CHECK)
    authority.transition(SystemMode.SAFE_IDLE, stopped=True, outputs_off=True,
                         safety_fresh=True, self_check_passed=True)
    authority.transition(SystemMode.AUTO_PENDING)
    authority.transition(SystemMode.AUTO_READY, stopped=True, outputs_off=True,
                         safety_fresh=True, perception_ready=True, localization_ready=True)


def command(authority):
    return AuthorizedAction("cmd", "intent", "permit", 1, authority.session_id,
                            authority.command_epoch, 1, 10, 20, True, "Y", 1, 4.0, action=3)


def permit(action):
    return PermitEvidence(action.permit_id, action.permit_generation, action.intent_id,
                          action.session_id, action.command_epoch, 10, 20, True,
                          allow_x_positive=True, allow_x_negative=True,
                          allow_y_positive=True, allow_y_negative=True,
                          allow_raise=True, allow_lower=True,
                          allow_grab_open=True, allow_grab_close=True, allow_auto_task=True)


def readiness(**overrides):
    values = dict(configuration_ready=True, hardware_ready=True, dual_lidar=True,
                  localization_ready=True, grab_tracking_ready=True, pit_perception_ready=True,
                  control_ready=True, safety_ready=True, x_verified=True, grab_verified=True,
                  y=FIXED, z=FIXED, grab_bottom_valid=True, target_valid=True,
                  wall_clearance_valid=True, raise_clearance_valid=True, jam_free_verified=True,
                  physical_output_enabled=True, automatic_control_enabled=True,
                  automatic_lowering_allowed=True, y_calibration_approved=True)
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
        self.assertEqual(SystemMode.REMOTE, authority.transition(SystemMode.AUTO_READY))
        self.assertFalse(authority.activate_new_task(True))
        ready_authority(authority)
        self.assertFalse(authority.activate_new_task(False))
        self.assertTrue(authority.activate_new_task(True))

    def test_old_command_after_epoch_change(self):
        authority = active_authority()
        action = command(authority)
        authority.transition(SystemMode.REMOTE)
        ready_authority(authority)
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
                    replace(permit(action), allow_y_positive=False)):
            self.assertFalse(bad.authorizes(action, 11))

    def test_variable_speed_profile(self):
        strategy = VariableSpeedStrategy(VARIABLE, ((0.05, 0.02, "JOG"),
                                                   (0.5, 0.1, "SLOW"), (2, 0.2, "FAST")))
        for position, phase in ((0, "FAST"), (3, "SLOW"), (3.8, "JOG"), (4, "AT_TARGET")):
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
        executor = TrolleyExecutor(authority, VariableSpeedStrategy(VARIABLE, ((0.05, 0.02, "JOG"),)),
                                   lease_sec=0.25)
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
        self.assertEqual(11.25, result.expire_stamp)


class R1Regressions(unittest.TestCase):
    def executor(self, kind=TrolleyExecutor):
        authority = active_authority()
        executor = kind(authority, FixedSlowStrategy(FIXED, STOP, 0.05, 0.2), lease_sec=0.25)
        action = replace(command(authority), axis=executor.axis, action=4 if kind.axis == "Z" else 3)
        return authority, executor, action

    def tick(self, executor, action, now=11, position=1, **changes):
        args = dict(readiness=readiness(), permit=permit(action), position=position,
                    feedback_fresh=True, speed=0.2)
        args.update(changes)
        return executor.tick(now, **args)

    def start(self, executor, action, now=11, position=1, **changes):
        args = dict(readiness=readiness(), permit=permit(action), position=position, feedback_fresh=True)
        args.update(changes)
        return executor.accept(action, now, **args)

    def test_two_consecutive_fixed_commands_y_and_z(self):
        for kind in (TrolleyExecutor, HoistExecutor):
            with self.subTest(axis=kind.axis):
                authority, executor, action = self.executor(kind)
                self.assertTrue(self.start(executor, action)[0])
                self.assertTrue(self.tick(executor, action).enable)
                self.assertFalse(self.tick(executor, action, 11.1, 4).enable)
                self.assertFalse(self.tick(executor, action, 11.4, 4).enable)
                self.assertEqual("COMPLETED", executor.context.state)
                self.assertFalse(self.tick(executor, action, 11.5, 1).enable)
                self.assertEqual(11.1, executor.strategy.stopping_since)
                second = replace(action, command_id="cmd2", sequence=2, target=8)
                self.assertTrue(self.start(executor, second, 12, 4)[0])
                self.assertIsNone(executor.strategy.stopping_since)
                self.assertTrue(self.tick(executor, second, 12, 4).enable)
                self.assertEqual(2, authority.last_sequence)

    def test_command_admitted_once_many_ticks(self):
        authority, executor, action = self.executor()
        self.assertTrue(self.start(executor, action)[0])
        requests = [self.tick(executor, action, time) for time in (11, 11.05, 11.1)]
        self.assertTrue(all(r.enable for r in requests))
        self.assertEqual([1, 1, 1], [r.command_sequence for r in requests])
        self.assertEqual([1, 2, 3], [r.actuation_sequence for r in requests])
        self.assertTrue(all(r.expire_stamp - r.issued_stamp <= 0.25 for r in requests))
        self.assertEqual(1, authority.last_sequence)
        self.assertFalse(self.start(executor, action)[0])

    def test_wrong_axis_and_invalid_target_do_not_burn_sequence(self):
        authority, executor, action = self.executor()
        wrong = HoistExecutor(authority, executor.strategy, lease_sec=0.25)
        self.assertFalse(self.start(wrong, action)[0])
        for bad in (replace(action, target=math.nan), replace(action, direction=True),
                    replace(action, direction=2), replace(action, target=0),
                    replace(action, action=4), replace(action, action=True)):
            self.assertFalse(self.start(executor, bad)[0])
            self.assertEqual(0, authority.last_sequence)
        self.assertTrue(self.start(executor, action)[0])

    def test_permit_axis_direction_mapping_is_exact(self):
        for (axis, direction), field in PERMISSION_FIELDS.items():
            one = replace(PermitEvidence(), **{field: True})
            for other_axis, other_dir in PERMISSION_FIELDS:
                self.assertEqual((axis, direction) == (other_axis, other_dir),
                                 direction_permission(one, other_axis, other_dir))
        self.assertTrue(direction_permission(PermitEvidence(), "ALL", 0))
        self.assertFalse(direction_permission(PermitEvidence(), "Y", True))
        self.assertFalse(direction_permission(PermitEvidence(), "ALL", 1))

    def test_wrong_direction_permit_does_not_burn_sequence(self):
        authority, executor, action = self.executor()
        self.assertFalse(self.start(executor, action, permit=replace(permit(action), allow_y_positive=False))[0])
        self.assertEqual(0, authority.last_sequence)

    def test_epoch_and_fresh_evidence_are_revalidated_each_tick(self):
        for failure in ("mode", "permit", "feedback", "readiness", "expiry", "clock"):
            with self.subTest(failure=failure):
                authority, executor, action = self.executor()
                self.start(executor, action)
                self.assertTrue(self.tick(executor, action).enable)
                args = {}
                now = 11.1
                if failure == "mode":
                    authority.transition(SystemMode.REMOTE)
                elif failure == "permit":
                    args["permit"] = replace(permit(action), allow_y_positive=False)
                elif failure == "feedback":
                    args["feedback_fresh"] = False
                elif failure == "readiness":
                    args["readiness"] = readiness(safety_ready=False)
                elif failure == "expiry":
                    now = 20
                else:
                    now = 10.9
                self.assertFalse(self.tick(executor, action, now, **args).enable)
                self.assertEqual("ABORTED", executor.context.state)
                self.assertFalse(self.tick(executor, action, 11.2).enable)

    def test_cancel_and_mode_abort_are_not_failure(self):
        for aborted, expected in ((False, "CANCELLED"), (True, "ABORTED")):
            authority, executor, action = self.executor()
            self.start(executor, action)
            self.assertFalse(executor.cancel(11.1, aborted=aborted).enable)
            self.assertEqual(expected, executor.context.state)
            self.assertFalse(self.start(executor, replace(action, sequence=2))[0])
            self.assertFalse(self.tick(executor, action, 11.2).enable)

    def test_new_command_cannot_overwrite_active_target(self):
        authority, executor, action = self.executor()
        self.start(executor, action)
        self.assertFalse(self.start(executor, replace(action, command_id="other", sequence=2))[0])
        self.assertEqual(action, executor.context.action)
        self.assertEqual(1, authority.last_sequence)

    def test_unknown_lease_never_admits(self):
        authority, executor, action = self.executor()
        executor.lease_sec = None
        self.assertFalse(self.start(executor, action)[0])
        self.assertEqual(0, authority.last_sequence)

    def test_unconfigured_strategy_never_consumes_sequence(self):
        authority, executor, action = self.executor()
        for strategy in (FixedSlowStrategy(), VariableSpeedStrategy(VARIABLE),
                         VariableSpeedStrategy(VARIABLE, (None,))):
            executor.strategy = strategy
            self.assertFalse(self.start(executor, action)[0])
            self.assertEqual(0, authority.last_sequence)

    def test_adapter_lease_expires_without_new_messages(self):
        authority, executor, action = self.executor()
        self.start(executor, action)
        request = self.tick(executor, action)
        guard = ActuationLeaseGuard(authority.session_id, authority.command_epoch, 0.25, clock=lambda: 11)
        self.assertTrue(guard.receive(request, 11))
        self.assertIsNotNone(guard.poll(11.24))
        self.assertIsNone(guard.poll(11.25))  # No request means all outputs OFF.
        self.assertFalse(any(guard.logical_outputs(11.25)))
        self.assertIsNone(guard.poll(11.26))

    def test_adapter_replay_restart_clock_and_bad_lease_release(self):
        authority, executor, action = self.executor()
        self.start(executor, action)
        request = self.tick(executor, action)
        bad_frames = [request, replace(request, session_id="old"),
                      replace(request, command_epoch=0), replace(request, actuation_sequence=0),
                      replace(request, expire_stamp=15), replace(request, direction=True),
                      replace(request, enable=False)]
        for bad in bad_frames:
            guard = ActuationLeaseGuard(authority.session_id, authority.command_epoch, 0.25, clock=lambda: 11)
            self.assertTrue(guard.receive(request, 11))
            self.assertFalse(guard.receive(bad, 11.1))
            self.assertIsNone(guard.poll(11.1))
        guard = ActuationLeaseGuard("restarted", authority.command_epoch, 0.25, clock=lambda: 11)
        self.assertFalse(guard.receive(request, 11))
        guard = ActuationLeaseGuard(authority.session_id, authority.command_epoch, 0.25, clock=lambda: 11)
        guard.receive(request, 11)
        guard.poll(11.1)
        self.assertIsNone(guard.poll(11.05))

    def test_stop_sequence_blocks_delayed_pre_stop_on(self):
        authority, executor, action = self.executor()
        self.start(executor, action)
        first = self.tick(executor, action)
        delayed = self.tick(executor, action, 11.05)
        stop = executor.cancel(11.1)
        guard = ActuationLeaseGuard(authority.session_id, authority.command_epoch, 0.25, clock=lambda: 11)
        self.assertTrue(guard.receive(first, 11))
        self.assertFalse(guard.receive(stop, 11.1))
        self.assertFalse(guard.receive(delayed, 11.11))
        self.assertFalse(any(guard.logical_outputs(11.11)))

    def test_mode_graph_prevents_startup_and_recovery_shortcuts(self):
        authority = ControlAuthority()
        proof = dict(stopped=True, outputs_off=True, safety_fresh=True,
                     perception_ready=True, localization_ready=True, reset_authorized=True)
        self.assertEqual(SystemMode.BOOT, authority.transition(SystemMode.AUTO_READY, **proof))
        authority.transition(SystemMode.SELF_CHECK)
        self.assertEqual(SystemMode.SELF_CHECK, authority.transition(SystemMode.SAFE_IDLE, **proof))
        authority.transition(SystemMode.SAFE_IDLE, self_check_passed=True, **proof)
        self.assertEqual(SystemMode.SAFE_IDLE, authority.transition(SystemMode.AUTO_READY, **proof))
        authority.transition(SystemMode.AUTO_PENDING)
        self.assertEqual(SystemMode.AUTO_PENDING, authority.transition(SystemMode.AUTO_READY))
        self.assertEqual(SystemMode.AUTO_READY, authority.transition(SystemMode.AUTO_READY, **proof))
        self.assertTrue(authority.activate_new_task(True))
        self.assertEqual(SystemMode.AUTO_ACTIVE, authority.transition(SystemMode.SAFE_IDLE))
        for latch in (SystemMode.FAULT_LATCHED, SystemMode.EMERGENCY_STOP):
            authority.transition(latch)
            self.assertEqual(latch, authority.transition(SystemMode.AUTO_READY, **proof))
            self.assertEqual(latch, authority.transition(SystemMode.SAFE_IDLE))
            self.assertEqual(SystemMode.SAFE_IDLE, authority.transition(SystemMode.SAFE_IDLE, **proof))
            self.assertFalse(authority.activate_new_task(True))

    def test_raise_lower_have_independent_conservative_evidence(self):
        for key in ("dual_lidar", "pit_perception_ready", "grab_bottom_valid",
                    "target_valid", "wall_clearance_valid"):
            result = readiness(**{key: False})
            self.assertFalse(result.z_lower_ready)
            self.assertTrue(result.z_raise_ready)
        result = readiness(raise_clearance_valid=False)
        self.assertTrue(result.z_lower_ready)
        self.assertFalse(result.z_raise_ready)
        for key in ("grab_tracking_ready", "jam_free_verified", "safety_ready"):
            result = readiness(**{key: False})
            self.assertFalse(result.z_lower_ready)
            self.assertFalse(result.z_raise_ready)

    def mapping(self):
        return {"contract_version": 4, "essential_signals": ["mode_auto", "safety_ok"],
                "optional_capabilities": {"e_stop": "NOT_CONFIGURED"},
                "physical_channels": {
                    device: [{"channel": i, "assignment": "NOT_CONFIGURED",
                              "signal": "NOT_CONFIGURED", "invert": "NOT_CONFIGURED"}
                             for i in range(count)]
                    for device, count in (("adam6052", 8), ("adam6251", 16))}}

    def test_canonical_di_is_derived_and_missing_safety_stays_unknown(self):
        config = self.mapping()
        config["physical_channels"]["adam6052"][0].update(
            assignment="ASSIGNED", signal="mode_auto", invert=False)
        mapping = derive_logical_inputs(config)
        values = normalize(mapping, {"adam6052": RawInput((True,) * 8, 10, 0, True)}, 10.1, 1)
        self.assertTrue(values["mode_auto"])
        self.assertIsNone(values["safety_ok"])
        self.assertIsNone(values["e_stop"])

    def test_canonical_di_rejects_parallel_duplicate_or_guessed_bindings(self):
        for failure in ("legacy", "duplicate_channel", "duplicate_signal", "polarity", "reserved"):
            config = self.mapping()
            rows = config["physical_channels"]["adam6052"]
            if failure == "legacy":
                config["digital_inputs"] = {}
            elif failure == "duplicate_channel":
                rows[1]["channel"] = 0
            else:
                rows[0].update(assignment="ASSIGNED", signal="mode_auto", invert=False)
                if failure == "duplicate_signal":
                    rows[1].update(assignment="ASSIGNED", signal="mode_auto", invert=False)
                elif failure == "polarity":
                    rows[0]["invert"] = "NOT_CONFIGURED"
                else:
                    rows[0]["assignment"] = "RESERVED"
            with self.assertRaises(ValueError):
                derive_logical_inputs(config)

    def test_manifest_requires_keyed_unique_complete_sensor_provenance(self):
        ref = dict(sensor_id="lidar-a", calibration_id="cal-a", extrinsic_version="ext-a",
                   intrinsic_version="intr-a")
        manifest = dict(run_id="run", task_id="task", cycle_id="cycle", map_id="map",
                        grab_geometry_version="grab", git_sha="a" * 40, config_hash="b" * 64,
                        required_sensor_ids=["lidar-a"], calibrations=[ref])
        self.assertTrue(manifest_ready(manifest))
        for bad in ([], [ref, ref], [dict(ref, sensor_id="lidar-b")],
                    [dict(ref, extrinsic_version="NOT_CONFIGURED")]):
            self.assertFalse(manifest_ready(dict(manifest, calibrations=bad)))
        self.assertFalse(manifest_ready(dict(manifest, required_sensor_ids=[])))

    def test_fail_cycle_reports_success_and_stopped_task_rejects_it(self):
        task = TaskCycles("run", "task")
        self.assertTrue(task.fail_cycle("EMPTY_GRAB", True))
        task.remote_abort()
        self.assertFalse(task.fail_cycle("EMPTY_GRAB"))
