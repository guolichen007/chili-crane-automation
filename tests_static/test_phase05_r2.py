import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_control/src"))
sys.path.insert(0, str(ROOT / "src/chili_crane_hardware/src"))
from chili_crane_control.execution import (
    ActionAuthorizationPolicy, AuthorizedAction, PermitEvidence, ActuationRequest,
    ExecutionStrategy, FixedSlowStrategy, ControlAuthority, GrabExecutor, BridgeServoExecutor,
)
from chili_crane_hardware.lease import ActuationLeaseGuard
from chili_crane_control.evidence_policy import SourceType, RuntimePolicy
from tests_static.test_architecture_freeze import MODULE


class R2Tests(unittest.TestCase):
    def test_output_and_auto_flags_are_readiness_gates(self):
        self.assertFalse(MODULE.readiness(physical_output_enabled=False).system_ready)
        self.assertFalse(MODULE.readiness(automatic_control_enabled=False).system_ready)
        self.assertFalse(MODULE.readiness(automatic_lowering_allowed=False).z_lower_ready)
        self.assertFalse(MODULE.readiness(y_calibration_approved=False).y_auto_ready)
        self.assertTrue(MODULE.readiness().system_ready)

    def test_unload_and_auto_permission_are_distinct(self):
        action = AuthorizedAction(axis="G", direction=1, cycle_phase="OPEN_GRAB")
        permit = PermitEvidence(allow_grab_open=True, allow_auto_task=True,
                                runtime_mode="synthetic_test", evidence_sources=(SourceType.SYNTHETIC,))
        self.assertFalse(ActionAuthorizationPolicy.authorizes(permit, action))
        self.assertTrue(ActionAuthorizationPolicy.authorizes(replace(permit, allow_unload=True), action))
        self.assertTrue(ActionAuthorizationPolicy.authorizes(
            permit, replace(action, cycle_phase="ENSURE_GRAB_OPEN")))
        self.assertFalse(ActionAuthorizationPolicy.authorizes(
            replace(permit, allow_auto_task=False), replace(action, cycle_phase="ENSURE_GRAB_OPEN")))
        self.assertFalse(ActionAuthorizationPolicy.authorizes(permit, replace(action, cycle_phase="")))

    def test_direction_only_rejects_before_lease(self):
        for axis, speed in [("X", False), ("Y", True)]:
            request = ActuationRequest(axis, 1, True, speed, source_command_id="cmd",
                session_id="s", command_epoch=1, command_sequence=1, actuation_sequence=1,
                issued_stamp=100, expire_stamp=100.25)
            guard = ActuationLeaseGuard("s", 1, .25, clock=lambda: 1)
            self.assertFalse(guard.receive(request, 100))
            self.assertFalse(any(guard.logical_outputs(1.1)))

    def test_ntp_does_not_extend_monotonic_deadline(self):
        request = ActuationRequest("Y", 1, True, False, source_command_id="cmd",
            session_id="s", command_epoch=1, command_sequence=1, actuation_sequence=1,
            issued_stamp=100, expire_stamp=100.25)
        guard = ActuationLeaseGuard("s", 1, .25, clock=lambda: 50)
        self.assertTrue(guard.receive(request, 100))
        self.assertIsNotNone(guard.poll(50.24))
        self.assertIsNone(guard.poll(50.25))

    def test_strategy_is_structural_and_position_strategies_reject_x_g(self):
        strategy = FixedSlowStrategy(MODULE.FIXED, MODULE.STOP, .05, .2)
        self.assertIsInstance(strategy, ExecutionStrategy)
        self.assertFalse(strategy.supports_axis("X"))
        self.assertFalse(strategy.supports_axis("G"))
        class ThirdParty:
            def configured(self): return True
            def supports_axis(self, axis): return axis == "Y"
            def begin(self, key): pass
            def step(self, position, target, **evidence): pass
        self.assertIsInstance(ThirdParty(), ExecutionStrategy)

    def test_production_rejects_every_nonphysical_source(self):
        policy = RuntimePolicy("production", True, True)
        self.assertTrue(policy.physical_permission((SourceType.PHYSICAL,)))
        for source in SourceType:
            if source != SourceType.PHYSICAL:
                self.assertFalse(policy.physical_permission((SourceType.PHYSICAL, source)))
        self.assertFalse(policy.physical_permission(()))
        self.assertFalse(RuntimePolicy("algorithm_dev", True, True).physical_permission((SourceType.PHYSICAL,)))
