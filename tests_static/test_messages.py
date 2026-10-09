import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MSG_DIR = ROOT / "src" / "chili_crane_msgs" / "msg"


class MessageContractTest(unittest.TestCase):
    def test_public_messages_have_validity_and_reason(self):
        for path in sorted(MSG_DIR.glob("*.msg")):
            with self.subTest(message=path.name):
                text = path.read_text(encoding="utf-8")
                self.assertIn("uint8 validity", text)
                self.assertIn("string reason", text)

    def test_safety_permissions_are_explicit(self):
        text = (MSG_DIR / "SafetyPermit.msg").read_text(encoding="utf-8")
        expected = {
            "allow_x_positive",
            "allow_x_negative",
            "allow_y_positive",
            "allow_y_negative",
            "allow_lower",
            "allow_raise",
            "allow_grab_open",
            "allow_grab_close",
            "allow_unload",
            "allow_auto_task",
        }
        actual = set(re.findall(r"^bool (allow_[a-z_]+)$", text, re.MULTILINE))
        self.assertEqual(expected, actual)
        for field in (
            "string permit_id",
            "uint64 permit_generation",
            "string evaluated_intent_id",
            "builtin_interfaces/Time issued_stamp",
            "builtin_interfaces/Time expire_stamp",
        ):
            self.assertIn(field, text)

    def test_authorized_command_binds_intent_permit_and_expiry(self):
        text = (MSG_DIR / "AuthorizedCommand.msg").read_text(encoding="utf-8")
        for field in (
            "string command_id",
            "string intent_id",
            "string permit_id",
            "uint64 permit_generation",
            "builtin_interfaces/Time issued_stamp",
            "builtin_interfaces/Time expire_stamp",
        ):
            self.assertIn(field, text)

    def test_execution_state_exposes_rejection_and_completion(self):
        text = (MSG_DIR / "CommandExecutionState.msg").read_text(
            encoding="utf-8"
        )
        for field in (
            "STATE_REJECTED",
            "STATE_EXECUTING",
            "STATE_COMPLETED",
            "bool accepted",
            "bool failed",
        ):
            self.assertIn(field, text)

    def test_mock_adapter_consumes_only_actuation_requests(self):
        adapter = (
            ROOT
            / "src"
            / "chili_crane_hardware"
            / "scripts"
            / "mock_hardware_adapter.py"
        ).read_text(encoding="utf-8")
        self.assertIn("control/actuation_request", adapter)
        self.assertNotIn("control/authorized_command", adapter)
        self.assertNotIn("control/requested_intent", adapter)
        self.assertNotIn("ControlIntent", adapter)

    def test_fail_safe_supervisor_sets_all_permissions_false(self):
        supervisor = (
            ROOT
            / "src"
            / "chili_crane_control"
            / "scripts"
            / "fail_safe_safety_supervisor.py"
        ).read_text(encoding="utf-8")
        safety = (MSG_DIR / "SafetyPermit.msg").read_text(encoding="utf-8")
        permissions = re.findall(
            r"^bool (allow_[a-z_]+)$", safety, re.MULTILINE
        )
        for permission in permissions:
            self.assertIn("permit.{} = False".format(permission), supervisor)

    def test_intent_and_authorized_command_actions_match(self):
        def action_pairs(name):
            text = (MSG_DIR / name).read_text(encoding="utf-8")
            pairs = re.findall(
                r"^uint8 (ACTION_[A-Z0-9_]+)=(\d+)$",
                text,
                re.MULTILINE,
            )
            return [(action, int(value)) for action, value in pairs]

        self.assertEqual(
            action_pairs("ControlIntent.msg"),
            action_pairs("AuthorizedCommand.msg"),
        )

    def test_grab_contract_has_conflict_state(self):
        text = (MSG_DIR / "GrabState.msg").read_text(encoding="utf-8")
        self.assertIn("OPENING_CONFLICT", text)
        self.assertNotIn("bool open_limit", text)
        self.assertNotIn("bool closed_limit", text)
        self.assertIn("float64 bottom_z_m", text)
        self.assertIn("geometry_msgs/PoseWithCovariance pose", text)

    def test_grab_io_contract_is_raw_hardware_only(self):
        text = (MSG_DIR / "GrabIoState.msg").read_text(encoding="utf-8")
        self.assertIn("bool open_limit", text)
        self.assertIn("bool closed_limit", text)
        self.assertNotIn("bottom_z_m", text)
        self.assertNotIn("geometry_msgs/Pose", text)

    def test_control_board_contract_has_safety_evidence(self):
        text = (MSG_DIR / "ControlBoardState.msg").read_text(encoding="utf-8")
        self.assertIn("bool e_stop_active", text)
        self.assertIn("bool io_heartbeat_ok", text)
        self.assertIn("bool manual_mode", text)

    def test_hoist_has_no_draw_wire_position_field(self):
        text = (MSG_DIR / "HoistState.msg").read_text(encoding="utf-8")
        self.assertNotIn("position_m", text)
        self.assertIn("bool upper_limit", text)
        self.assertIn("bool lower_limit", text)
        self.assertIn("float64 evidence_age_sec", text)

    def test_servo_contract_reserves_generic_drive_health(self):
        text = (MSG_DIR / "ServoState.msg").read_text(encoding="utf-8")
        for field in (
            "bool drive_ready",
            "bool servo_enabled",
            "bool positive_limit",
            "bool negative_limit",
            "bool communication_ok",
            "int32 fault_code",
            "uint64 source_counter",
        ):
            self.assertIn(field, text)


if __name__ == "__main__":
    unittest.main()
