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
            "allow_x_move",
            "allow_y_move",
            "allow_lower",
            "allow_raise",
            "allow_grab_open",
            "allow_grab_close",
            "allow_unload",
            "allow_auto_task",
        }
        actual = set(re.findall(r"^bool (allow_[a-z_]+)$", text, re.MULTILINE))
        self.assertEqual(expected, actual)

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


if __name__ == "__main__":
    unittest.main()
