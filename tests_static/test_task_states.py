import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TASK_MSG = ROOT / "src" / "chili_crane_msgs" / "msg" / "TaskStatus.msg"


class TaskStateContractTest(unittest.TestCase):
    def test_required_task_states_are_unique(self):
        text = TASK_MSG.read_text(encoding="utf-8")
        pairs = re.findall(r"^uint8 ([A-Z][A-Z0-9_]*)=(\d+)$", text, re.MULTILINE)
        task_pairs = [
            (name, int(value))
            for name, value in pairs
            if name
            not in {"UNKNOWN", "NOT_CONFIGURED", "STALE", "DEGRADED", "VALID"}
        ]
        names = [name for name, _ in task_pairs]
        values = [value for _, value in task_pairs]
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual(len(values), len(set(values)))
        for required in (
            "IDLE",
            "SCANNING_PIT",
            "LOWERING",
            "LOAD_VERIFY",
            "UNLOAD_VERIFY",
            "DONE",
            "FAULT",
            "ABORTED",
        ):
            self.assertIn(required, names)


if __name__ == "__main__":
    unittest.main()
