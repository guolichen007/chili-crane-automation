import re
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
TASK_CONTRACT = ROOT / "config" / "contracts" / "task_states.yaml"
TASK_HEADER = (
    ROOT
    / "src"
    / "chili_crane_core"
    / "include"
    / "chili_crane_core"
    / "task_state.hpp"
)
TASK_MESSAGE = ROOT / "src" / "chili_crane_msgs" / "msg" / "TaskStatus.msg"


def expected_states():
    data = yaml.safe_load(TASK_CONTRACT.read_text(encoding="utf-8"))
    return [(item["name"], item["value"]) for item in data["states"]]


def message_states(names):
    text = TASK_MESSAGE.read_text(encoding="utf-8")
    pairs = re.findall(r"^uint8 ([A-Z][A-Z0-9_]*)=(\d+)$", text, re.MULTILINE)
    by_name = {name: int(value) for name, value in pairs}
    return [(name, by_name[name]) for name in names if name in by_name]


def header_states():
    text = TASK_HEADER.read_text(encoding="utf-8")
    match = re.search(
        r"enum class TaskState\s*:\s*std::uint8_t\s*\{(?P<body>.*?)\};",
        text,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError("TaskState enum not found")
    states = []
    next_value = 0
    for raw_item in match.group("body").split(","):
        item = raw_item.strip()
        if not item:
            continue
        parts = [part.strip() for part in item.split("=", 1)]
        name = parts[0]
        if len(parts) == 2:
            next_value = int(parts[1], 0)
        states.append((name, next_value))
        next_value += 1
    return states


class TaskStateConsistencyTest(unittest.TestCase):
    def test_yaml_message_and_cpp_are_exactly_equal(self):
        expected = expected_states()
        names = [name for name, _ in expected]
        self.assertEqual(expected, message_states(names))
        self.assertEqual(expected, header_states())

    def test_required_terminal_and_override_states_exist(self):
        names = {name for name, _ in expected_states()}
        self.assertTrue({"DONE", "FAULT", "ABORTED"}.issubset(names))

    def test_all_cpp_task_references_exist_in_contract(self):
        expected = {name for name, _ in expected_states()}
        for path in (ROOT / "src/chili_crane_core").rglob("*.hpp"):
            names = set(re.findall(r"TaskState::([A-Z_]+)", path.read_text(encoding="utf-8")))
            self.assertTrue(names.issubset(expected), str(path))


if __name__ == "__main__":
    unittest.main()
