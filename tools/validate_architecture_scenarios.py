#!/usr/bin/env python3
"""Run synthetic scenarios only, not hardware or ROS validation."""
import importlib.util
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_control/src"))
sys.path.insert(0, str(ROOT / "src/chili_crane_hardware/src"))


def main():
    spec = importlib.util.spec_from_file_location(
        "architecture_scenarios", ROOT / "src/chili_crane_control/test/test_architecture.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(module))
    print(json.dumps({"scope": "SYNTHETIC_POLICY_ONLY", "tests": result.testsRun,
                      "MOCK_SCENARIO_STATUS": "PASS" if result.wasSuccessful() else "FAIL",
                      "HARDWARE": "NOT_RUN", "BAG": "NOT_RUN", "FIELD": "NOT_RUN"}))
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
