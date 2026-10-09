"""Import pure Phase 0.5 tests without ROS."""
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_control/src"))
sys.path.insert(0, str(ROOT / "src/chili_crane_hardware/src"))
SPEC = importlib.util.spec_from_file_location(
    "phase05_tests", ROOT / "src/chili_crane_control/test/test_architecture.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
ArchitectureScenarios = MODULE.ArchitectureScenarios
