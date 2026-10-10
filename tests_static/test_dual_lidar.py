import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_slam/src"))
SPEC = importlib.util.spec_from_file_location("dual_lidar_tests", ROOT / "src/chili_crane_slam/test/test_dual_lidar.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
SynchronizerTests = MODULE.SynchronizerTests
MergerTests = MODULE.MergerTests
NormalizationTests = MODULE.NormalizationTests
HealthTests = MODULE.HealthTests
