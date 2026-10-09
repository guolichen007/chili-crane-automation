"""Run the same pure hardware tests on Windows and in the ROS2 build job."""
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/chili_crane_hardware/src"))
sys.path.insert(0, str(ROOT / "src/chili_crane_control/src"))
SPEC = importlib.util.spec_from_file_location(
    "chili_hardware_tests", ROOT / "src/chili_crane_hardware/test/test_protocols.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
AddressAndOutputTest = MODULE.AddressAndOutputTest
TcpTransportTest = MODULE.TcpTransportTest
InputAndModeTest = MODULE.InputAndModeTest
PullWireTest = MODULE.PullWireTest
RtuFrameTest = MODULE.RtuFrameTest
SessionLatchTest = MODULE.SessionLatchTest
