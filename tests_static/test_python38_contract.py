import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKER_PATH = ROOT / "tools" / "check_repo_contracts.py"
SPEC = importlib.util.spec_from_file_location("python38_checker", CHECKER_PATH)
CHECKER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(CHECKER)


class Python38ContractTest(unittest.TestCase):
    def test_first_party_python_is_python38_compatible(self):
        errors = []
        CHECKER.check_python38_compatibility(ROOT, errors)
        self.assertEqual([], errors)


if __name__ == "__main__":
    unittest.main()
