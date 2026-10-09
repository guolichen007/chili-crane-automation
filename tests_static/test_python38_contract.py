import importlib.util
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("python310_checker", ROOT / "tools/check_repo_contracts.py")
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


class Python310ContractTest(unittest.TestCase):
    def test_first_party_python310_compatibility(self):
        errors = []
        CHECKER.check_python310_compatibility(ROOT, errors)
        self.assertEqual([], errors)


if __name__ == "__main__":
    unittest.main()
