import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKER_PATH = ROOT / "tools" / "check_repo_contracts.py"
SPEC = importlib.util.spec_from_file_location("check_repo_contracts", CHECKER_PATH)
CHECKER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(CHECKER)


class RepositoryContractTest(unittest.TestCase):
    def test_all_repository_contracts(self):
        self.assertEqual([], CHECKER.collect_errors(ROOT))


if __name__ == "__main__":
    unittest.main()
