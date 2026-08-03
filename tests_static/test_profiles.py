import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_profile(name):
    path = ROOT / "config" / "profiles" / f"{name}.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))


class RuntimeProfileTest(unittest.TestCase):
    def test_mapping_and_localization_are_mutually_distinct(self):
        mapping = load_profile("mapping")
        localization = load_profile("localization")
        self.assertEqual("mapping", mapping["runtime"]["mode"])
        self.assertEqual("localization", localization["runtime"]["mode"])
        self.assertTrue(mapping["map_lifecycle"]["map_mutation_allowed"])
        self.assertFalse(
            localization["map_lifecycle"]["map_mutation_allowed"]
        )
        self.assertTrue(localization["map_lifecycle"]["require_frozen_map"])

    def test_all_profiles_fail_closed(self):
        for name in ("mapping", "localization", "mock", "replay"):
            with self.subTest(profile=name):
                profile = load_profile(name)
                self.assertEqual("mock", profile["runtime"]["hardware_adapter"])
                self.assertFalse(
                    profile["safety"]["default_allow_auto_task"]
                )

    def test_mock_is_not_valid_by_default(self):
        profile = load_profile("mock")
        self.assertEqual("localization", profile["runtime"]["mode"])
        self.assertFalse(
            profile["mock_hardware_adapter"]["configured_valid_mock"]
        )

    def test_mock_params_match_node_private_namespace(self):
        for name in ("mapping", "localization", "mock", "replay"):
            with self.subTest(profile=name):
                profile = load_profile(name)
                self.assertIn("mock_hardware_adapter", profile)
                self.assertIn(
                    "configured_valid_mock",
                    profile["mock_hardware_adapter"],
                )

    def test_runtime_modes_are_only_mapping_localization_or_replay(self):
        allowed = {"mapping", "localization", "replay"}
        for name in ("mapping", "localization", "mock", "replay"):
            with self.subTest(profile=name):
                profile = load_profile(name)
                self.assertIn(profile["runtime"]["mode"], allowed)


if __name__ == "__main__":
    unittest.main()
