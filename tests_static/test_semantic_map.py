import json
import unittest
from pathlib import Path

import jsonschema
import yaml


ROOT = Path(__file__).resolve().parents[1]


class SemanticMapContractTest(unittest.TestCase):
    def test_template_is_explicitly_unconfigured(self):
        path = ROOT / "config" / "semantic" / "track_01.template.yaml"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.assertEqual("NOT_CONFIGURED", data["config_state"])
        self.assertEqual("map", data["frame_id"])
        self.assertIsNone(data["track"]["origin_map_m"])
        self.assertEqual([], data["pits"])

    def test_schema_requires_core_semantics(self):
        path = ROOT / "docs" / "api" / "semantic_map.schema.json"
        schema = json.loads(path.read_text(encoding="utf-8"))
        jsonschema.Draft202012Validator.check_schema(schema)
        required = set(schema["required"])
        self.assertTrue(
            {
                "map_id",
                "semantic_version",
                "track",
                "pits",
                "unloading_stations",
                "safe_wait_pose",
            }.issubset(required)
        )

    def test_cpp_contract_uses_schema_names_without_aliases(self):
        header = (
            ROOT
            / "src"
            / "chili_crane_core"
            / "include"
            / "chili_crane_core"
            / "semantic_map.hpp"
        ).read_text(encoding="utf-8")
        self.assertIn("std::uint32_t schema_version", header)
        self.assertIn("std::string semantic_version", header)
        self.assertIn("Validity config_state", header)
        self.assertNotRegex(header, r"std::string\s+version\s*;")

    def test_template_validates_but_cannot_claim_valid(self):
        template = yaml.safe_load(
            (
                ROOT
                / "config"
                / "semantic"
                / "track_01.template.yaml"
            ).read_text(encoding="utf-8")
        )
        schema = json.loads(
            (
                ROOT / "docs" / "api" / "semantic_map.schema.json"
            ).read_text(encoding="utf-8")
        )
        jsonschema.validate(template, schema)
        template["config_state"] = "VALID"
        with self.assertRaises(jsonschema.ValidationError):
            jsonschema.validate(template, schema)

    def test_manifest_requires_provenance_versions(self):
        path = ROOT / "docs" / "api" / "map_manifest.schema.json"
        schema = json.loads(path.read_text(encoding="utf-8"))
        jsonschema.Draft202012Validator.check_schema(schema)
        required = set(schema["required"])
        self.assertTrue(
            {
                "source_git_sha",
                "source_run_id",
                "created_at_utc",
                "semantic_map_version",
                "lidar_extrinsic_version",
                "servo_calibration_version",
                "checksums_sha256",
            }.issubset(required)
        )


if __name__ == "__main__":
    unittest.main()
