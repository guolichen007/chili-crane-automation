"""Phase1C regression: receive freshness differs from clock status and frames."""
import sys
import unittest
from pathlib import Path
import numpy as np
from chili_crane_slam.preview_contract import receive_fresh, preview_transform


class PreviewContracts(unittest.TestCase):
    def test_preview_ttl_monotonic_no_source_age_gate(self):
        self.assertTrue(receive_fresh(1, .8, .5))
        self.assertFalse(receive_fresh(1, .8, .1))
        self.assertFalse(receive_fresh(1, .8, None))
        self.assertFalse(receive_fresh(1, 2, .5))

    def test_candidate_frame_conversion_missing_never_identity(self):
        t = np.eye(4)
        t[0, 3] = 1
        rig = {"mount_revision": "synthetic-r1", "rig_frame": "rig", "parent_mode": "DYNAMIC_PARENT"}
        candidate = {"mount_revision": "synthetic-r1", "status": "CANDIDATE",
                     "target_frame": "a", "source_frame": "b", "T_target_source": t.tolist()}
        np.testing.assert_allclose(preview_transform("b", "a", rig, [], candidate), t)
        with self.assertRaisesRegex(ValueError, "NOT_AVAILABLE_FRAME_MISMATCH"):
            preview_transform("a", "rig", rig, [], candidate)

    def test_python_scripts_git_executable_registration(self):
        import subprocess
        root = Path(__file__).resolve().parents[1]
        lines = subprocess.check_output(["git", "ls-files", "--stage", "src"], cwd=root, text=True)
        for line in lines.splitlines():
            path = line.split("\t", 1)[1]
            if "/scripts/" in path and path.endswith(".py"):
                if path == "src/chili_crane_hardware/scripts/er1_clock_observer.py":
                    self.assertTrue(line.startswith("100644 "), path)  # Frozen implementation data.
                else:
                    self.assertTrue(line.startswith("100755 "), path)
