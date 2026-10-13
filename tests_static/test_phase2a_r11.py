"""Narrow Clearance age/identity contract; no registration or geometry changes."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src/chili_crane_perception/src"))
from chili_crane_perception.state_contract import synchronize_clearance_age


class ClearanceAgeTests(unittest.TestCase):
    def message(self):
        return SimpleNamespace(header=SimpleNamespace(stamp=123),
            evidence=SimpleNamespace(measurement_stamp=123,source_counter=456,evidence_age_sec=.02),
            evidence_age_sec=.02,publish_latency_sec=0.)

    def test_outer_inner_age_match_and_measurement_identity_unchanged(self):
        result=self.message()
        age=synchronize_clearance_age(result,.02,10.,10.05)
        self.assertAlmostEqual(.07,age)
        self.assertEqual(result.evidence_age_sec,result.evidence.evidence_age_sec)
        self.assertEqual(123,result.header.stamp)
        self.assertEqual(123,result.evidence.measurement_stamp)
        self.assertEqual(456,result.evidence.source_counter)

    def test_same_geometry_age_monotonic_and_exceeds_unchanged_ttl(self):
        result=self.message()
        ages=[synchronize_clearance_age(result,.02,10.,t) for t in (10.01,10.06,10.11,10.21,10.31)]
        self.assertTrue(all(a<b for a,b in zip(ages,ages[1:])))
        self.assertGreater(ages[-1],.2)
        self.assertEqual(456,result.evidence.source_counter)

    def test_missing_or_invalid_age_is_not_young_nested_evidence(self):
        for upstream,received,now in ((None,None,10.),(float("nan"),10.,10.1),(-1.,10.,10.1),(.02,10.,9.)):
            result=self.message()
            self.assertEqual(1e9,synchronize_clearance_age(result,upstream,received,now))
            self.assertEqual(result.evidence_age_sec,result.evidence.evidence_age_sec)
            self.assertEqual(123,result.evidence.measurement_stamp)


if __name__=="__main__":
    unittest.main()
