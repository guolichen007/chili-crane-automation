"""Regression for 5Hz cloud / independent 20Hz geometry callback ordering."""
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/chili_crane_perception/src'))
from chili_crane_perception.grab_mask_pairs import GrabMaskPairs


def observation(ns,counter=1,frame='map'):
    return S(header=S(stamp=S(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000),frame_id=frame),
             evidence=S(source_counter=counter))


class MaskPairsTests(unittest.TestCase):
    def test_cloud_uses_original_mid_pair_even_after_next_20hz_observation(self):
        pairs=GrabMaskPairs();g=observation(10_000_000_000);c=observation(10_000_000_000)
        pairs.add('grab',g,1.);pairs.add('clearance',c,1.01)
        pairs.add('grab',observation(10_050_000_000,2),1.05)
        pairs.add('clearance',observation(10_050_000_000,2),1.06)
        (actual_g,arrival_g),(actual_c,arrival_c)=pairs.closest(10.,.03)
        self.assertIs(actual_g,g);self.assertIs(actual_c,c)
        self.assertEqual((arrival_g,arrival_c),(1.,1.01))
        with self.assertRaisesRegex(ValueError,'TIME_ALIGNMENT'):pairs.closest(11.,.03)

    def test_counter_or_frame_mismatch_does_not_make_a_pair(self):
        for c in (observation(10_000_000_000,2),observation(10_000_000_000,1,'other')):
            pairs=GrabMaskPairs();pairs.add('grab',observation(10_000_000_000),1.)
            pairs.add('clearance',c,1.)
            with self.assertRaisesRegex(ValueError,'TIME_ALIGNMENT'):pairs.closest(10.,.03)

    def test_capacity_bounds_retention_and_rejects_evicted_epoch(self):
        pairs=GrabMaskPairs(capacity=4)
        for i in range(20):
            for kind in ('grab','clearance'):pairs.add(kind,observation(10_000_000_000+i*50_000_000,i+1),i)
        self.assertTrue(all(len(samples)==4 for samples in pairs.samples.values()))
        with self.assertRaisesRegex(ValueError,'TIME_ALIGNMENT'):pairs.closest(10.,.03)
        self.assertEqual(pairs.closest(10.95,.03)[0][0].evidence.source_counter,20)


if __name__=='__main__':unittest.main()
