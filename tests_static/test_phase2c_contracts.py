import dataclasses
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/chili_crane_control/src'))
from chili_crane_control.interaction_contracts import Evidence,TargetBinding,BindingLedger


def binding():
    return TargetBinding('run','task','task:1',1,'binding','proposal','candidate','map-id','v1','pit',
        'b'*32+':snapshot','b'*32,2,'transform','chain-cal','geofence','v2','a'*64,'kernel',
        (1.,1.,3.),((0.,0.),(1.1,0.),(1.1,1.1),(0.,1.1)),1.,.9,1.1,.3,10_100_000_000,1,Evidence.at(10.))


class ContractTests(unittest.TestCase):
    def test_binding_digest_and_immutable_nested_fields(self):
        b=binding();self.assertEqual(b.digest(),b.binding_digest_sha256)
        self.assertEqual(binding().binding_digest_sha256,b.binding_digest_sha256)
        with self.assertRaises(dataclasses.FrozenInstanceError):b.candidate_id='other'
        with self.assertRaises(TypeError):b.capture_footprint[0][0]=99
        with self.assertRaisesRegex(ValueError,'DIGEST'):
            dataclasses.replace(b,target_suspension_map=(2.,1.,3.))

    def test_same_cycle_swap_and_lineage_change_rejected(self):
        ledger=BindingLedger();b=ledger.bind(binding())
        with self.assertRaisesRegex(ValueError,'ALREADY_BOUND'):ledger.bind(b)
        with self.assertRaisesRegex(ValueError,'LINEAGE'):
            ledger.check_lineage(b,('new-map',)+b.lineage[1:])
        self.assertEqual(ledger.bindings[('run','task','task:1')],b)

    def test_invalid_nan_and_control_binding_rejected(self):
        b=binding()
        for update in (dict(surface_upper_m=float('nan')),dict(valid_for_control=True),dict(cycle_index=0)):
            with self.assertRaises(ValueError):dataclasses.replace(b,binding_digest_sha256='',**update)

    def test_msgs_cmake_and_complete_load_migration(self):
        names=('TargetBinding','ContactEvidence','SurfaceInteractionEvidence','ClosureEvidence','PayloadEvidence',
               'LiftEvidence','UnloadEvidence','GraspInteractionState')
        cmake=(ROOT/'src/chili_crane_msgs/CMakeLists.txt').read_text(encoding='utf-8')
        for name in names:
            message=(ROOT/'src/chili_crane_msgs/msg'/f'{name}.msg').read_text(encoding='utf-8')
            self.assertIn('EvidenceMetadata evidence',message)
            self.assertIn('bool valid_for_control',message)
            self.assertIn(f'msg/{name}.msg',cmake)
        self.assertIn('EvidenceMetadata evidence',(ROOT/'src/chili_crane_msgs/msg/LoadState.msg').read_text())

    def test_counter_epoch_and_source_freshness(self):
        self.assertTrue(Evidence.at(10.).fresh(10.1,.3,'SYNTHETIC'))
        for e in (Evidence.at(10.,validity=1),Evidence.at(10.,reported_age=3.),Evidence.at(10.,source_type='UNKNOWN')):
            self.assertFalse(e.fresh(10.1,.3))
        self.assertFalse(Evidence.at(10.).fresh(9.9,.3))
