"""B2 conservative independent Grab/static mask counterexamples."""
from dataclasses import replace
import sys
from pathlib import Path
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests_static'))
sys.path.insert(0,str(ROOT/'src/chili_crane_perception/src'))
from chili_crane_perception.surface_mask import GrabMask, PitSurfaceBuilder
from test_phase2b_surface import fixture, cloud


def mask_fixture():
    c,s,ctx=fixture()
    c.update(dynamic_mask_dilation_m=.05,static_masks_state='VALID',static_masks=[])
    grab=GrabMask(10.,'map','map1','v1','pit','cal1','SYNTHETIC','TRACKED',True,True,
                  ((-.4,-.4),(.4,-.4),(.4,.4),(-.4,.4)),.9,1.8,0.,'SYNTHETIC_VALIDATED')
    return c,s,ctx,grab


def ingest(b,p,sensors,ctx,grab,epoch=10.,counter=1):
    return b.ingest(p,sensors,dict(epoch=epoch,source_counter=counter,frame_id='cloud',span_sec=.02,
        source_type='SYNTHETIC',calibration_id='cal1'),replace(ctx,measurement_epoch=epoch),
        replace(grab,epoch=epoch) if grab is not None else None,epoch+.01)


class MaskTests(unittest.TestCase):
    def setUp(self):
        self.c,self.s,self.ctx,self.grab=mask_fixture()
        self.b=PitSurfaceBuilder(self.c,self.s)
        self.p,self.sensors=cloud(self.b.grid)

    def test_connected_grab_points_never_become_material(self):
        # Same contiguous height as material, not a separable isolated cluster.
        self.assertTrue(ingest(self.b,self.p,self.sensors,self.ctx,self.grab))
        self.assertTrue(ingest(self.b,self.p,self.sensors,self.ctx,self.grab,10.05,2))
        snapshot=self.b.grid.snapshot(10.06)
        self.assertGreater(self.b.masked_count,0)
        for c in snapshot.cells:
            x,y=self.b.grid.center(c.index)
            if abs(x) <= .4 and abs(y) <= .4:
                self.assertFalse(c.observed)
        self.assertEqual(self.grab.bottom_lower_bound_m,.9)

    def test_unknown_stale_moving_mask_invalidates_prior_surface(self):
        for grab in (None,replace(self.grab,tracking_state='LOST'),replace(self.grab,motion_stable=False),
                     replace(self.grab,evidence_age_sec=1.)):
            self.b=PitSurfaceBuilder(self.c,self.s)
            self.assertTrue(ingest(self.b,self.p,self.sensors,self.ctx,self.grab))
            self.assertFalse(ingest(self.b,self.p,self.sensors,self.ctx,grab,10.05,2))
            self.assertFalse(any(c.observed for c in self.b.grid.snapshot(10.06).cells))

    def test_map_and_calibration_mismatch(self):
        for grab in (replace(self.grab,map_id='other'),replace(self.grab,frame_id='cloud'),
                     replace(self.grab,calibration_id='other'),replace(self.grab,extrinsic_state='BENCH_VALIDATED')):
            self.assertFalse(ingest(self.b,self.p,self.sensors,self.ctx,grab))

    def test_vertical_mask_not_column_to_infinity(self):
        p=self.p.copy();p[:,2]=.5
        self.assertTrue(ingest(self.b,p,self.sensors,self.ctx,self.grab))
        self.assertEqual(self.b.masked_count,0)

    def test_static_self_mask_and_unknown_configuration(self):
        self.c['static_masks']=[dict(mask_id='beam',footprint_xy_m=[[-2,-2],[-1,-2],[-1,2],[-2,2]],bottom_m=0.,top_m=3.)]
        b=PitSurfaceBuilder(self.c,self.s)
        self.assertTrue(ingest(b,self.p,self.sensors,self.ctx,self.grab))
        self.assertGreater(b.masked_count,self.b.masked_count)
        self.c['static_masks_state']='NOT_CONFIGURED'
        with self.assertRaisesRegex(ValueError,'STATIC_SELF_MASKS_NOT_CONFIGURED'):
            PitSurfaceBuilder(self.c,self.s)

    def test_conservative_dilation_masks_edge(self):
        p=np.tile([[.44,0,1.]],(6,1));sensors=np.array([0,1]*3)
        self.assertFalse(ingest(self.b,p,sensors,self.ctx,self.grab))
        self.assertEqual(self.b.masked_count,6)
