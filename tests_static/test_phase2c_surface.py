from dataclasses import replace
from pathlib import Path
import sys
import unittest
import os
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tests_static'),str(ROOT/'src/chili_crane_perception/src'),str(ROOT/'src/chili_crane_control/src')]
from test_phase2c_evidence import configuration
from chili_crane_control.interaction_contracts import TargetBinding,Evidence
from chili_crane_perception.phase2b_synthetic import fixture,scene,ingest
from chili_crane_perception.surface_mask import PitSurfaceBuilder
from chili_crane_perception.surface_interaction import SurfaceInteractionLifecycle


@unittest.skipUnless(os.environ.get('CHILI_RUN_LEGACY_R1_ARCHITECTURE')=='1','LEGACY_R1_ARCHITECTURE local refresh; not R2 acceptance')
class SurfaceTests(unittest.TestCase):
    def setup_surface(self):
        c,s,gc,ctx,grab=fixture(ROOT/'config/safety/pit_operational_geofence.template.yaml')
        builder=PitSurfaceBuilder(c,s);p,sensors=scene(builder.grid,'flat')
        for i in range(2):self.assertTrue(ingest(builder,p,sensors,ctx,grab,10.+i*.05,i+1))
        snapshot=builder.grid.snapshot(10.06)
        b=TargetBinding('run','task','task:1',1,'binding','proposal','candidate',ctx.map_id,ctx.semantic_version,ctx.pit_id,
            snapshot.snapshot_id,snapshot.surface_session_id,snapshot.generation,ctx.transform_id,ctx.calibration_id,
            gc['geofence_id'],gc['geofence_version'],'a'*64,c['kernel']['kernel_id'],(1.,1.,3.),
            ((.5,.5),(1.,.5),(1.,1.),(.5,1.)),1.,.95,1.05,.1,10_060_000_000,2,
            Evidence.at(10.05,2,calibration_id=ctx.calibration_id,frame_id='map'))
        life=SurfaceInteractionLifecycle(builder.grid,configuration());life.bind(b,snapshot,10.06)
        return builder,p,sensors,ctx,grab,b,snapshot,life

    def test_local_idempotent_invalidation_frozen_snapshot_and_post_clear_refresh(self):
        builder,p,sensors,ctx,grab,b,frozen,life=self.setup_surface()
        frozen_observed={v.index for v in frozen.cells if v.observed}
        e=Evidence.at(10.1,3,calibration_id=ctx.calibration_id)
        self.assertTrue(life.interact('binding:interaction',10.1,3,(),e,10.1))
        generation=builder.grid.generation
        self.assertFalse(life.interact('binding:interaction',10.1,3,(),e,10.1))
        self.assertEqual(builder.grid.generation,generation)
        disturbed=builder.grid.snapshot(10.11)
        self.assertTrue(all(not disturbed.cells[i].observed for i in life.affected))
        self.assertTrue(any(frozen.cells[i].observed for i in life.affected))
        self.assertEqual({v.index for v in frozen.cells if v.observed},frozen_observed)
        unaffected={v.index for v in disturbed.cells if v.observed};self.assertTrue(unaffected)
        # While occluded, even newer points cannot revive the affected cells.
        keep=life.filter_points(p,np.full(len(p),10.15))
        self.assertTrue(ingest(builder,p[keep],sensors[keep],ctx,grab,10.15,3))
        self.assertFalse(life.refresh(builder.grid.snapshot(10.16),10.16).ready)
        life.allow_observation(b.binding_id,b.binding_digest_sha256,
            Evidence.at(10.2,4,calibration_id=ctx.calibration_id),.5,.3,10.2)
        old=life.filter_points(p,np.full(len(p),10.1))
        self.assertLess(np.count_nonzero(old),len(p))
        for epoch,counter in ((10.25,4),(10.3,5)):
            keep=life.filter_points(p,np.full(len(p),epoch))
            self.assertTrue(keep.all())
            raised=replace(grab,bottom_lower_bound_m=3.,top_upper_bound_m=4.)
            self.assertTrue(ingest(builder,p[keep],sensors[keep],ctx,raised,epoch,counter))
        refreshed=life.refresh(builder.grid.snapshot(10.31),10.31)
        self.assertTrue(refreshed.ready)
        self.assertGreater(refreshed.surface_generation,b.surface_generation)
        self.assertIs(life.frozen_snapshot,frozen)

    def test_swept_area_extends_local_mask_without_clearing_whole_pit(self):
        builder,p,sensors,ctx,grab,b,frozen,life=self.setup_surface()
        e=Evidence.at(10.1,3,calibration_id=ctx.calibration_id)
        life.interact('binding:interaction',10.1,3,(),e,10.1);first=len(life.affected)
        swept=(((1.,.5),(1.5,.5),(1.5,1.),(1.,1.)),)
        life.interact('binding:interaction',10.1,3,swept,replace(e,epoch_ns=10_150_000_000,counter=4),10.15)
        self.assertGreater(len(life.affected),first)
        self.assertLess(len(life.affected),builder.grid.pit_active_cell_count)

    def test_uncalibrated_physical_and_stale_wrong_binding_clear_rejected(self):
        builder,p,sensors,ctx,grab,b,frozen,life=self.setup_surface()
        e=Evidence.at(10.1,3,calibration_id=ctx.calibration_id)
        life.interact('binding:interaction',10.1,3,(),e,10.1)
        with self.assertRaises(ValueError):life.allow_observation('wrong',b.binding_digest_sha256,e,1.,1.,10.1)
        with self.assertRaises(ValueError):life.allow_observation(b.binding_id,b.binding_digest_sha256,e,.01,.01,10.1)
        with self.assertRaises(ValueError):life.interact('other',10.1,3,(),e,10.1)
        life.config.update(source_type='PHYSICAL',config_state='NOT_CONFIGURED')
        with self.assertRaisesRegex(ValueError,'PHYSICAL_INTERACTION_NOT_CONFIGURED'):
            life.interact('binding:interaction',10.1,3,(),replace(e,source_type='PHYSICAL'),10.1)
