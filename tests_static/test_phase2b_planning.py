"""B3 exact footprint/coverage/provenance gates before explainable ranking."""
import copy
from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/chili_crane_control/src'))
from chili_crane_control.geofence import OperationalGeofence
from chili_crane_control.grasp_planning import GraspPlanner
from test_operational_geofence import geofence_fixture
from test_phase2b_surface import fixture,cloud,add
from chili_crane_perception.pit_surface import PitSurfaceGrid


def planning_fixture():
    c,s,ctx=fixture()
    gc,gs,_,_,_=geofence_fixture()
    s.update(track=gs['track'])
    gc.update(semantic_map_id=s['map_id'],semantic_version=s['semantic_version'],pit_id=c['pit_id'])
    gc['axis_to_map'].update(map_id=s['map_id'],semantic_version=s['semantic_version'],pit_id=c['pit_id'])
    gc['static_margin'].update(grab_horizontal_radius_m=.3,maximum_swing_m=.02,position_uncertainty_m=.02,
        calibration_uncertainty_m=.02,stop_margin_m=.02,additional_margin_m=.02)
    c.update(maximum_candidates=256,kernel=dict(state='SYNTHETIC_NON_PRODUCTION',kernel_id='kernel1',
        provenance='synthetic-model-v1',footprint_xy_m=[[-.25,-.25],[.25,-.25],[.25,.25],[-.25,.25]],
        capture_depth_m=.3,yaw_control_available=False,yaw_mode='FIXED_MECHANICAL',fixed_yaw_rad=0.),
        planning=dict(minimum_observed_fraction=1.,maximum_unknown_fraction=0.,maximum_slope_rad=.6,
            maximum_roughness_m=.1,wall_margin_m=.1,distance_acceleration_enabled=False,
            disturbance_dilation_m=.05,recent_grab_radius_m=.75,recent_grab_ttl_sec=3.,maximum_recent_events=4))
    return c,s,ctx,OperationalGeofence(gc,s)


class PlanningTests(unittest.TestCase):
    def setUp(self):
        self.c,self.s,self.ctx,self.geofence=planning_fixture()
        self.grid=PitSurfaceGrid(self.c,self.s)
        self.planner=GraspPlanner(self.c,self.geofence)

    def snapshot(self,height=lambda x,y:1.):
        p,sensors=cloud(self.grid,height)
        self.assertTrue(add(self.grid,p,sensors,self.ctx))
        self.assertTrue(add(self.grid,p,sensors,self.ctx,10.05,2))
        return self.grid.snapshot(10.06)

    def plan(self,snapshot,**kw):
        return self.planner.plan(snapshot,kw.get('now',10.06),kw.get('context',snapshot.context),
                                 kw.get('generation',snapshot.generation))

    def test_flat_candidates_exact_and_not_control(self):
        snap=self.snapshot();proposal=self.plan(snap)
        self.assertIsNotNone(proposal.selected)
        self.assertFalse(proposal.valid_for_control)
        for candidate in proposal.candidates:
            if candidate.accepted:
                self.assertTrue(self.geofence.safe_region.covers(__import__('shapely').geometry.Point(candidate.target_xy)))
                self.assertGreaterEqual(candidate.wall_clearance_m,.2-1e-10)
                self.assertLessEqual(candidate.material_volume_proxy_lower_bound_m3,candidate.material_volume_proxy_m3)
                self.assertAlmostEqual(candidate.observed_area_fraction,1.)

    def test_hole_cannot_be_filled_by_height_or_score(self):
        snap=self.snapshot()
        snap=replace(snap,cells=tuple(replace(c,observed=False) for c in snap.cells))
        self.assertIsNone(self.plan(snap).selected)

    def test_partial_coverage_hard_rejected(self):
        snap=self.snapshot()
        cells=tuple(replace(c,observed=False) if c.index%2==0 else c for c in snap.cells)
        result=self.plan(replace(snap,cells=cells))
        self.assertIsNone(result.selected)
        self.assertTrue(any('UNKNOWN_OR_INSUFFICIENT_OBSERVED_COVERAGE' in c.rejection_reasons for c in result.candidates))

    def test_stale_generation_and_spatial_mismatch(self):
        snap=self.snapshot()
        for kw in (dict(now=11.),dict(generation=snap.generation+1),dict(context=replace(snap.context,map_id='other'))):
            self.assertIsNone(self.plan(snap,**kw).selected)

    def test_big_footprint_cannot_escape_exact_wall_gate(self):
        self.c['kernel']['footprint_xy_m']=[[-3,-3],[3,-3],[3,3],[-3,3]]
        self.grid=PitSurfaceGrid(self.c,self.s)
        self.planner=GraspPlanner(self.c,self.geofence)
        result=self.plan(self.snapshot())
        self.assertIsNone(result.selected)
        self.assertTrue(all('WALL_OR_PIT_GEOMETRY' in c.rejection_reasons for c in result.candidates))

    def test_excess_slope_and_invalid_height_bounds_rejected(self):
        snap=self.snapshot()
        for changes in (dict(slope=1.),dict(z_lower_bound=5.),dict(height_uncertainty=float('nan'))):
            result=self.plan(replace(snap,cells=tuple(replace(c,**changes) for c in snap.cells)))
            self.assertIsNone(result.selected)

    def test_yaw_and_physical_kernel_not_guessed(self):
        for changes in (dict(yaw_control_available=True),dict(state='NOT_CONFIGURED')):
            c=copy.deepcopy(self.c);c['kernel'].update(changes)
            with self.assertRaises(ValueError):
                GraspPlanner(c,self.geofence)

    def test_candidate_count_is_bounded(self):
        self.c['maximum_candidates']=3
        self.grid=PitSurfaceGrid(self.c,self.s)
        self.planner=GraspPlanner(self.c,self.geofence)
        self.assertLessEqual(len(self.plan(self.snapshot()).candidates),3)
