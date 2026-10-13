"""B4 invalidation is hard truth removal; penalty never substitutes for it."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
from shapely.geometry import Polygon
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests_static'))
sys.path.insert(0,str(ROOT/'src/chili_crane_control/src'))
sys.path.insert(0,str(ROOT/'src/chili_crane_perception/src'))
from chili_crane_perception.pit_surface import PitSurfaceGrid
from chili_crane_perception.surface_disturbance import SurfaceDisturbanceTracker,SurfaceGraspEvent
from chili_crane_control.grasp_planning import GraspPlanner,rank_candidates
from test_phase2b_planning import planning_fixture
from test_phase2b_surface import cloud,add


class DisturbanceTests(unittest.TestCase):
    def setUp(self):
        self.c,self.s,self.ctx,self.g=planning_fixture()
        self.grid=PitSurfaceGrid(self.c,self.s)
        self.tracker=SurfaceDisturbanceTracker(self.c)
        self.p,self.sensors=cloud(self.grid)
        add(self.grid,self.p,self.sensors,self.ctx)
        add(self.grid,self.p,self.sensors,self.ctx,10.05,2)
        self.event=SurfaceGraspEvent('event1',1,10.1,'SYNTHETIC','map1','v1','pit','transform1','kernel1',
                                     ((-.5,-.5),(.5,-.5),(.5,.5),(-.5,.5)))

    def test_immediate_invalidation_and_reobservation_only(self):
        old=self.grid.snapshot(10.06)
        self.tracker.apply(self.grid,self.event,10.11)
        new=self.grid.snapshot(10.11)
        self.assertEqual(new.last_disturbance_id,self.event.event_id)
        self.assertEqual(new.disturbance_counter,self.event.source_counter)
        self.assertGreater(new.generation,old.generation)
        self.assertFalse(new.cells[8*new.width+8].observed)
        planner=GraspPlanner(self.c,self.g)
        self.assertIsNone(planner.plan(old,10.11,old.context,new.generation).selected)
        # A late frame newer than the previous scan but before the event stays invalid.
        self.assertTrue(add(self.grid,self.p,self.sensors,self.ctx,10.08,3))
        self.assertFalse(self.grid.snapshot(10.12).cells[8*new.width+8].observed)
        add(self.grid,self.p,self.sensors,self.ctx,10.15,4)
        add(self.grid,self.p,self.sensors,self.ctx,10.2,5)
        self.assertTrue(self.grid.snapshot(10.21).cells[8*new.width+8].observed)

    def test_duplicate_future_physical_mismatched_events_rejected(self):
        for event in (replace(self.event,source_type='PHYSICAL'),replace(self.event,map_id='other'),
                      replace(self.event,epoch=11.)):
            with self.assertRaises(ValueError):
                self.tracker.apply(self.grid,event,10.11)
        self.tracker.apply(self.grid,self.event,10.11)
        with self.assertRaises(ValueError):
            self.tracker.apply(self.grid,self.event,10.12)

    def test_recent_penalty_bounded_context_and_expiry(self):
        self.tracker.apply(self.grid,self.event,10.11)
        fp=Polygon(self.event.footprint_xy)
        self.assertGreater(self.tracker.penalty(fp,10.12,self.ctx.key),0)
        self.assertEqual(self.tracker.penalty(fp,10.12,'other-context'),0)
        self.assertEqual(self.tracker.penalty(fp,14.),0)

    def test_ranking_components_are_explainable(self):
        snap=self.grid.snapshot(10.06)
        result=GraspPlanner(self.c,self.g).plan(snap,10.06,snap.context,snap.generation,self.tracker)
        best=rank_candidates(result.candidates,self.c['planning']['volume_equivalence_epsilon'])[0]
        self.assertEqual(result.selected,best)

    def test_esdf_cannot_skip_exact_polygon_gate(self):
        self.c['planning']['distance_acceleration_enabled']=True
        self.c['kernel']['footprint_xy_m']=[[-3,-3],[3,-3],[3,3],[-3,3]]
        self.grid=PitSurfaceGrid(self.c,self.s)
        add(self.grid,self.p,self.sensors,self.ctx)
        add(self.grid,self.p,self.sensors,self.ctx,10.05,2)
        snapshot=self.grid.snapshot(10.06)
        # Worst optimistic accelerator: it never rejects. Exact geometry still must.
        with patch('chili_crane_control.distance_acceleration.RejectionDistanceField.definitely_reject',return_value=False):
            result=GraspPlanner(self.c,self.g).plan(snapshot,10.06,snapshot.context,snapshot.generation)
        self.assertIsNone(result.selected)
        self.assertTrue(all('WALL_OR_PIT_GEOMETRY' in c.rejection_reasons for c in result.candidates))
