"""FIELD-readiness is software-only; every numeric fixture is NON_PRODUCTION."""
import copy
from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
from shapely.geometry import Point, Polygon
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tests_static'),str(ROOT/'src/chili_crane_control/src'),str(ROOT/'src/chili_crane_perception/src'),
             str(ROOT/'src/chili_crane_slam/src')]
from test_phase2b_planning import planning_fixture
from test_phase2b_surface import cloud,add
from test_phase2b_mask import mask_fixture
from chili_crane_perception.pit_surface import PitSurfaceGrid
from chili_crane_perception.surface_mask import PitSurfaceBuilder
from chili_crane_perception.surface_payload import sparse_cells,expand_sparse
from chili_crane_control.grasp_planning import GraspPlanner,rank_candidates


class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.c,self.s,self.ctx,self.geo=planning_fixture()

    def populated(self,c=None,s=None,height=lambda x,y:1.):
        grid=PitSurfaceGrid(c or self.c,s or self.s)
        p,sensors=cloud(grid,height)
        self.assertTrue(add(grid,p,sensors,self.ctx))
        self.assertTrue(add(grid,p,sensors,self.ctx,10.05,2))
        return grid,grid.snapshot(10.06)

    def test_lineage_excludes_frame_evidence_but_not_true_identity(self):
        t=np.eye(4);t[2,3]=.0002
        different=replace(self.ctx,measurement_epoch=11.,uncertainty_m=.02,transform=tuple(t.ravel()),
                          transform_id='frame-2',motion_evidence_id='motion-2',crane_pose_evidence_id='pose-2')
        self.assertEqual(self.ctx.lineage_key,different.lineage_key)
        for key,value in dict(calibration_id='chain-2',field_extrinsic_id='extrinsic-2',axis_to_map_id='axis-2',
                              map_id='map-2',semantic_version='v2',pit_id='pit-2',sensor_calibration_ids=('new','s1')).items():
            self.assertNotEqual(self.ctx.lineage_key,replace(self.ctx,**{key:value}).lineage_key)

    def test_stationary_jitter_fuses_and_expands_height_uncertainty(self):
        for kind in ('translation','rotation','uncertainty','motion'):
            g=PitSurfaceGrid(self.c,self.s);p,sensors=cloud(g)
            self.assertTrue(add(g,p,sensors,self.ctx))
            t=np.eye(4)
            if kind=='translation':t[2,3]=.0005
            if kind=='rotation':
                a=.0005;t[:3,:3]=[[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]]
            ctx=replace(self.ctx,transform=tuple(t.ravel()),uncertainty_m=.02 if kind=='uncertainty' else .01,
                        motion_evidence_id='next',crane_pose_evidence_id='next',transform_id='next-frame')
            self.assertTrue(add(g,p,sensors,ctx,10.05,2),g.reason)
            self.assertEqual(len(g.frames),2,kind)
            center=g.snapshot(10.06).cells[100]
            self.assertTrue(center.observed)
            self.assertGreater(center.height_uncertainty,.02)
            self.assertLess(center.z_lower_bound,center.z_median)

    def test_identity_changes_reset_and_pose_walk_cannot_evade_reference(self):
        for key in ('calibration_id','field_extrinsic_id','axis_to_map_id'):
            g,snap=self.populated();p,sensors=cloud(g)
            self.assertTrue(add(g,p,sensors,replace(self.ctx,**{key:'changed'}),10.1,3))
            self.assertEqual(len(g.frames),1)
            self.assertFalse(any(v.observed for v in g.snapshot(10.11).cells))
        for key in ('map_id','pit_id','semantic_version'):
            g,_=self.populated();p,sensors=cloud(g)
            self.assertFalse(add(g,p,sensors,replace(self.ctx,**{key:'changed'}),10.1,3))
            self.assertFalse(g.frames)
        g,_=self.populated();p,sensors=cloud(g)
        for i,delta in enumerate((.0008,.0016),3):
            t=np.eye(4);t[2,3]=delta
            ok=add(g,p,sensors,replace(self.ctx,transform=tuple(t.ravel())),10+i*.02,i)
            self.assertEqual(ok,i==3)
        self.assertEqual(g.reason,'SPATIAL_MOTION_EXCEEDS_STATIONARY_WINDOW')

    def test_fresh_but_not_contemporaneous_grab_rejected(self):
        c,s,ctx,grab=mask_fixture();b=PitSurfaceBuilder(c,s);p,sensors=cloud(b.grid)
        frame=dict(epoch=10.,source_counter=1,frame_id='cloud',span_sec=.02,source_type='SYNTHETIC',calibration_id=ctx.calibration_id)
        self.assertFalse(b.ingest(p,sensors,frame,ctx,replace(grab,epoch=9.9),10.01))
        self.assertEqual(b.grid.reason,'GRAB_MASK_TIME_ALIGNMENT_REJECTED')
        self.assertFalse(b.grid.frames)
        self.assertEqual(b.grid.snapshot(10.02).reason,'GRAB_MASK_TIME_ALIGNMENT_REJECTED')

    def test_distinct_chain_and_extrinsic_correct_but_mismatch_rejected(self):
        c,s,ctx,grab=mask_fixture();ctx=replace(ctx,calibration_id='full-chain',field_extrinsic_id=grab.calibration_id)
        for mismatch in ('none','extrinsic','chain'):
            b=PitSurfaceBuilder(c,s);p,sensors=cloud(b.grid)
            frame=dict(epoch=10.,source_counter=1,frame_id='cloud',span_sec=.02,source_type='SYNTHETIC',
                       calibration_id='wrong' if mismatch=='chain' else 'full-chain')
            active=replace(grab,calibration_id='wrong') if mismatch=='extrinsic' else grab
            self.assertEqual(b.ingest(p,sensors,frame,ctx,active,10.01),mismatch=='none')

    def test_best_late_mound_and_permuted_traversal(self):
        c=copy.deepcopy(self.c);c['maximum_candidates']=4;c['kernel']['capture_depth_m']=2.
        c['planning']['maximum_slope_rad']=1.5
        grid,snap=self.populated(c,height=lambda x,y:.3+.6*np.exp(-((x-1.1)**2+(y-1.1)**2)))
        results=[]
        for cells in (snap.cells,tuple(reversed(snap.cells)),tuple(np.random.default_rng(18).permutation(snap.cells))):
            planner=GraspPlanner(c,self.geo)
            result=planner.plan(replace(snap,cells=cells),10.06,snap.context,snap.generation)
            self.assertIsNotNone(result.selected)
            results.append(result.selected.candidate_id)
            self.assertGreater(result.selected.target_xy[0],.5)
            self.assertGreater(result.selected.target_xy[1],.5)
            self.assertEqual(planner.metrics['candidate_centers_scanned'],256)
            self.assertTrue(result.search_complete)
            self.assertFalse(result.global_optimum_proven)
        self.assertEqual(len(set(results)),1)

    def test_truncated_search_is_never_advertised_as_optimum(self):
        c=copy.deepcopy(self.c);c['planning']['maximum_evaluated_cells']=10
        _,snap=self.populated(c);result=GraspPlanner(c,self.geo).plan(snap,10.06,snap.context,snap.generation)
        self.assertFalse(result.search_complete)
        self.assertFalse(result.global_optimum_proven)
        self.assertIn('SEARCH_TRUNCATED',result.reason)
        self.assertIn('GLOBAL_OPTIMUM_NOT_PROVEN',result.reason)

    def test_empty_floor_reserve_and_material_gate(self):
        for height in (.0,.03,.06):
            _,snap=self.populated(height=lambda x,y:height)
            result=GraspPlanner(self.c,self.geo).plan(snap,10.06,snap.context,snap.generation)
            self.assertIsNone(result.selected)
            self.assertTrue(any('MATERIAL_INSUFFICIENT' in c.rejection_reasons
                                or 'FLOOR_RESERVE_INSUFFICIENT' in c.rejection_reasons for c in result.candidates))

    def test_near_volume_recent_grab_loses_but_large_volume_wins(self):
        _,snap=self.populated();result=GraspPlanner(self.c,self.geo).plan(snap,10.06,snap.context,snap.generation)
        a=result.selected
        b=replace(a,candidate_id='alternative',recent_grab_penalty=0.,material_volume_proxy_lower_bound_m3=.0995)
        a=replace(a,recent_grab_penalty=1.,material_volume_proxy_lower_bound_m3=.1)
        self.assertEqual(rank_candidates((a,b),.001)[0],b)
        a=replace(a,material_volume_proxy_lower_bound_m3=.12)
        self.assertEqual(rank_candidates((a,b),.001)[0],a)

    def test_esdf_really_skips_exact_checks_and_optimism_never_grants_safety(self):
        c=copy.deepcopy(self.c);c['planning']['distance_acceleration_enabled']=True
        _,snap=self.populated(c)
        with patch('chili_crane_control.distance_acceleration.RejectionDistanceField.definitely_reject',return_value=True):
            planner=GraspPlanner(c,self.geo);result=planner.plan(snap,10.06,snap.context,snap.generation)
            self.assertIsNone(result.selected)
            self.assertGreater(planner.metrics['esdf_reject_count'],0)
            self.assertEqual(planner.metrics['exact_geometry_check_count'],0)
        c['resolution_m']=.5
        _,snap=self.populated(c)
        with patch('chili_crane_control.distance_acceleration.RejectionDistanceField.definitely_reject',return_value=False):
            planner=GraspPlanner(c,self.geo);result=planner.plan(snap,10.06,snap.context,snap.generation)
            for candidate in result.candidates:
                if candidate.accepted:
                    fp=Polygon(candidate.footprint_xy)
                    self.assertTrue(self.geo.polygon.covers(fp))
                    self.assertGreaterEqual(candidate.wall_clearance_m,.2-1e-9)

    def test_l_shaped_active_coverage_not_bbox(self):
        s=copy.deepcopy(self.s);s['pits'][0]['polygon_map_m']=[[-2,-2],[2,-2],[2,0],[0,0],[0,2],[-2,2]]
        grid,snap=self.populated(s=s)
        self.assertEqual(grid.pit_active_cell_count,192)
        self.assertEqual(snap.observed_cell_count,192)
        self.assertEqual(snap.observed_cell_count/snap.pit_active_cell_count,1.)
        self.assertLess(snap.support_qualified_cell_count,snap.observed_cell_count)

    def test_real_esdf_reject_reduces_expensive_geometry_work(self):
        c=copy.deepcopy(self.c);c['planning']['distance_acceleration_enabled']=True
        # The frozen geofence already excludes the closest wall centers. A larger
        # NON_PRODUCTION kernel creates genuine ESDF rejects inside that domain.
        c['kernel']['footprint_xy_m']=[[-1.5,-1.5],[1.5,-1.5],[1.5,1.5],[-1.5,1.5]]
        _,snap=self.populated(c);planner=GraspPlanner(c,self.geo)
        result=planner.plan(snap,10.06,snap.context,snap.generation)
        self.assertGreater(planner.metrics['esdf_reject_count'],0)
        self.assertLess(planner.metrics['exact_geometry_check_count'],planner.metrics['esdf_checked_count'])
        for candidate in result.candidates:
            if candidate.accepted:self.assertGreaterEqual(candidate.wall_clearance_m,.2-1e-9)

    def test_sparse_capacities_unknown_implicit_and_bad_indices_rejected(self):
        for resolution in (.5,.25,.125):
            c=copy.deepcopy(self.c);c['resolution_m']=resolution;c['maximum_grid_cells']=1024
            grid=PitSurfaceGrid(c,self.s);p,sensors=cloud(grid)
            keep=p[:,0]>0
            add(grid,p[keep],sensors[keep],self.ctx);add(grid,p[keep],sensors[keep],self.ctx,10.05,2)
            snap=grid.snapshot(10.06);sparse=sparse_cells(snap)
            self.assertEqual(len(sparse),grid.width*grid.height//2)
            expanded=expand_sparse(sparse,snap.width,snap.height,c['maximum_grid_cells'])
            self.assertEqual(sum(v.observed for v in expanded),len(sparse))
            self.assertTrue(all(np.isnan(v.z_median) for v in expanded if not v.observed))
            for invalid in (sparse+(sparse[0],),(replace(sparse[0],index=1024),),
                            (replace(sparse[0],observed=False),)):
                with self.assertRaises(ValueError):expand_sparse(invalid,snap.width,snap.height,1024)

    def test_generation_cache_age_and_disturbance_key(self):
        grid,snap=self.populated();planner=GraspPlanner(self.c,self.geo)
        first=planner.plan(snap,10.06,snap.context,snap.generation)
        for now in (10.07,10.08,10.09):
            self.assertIs(planner.plan(snap,now,snap.context,snap.generation),first)
            self.assertIs(grid.snapshot(now),snap)
        self.assertEqual(planner.compute_count,1)
        self.assertEqual(planner.cache_hit_count,3)
        self.assertIsNone(planner.plan(snap,11.,snap.context,snap.generation).selected)
        self.assertEqual(planner.compute_count,1)
        new=replace(snap,snapshot_id='new-disturbance',generation=snap.generation+1,disturbance_counter=1)
        planner.plan(new,10.09,new.context,new.generation)
        self.assertEqual(planner.compute_count,2)

    def test_production_new_numbers_are_unconfigured(self):
        import yaml
        c=yaml.safe_load((ROOT/'config/perception/phase2b_surface.template.yaml').read_text(encoding='utf-8'))
        for key in ('maximum_stationary_transform_translation_delta_m','maximum_stationary_transform_rotation_delta_rad',
                    'maximum_grab_mask_time_skew_sec'):
            self.assertIsNone(c[key])
        for key in ('maximum_evaluated_cells','minimum_material_volume_proxy_lower_bound_m3',
                    'minimum_conservative_material_height_above_pit_bottom_m','volume_equivalence_epsilon'):
            self.assertIsNone(c['planning'][key])

    def test_publication_rechecks_age_after_slow_compute_without_renewing_epoch(self):
        _,snap=self.populated();planner=GraspPlanner(self.c,self.geo)
        result=planner.plan(snap,10.06,snap.context,snap.generation)
        self.assertIsNotNone(result.selected)
        self.assertIs(planner.publication_gate(result,snap,10.07,.02,snap.context),result)
        # Time advances while computing; source epoch is not changed to hide delay.
        for now,age,reason in ((10.07,.6,'SURFACE_STALE_AT_PUBLICATION'),
                               (10.3,.25,'SPATIAL_CONTEXT_STALE_AT_PUBLICATION'),
                               (10.07,float('nan'),'SURFACE_STALE_AT_PUBLICATION')):
            rejected=planner.publication_gate(result,snap,now,age,snap.context)
            self.assertIsNone(rejected.selected)
            self.assertEqual(rejected.measurement_epoch,result.measurement_epoch)
            self.assertEqual(rejected.reason,reason)

    def test_ros_qos_and_sparse_cdr_bounds(self):
        try:
            import rclpy
            from chili_crane_msgs.msg import PitSurfaceSnapshot
        except ImportError:self.skipTest('NOT_RUN: generated ROS2 messages unavailable on Windows')
        from rclpy.qos import ReliabilityPolicy,DurabilityPolicy
        from rclpy.serialization import serialize_message
        from chili_crane_perception.surface_runtime import QOS,SNAPSHOT_QOS,PROPOSAL_QOS,EVENT_QOS
        from chili_crane_perception.surface_transport import snapshot_to_message,snapshot_from_message
        self.assertEqual(QOS.reliability,ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(SNAPSHOT_QOS.durability,DurabilityPolicy.TRANSIENT_LOCAL)
        for qos in (SNAPSHOT_QOS,PROPOSAL_QOS,EVENT_QOS):
            self.assertEqual(qos.depth,1);self.assertEqual(qos.reliability,ReliabilityPolicy.RELIABLE)
        for r in (.5,.25,.125):
            c=copy.deepcopy(self.c);c['resolution_m']=r
            grid,snap=self.populated(c)
            msg=snapshot_to_message(snap,10.06,2)
            self.assertEqual(len(msg.cells),snap.observed_cell_count)
            self.assertLess(len(serialize_message(msg)),16384+192*len(msg.cells))
            decoded=snapshot_from_message(msg,c)
            self.assertEqual(decoded.pit_active_cell_count,snap.pit_active_cell_count)
            msg.cell_encoding='DENSE_V1'
            with self.assertRaises(ValueError):snapshot_from_message(msg,c)
