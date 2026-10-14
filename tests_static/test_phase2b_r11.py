"""Two P1 closeouts only: real scan time and producer lifecycle identity."""
import ast
from dataclasses import replace
from pathlib import Path
import sys
import unittest
import uuid
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tests_static'),str(ROOT/'src/chili_crane_control/src'),
             str(ROOT/'src/chili_crane_perception/src'),str(ROOT/'src/chili_crane_slam/src')]
from test_phase2b_mask import mask_fixture
from test_phase2b_surface import cloud,add
from test_phase2b_planning import planning_fixture
from chili_crane_perception.surface_time import scan_epochs
from chili_crane_perception.surface_session import SurfaceSessionGate,snapshot_identity
from chili_crane_perception.surface_mask import PitSurfaceBuilder
from chili_crane_perception.pit_surface import PitSurfaceGrid
from chili_crane_control.grasp_planning import GraspPlanner


class SealTests(unittest.TestCase):
    def scan_case(self,span=.1,phase=0.,grab_basis='mid',bad=None):
        c,s,ctx,grab=mask_fixture();c['maximum_frame_span_sec']=.12
        b=PitSurfaceBuilder(c,s);p,sensors=cloud(b.grid)
        # True separate sensor scans, exactly as the frozen merger concatenates.
        order=np.argsort(sensors,kind='stable');p,sensors=p[order],sensors[order]
        times=np.empty(len(p))
        for sensor in (0,1):
            ids=np.flatnonzero(sensors==sensor)
            times[ids]=np.linspace(10.+sensor*phase,10.+sensor*phase+span,len(ids))
        header=10.+phase
        if bad=='regression':times[5]=times[4]-.01
        if bad=='nonfinite':times[5]=np.nan
        try:
            t=scan_epochs(header,times,sensors)
        except ValueError:
            if bad:return None
            raise
        epoch=t.mid_epoch if grab_basis=='mid' else header if grab_basis=='header' else 9.95
        frame=dict(epoch=header,mid_epoch=t.mid_epoch,span_sec=t.span_sec,source_counter=1,
                   frame_id='cloud',source_type='SYNTHETIC',calibration_id=ctx.calibration_id)
        accepted=b.ingest(p,sensors,frame,replace(ctx,measurement_epoch=header),replace(grab,epoch=epoch),10.12)
        return accepted,b,t

    def test_first_point_header_and_true_mid_80_and_100ms_pass(self):
        for span in (.08,.1):
            ok,b,t=self.scan_case(span)
            self.assertTrue(ok,b.grid.reason)
            self.assertEqual(t.header_epoch,10.)
            self.assertAlmostEqual(t.mid_epoch,10.+span/2)
            self.assertEqual(b.grid.last_epoch,10.,'header identity must not be silently changed')
            self.assertEqual(b.grid.context.measurement_epoch,10.)

    def test_first_point_grab_and_old_fresh_grab_fail(self):
        for basis in ('header','old'):
            ok,b,t=self.scan_case(grab_basis=basis)
            self.assertFalse(ok)
            self.assertEqual(b.grid.reason,'GRAB_MASK_TIME_ALIGNMENT_REJECTED')
            self.assertGreater(t.mid_epoch-t.header_epoch,b.grid.config['maximum_grab_mask_time_skew_sec'])

    def test_dual_scan_phase_uses_merged_min_max_not_either_header(self):
        ok,b,t=self.scan_case(span=.09,phase=.004)
        self.assertTrue(ok,b.grid.reason)
        self.assertAlmostEqual(t.start_epoch,10.)
        self.assertAlmostEqual(t.end_epoch,10.094)
        self.assertAlmostEqual(t.mid_epoch,10.047)
        self.assertNotEqual(t.mid_epoch,t.header_epoch)

    def test_point_time_regression_nonfinite_and_bad_header_rejected(self):
        for bad in ('regression','nonfinite'):
            self.assertIsNone(self.scan_case(bad=bad))
        with self.assertRaisesRegex(ValueError,'HEADER_OUTSIDE_POINT_TIME_RANGE'):
            scan_epochs(9.,[10.,10.1],[0,0])

    def test_mid_epoch_missing_never_falls_back_to_header(self):
        c,s,ctx,grab=mask_fixture();b=PitSurfaceBuilder(c,s);p,sensors=cloud(b.grid)
        frame=dict(epoch=10.,span_sec=.02,source_counter=1,frame_id='cloud',source_type='SYNTHETIC',calibration_id=ctx.calibration_id)
        self.assertFalse(b.ingest(p,sensors,frame,ctx,grab,10.01))
        self.assertFalse(b.grid.frames)

    def test_restart_identity_unique_same_generation_and_same_config(self):
        c,s,ctx,g=planning_fixture();a=PitSurfaceGrid(c,s);b=PitSurfaceGrid(c,s)
        p,sensors=cloud(a)
        for grid in (a,b):
            add(grid,p,sensors,ctx);add(grid,p,sensors,ctx,10.05,2)
        sa,sb=a.snapshot(10.06),b.snapshot(10.06)
        self.assertEqual(sa.generation,sb.generation)
        self.assertNotEqual(sa.surface_session_id,sb.surface_session_id)
        self.assertNotEqual(sa.snapshot_id,sb.snapshot_id)
        planner=GraspPlanner(c,g)
        pa=planner.plan(sa,10.06,sa.context,sa.generation)
        planner.reset_cache()
        self.assertIsNone(planner.cached_proposal)
        pb=planner.plan(sb,10.06,sb.context,sb.generation)
        self.assertNotEqual(pa.proposal_id,pb.proposal_id)
        self.assertEqual(pb.surface_session_id,sb.surface_session_id)
        self.assertEqual(planner.cache_key[0],sb.surface_session_id)

    def test_session_switch_resets_floor_137_to_1_and_retired_never_returns(self):
        gate=SurfaceSessionGate();a,b=uuid.uuid4().hex,uuid.uuid4().hex
        self.assertEqual(gate.begin(a),'NEW_SESSION');self.assertTrue(gate.accept(137,200))
        self.assertEqual(gate.begin(b),'NEW_SESSION')
        self.assertEqual(gate.generation_floor,-1)
        self.assertTrue(gate.accept(1,1))
        self.assertEqual(gate.begin(a),'RETIRED_SESSION_REPLAY')
        self.assertEqual(gate.session_id,b)
        self.assertFalse(gate.accept(0,2));self.assertFalse(gate.accept(1,2))
        self.assertTrue(gate.accept(2,2))

    def test_session_history_bounded_fail_closed_not_lru_replay(self):
        gate=SurfaceSessionGate()
        self.assertEqual(gate.begin(''),'INVALID_SESSION')
        for _ in range(129):self.assertEqual(gate.begin(uuid.uuid4().hex),'NEW_SESSION')
        self.assertEqual(gate.begin(uuid.uuid4().hex),'SESSION_HISTORY_CAPACITY_EXCEEDED')
        self.assertEqual(len(gate.retired),128)
        self.assertFalse(gate.accept(1,1))

    def test_invalid_higher_generation_is_also_a_replay_barrier(self):
        gate=SurfaceSessionGate();gate.begin(uuid.uuid4().hex)
        self.assertTrue(gate.note(1));self.assertTrue(gate.accept(1,1))
        self.assertTrue(gate.note(2))  # Higher STALE producer snapshot, not admitted.
        self.assertFalse(gate.note(1));self.assertFalse(gate.note(2))
        self.assertTrue(gate.note(3));self.assertTrue(gate.accept(3,3))

    def test_frozen_algorithms_and_physical_disturbance_unchanged(self):
        # Only lifecycle methods/identity fields may change in existing algorithms.
        import subprocess
        base='8ee41886dfb8d084b2b01d505b770c98bf2fefde'
        files={'src/chili_crane_perception/src/chili_crane_perception/pit_surface.py':
               ('PitSurfaceGrid.ingest','PitSurfaceGrid.expire','PitSurfaceGrid.invalidate'),
               'src/chili_crane_control/src/chili_crane_control/grasp_planning.py':
               ('rank_candidates','GraspCaptureKernel.configure','GraspCaptureKernel.footprint'),
               'src/chili_crane_perception/src/chili_crane_perception/surface_disturbance.py':
               ('SurfaceDisturbanceTracker.apply','SurfaceDisturbanceTracker.penalty'),
               'src/chili_crane_control/src/chili_crane_control/distance_acceleration.py':
               ('RejectionDistanceField.definitely_reject',)}
        def functions(text):
            tree=ast.parse(text);found={}
            for item in tree.body:
                if isinstance(item,ast.FunctionDef):found[item.name]=ast.dump(item,include_attributes=False)
                if isinstance(item,ast.ClassDef):
                    for method in item.body:
                        if isinstance(method,ast.FunctionDef):found[item.name+'.'+method.name]=ast.dump(method,include_attributes=False)
            return found
        for path,names in files.items():
            before=functions(subprocess.check_output(['git','show',base+':'+path],cwd=ROOT,text=True,encoding='utf-8'))
            after=functions((ROOT/path).read_text(encoding='utf-8'))
            for name in names:self.assertEqual(before[name],after[name],name)

    def test_ros_identity_roundtrip_and_cross_field_session_rejected(self):
        try:
            from chili_crane_msgs.msg import PitSurfaceSnapshot
        except ImportError:self.skipTest('NOT_RUN: ROS2 generated messages unavailable on Windows')
        from chili_crane_perception.surface_transport import snapshot_to_message,snapshot_from_message,proposal_to_message
        c,s,ctx,g=planning_fixture();grid=PitSurfaceGrid(c,s);p,sensors=cloud(grid)
        add(grid,p,sensors,ctx);add(grid,p,sensors,ctx,10.05,2)
        snap=grid.snapshot(10.06);msg=snapshot_to_message(snap,10.06,2)
        decoded=snapshot_from_message(msg,c)
        self.assertEqual(decoded.surface_session_id,snap.surface_session_id)
        planner=GraspPlanner(c,g);proposal=planner.plan(decoded,10.06,decoded.context,decoded.generation)
        self.assertEqual(proposal_to_message(proposal,msg,.01).surface_session_id,snap.surface_session_id)
        msg.surface_session_id=uuid.uuid4().hex
        with self.assertRaisesRegex(ValueError,'IDENTITY'):snapshot_from_message(msg,c)
