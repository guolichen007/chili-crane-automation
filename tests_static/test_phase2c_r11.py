"""R1.1 deterministic counterexamples. No ROS or field acceptance claims."""
from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tests_static'),str(ROOT/'src/chili_crane_commissioning/src'),
             str(ROOT/'src/chili_crane_control/src'),str(ROOT/'src/chili_crane_perception/src')]
from test_phase2c_evidence import Harness,configuration
from test_phase2c_contracts import binding
from chili_crane_control.grasp_interaction import GraspInteractionEngine
from chili_crane_control.interaction_contracts import Evidence
from chili_crane_perception.phase2c_synthetic import SyntheticInteractionRun
from chili_crane_commissioning.recording import RunRecorder
from chili_crane_commissioning.bag import BagRecorder


def successor(b,now,**changes):
    return replace(b,cycle_id=b.cycle_id+':next',cycle_index=b.cycle_index+1,binding_id=b.binding_id+':next',
                   binding_epoch_ns=int(round(now*1e9)),evidence=replace(b.evidence,epoch_ns=int(round(now*1e9))),
                   binding_digest_sha256='',**changes)


class LifecycleTests(unittest.TestCase):
    def test_shadow_composition_and_production_isolation(self):
        import ast
        stack=ast.parse((ROOT/'src/chili_crane_bringup/launch/phase2c_shadow_stack.launch.py').read_text(encoding='utf-8'))
        calls=[n for n in ast.walk(stack) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)]
        nodes=[n for n in calls if n.func.id=='Node']
        executables={ast.literal_eval(k.value) for n in nodes for k in n.keywords if k.arg=='executable'}
        self.assertEqual(executables,{'pit_surface_shadow_node.py','grasp_proposal_shadow_node.py',
            'grasp_interaction_shadow_node.py','commissioning_console.py'})
        declarations={ast.literal_eval(n.args[0]):n for n in calls if n.func.id=='DeclareLaunchArgument'}
        self.assertEqual(ast.literal_eval(next(k.value for k in declarations['commissioning'].keywords
                                               if k.arg=='default_value')),'false')
        text=ast.unparse(stack)
        self.assertIn('interaction_config_file=interaction',text)
        self.assertNotIn('--enable-isolated-bench-output',text)
        for path in (ROOT/'src/chili_crane_bringup/launch').glob('*production*'):
            self.assertNotIn('commissioning',path.read_text(encoding='utf-8'))

    def test_payload_confirmed_while_refresh_pending_and_old_generation_rejected(self):
        h=Harness();h.warm();h.interact();r=h.lift(refresh=False)
        self.assertEqual(r.payload['state'],'PAYLOAD_CONFIRMED')
        self.assertFalse(r.lift['refresh_ready']);self.assertFalse(r.next_cycle_surface_ready)
        self.assertEqual(r.lift['updated_clear_state'],'UNKNOWN')
        self.assertTrue(all(r.diagnostics[k] for k in ('closure_confirmed','interaction_committed','bound_clear',
            'raise_seen','vertical_settled','load_baseline_valid','load_sensor_fresh','load_source_calibration_allowed',
            'settling_window_valid','payload_gate_pass')))
        h.engine.evaluate('ABORTED',None,None,None,None,h.t)
        with self.assertRaisesRegex(ValueError,'NEW_SURFACE'):h.engine.bind(successor(h.binding,h.t),h.t)

    def test_surface_restart_active_rejects_and_requires_abort(self):
        h=Harness();b=h.binding
        with self.assertRaisesRegex(ValueError,'ACTIVE_CYCLE_ABORT_REQUIRED'):
            h.engine.observe_surface_session('d'*32,Evidence.at(10.2),10.2)
        r=h.step();self.assertTrue(r.abort_required);self.assertIs(h.engine.binding,b)
        with self.assertRaisesRegex(ValueError,'ACTIVE_CYCLE'):
            h.engine.bind(successor(b,h.t,surface_session_id='d'*32,surface_snapshot_id='d'*32+':new'),h.t)

    def test_surface_restart_after_complete_and_abort_accepts_full_rescan_generation_reset(self):
        for terminal in ('COMPLETE','ABORTED'):
            h=Harness();h.warm();h.interact();h.lift(refresh=False)
            h.engine.evaluate(terminal,None,None,None,None,h.t)
            e=Evidence.at(h.t+.05)
            self.assertTrue(h.engine.observe_surface_session('d'*32,e,e.epoch))
            self.assertIsNone(h.engine.last_refresh);self.assertFalse(h.engine.committed)
            self.assertEqual(h.engine.last_phase,terminal)
            self.assertFalse(h.engine.evaluate('SCAN',None,None,None,None,e.epoch).abort_required)
            with self.assertRaisesRegex(ValueError,'NEW_SESSION'):h.engine.bind(successor(h.binding,e.epoch),e.epoch)
            fresh=successor(h.binding,e.epoch,surface_session_id='d'*32,surface_snapshot_id='d'*32+':new',surface_generation=1)
            h.engine.bind(fresh,e.epoch)
            self.assertEqual(h.engine.binding.surface_generation,1);self.assertFalse(h.engine.rescan_required)
            # Old-session refresh and proposal cannot cross the new binding.
            h.binding=fresh;h.t=e.epoch
            h.engine.evaluate('ABORTED',None,None,None,None,h.t)
            with self.assertRaisesRegex(ValueError,'RETIRED'):
                h.engine.observe_surface_session('b'*32,Evidence.at(h.t+.05),h.t+.05)

    def fill_run_ledger(self):
        c=configuration();engine=GraspInteractionEngine(c);initial=binding();limit=c['maximum_bindings']
        self.assertEqual(limit,1024)
        for i in range(limit):
            now=round(10.1+i*.01,2)
            b=replace(initial,cycle_id='task:'+str(i+1),cycle_index=i+1,binding_id='binding:'+str(i+1),
                binding_epoch_ns=int(round(now*1e9)),evidence=Evidence.at(now),binding_digest_sha256='')
            engine.bind(b,now);engine.evaluate('ABORTED',None,None,None,None,now)
        now=round(now+.01,2);overflow=successor(b,now)
        with self.assertRaisesRegex(ValueError,'CAPACITY'):engine.bind(overflow,now)
        self.assertEqual(len(engine.ledger.bindings),limit)
        return engine,overflow,now

    def test_same_run_ledger_overflow(self):
        engine,overflow,now=self.fill_run_ledger()
        self.assertEqual(len(engine.ledger.bindings),engine.config['maximum_bindings'])

    def test_new_run_after_terminal_rollover(self):
        engine,overflow,now=self.fill_run_ledger()
        new=replace(overflow,run_id='new-run',binding_digest_sha256='')
        engine.bind(new,now)
        self.assertEqual(len(engine.ledger.bindings),1);self.assertEqual(engine.active_run_id,'new-run')

    def test_run_swap_during_active_cycle_rejects_and_blocks_evidence(self):
        h=Harness();other=successor(h.binding,10.2,run_id='new-run')
        with self.assertRaisesRegex(ValueError,'RUN_SWAP_DURING_ACTIVE_CYCLE'):h.engine.bind(other,10.2)
        self.assertTrue(h.step().abort_required)
        self.assertEqual(h.engine.active_run_id,'run')


class VisibilityTests(unittest.TestCase):
    def run_visibility(self,visibility):
        run=SyntheticInteractionRun(configuration(),ROOT/'config/safety/pit_operational_geofence.template.yaml')
        run.warm();run.interact();run.visibility=visibility;run.lift()
        return run

    def test_partial_post_grab_visibility_candidate_local_recovery(self):
        run=self.run_visibility('partial')
        self.assertEqual(run.last_result.payload['state'],'PAYLOAD_CONFIRMED')
        r=run.refresh
        self.assertGreater(r.refreshed_cell_count,0);self.assertGreater(r.unknown_cell_count,0)
        self.assertEqual(r.affected_cell_count,r.refreshed_cell_count+r.unknown_cell_count)
        self.assertIsNone(run.proposal().selected)
        self.assertFalse(run.last_result.next_cycle_surface_ready)
        snap=run.life.planning_snapshot(run.builder.grid.snapshot(run.t),run.t)
        for i in run.life.affected:
            if snap.cells[i].observed:self.assertGreater(snap.cells[i].last_observed_epoch,run.life.observation_after)
            else:self.assertEqual(snap.cells[i].validity,'UNKNOWN')
        frozen=run.life.frozen_snapshot
        self.assertIsNone(run.planner.plan(frozen,run.t,snap.context,snap.generation).selected)
        run.visibility='recovery'
        for _ in range(8):run.step('RAISE',opening='CLOSED',closed=True,opened=False,value=101.)
        proposal=run.proposal();self.assertIsNotNone(proposal.selected)
        self.assertTrue(proposal.selected.accepted);self.assertGreater(proposal.generation,run.binding.surface_generation)
        self.assertGreater(run.refresh.unknown_cell_count,0)
        self.assertTrue(run.last_result.next_cycle_surface_ready)
        run.engine.evaluate('ABORTED',None,None,None,None,run.t)
        c=proposal.selected
        new=successor(run.binding,run.t,surface_generation=proposal.generation,
            surface_snapshot_id=proposal.snapshot_id,proposal_id=proposal.proposal_id,candidate_id=c.candidate_id,
            target_suspension_map=(*c.target_xy,0.),capture_footprint=c.footprint_xy,
            surface_nominal_m=c.surface_nominal_m,surface_lower_m=c.surface_lower_m,surface_upper_m=c.surface_upper_m)
        run.engine.bind(new,run.t)
        self.assertIs(run.life.frozen_snapshot,frozen)

    def test_post_grab_occlusion_then_recovery_never_empty(self):
        run=self.run_visibility('occluded')
        self.assertEqual(run.last_result.payload['state'],'PAYLOAD_CONFIRMED')
        self.assertFalse(run.refresh.ready);self.assertIsNone(run.proposal().selected)
        snap=run.life.planning_snapshot(run.builder.grid.snapshot(run.t),run.t)
        self.assertTrue(all(not snap.cells[i].observed and snap.cells[i].validity=='UNKNOWN' for i in run.life.affected))
        run.visibility='recovery'
        for _ in range(8):run.step('RAISE',opening='CLOSED',closed=True,opened=False,value=101.)
        self.assertIsNotNone(run.proposal().selected)

    def test_terminal_restart_actual_phase2b_full_scan_new_proposal_binding(self):
        from chili_crane_perception.surface_mask import PitSurfaceBuilder
        from chili_crane_perception.surface_interaction import SurfaceInteractionLifecycle
        from chili_crane_perception.phase2b_synthetic import ingest,fixture
        for terminal in ('COMPLETE','ABORTED'):
            run=self.run_visibility('occluded');old=run.binding;run.engine.evaluate(terminal,None,None,None,None,run.t)
            run.engine.observe_surface_session('d'*32,Evidence.at(run.t),run.t)
            _,semantic,_,ctx,grab=fixture(ROOT/'config/safety/pit_operational_geofence.template.yaml')
            run.builder=PitSurfaceBuilder(run.builder.grid.config,semantic,'d'*32)
            run.life=SurfaceInteractionLifecycle(run.builder.grid,configuration())
            for i in range(2):
                run.t+=.05
                self.assertTrue(ingest(run.builder,run.points,run.sensors,ctx,
                    replace(grab,bottom_lower_bound_m=4.,top_upper_bound_m=5.),run.t,i+1))
            run.t+=.01
            fresh=run.next_candidate();self.assertIsNotNone(fresh)
            self.assertEqual(fresh.surface_session_id,'d'*32)
            self.assertLessEqual(fresh.surface_generation,old.surface_generation)
            self.assertNotEqual(fresh.proposal_id,old.proposal_id)
            run.engine.bind(fresh,run.t)
            run.life.bind(fresh,run.builder.grid.snapshot(run.t),run.t)
            self.assertIs(run.engine.binding,fresh)


class Process:
    def __init__(self):self.alive=True;self.signals=[]
    def poll(self):return None if self.alive else 0
    def send_signal(self,s):self.signals.append(s);self.alive=False
    def wait(self,timeout):return 0


class StorageTests(unittest.TestCase):
    def config(self):
        return dict(config_state='VALID',enabled=True,storage_id='sqlite3',max_bag_size_bytes=1000000,
            max_bag_duration_sec=20,max_total_storage_bytes=2000000,max_recording_duration_sec=30,
            minimum_free_space_bytes=0,global_recording_budget_bytes=10000000,retention_policy='STOP_ONLY',
            storage_check_interval_sec=1.,
            profiles={'state_only':{'topics':['phase2c/interaction_state']}})

    def test_global_recording_quota_stops_new_and_current_bag_preserves_all_history(self):
        with tempfile.TemporaryDirectory() as root:
            recorder=RunRecorder(root,'run','a'*40,'b'*64);config=self.config()
            process=Process();bag=BagRecorder(config,recorder,'crane_01',executable='ros2',process_factory=lambda *a,**k:process)
            bag.start('state_only');self.assertTrue(bag.status()['rosbag_process_alive'])
            history=Path(root)/'pinned-fault-evidence';history.mkdir();evidence=history/'evidence.bin'
            evidence.write_bytes(b'x'*config['global_recording_budget_bytes'])
            with patch('chili_crane_commissioning.bag.time.monotonic',return_value=bag.next_storage_check):bag.tick()
            self.assertFalse(process.alive);self.assertTrue(evidence.exists())
            self.assertIn(b'STORAGE_LIMIT_GLOBAL_BUDGET',recorder.read('events'))
            with self.assertRaisesRegex(PermissionError,'STORAGE_LIMIT'):bag.start('state_only')
            self.assertEqual(len(process.signals),1)
            self.assertFalse(json.loads(recorder.read('manifest'))['rosbag_enabled'])

    def test_minimum_free_space_and_unknown_field_capacity_fail_closed(self):
        with tempfile.TemporaryDirectory() as root:
            recorder=RunRecorder(root,'run','a'*40,'b'*64);c=self.config();c['minimum_free_space_bytes']=100
            bag=BagRecorder(c,recorder,'crane_01',executable='ros2')
            from types import SimpleNamespace
            with patch('chili_crane_commissioning.bag.shutil.disk_usage',return_value=SimpleNamespace(free=99)):
                with self.assertRaisesRegex(PermissionError,'MINIMUM_FREE'):bag.start('state_only')
            for value in (None,True,-1):
                c['global_recording_budget_bytes']=value
                with self.assertRaisesRegex(PermissionError,'NOT_CONFIGURED'):BagRecorder(c,recorder,'crane_01',executable='ros2').start('state_only')
            c=self.config();c['retention_policy']='DELETE_OLDEST_UNPINNED'
            with self.assertRaisesRegex(PermissionError,'RETENTION_UNSUPPORTED'):BagRecorder(c,recorder,'crane_01',executable='ros2').start('state_only')

    def test_run_manifest_truthful_jsonl_only_running_and_exited_process(self):
        with tempfile.TemporaryDirectory() as root:
            recorder=RunRecorder(root,'run','a'*40,'b'*64);process=Process()
            bag=BagRecorder(self.config(),recorder,'crane_01',executable='ros2',process_factory=lambda *a,**k:process)
            m=json.loads(recorder.read('manifest'));self.assertTrue(m['journal_enabled']);self.assertTrue(m['rosbag_configured'])
            self.assertFalse(m['recording_enabled']);self.assertFalse(m['rosbag_process_alive'])
            bag.start('state_only');self.assertTrue(json.loads(recorder.read('manifest'))['rosbag_enabled'])
            process.alive=False;bag.tick()
            self.assertFalse(json.loads(recorder.read('manifest'))['rosbag_process_alive'])


if __name__=='__main__':unittest.main()
