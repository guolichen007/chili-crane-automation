"""R2 acceptance: original owner, actual B grid/planner, frozen C evidence."""
from dataclasses import replace
from pathlib import Path
import subprocess
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tests_static'),str(ROOT/'src/chili_crane_control/src'),
             str(ROOT/'src/chili_crane_perception/src')]
from test_phase2c_evidence import configuration
from chili_crane_control.cycle import TaskCycles,CycleState
from chili_crane_control.interaction_contracts import Evidence,TargetBinding
from chili_crane_control.snapshot_cycle import CycleObservation,ScanProof,SnapshotCycleGate
from chili_crane_control.grasp_interaction import (GraspInteractionEngine,GrabObservation,HoistObservation,
    IoObservation,CartObservation,ZoneObservation,evidence_allows_advance)
from chili_crane_control.load_features import LoadSample
from chili_crane_control.grasp_planning import GraspPlanner
from chili_crane_control.geofence import OperationalGeofence
from chili_crane_perception.phase2b_synthetic import fixture,scene
from chili_crane_perception.snapshot_surface import CycleSnapshotWindow


class SnapshotHarness:
    def __init__(self):
        self.c,self.semantic,self.gc,self.ctx,self.grab=fixture(ROOT/'config/safety/pit_operational_geofence.template.yaml')
        self.c.update(frame_ttl_sec=2.,surface_ttl_sec=2.5,window_sec=3.,maximum_window_frames=4)
        self.window=CycleSnapshotWindow(self.c,self.semantic,'c'*32)
        self.cycles=TaskCycles('synthetic-run','synthetic-task')
        self.t=10.;self.counter=1;self.binding=None;self.result=None
        self.xyz,self.sensors=scene(self.window.builder.grid,'flat')
        self.planner=GraspPlanner(self.c,OperationalGeofence(self.gc,self.semantic))
        config=configuration();config['surface_mode']='SNAPSHOT_PER_CYCLE'
        self.engine=GraspInteractionEngine(config,self.semantic)
        self.observe_cycle()

    def evidence(self,calibration):return Evidence.at(self.t,self.counter,calibration_id=calibration)

    def observe_cycle(self):
        return self.window.observe(CycleObservation(self.evidence('synthetic-cycle'),self.cycles.run_id,
            self.cycles.task_id,self.cycles.cycle_id,self.cycles.cycle_index,self.cycles.state),self.t)

    def scan(self,epoch=None):
        self.t+=.05;self.counter+=1;self.observe_cycle()
        epoch=self.t if epoch is None else epoch
        frame=dict(epoch=epoch,mid_epoch=epoch,source_counter=self.counter,frame_id=self.ctx.source_frame,
                   span_sec=.02,source_type=self.ctx.source_type,calibration_id=self.ctx.calibration_id)
        accepted=self.window.ingest(self.xyz,self.sensors,frame,replace(self.ctx,measurement_epoch=epoch),
                                   replace(self.grab,epoch=epoch),self.t+.01)
        self.window.snapshot(self.t+.01)
        return accepted

    def proposal_and_binding(self):
        snapshot=self.window.snapshot(self.t)
        proposal=self.planner.plan(snapshot,self.t,snapshot.context,snapshot.generation)
        c=proposal.selected
        if c is None:raise ValueError('NO_REAL_B_CANDIDATE')
        p=proposal.geofence_provenance
        e=Evidence.at(snapshot.window_end,self.window.builder.grid.last_counter,
            calibration_id=self.ctx.calibration_id,config_version=self.c['config_version'])
        b=TargetBinding(self.cycles.run_id,self.cycles.task_id,self.cycles.cycle_id,self.cycles.cycle_index,
            self.cycles.cycle_id+':binding',proposal.proposal_id,c.candidate_id,self.ctx.map_id,self.ctx.semantic_version,
            self.ctx.pit_id,snapshot.snapshot_id,snapshot.surface_session_id,snapshot.generation,self.ctx.transform_id,
            self.ctx.calibration_id,p['geofence_id'],p['geofence_version'],p['config_sha256'],proposal.kernel_id,
            (*c.target_xy,0.),c.footprint_xy,c.surface_nominal_m,c.surface_lower_m,c.surface_upper_m,
            c.material_volume_proxy_lower_bound_m3,int(round(self.t*1e9)),e.counter,e)
        proof=ScanProof((b.run_id,b.task_id,b.cycle_id,b.cycle_index),self.window.gate.scan_start_ns,
            int(round(snapshot.window_start*1e9)),int(round(snapshot.window_end*1e9)),snapshot.surface_session_id,
            snapshot.generation,snapshot.snapshot_id,e)
        return proposal,b,proof

    def bind(self):
        for _ in range(3):assert self.scan()
        assert self.cycles.accept_scan(self.counter,fresh=True)
        self.t+=.01;self.counter+=1;self.observe_cycle()
        _,self.binding,proof=self.proposal_and_binding()
        self.engine.bind(self.binding,self.t,proof)
        self.window.gate.freeze(proof.key)
        return self.binding,proof

    def advance(self,mock=False):
        assert self.cycles.advance(evidence_verified=mock or evidence_allows_advance(self.cycles,self.result))

    def step(self,z=4.,motion='STOPPED',opening='OPEN',closed=False,opened=True,value=100.,stable=True,empty=False):
        self.t+=.05;self.counter+=1;self.observe_cycle();e=self.evidence('synthetic-extrinsic')
        b=self.binding
        g=GrabObservation(e,z-.05,z+.05,z+.8,z,b.capture_footprint,opening,'synthetic-extrinsic',motion_stable=stable)
        h=HoistObservation(replace(e,calibration_id='synthetic-hoist'),motion)
        io=IoObservation(replace(e,calibration_id='synthetic-io'),opened,closed)
        load=LoadSample(replace(e,calibration_id='synthetic-load'),value,'raw',suspended_empty_known=empty)
        cart=CartObservation(replace(e,calibration_id='synthetic-cart'),'station',True,True,False,False)
        zone=ZoneObservation(replace(e,calibration_id='synthetic-zone'),(-3.,3.,-3.,3.,3.5,5.),'synthetic-zone',True)
        self.result=self.engine.evaluate(self.cycles.state,g,h,io,load,self.t,b.lineage,cart=cart,zone=zone)
        return self.result

    def grasp(self):
        self.bind();self.advance(mock=True);self.step(empty=True);self.advance(mock=True)
        for _ in range(20):self.step(empty=True)
        self.advance(mock=True);self.step();self.advance(mock=True)
        for i in range(30):self.step(z=4.-3.2*(i+1)/30,motion='DOWN',stable=False)
        for i in range(6):self.step(z=.8-.005*i,motion='DOWN',stable=False)
        self.advance()
        for _ in range(8):self.step(z=.775,opening='CLOSED',closed=True,opened=False)
        self.advance();self.step(z=.775,opening='CLOSED',closed=True,opened=False);self.advance()
        for i in range(30):self.step(z=.775+3.225*(i+1)/30,motion='UP',opening='CLOSED',closed=True,opened=False,value=101.,stable=False)
        for _ in range(25):self.step(opening='CLOSED',closed=True,opened=False,value=101.)
        return self.result

    def complete(self):
        self.grasp();self.advance()
        for _ in range(3):self.step(opening='CLOSED',closed=True,opened=False,value=101.)
        self.advance(mock=True);self.step(opening='CLOSED',closed=True,opened=False,value=101.)
        self.advance(mock=True);self.step(opening='CLOSED',closed=True,opened=False,value=101.)
        self.advance(mock=True)
        for _ in range(15):self.step(value=100.)
        self.advance(mock=True)
        for _ in range(12):self.step(value=100.)
        self.advance();self.step()


class SnapshotCycleTests(unittest.TestCase):
    def test_scan_build_snapshot_and_actual_phase2b_candidate(self):
        h=SnapshotHarness();b,p=h.bind()
        self.assertGreater(b.surface_generation,0);self.assertTrue(p.matches_binding(b,h.t,1.,'SYNTHETIC'))
        self.assertGreater(p.window_start_ns,p.scan_start_ns)

    def test_plan_and_binding_freeze_actual_grid_even_after_ttl(self):
        h=SnapshotHarness();h.bind();snapshot=h.window.snapshot(h.t)
        for _ in range(50):
            self.assertFalse(h.scan());self.assertIs(h.window.snapshot(h.t),snapshot)
        self.assertEqual(h.window.builder.grid.generation,snapshot.generation)

    def test_clouds_in_lower_close_and_raise_never_mutate_surface(self):
        for phase in (CycleState.LOWER,CycleState.CLOSE,CycleState.RAISE):
            h=SnapshotHarness();h.bind();h.cycles.state=phase
            original=h.window.snapshot(h.t);h.xyz[:,2]=1000.
            for _ in range(10):self.assertFalse(h.scan())
            self.assertIs(h.window.snapshot(h.t),original)

    def test_original_owner_payload_and_pit_exit_use_bound_surface(self):
        h=SnapshotHarness();r=h.grasp()
        self.assertEqual(h.cycles.state,CycleState.RAISE)
        self.assertEqual(r.payload['state'],'PAYLOAD_CONFIRMED');self.assertTrue(r.lift['pit_exit_confirmed'])
        self.assertFalse(r.interaction['invalidate']);self.assertFalse(r.lift['clear_of_updated_surface'])
        self.assertFalse(r.next_cycle_surface_ready);self.assertIsNone(h.engine.last_refresh)
        self.assertNotIn('SURFACE_INVALIDATION_REQUIRED',h.engine.events)
        self.assertTrue(evidence_allows_advance(h.cycles,r))

    def test_complete_does_not_require_next_plan(self):
        h=SnapshotHarness();h.complete()
        self.assertEqual(h.cycles.state,CycleState.COMPLETE)
        self.assertFalse(h.result.next_cycle_surface_ready)

    def test_next_cycle_clears_entire_window_and_rejects_old_frames_and_proposal(self):
        h=SnapshotHarness();h.complete();old=h.binding;old_snapshot=h.window.frozen
        self.assertTrue(h.cycles.next_cycle(continue_task=True));h.t+=.05;h.counter+=1;h.observe_cycle()
        self.assertFalse(h.window.builder.grid.frames)
        self.assertIsNone(h.window.frozen)
        self.assertFalse(h.scan(epoch=old_snapshot.window_end))
        stale=replace(old,cycle_id=h.cycles.cycle_id,cycle_index=2,binding_id='replayed',
            binding_epoch_ns=int(h.t*1e9),binding_digest_sha256='')
        with self.assertRaises(ValueError):h.engine.bind(stale,h.t)
        forged_fresh=replace(stale,evidence=replace(stale.evidence,epoch_ns=int(h.t*1e9)),binding_digest_sha256='')
        old_proof=ScanProof((stale.run_id,stale.task_id,stale.cycle_id,stale.cycle_index),h.window.gate.scan_start_ns,
            int(old_snapshot.window_start*1e9),int(old_snapshot.window_end*1e9),old.surface_session_id,
            old.surface_generation,old.surface_snapshot_id,forged_fresh.evidence)
        with self.assertRaisesRegex(ValueError,'NEW_SCAN_PROOF'):h.engine.bind(forged_fresh,h.t,old_proof)
        h.xyz[:,2]=.65
        for _ in range(3):self.assertTrue(h.scan())
        self.assertTrue(h.cycles.accept_scan(h.counter,fresh=True));h.t+=.01;h.counter+=1;h.observe_cycle()
        _,new,proof=h.proposal_and_binding();h.engine.bind(new,h.t,proof)
        self.assertGreater(new.surface_generation,old.surface_generation);self.assertIsNone(h.engine.last_refresh)
        self.assertLess(new.surface_nominal_m,old.surface_nominal_m-.1)

    def test_active_surface_restart_requires_abort(self):
        h=SnapshotHarness();h.grasp()
        with self.assertRaisesRegex(ValueError,'ABORT_REQUIRED'):
            h.engine.observe_surface_session('d'*32,h.evidence(h.ctx.calibration_id),h.t)
        self.assertTrue(h.step(opening='CLOSED',closed=True,opened=False,value=101.).abort_required)

    def test_missing_pit_geometry_does_not_allow_payload_to_advance_raise(self):
        h=SnapshotHarness();h.engine.config['pit_geometry_state']='NOT_CONFIGURED';r=h.grasp()
        self.assertEqual(r.payload['state'],'PAYLOAD_CONFIRMED')
        self.assertFalse(r.lift['pit_exit_confirmed']);self.assertFalse(evidence_allows_advance(h.cycles,r))

    def test_geometry_provenance_change_revokes_pit_exit_and_blocks_cycle(self):
        h=SnapshotHarness();h.grasp();h.engine.config['geometry_calibration_id']='different'
        self.assertTrue(h.step(opening='CLOSED',closed=True,opened=False,value=101.).abort_required)
        self.assertIsNone(h.engine.pit_exit)

    def test_new_scan_proof_cannot_be_forged_by_new_generation_only(self):
        h=SnapshotHarness();b,p=h.bind()
        for changed in (replace(p,scan_start_ns=p.window_start_ns),replace(p,key=('other',)+p.key[1:]),
                        replace(p,window_end_ns=p.window_end_ns-1),replace(p,session='d'*32)):
            self.assertFalse(changed.matches_binding(b,h.t,1.,'SYNTHETIC'))

    def test_gate_blocks_stale_cycle_source_and_phase_regression(self):
        h=SnapshotHarness();h.bind();h.cycles.state=CycleState.RAISE;h.t+=.05;h.counter+=1;h.observe_cycle()
        self.assertFalse(h.window.gate.scanning(h.t+3.))
        h.cycles.state=CycleState.LOWER;h.t+=.05;h.counter+=1
        with self.assertRaisesRegex(ValueError,'REGRESSION'):h.observe_cycle()
        self.assertTrue(h.window.gate.blocked)

    def test_planner_gate_only_scan_or_plan_and_new_scan_identity(self):
        h=SnapshotHarness();h.scan();h.scan();_,b,p=h.proposal_and_binding()
        self.assertTrue(h.window.gate.accepts(p,h.t));h.window.gate.freeze(p.key)
        self.assertFalse(h.window.gate.accepts(p,h.t))

    def test_a_b_models_cycles_and_safety_authority_are_exact_base_bytes(self):
        paths=['src/chili_crane_control/src/chili_crane_control/'+n+'.py' for n in
               ('cycle','grasp_planning','geofence','execution','executor_node','contracts','evidence_policy')]
        paths += ['src/chili_crane_perception/src/chili_crane_perception/'+n+'.py' for n in
                  ('pit_surface','surface_mask','geometry','pipeline')]
        for p in paths:
            self.assertEqual((ROOT/p).read_bytes(),subprocess.check_output(['git','show',
                '7f8378d9e670c4145836264e268ce3a500a438a7:'+p],cwd=ROOT),p)


if __name__=='__main__':unittest.main()
