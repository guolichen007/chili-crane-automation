from dataclasses import replace
from pathlib import Path
import sys
import unittest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tests_static'),str(ROOT/'src/chili_crane_control/src')]
from test_phase2c_contracts import binding
from chili_crane_control.interaction_contracts import Evidence
from chili_crane_control.load_features import LoadSample,LoadFeatureEstimator
from chili_crane_control.grasp_interaction import (GraspInteractionEngine,GrabObservation,HoistObservation,IoObservation,
    RefreshedSurface,CartObservation,ZoneObservation,evidence_allows_advance)
from chili_crane_control.cycle import TaskCycles,CycleState


def configuration():
    config=yaml.safe_load((ROOT/'config/algorithms/phase2c_interaction.synthetic.yaml').read_text())
    config['surface_mode']='LEGACY_R1_ARCHITECTURE'  # Historical evidence regression, not R2 acceptance.
    return config


class Harness:
    def __init__(self):
        self.config=configuration();self.engine=GraspInteractionEngine(self.config);self.binding=binding()
        self.engine.bind(self.binding,10.1);self.t=10.1;self.counter=1
        self.result=None

    def step(self,phase='WAIT_STABLE',z=3.,motion='STOPPED',opening='OPEN',closed=False,opened=True,
             value=100.,stable=True,empty=False,refresh=False,load_validity=4,cart=False,dt=.05):
        self.t+=dt;self.counter+=1
        e=Evidence.at(self.t,self.counter,calibration_id='synthetic-extrinsic')
        g=GrabObservation(e,z-.05,z+.05,z+.8,z,self.binding.capture_footprint,opening,'synthetic-extrinsic',motion_stable=stable)
        h=HoistObservation(replace(e,calibration_id='synthetic-hoist'),motion)
        io=IoObservation(replace(e,calibration_id='synthetic-io'),opened,closed)
        load=LoadSample(replace(e,calibration_id='synthetic-load',validity=load_validity),value,'raw',suspended_empty_known=empty)
        r=RefreshedSurface(replace(e,calibration_id=self.binding.spatial_calibration_id),self.binding.binding_id,
            self.binding.binding_digest_sha256,'refreshed',self.binding.surface_generation+1,self.binding.surface_session_id,
            1.05,.85,.95,True,self.binding.lineage) if refresh else None
        c=CartObservation(replace(e,calibration_id='synthetic-cart'),'station',True,True,False,False) if cart else None
        zone=ZoneObservation(replace(e,calibration_id='synthetic-zone'),(-1.,2.,-1.,2.,2.5,4.),'synthetic-zone',True) if cart else None
        self.result=self.engine.evaluate(phase,g,h,io,load,self.t,self.binding.lineage,r,c,zone)
        return self.result

    def warm(self):
        for _ in range(20):self.step(empty=True)
        assert self.engine.load.features.baseline_valid

    def interact(self,value=100.):
        for i in range(30):self.step('LOWER',z=3.-2.2*(i+1)/30,motion='DOWN',value=value,stable=False)
        # Evidence A needs real descending penetration persistence, not a timer
        # repeatedly evaluating one stopped measurement.
        for i in range(6):self.step('LOWER',z=.8-.005*i,motion='DOWN',value=value,stable=False)
        assert self.engine.committed
        for _ in range(6):self.step('CLOSE',z=.775,opening='CLOSED',closed=True,opened=False,value=value)
        assert self.result.closure['state']=='CLOSURE_CONFIRMED'

    def lift(self,value=101.,refresh=True):
        for i in range(30):self.step('RAISE',z=.775+2.225*(i+1)/30,motion='UP',opening='CLOSED',closed=True,
            opened=False,value=value,stable=False,refresh=refresh)
        for _ in range(35):self.step('RAISE',opening='CLOSED',closed=True,opened=False,value=value,refresh=refresh)
        return self.result


class EvidenceTests(unittest.TestCase):
    def test_cart_zone_and_sensor_role_replay_cannot_change_same_epoch_fact(self):
        from types import SimpleNamespace
        a=Harness();e=Evidence.at(10.2,3)
        for key in ('cart','zone','grab','hoist'):
            self.assertTrue(a.engine._ordered(key,SimpleNamespace(evidence=e),(False,)))
            self.assertTrue(a.engine._ordered(key,SimpleNamespace(evidence=e),(False,)))
            self.assertFalse(a.engine._ordered(key,SimpleNamespace(evidence=e),(True,)))
            self.assertFalse(a.engine._ordered(key,SimpleNamespace(evidence=replace(e,epoch_ns=e.epoch_ns+1,counter=2)),(False,)))
            self.assertFalse(a.engine._ordered(key,SimpleNamespace(evidence=replace(e,epoch_ns=e.epoch_ns+1,counter=4,
                calibration_id='changed-source-role')),(False,)))

    def test_calibrated_contact_requires_joint_overlap_descent_and_persistence(self):
        a=Harness();a.warm()
        states=[]
        for i in range(32):
            r=a.step('LOWER',z=1.08-.003*i,motion='DOWN',value=101.,stable=False)
            states.append(r.contact['state'])
        self.assertIn('CONTACT_CONFIRMED',states)
        self.assertTrue(r.contact['overlap'])
        self.assertTrue(r.contact['descending'])
        self.assertTrue(r.contact['load_change'])
        self.assertTrue(r.interaction['invalidate'])

    def test_sensor_fault_and_stale_source_cannot_preserve_payload_confirmation(self):
        a=Harness();a.warm();a.interact();a.lift()
        self.assertEqual(a.result.payload['state'],'PAYLOAD_CONFIRMED')
        for _ in range(8):
            r=a.step('RAISE',opening='CLOSED',closed=True,opened=False,value=101.,refresh=True,load_validity=1)
        self.assertNotEqual(r.payload['state'],'PAYLOAD_CONFIRMED')
        self.assertFalse(r.load_features.fresh)
        self.assertFalse(a.engine.load.current(a.t+2.).fresh)

    def test_geometry_alone_possible_and_load_spike_without_overlap_not_confirmed(self):
        a=Harness();a.warm()
        for i in range(12):r=a.step('LOWER',z=1.1-.01*i,motion='DOWN',stable=False)
        self.assertEqual(r.contact['state'],'CONTACT_POSSIBLE')
        b=Harness();b.warm()
        for i in range(12):r=b.step('LOWER',z=2.8-.01*i,motion='DOWN',value=110.,stable=False)
        self.assertNotEqual(r.contact['state'],'CONTACT_CONFIRMED')

    def test_io_closed_lidar_open_and_dual_limits_conflict(self):
        h=Harness();h.warm()
        for opened in (False,True):
            r=h.step('CLOSE',closed=True,opened=opened,opening='OPEN')
            self.assertEqual(r.closure['state'],'CLOSURE_CONFLICT')

    def test_empty_grasp_commits_interaction_but_not_payload(self):
        h=Harness();h.warm();h.interact();r=h.lift(value=100.)
        self.assertEqual(r.interaction['state'],'INTERACTION_COMMITTED')
        self.assertTrue(r.interaction['invalidate'])
        self.assertEqual(r.payload['state'],'PAYLOAD_NOT_CONFIRMED')
        self.assertEqual(h.engine.events.count('INTERACTION_COMMITTED'),1)

    def test_successful_stable_lift_calibrated_delta_payload(self):
        h=Harness();h.warm();h.interact();r=h.lift()
        self.assertEqual(r.payload['state'],'PAYLOAD_CONFIRMED')
        self.assertTrue(r.lift['clear_of_bound_surface']);self.assertTrue(r.lift['clear_of_updated_surface'])
        self.assertTrue(r.lift['clear_of_pit']);self.assertTrue(r.lift['safe_lift_height'])

    def test_accelerating_load_spike_never_final_payload(self):
        h=Harness();h.warm();h.interact()
        for i in range(20):
            r=h.step('RAISE',z=.775+.009*i*i,motion='UP',opening='CLOSED',closed=True,opened=False,
                     value=110.,stable=False,refresh=True)
            self.assertNotEqual(r.payload['state'],'PAYLOAD_CONFIRMED')

    def test_load_not_configured_and_field_unknown_never_confirm(self):
        h=Harness();h.warm();h.interact()
        for _ in range(10):r=h.step('RAISE',opening='CLOSED',closed=True,opened=False,value=110.,load_validity=1,refresh=True)
        self.assertEqual(r.payload['state'],'PAYLOAD_CANDIDATE')
        h=Harness();h.warm();h.interact();h.engine.config['load']['calibration_state']='NOT_CONFIGURED'
        h.engine.load.config['calibration_state']='NOT_CONFIGURED'
        r=h.lift();self.assertNotEqual(r.payload['state'],'PAYLOAD_CONFIRMED')

    def test_binding_swap_lineage_and_old_generation_rejected(self):
        h=Harness();h.warm()
        attack=replace(h.binding,candidate_id='new-candidate',binding_epoch_ns=int(h.t*1e9),
            evidence=replace(h.binding.evidence,epoch_ns=int(h.t*1e9)),binding_digest_sha256='')
        with self.assertRaisesRegex(ValueError,'REBIND'):h.engine.bind(attack,h.t)
        h.interact();r=h.lift();h.engine.last_phase='COMPLETE'
        old=replace(h.binding,cycle_id='task:2',cycle_index=2,binding_id='new',binding_epoch_ns=int(h.t*1e9),
                    evidence=replace(h.binding.evidence,epoch_ns=int(h.t*1e9)),binding_digest_sha256='')
        with self.assertRaisesRegex(ValueError,'NEW_SURFACE'):h.engine.bind(old,h.t)
        e=Evidence.at(h.t,h.counter+1,calibration_id='synthetic-extrinsic')
        g=GrabObservation(e,2.95,3.05,3.8,3.,h.binding.capture_footprint,'CLOSED','synthetic-extrinsic')
        r=h.engine.evaluate('RAISE',g,HoistObservation(e,'STOPPED'),None,None,h.t,('wrong',)+h.binding.lineage[1:])
        self.assertTrue(r.abort_required)

    def test_updated_clear_requires_real_refresh_and_pit_calibration(self):
        h=Harness();h.warm();h.interact();r=h.lift(refresh=False)
        self.assertTrue(r.lift['clear_of_bound_surface']);self.assertFalse(r.lift['clear_of_updated_surface'])
        self.assertEqual(r.payload['state'],'PAYLOAD_CONFIRMED')
        self.assertFalse(r.next_cycle_surface_ready)
        self.assertEqual(r.lift['updated_clear_state'],'UNKNOWN')
        h.engine.config['pit_geometry_state']='NOT_CONFIGURED'
        r=h.step('RAISE',opening='CLOSED',closed=True,opened=False,value=101.,refresh=True)
        self.assertFalse(r.lift['clear_of_pit'])

    def test_unload_safe_zone_cart_open_and_load_drop(self):
        h=Harness();h.warm();h.interact();h.lift()
        for _ in range(60):r=h.step('VERIFY_EMPTY',value=100.,cart=True,refresh=True)
        self.assertEqual(r.unload['state'],'UNLOAD_CONFIRMED')

    def test_load_robust_features_baseline_freeze_stale_regression_hysteresis(self):
        estimator=LoadFeatureEstimator(configuration()['load'])
        for i in range(20):
            e=Evidence.at(1+i*.05,i+1,calibration_id='synthetic-load')
            f=estimator.observe(LoadSample(e,100.,'raw',suspended_empty_known=True),e.epoch,True)
        baseline=f.empty_baseline
        for i in range(40):
            e=Evidence.at(2+i*.05,i+21,calibration_id='synthetic-load')
            f=estimator.observe(LoadSample(e,101.,'raw'),e.epoch,False)
        self.assertEqual(f.empty_baseline,baseline);self.assertTrue(f.change_latched)
        duplicate=estimator.observe(estimator.last_sample,estimator.last_sample.evidence.epoch+.01)
        self.assertEqual(duplicate.change_persistence_sec,f.change_persistence_sec)
        self.assertFalse(estimator.current(10.).fresh)
        self.assertFalse(estimator.observe(LoadSample(Evidence.at(1.,1,calibration_id='synthetic-load'),100.,'raw'),1.).fresh)

    def test_admission_uses_existing_cycles_not_second_fsm(self):
        h=Harness();h.warm();h.interact()
        cycles=TaskCycles('run','task');cycles.state=CycleState.VERIFY_LOAD
        r=h.step('VERIFY_LOAD',z=.775,opening='CLOSED',closed=True,opened=False)
        self.assertTrue(evidence_allows_advance(cycles,r))
        self.assertNotEqual(r.payload['state'],'PAYLOAD_CONFIRMED')
        cycles.state=CycleState.RAISE
        self.assertFalse(evidence_allows_advance(cycles,r))
        r=h.lift();self.assertTrue(evidence_allows_advance(cycles,r))
