"""R1.2 owner-driven cycle, historical exit and bounded storage counterexamples."""
from dataclasses import replace
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tests_static'),str(ROOT/'src/chili_crane_control/src'),
             str(ROOT/'src/chili_crane_perception/src'),str(ROOT/'src/chili_crane_commissioning/src'),
             str(ROOT/'src/chili_crane_hardware/src')]
from test_phase2c_evidence import Harness,configuration
from test_phase2c_r11 import Process,successor
import test_phase2c_r11 as r11
from chili_crane_control.cycle import TaskCycles,CycleState
from chili_crane_control.grasp_interaction import evidence_allows_advance
from chili_crane_perception.phase2c_synthetic import SyntheticInteractionRun
from chili_crane_commissioning.recording import RunRecorder
from chili_crane_commissioning.bag import BagRecorder
from chili_crane_commissioning.backend import ConsoleBackend
import yaml


class OwnedCycleRun:
    """Original TaskCycles owns every phase. No actuation or second FSM."""
    def __init__(self):
        self.run=SyntheticInteractionRun(configuration(),ROOT/'config/safety/pit_operational_geofence.template.yaml')
        self.cycles=TaskCycles(self.run.binding.run_id,self.run.binding.task_id)
        assert self.cycles.accept_scan(2,fresh=True)
        # Existing positioning policy is explicitly verified by this mock owner.
        assert self.cycles.advance(evidence_verified=True)
        assert self.cycles.advance(evidence_verified=True)
        for _ in range(20):self.step(empty=True)
        assert self.cycles.advance(evidence_verified=True)
        self.step()
        assert self.cycles.advance(evidence_verified=True)
        for i in range(30):self.step(z=4.-3.2*(i+1)/30,motion='DOWN',stable=False)
        for i in range(6):self.step(z=.8-.005*i,motion='DOWN',stable=False)
        assert self.advance()
        for _ in range(8):self.step(z=.775,opening='CLOSED',closed=True,opened=False)
        assert self.advance()
        self.step(z=.775,opening='CLOSED',closed=True,opened=False)
        assert self.advance()
        self.run.visibility='partial'
        for i in range(30):self.step(z=.775+3.225*(i+1)/30,motion='UP',opening='CLOSED',closed=True,
                                    opened=False,value=101.,stable=False)
        for _ in range(40):self.step(opening='CLOSED',closed=True,opened=False,value=101.)

    def step(self,**values):return self.run.step(self.cycles.state.name,**values)
    def advance(self):return self.cycles.advance(evidence_verified=evidence_allows_advance(self.cycles,self.run.last_result))

    def unload_without_refresh(self):
        assert self.advance() and self.cycles.state==CycleState.MOVE_TO_UNLOAD
        self.run.visibility='occluded'
        for _ in range(30):self.step(opening='CLOSED',closed=True,opened=False,value=101.)
        assert not self.run.last_result.lift['refresh_ready']
        assert self.run.last_result.lift['pit_exit_confirmed']
        assert self.cycles.advance(evidence_verified=True)  # mock transport positioned
        self.step(opening='CLOSED',closed=True,opened=False,value=101.)
        assert self.cycles.advance(evidence_verified=self.run.last_result.unload['aligned'])
        self.step(opening='CLOSED',closed=True,opened=False,value=101.)
        assert self.cycles.advance(evidence_verified=self.run.last_result.unload['in_safe_zone'])
        self.step(value=100.)
        assert self.cycles.advance(evidence_verified=self.run.last_result.unload['open_confirmed'])
        for _ in range(60):self.step(value=100.)
        return self.run.last_result


class CurrentCycleTests(unittest.TestCase):
    def test_partial_refresh_current_cycle_exit(self):
        owned=OwnedCycleRun();r=owned.run.last_result
        self.assertEqual((owned.run.refresh.affected_cell_count,owned.run.refresh.refreshed_cell_count,
                          owned.run.refresh.unknown_cell_count),(9,2,7))
        self.assertEqual(r.payload['state'],'PAYLOAD_CONFIRMED')
        self.assertFalse(r.lift['clear_of_updated_surface']);self.assertFalse(r.next_cycle_surface_ready)
        self.assertTrue(r.lift['clear_of_pit']);self.assertTrue(r.lift['safe_lift_height'])
        self.assertTrue(r.lift['pit_exit_confirmed']);self.assertTrue(owned.advance())
        self.assertEqual(owned.cycles.state,CycleState.MOVE_TO_UNLOAD)

    def test_stale_refresh_after_pit_exit(self):
        h=Harness();h.warm();h.interact();h.lift()
        confirmed=h.engine.pit_exit
        r=h.step('MOVE_TO_UNLOAD',opening='CLOSED',closed=True,opened=False,value=101.,refresh=False)
        self.assertTrue(r.lift['pit_exit_confirmed']);self.assertIs(h.engine.pit_exit,confirmed)
        self.assertFalse(r.lift['refresh_ready']);self.assertEqual(h.engine.events.count('PIT_EXIT_CONFIRMED'),1)

    def test_unload_with_latched_pit_exit(self):
        owned=OwnedCycleRun();r=owned.unload_without_refresh()
        self.assertEqual(r.unload['state'],'UNLOAD_CONFIRMED')
        self.assertTrue(r.lift['pit_exit_confirmed']);self.assertFalse(r.lift['refresh_ready'])

    def test_complete_without_next_plan_and_next_bind_still_requires_new_b_proposal(self):
        owned=OwnedCycleRun();r=owned.unload_without_refresh();run=owned.run
        self.assertEqual(r.unload['state'],'UNLOAD_CONFIRMED');self.assertFalse(r.next_cycle_surface_ready)
        self.assertTrue(owned.advance());self.assertEqual(owned.cycles.state,CycleState.COMPLETE)
        owned.step(value=100.)
        self.assertTrue(owned.cycles.next_cycle(continue_task=True));self.assertEqual(owned.cycles.state,CycleState.SCAN)
        self.assertIsNone(run.next_candidate())
        old=successor(run.binding,run.t)
        with self.assertRaisesRegex(ValueError,'NEW_SURFACE'):run.engine.bind(old,run.t)
        # Fresh B scan/proposal is independently necessary for the next binding.
        run.visibility='recovery'
        for _ in range(8):
            # The previous terminal cycle's observation adapter continues scanning;
            # it does not advance the next owner or mutate the previous binding.
            run.step('COMPLETE',value=100.)
        fresh=run.next_candidate();self.assertIsNotNone(fresh)
        self.assertGreater(fresh.surface_generation,run.binding.surface_generation)
        proposal=run.proposal();self.assertTrue(proposal.selected.accepted)
        self.assertTrue(owned.cycles.accept_scan(run.builder.grid.last_counter,fresh=True))
        run.engine.bind(fresh,run.t)
        self.assertIsNone(run.engine.pit_exit)

    def test_pit_exit_provenance_conflict_and_phase_regression_fail_closed(self):
        for attack in ('lineage','geometry','semantic','phase'):
            h=Harness();h.warm();h.interact();h.lift(refresh=False)
            self.assertIsNotNone(h.engine.pit_exit)
            if attack=='lineage':
                r=h.engine.evaluate('MOVE_TO_UNLOAD',None,None,None,None,h.t,('wrong',)+h.binding.lineage[1:])
            elif attack=='geometry':
                h.engine.config['geometry_calibration_id']='different'
                r=h.step('MOVE_TO_UNLOAD')
            elif attack=='semantic':
                h.engine.config['pit_rim_elevation_m']=2.1
                r=h.step('MOVE_TO_UNLOAD')
            else:
                h.step('MOVE_TO_UNLOAD',opening='CLOSED',closed=True,opened=False,value=101.)
                r=h.step('RAISE',opening='CLOSED',closed=True,opened=False,value=101.)
            self.assertTrue(r.abort_required,attack);self.assertIsNone(h.engine.pit_exit)
            self.assertTrue(h.step('VERIFY_EMPTY',cart=True).abort_required)

    def test_no_pit_exit_from_missing_geometry_calibration_or_without_raise(self):
        for attack in ('pit_geometry','height','not_raise','binding_chronology'):
            h=Harness();h.warm();h.interact()
            if attack=='pit_geometry':h.engine.config['pit_geometry_state']='NOT_CONFIGURED'
            if attack=='height':h.engine.config['safe_lift_height_m']=4.
            if attack=='binding_chronology':h.engine.lower_started=h.binding.binding_epoch_ns/1e9-.05
            if attack=='not_raise':
                for _ in range(40):r=h.step('VERIFY_LOAD',opening='CLOSED',closed=True,opened=False,value=101.)
            else:r=h.lift(refresh=False)
            self.assertFalse(r.lift['pit_exit_confirmed']);self.assertIsNone(h.engine.pit_exit)

    def test_abort_and_session_run_conflicts_clear_exit(self):
        for attack in ('abort','session','run'):
            h=Harness();h.warm();h.interact();h.lift(refresh=False)
            if attack=='abort':h.engine.evaluate('ABORTED',None,None,None,None,h.t)
            else:
                values={'run_id':'other'} if attack=='run' else {'surface_session_id':'d'*32,'surface_snapshot_id':'d'*32+':new'}
                with self.assertRaises(ValueError):h.engine.bind(successor(h.binding,h.t,**values),h.t)
            self.assertIsNone(h.engine.pit_exit)


class DiagnosticTests(unittest.TestCase):
    def assert_aggregate(self,result):
        d=result.diagnostics
        keys=('closure_confirmed','interaction_committed','raise_seen','bound_clear','vertical_settled',
              'settling_window_valid','load_sensor_fresh','load_baseline_valid','load_source_calibration_allowed',
              'payload_delta_gate_pass')
        self.assertEqual(d['payload_gate_pass'],all(d[k] for k in keys))
        self.assertEqual(d['payload_gate_pass'],result.payload['state']=='PAYLOAD_CONFIRMED')
        for key in ('pit_rim_clear','safe_lift_height','pit_exit_confirmed','surface_refresh_ready','updated_clear','next_plan_ready'):
            self.assertIn(key,d)

    def test_payload_gate_diagnostic_truth_table(self):
        h=Harness();h.warm();h.interact();self.assert_aggregate(h.lift(refresh=False))
        h=Harness();h.warm();h.interact()
        for i in range(25):r=h.step('RAISE',z=.775+.012*i*i,motion='UP',opening='CLOSED',closed=True,
                                  opened=False,value=110.,stable=False)
        self.assertTrue(r.diagnostics['payload_delta_gate_pass']);self.assertFalse(r.diagnostics['payload_gate_pass'])
        self.assert_aggregate(r)
        h=Harness();h.warm();h.interact()
        for _ in range(40):r=h.step('RAISE',z=1.1,opening='CLOSED',closed=True,opened=False,value=101.)
        self.assertTrue(r.diagnostics['payload_delta_gate_pass']);self.assertFalse(r.diagnostics['bound_clear'])
        self.assert_aggregate(r)
        h=Harness();h.warm();h.interact();h.engine.load.config['calibration_state']='NOT_CONFIGURED'
        r=h.lift(refresh=False);self.assertTrue(r.diagnostics['payload_delta_gate_pass'])
        self.assertFalse(r.diagnostics['payload_gate_pass']);self.assert_aggregate(r)


class StorageRateTests(unittest.TestCase):
    def test_storage_scan_rate_bounded_for_backend_ticks(self):
        with tempfile.TemporaryDirectory() as root:
            recorder=RunRecorder(root,'rate','a'*40,'b'*64);c=r11.StorageTests().config()
            config=yaml.safe_load((ROOT/'config/commissioning/console.yaml').read_text(encoding='utf-8'))
            backend=ConsoleBackend(config,recorder);p=Process()
            backend.bag=BagRecorder(c,recorder,'crane_01',executable='ros2',process_factory=lambda *a,**k:p)
            with patch('chili_crane_commissioning.bag.time.monotonic',return_value=100.):backend.bag.start('state_only')
            for i in range(200):
                with patch('chili_crane_commissioning.bag.time.monotonic',return_value=100.+i*.01):backend.tick()
            self.assertEqual(backend.bag.storage_scan_count,2)  # start + one interval, not 200 ticks
            with patch('chili_crane_commissioning.bag.time.monotonic',return_value=102.):backend.tick()
            self.assertEqual(backend.bag.storage_scan_count,3);self.assertTrue(p.alive)
            backend.bag.stop()

    def test_invalid_storage_interval_and_scan_oserror_fail_closed(self):
        with tempfile.TemporaryDirectory() as root:
            recorder=RunRecorder(root,'invalid','a'*40,'b'*64)
            for interval in (None,False,0.,-1.,float('nan'),float('inf'),.9,5.1):
                c=r11.StorageTests().config();c['storage_check_interval_sec']=interval
                bag=BagRecorder(c,recorder,'crane_01',executable='ros2')
                with self.assertRaisesRegex(PermissionError,'NOT_CONFIGURED'):bag.start('state_only')
            p=Process();bag=BagRecorder(r11.StorageTests().config(),recorder,'crane_01',executable='ros2',process_factory=lambda *a,**k:p)
            bag.start('state_only')
            with patch('chili_crane_commissioning.bag.time.monotonic',return_value=bag.next_storage_check),\
                    patch.object(Path,'rglob',side_effect=OSError('scan failure')):
                bag.tick()
            self.assertFalse(p.alive);self.assertIn(b'STORAGE_LIMIT',recorder.read('events'))

    def test_entry_bound_counts_even_symlinks(self):
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as root:
            recorder=RunRecorder(root,'entries','a'*40,'b'*64)
            bag=BagRecorder(r11.StorageTests().config(),recorder,'crane_01',executable='ros2')
            link=Mock();link.is_symlink.return_value=True
            with patch.object(Path,'rglob',return_value=(link for _ in range(100001))):
                with self.assertRaisesRegex(PermissionError,'DIRECTORY_BOUND'):bag.start('state_only')
            self.assertIsNone(bag.process)


class PackagingTests(unittest.TestCase):
    def test_bringup_commissioning_dependency_and_production_still_disabled(self):
        dependencies={n.text for n in ET.parse(ROOT/'src/chili_crane_bringup/package.xml').getroot().findall('exec_depend')}
        self.assertIn('chili_crane_commissioning',dependencies)
        for file in (ROOT/'src/chili_crane_bringup/launch').glob('*production*'):
            self.assertNotIn('commissioning',file.read_text(encoding='utf-8'))

    def test_ctest_integrated_scenarios_budget_explicit_and_internal_wait_unchanged(self):
        text=(ROOT/'src/chili_crane_perception/CMakeLists.txt').read_text(encoding='utf-8')
        self.assertIn('test_phase2c_delivery.py\n    TIMEOUT 180',text)
        import subprocess
        base=subprocess.check_output(['git','show','7389e88a4f45268b302320e0f805ca5da953c166:tools/validate_phase2c_ros.py'],cwd=ROOT).decode()
        current=(ROOT/'tools/validate_phase2c_ros.py').read_text(encoding='utf-8')
        self.assertEqual(base.count('timeout=9.'),current.count('timeout=9.'))


if __name__=='__main__':unittest.main()
