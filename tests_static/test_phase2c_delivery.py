"""Phase2C integrated counterexamples and exact-base freeze, no FIELD claims."""
import ast
import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
BASE='077d0bfe44d8fe34fc7572f52cff65362cbc83d8'
sys.path[:0]=[str(ROOT/'src/chili_crane_perception/src'),str(ROOT/'src/chili_crane_control/src')]


class DeliveryTests(unittest.TestCase):
    def test_ros_node_does_not_overwrite_rclpy_read_only_properties(self):
        tree=ast.parse((ROOT/'src/chili_crane_control/src/chili_crane_control/interaction_node.py').read_text(encoding='utf-8'))
        for n in ast.walk(tree):
            if isinstance(n,ast.Attribute) and isinstance(n.ctx,ast.Store) and isinstance(n.value,ast.Name) and n.value.id=='self':
                self.assertNotIn(n.attr,{'context','publishers','subscriptions','timers','clients','services','guards'})

    def test_four_integrated_scenarios_use_actual_grid_and_planner(self):
        spec=importlib.util.spec_from_file_location('phase2c_acceptance',ROOT/'tools/validate_phase2c_synthetic.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        result=module.validate()
        self.assertEqual(len(result['scenarios']),4)
        self.assertFalse(any(result['counters'].values()))
        cases={c['scenario']:c for c in result['scenarios']}
        self.assertEqual(cases['SUCCESSFUL_GRASP']['payload_after_lift'],'PAYLOAD_CONFIRMED')
        self.assertEqual(cases['FAILED_GRASP_AFTER_INTERACTION']['payload_after_lift'],'PAYLOAD_NOT_CONFIRMED')
        for name in ('SUCCESSFUL_GRASP','FAILED_GRASP_AFTER_INTERACTION'):
            self.assertTrue(cases[name]['refreshed'])
            self.assertGreater(cases[name]['current_generation'],cases[name]['frozen_generation'])
        self.assertFalse(cases['LOAD_ACCELERATION_FALSE_POSITIVE']['refreshed'])
        self.assertFalse(result['physical_output_enabled'])

    def test_phase2a_b_clock_vendor_and_original_cycles_are_exactly_frozen(self):
        # Compare actual Git blob bytes, not text normalized by the OS. New
        # optional adapters may change; the already sealed algorithms may not.
        paths=[
            'src/chili_crane_perception/src/chili_crane_perception/'+name+'.py' for name in
            ('pit_surface','surface_mask','surface_disturbance','surface_time','surface_session')]
        paths += ['src/chili_crane_control/src/chili_crane_control/'+name+'.py' for name in
                  ('grasp_planning','cycle')]
        paths += ['src/chili_crane_slam/src/chili_crane_slam/'+name+'.py' for name in
                  ('dual_lidar','dual_lidar_node','timebase')]
        paths += ['scripts/setup/install_rslidar_sdk.sh']
        for relative in paths:
            expected=subprocess.check_output(['git','show',BASE+':'+relative],cwd=ROOT)
            self.assertEqual(hashlib.sha256((ROOT/relative).read_bytes()).digest(),hashlib.sha256(expected).digest(),relative)

    def test_only_evidence_and_isolated_console_publishers(self):
        text=(ROOT/'src/chili_crane_control/src/chili_crane_control/interaction_node.py').read_text(encoding='utf-8')
        console=(ROOT/'src/chili_crane_commissioning/src/chili_crane_commissioning/node.py').read_text(encoding='utf-8')
        for source in (text,console):
            tree=ast.parse(source)
            publishers=[ast.unparse(n.args[0]) for n in ast.walk(tree) if isinstance(n,ast.Call)
                        and isinstance(n.func,ast.Attribute) and n.func.attr=='create_publisher']
            self.assertFalse({'SafetyPermit','AuthorizedCommand','ActuationRequest'} & set(publishers))
        self.assertIn('LIVE_SPATIAL_LINEAGE_STALE_OR_CONFLICT',text)
        self.assertIn('proposal.validity in (3,4) and proposal.selected_candidate_id',text)
        self.assertNotIn('from chili_crane_perception',text)

    def test_normal_and_symlink_ci_keep_old_and_add_new_acceptance(self):
        text=(ROOT/'scripts/validation/ubuntu22_ros2_phase0_validate.sh').read_text(encoding='utf-8')
        for tool in ('validate_phase2a_ros.py','validate_phase2b_ros.py','validate_phase2b_r11_ros.py','validate_phase2c_ros.py'):
            self.assertGreaterEqual(text.count(tool),2,tool)
        self.assertIn('validate_phase2c_synthetic.py',text)
        self.assertIn('python3 -m compileall',text)
        self.assertIn('python3 -m pytest',text)


if __name__=='__main__':unittest.main()
