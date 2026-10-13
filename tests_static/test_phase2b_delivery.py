"""Phase2B interface, shadow-isolation and native deterministic acceptance."""
import ast
import importlib.util
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/chili_crane_perception/src'),str(ROOT/'src/chili_crane_control/src')]


class DeliveryTests(unittest.TestCase):
    def test_phase2a_frozen_content(self):
        import hashlib
        import yaml
        manifest=yaml.safe_load((ROOT/'config/contracts/phase2a_software_frozen.yaml').read_text(encoding='utf-8'))
        for path,digest in manifest['files'].items():
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),digest,path)

    def test_shadow_publisher_types_and_no_execution_startup(self):
        publishers=set()
        for relative in ('src/chili_crane_perception/src/chili_crane_perception/surface_runtime.py',
                         'src/chili_crane_control/src/chili_crane_control/grasp_runtime.py'):
            tree=ast.parse((ROOT/relative).read_text(encoding='utf-8'))
            for node in ast.walk(tree):
                if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='create_publisher':
                    publishers.add(ast.unparse(node.args[0]))
        self.assertEqual(publishers,{'PitSurfaceSnapshot','PitSurfaceStatus','PointCloud2','MarkerArray','GraspProposal'})
        text=(ROOT/'src/chili_crane_bringup/launch/phase2b_surface_shadow.launch.py').read_text(encoding='utf-8')
        for forbidden in ('action_executor','safety_supervisor','adam_do','mock_system','production.launch'):
            self.assertNotIn(forbidden,text)

    def test_typed_grid_bounded_separate_from_status(self):
        msgs=ROOT/'src/chili_crane_msgs/msg'
        self.assertIn('[<=100000] cells',(msgs/'PitSurfaceSnapshot.msg').read_text())
        self.assertNotIn('PitSurfaceCell',(msgs/'PitSurfaceStatus.msg').read_text())
        self.assertIn('[<=4096] candidates',(msgs/'GraspProposal.msg').read_text())

    def test_interpolated_or_unproven_cells_cannot_rank(self):
        from dataclasses import replace
        from test_phase2b_planning import planning_fixture
        from test_phase2b_surface import cloud,add
        from chili_crane_perception.pit_surface import PitSurfaceGrid
        from chili_crane_control.grasp_planning import GraspPlanner
        c,s,ctx,g=planning_fixture();grid=PitSurfaceGrid(c,s)
        p,sensors=cloud(grid);add(grid,p,sensors,ctx);add(grid,p,sensors,ctx,10.05,2)
        snapshot=grid.snapshot(10.06);planner=GraspPlanner(c,g)
        for changes in (dict(sample_count=0),dict(sensor_support_mask=0),dict(frame_count=0),dict(validity='UNKNOWN')):
            modified=replace(snapshot,cells=tuple(replace(cell,**changes) for cell in snapshot.cells))
            self.assertIsNone(planner.plan(modified,10.06,modified.context,modified.generation).selected)

    def test_ros_typed_roundtrip(self):
        try:
            from chili_crane_msgs.msg import PitSurfaceSnapshot
        except ImportError:
            self.skipTest('NOT_RUN: generated ROS2 messages unavailable on Windows')
        from test_phase2b_planning import planning_fixture
        from test_phase2b_surface import cloud,add
        from chili_crane_perception.pit_surface import PitSurfaceGrid
        from chili_crane_perception.surface_transport import snapshot_to_message,snapshot_from_message
        c,s,ctx,g=planning_fixture();grid=PitSurfaceGrid(c,s)
        p,sensors=cloud(grid);add(grid,p,sensors,ctx);add(grid,p,sensors,ctx,10.05,2)
        snapshot=grid.snapshot(10.06)
        message=snapshot_to_message(snapshot,10.06,2)
        decoded=snapshot_from_message(message,c)
        self.assertEqual(decoded.snapshot_id,snapshot.snapshot_id)
        self.assertEqual(decoded.context.key,snapshot.context.key)
        self.assertEqual(decoded.cells[100].sensor_support_mask,3)
        self.assertEqual(message.status.header.stamp,message.status.evidence.measurement_stamp)

    def test_deterministic_sequence_smoke(self):
        spec=importlib.util.spec_from_file_location('phase2b_validation',ROOT/'tools/validate_phase2b_synthetic.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        result=module.validate(1)
        self.assertEqual(result['status'],'PASS')
        self.assertEqual(set(result['counts'].values()),{0})
        self.assertEqual(len(result['scenarios']),21)
