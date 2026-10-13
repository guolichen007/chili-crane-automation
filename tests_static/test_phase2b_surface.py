"""B1 observed-only counterexamples, explicit NON_PRODUCTION synthetic policy."""
import copy
from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/chili_crane_perception/src'))
from chili_crane_perception.pit_surface import PitSurfaceGrid, PlanningSpatialContext


def fixture():
    c = dict(config_state='VALID', config_version='SYNTHETIC_NON_PRODUCTION', source_type='SYNTHETIC',
        pit_id='pit', resolution_m=.25, frame_ttl_sec=.2, surface_ttl_sec=.5, window_sec=.4,
        maximum_frame_span_sec=.03, epoch_tolerance_sec=1e-6, sensor_uncertainty_m=.01,
        single_sensor_extra_uncertainty_m=.02, mad_scale=3., max_height_dispersion_m=.3,
        minimum_neighborhood_coverage=.5, minimum_cell_samples=4, minimum_cell_frames=2,
        maximum_input_points=20000, maximum_grid_cells=1024, maximum_window_frames=4,
        physical_output_enabled=False, automatic_control_enabled=False, camera_control_authority=False)
    s = dict(config_state='VALID', schema_version=1, map_id='map1', semantic_version='v1', frame_id='map',
        pits=[dict(pit_id='pit', polygon_map_m=[[-2,-2],[2,-2],[2,2],[-2,2]], pit_bottom_z_m=0.,
                   wall_top_z_m=3., safety_inset_m=.1)])
    context = PlanningSpatialContext('cloud', 'map', 'map1', 'v1', 'pit', 'transform1', 'cal1',
        'SYNTHETIC_VALIDATED', 'SYNTHETIC', 10., .01, tuple(np.eye(4).ravel()), ('s0', 's1'), True, 'motion1')
    return c, s, context


def cloud(grid, height=lambda x,y: 1.):
    xy = np.array([grid.center(i) for i in range(grid.width*grid.height)])
    p = np.repeat(np.c_[xy, [height(x,y) for x,y in xy]], 6, axis=0)
    return p, np.tile([0,1,0,1,0,1], len(xy))


def add(grid, p, sensors, context, epoch=10., counter=1):
    return grid.ingest(p, sensors, dict(epoch=epoch, source_counter=counter, frame_id='cloud', span_sec=.02,
        source_type=context.source_type, calibration_id=context.calibration_id), replace(context, measurement_epoch=epoch), epoch+.01)


class SurfaceTests(unittest.TestCase):
    def setUp(self):
        self.c, self.s, self.context = fixture()
        self.g = PitSurfaceGrid(self.c, self.s)

    def observed(self, p=None, sensors=None):
        if p is None:
            p, sensors = cloud(self.g)
        self.assertTrue(add(self.g, p, sensors, self.context))
        self.assertTrue(add(self.g, p, sensors, self.context, 10.05, 2))
        return self.g.snapshot(10.06)

    def test_flat_bounds_and_plane(self):
        s = self.observed()
        center = s.cells[8*self.g.width+8]
        self.assertEqual(center.sensor_support_mask, 3)
        self.assertLess(center.z_lower_bound, 1.)
        self.assertGreater(center.z_upper_bound, 1.)
        self.assertAlmostEqual(center.z_median, 1.)
        self.assertAlmostEqual(center.slope, 0.)
        self.assertFalse(s.valid_for_control)

    def test_outlier_median_not_mean(self):
        p, sensors = cloud(self.g)
        p[::6, 2] = 2.9
        s = self.observed(p, sensors)
        self.assertAlmostEqual(s.cells[100].z_median, 1.)

    def test_holes_are_unknown_and_not_empty(self):
        p, sensors = cloud(self.g)
        keep = np.linalg.norm(p[:,:2], axis=1) > .5
        s = self.observed(p[keep], sensors[keep])
        self.assertFalse(s.cells[8*self.g.width+8].observed)
        self.assertTrue(np.isnan(s.cells[8*self.g.width+8].z_median))

    def test_single_sensor_degraded_extra_bound(self):
        p, sensors = cloud(self.g)
        for sensor in (0,1):
            self.g = PitSurfaceGrid(self.c, self.s)
            s = self.observed(p, np.full(len(p),sensor))
            cell = s.cells[100]
            self.assertEqual(cell.sensor_support_mask, 1 << sensor)
            self.assertEqual(cell.validity, 'DEGRADED')
            self.assertAlmostEqual(cell.height_uncertainty, .04)

    def test_plane_only_observed_neighbors(self):
        p, sensors = cloud(self.g, lambda x,y: 1.+.2*x)
        s = self.observed(p,sensors)
        self.assertAlmostEqual(s.cells[100].slope, np.arctan(.2))
        self.assertLess(s.cells[100].roughness, 1e-8)

    def test_ttl_generation_and_window_bounded(self):
        self.observed()
        for i in range(3,12):
            p, sensors = cloud(self.g)
            self.assertTrue(add(self.g,p,sensors,self.context,10+i*.02,i+1))
        self.assertLessEqual(len(self.g.frames),4)
        old = self.g.snapshot(10.23)
        new = self.g.snapshot(11.)
        self.assertGreater(new.generation,old.generation)
        self.assertFalse(any(c.observed for c in new.cells))
        self.assertEqual(new.validity,'STALE')

    def test_spatial_identity_even_same_frame_needs_proof(self):
        p, sensors = cloud(self.g)
        for changes in (dict(source_frame='wrong'),dict(transform_state='NOT_CONFIGURED'),dict(map_id='other'),
                        dict(platform_motion_stable=False),dict(uncertainty_m=float('nan'))):
            self.assertFalse(add(self.g,p,sensors,replace(self.context,**changes)))
            self.assertFalse(any(c.observed for c in self.g.snapshot(10.01).cells))

    def test_physical_requires_field_chain(self):
        self.c['source_type']='PHYSICAL'
        g=PitSurfaceGrid(self.c,self.s)
        p,sensors=cloud(g)
        ctx=replace(self.context,source_type='PHYSICAL',transform_state='FIELD_VALIDATED')
        self.assertFalse(add(g,p,sensors,ctx))
        self.assertIn('FIELD_SPATIAL_CHAIN',g.reason)

    def test_duplicates_and_capacity_fail_closed(self):
        p,sensors=cloud(self.g)
        self.assertTrue(add(self.g,p,sensors,self.context))
        self.assertFalse(add(self.g,p,sensors,self.context))
        self.assertEqual(self.g.reason,'DUPLICATE_OR_REGRESSING_FRAME')
        self.assertFalse(add(self.g,np.tile(p,(20,1)),np.tile(sensors,20),self.context,10.1,2))

    def test_template_is_not_configured(self):
        import yaml
        c=yaml.safe_load((ROOT/'config/perception/phase2b_surface.template.yaml').read_text(encoding='utf-8'))
        with self.assertRaises(ValueError):
            PitSurfaceGrid(c,self.s)
