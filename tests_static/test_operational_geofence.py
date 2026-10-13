"""OperationalGeofence necessary-condition policies, synthetic fixtures only."""
import copy
import sys
import unittest
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src/chili_crane_control/src"))
from chili_crane_control.geofence import OperationalGeofence


def pit():
    return dict(pit_id="SYNTHETIC_PIT", config_state="VALID",
                polygon_map_m=[[-4,-4],[4,-4],[4,4],[-4,4]],
                wall_top_z_m=3, pit_bottom_z_m=0, safety_inset_m=.1)


def geofence_fixture():
    import yaml
    c = yaml.safe_load((ROOT/"config/safety/pit_operational_geofence.template.yaml").read_text(encoding="utf-8"))
    s = dict(config_state="VALID", schema_version=1, frame_id="map", map_id="SYNTHETIC_MAP",
             semantic_version="synthetic-v1", pits=[pit()],track=dict(track_id="SYNTHETIC_TRACK",
                 origin_map_m=[0,0],direction_map_xy=[1,0],valid_x_range_m=[-8,8]))
    c.update(config_state="VALID", config_version="synthetic-policy-v1", geofence_id="SYNTHETIC_GEOFENCE",
             geofence_version="synthetic-v1", semantic_map_id=s["map_id"], semantic_version=s["semantic_version"],
             pit_id=pit()["pit_id"], source_type="SYNTHETIC", evidence_timeout_sec=.2, direction_probe_m=.1)
    for axis in ("x", "y"):
        c[axis]["calibration_id"] = "SYNTHETIC_"+axis
    c["y"].update(limit_semantics="PIT_DIRECTION_LIMIT", negative_limit_signal="y_left_limit",
                  positive_limit_signal="y_right_limit", negative_expected_window_m=[-2.3,-2.1],
                  positive_expected_window_m=[2.1,2.3])
    c["axis_to_map"].update(calibration_id="SYNTHETIC_AXIS_TO_MAP",state="SYNTHETIC_VALIDATED",
        map_id=s["map_id"],semantic_version=s["semantic_version"],pit_id=c["pit_id"],
        x_calibration_id=c["x"]["calibration_id"],y_calibration_id=c["y"]["calibration_id"],
        origin_map_xy_m=[0,0],x_direction_map_xy=[1,0],y_direction_map_xy=[0,1],
        x_scale=1,y_scale=1,x_offset_m=0,y_offset_m=0)
    c["static_margin"].update(grab_horizontal_radius_m=1,maximum_swing_m=.1,position_uncertainty_m=.1,
        calibration_uncertainty_m=.1,stop_margin_m=.1,additional_margin_m=.1)
    x = dict(validity="VALID",position_m=0.,fault=False,homed=True,communication_ok=True,
             positive_limit=False,negative_limit=False,calibration_id=c["x"]["calibration_id"],
             evidence_age_sec=0.,evidence={"source_type":"SYNTHETIC"})
    y = dict(validity="VALID",position_m=0.,fault=False,left_limit=False,right_limit=False,
             known_signals=["y_left_limit","y_right_limit"],calibration_id=c["y"]["calibration_id"],
             evidence_age_sec=0.,evidence={"source_type":"SYNTHETIC"})
    d = dict(wall_collision_risk=False,clearance_defined=True,fresh=True,measurement_accepted=True,
             tracking_state="TRACKED",map_id=s["map_id"],semantic_version=s["semantic_version"],pit_id=c["pit_id"])
    return c,s,x,y,d


class GeofenceTests(unittest.TestCase):
    def setUp(self):
        self.c,self.s,self.x,self.y,self.d = geofence_fixture()
        self.g = OperationalGeofence(self.c,self.s)

    def evaluate(self):
        return self.g.evaluate(self.x,self.y,self.d)

    def test_template_fail_closed_and_does_not_copy_pit(self):
        import yaml
        c=yaml.safe_load((ROOT/"config/safety/pit_operational_geofence.template.yaml").read_text(encoding="utf-8"))
        self.assertNotIn("polygon_map_m",str(c))
        with self.assertRaises(ValueError):
            OperationalGeofence(c,self.s)

    def test_identity_mapping_and_provenance(self):
        self.c["axis_to_map"]["origin_map_xy_m"]=[1,1]
        g=OperationalGeofence(self.c,self.s)
        np.testing.assert_allclose(g.point(0,0),[1,1])
        r=self.evaluate()
        self.assertTrue(r["x_positive"])
        self.assertFalse(r["valid_for_control"])
        self.assertFalse(r["production_control_eligible"])
        self.assertEqual(64,len(r["provenance"]["config_sha256"]))

    def test_directional_escape_is_not_all_or_nothing(self):
        self.x["position_m"]=2.36
        r=self.evaluate()
        self.assertFalse(r["x_positive"])
        self.assertTrue(r["x_negative"])
        self.y.update(position_m=-2.2,left_limit=True)
        r=self.evaluate()
        self.assertFalse(r["y_negative"])
        self.assertTrue(r["y_positive"])
        self.assertTrue(r["stop_deenergize"])

    def test_limit_conflict_mismatch_expected_and_unknown(self):
        for changes,reason in ((dict(left_limit=True,right_limit=True),"Y_LIMIT_CONFLICT"),
             (dict(left_limit=True),"Y_POSITION_LIMIT_MISMATCH"),
             (dict(position_m=-2.2),"Y_EXPECTED_LIMIT_NOT_ASSERTED"),
             (dict(known_signals=[]),"Y_LIMIT_UNKNOWN")):
            y=dict(self.y,**changes)
            r=self.g.evaluate(self.x,y,self.d)
            self.assertIn(reason,r["reasons"])
            self.assertFalse(r["automatic_eligibility"])

    def test_servo_stale_unhomed_fault_calibration_and_stop_gate(self):
        for change in (dict(homed=False),dict(communication_ok=False),dict(fault=True),
                       dict(evidence_age_sec=.3),dict(calibration_id="wrong"),dict(position_m=float("nan"))):
            r=self.g.evaluate(dict(self.x,**change),self.y,self.d)
            self.assertFalse(r["x_geometry_eligible"])
            self.assertFalse(r["automatic_eligibility"])
        self.assertFalse(self.g.evaluate(self.x,self.y,self.d,age_x=.3)["automatic_eligibility"])

    def test_map_axis_identity_gate(self):
        for key in ("map_id","semantic_version","pit_id","x_calibration_id","calibration_id"):
            c=copy.deepcopy(self.c); c["axis_to_map"][key]="wrong" if key!="calibration_id" else "NOT_CONFIGURED"
            with self.assertRaises(ValueError):
                OperationalGeofence(c,self.s)

    def test_track_range_is_an_independent_constraint(self):
        s=copy.deepcopy(self.s);s["track"]["valid_x_range_m"]=[-1,1]
        g=OperationalGeofence(self.c,s)
        self.x["position_m"]=1.2
        self.assertFalse(g.evaluate(self.x,self.y,self.d)["automatic_eligibility"])
        self.x["position_m"]=.95
        result=g.evaluate(self.x,self.y,self.d)
        self.assertFalse(result["x_positive"])
        self.assertTrue(result["x_negative"])

    def test_concave_erosion_and_margin_monotonic(self):
        from shapely.geometry import Point
        s=copy.deepcopy(self.s)
        s["pits"][0]["polygon_map_m"]=[[-5,-5],[5,-5],[5,5],[1,5],[1,1],[-1,1],[-1,5],[-5,5]]
        g=OperationalGeofence(self.c,s)
        self.assertFalse(g.safe_region.covers(Point(0,2)))
        for delta in (.01,.1,.3):
            c=copy.deepcopy(self.c);c["static_margin"]["additional_margin_m"]+=delta
            smaller=OperationalGeofence(c,s)
            self.assertTrue(g.safe_region.buffer(1e-10).covers(smaller.safe_region))
            for x,y in np.random.default_rng(9).uniform(-5,5,(500,2)):
                p=Point(x,y)
                if smaller.safe_region.covers(p):
                    self.assertGreaterEqual(smaller.polygon.boundary.distance(p)+1e-10,smaller.margin)

    def test_fixed_prior_cannot_hide_dynamic_collision_or_stale(self):
        for changes in (dict(wall_collision_risk=True),dict(fresh=False),dict(map_id="old"),
                        dict(clearance_defined=False),dict(measurement_accepted=False)):
            d=dict(self.d,**changes);snapshot=copy.deepcopy(d)
            r=self.g.evaluate(self.x,self.y,d)
            self.assertTrue(r["inside"])
            self.assertFalse(r["automatic_eligibility"])
            self.assertEqual(snapshot,d)

    def test_physical_requires_field_axis_and_stop_proof(self):
        c=copy.deepcopy(self.c);c["source_type"]="PHYSICAL"
        with self.assertRaises(ValueError):
            OperationalGeofence(c,self.s)
        c["axis_to_map"]["state"]="FIELD_VALIDATED"
        g=OperationalGeofence(c,self.s)
        x=copy.deepcopy(self.x);y=copy.deepcopy(self.y)
        x["evidence"]["source_type"]=y["evidence"]["source_type"]="PHYSICAL"
        r=g.evaluate(x,y,self.d)
        self.assertFalse(r["automatic_eligibility"])
        self.assertIn("STOP_MODEL_NOT_CONFIGURED",r["reasons"])
