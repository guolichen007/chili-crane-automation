"""R1 counterexamples: all coordinates here are non-production synthetic fixtures."""
import copy
import sys
import time
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT = Path(__file__).resolve().parents[1]
for package in ("perception", "slam"):
    sys.path.insert(0, str(ROOT / ("src/chili_crane_"+package+"/src")))
from chili_crane_perception.synthetic import fixture, scene, pit, box_shape
from chili_crane_perception.pipeline import GrabPerceptionPipeline, LatestFrameSlot
from chili_crane_perception.geometry import GrabGeometryModel, PitBoundaryModel, envelope
from chili_crane_perception.registration import GrabModelRegistration
from chili_crane_perception.state_contract import epoch_diagnostic, validity_for_reason, field_extrinsic_eligible
from chili_crane_slam.pointcloud import CANONICAL


def core_fixture():
    c = fixture()
    c["model"]["TRACKING_CORE"] = box_shape([.8,.6,.3])
    return c


def connected_material(config, pose, stamp, opening="CLOSED", penetration=0., rough=False,
                       noise=.001, seed=1, occlusion=0., one_sensor=False, span=.02, count_side=101):
    # A continuous surface (not isolated outliers), grid spacing below cluster radius.
    model = config["model"]
    if opening == "TRANSITION":
        model = dict(model, OPEN=model["TRACKING_CORE"])
    grab = scene(model,pose,stamp,opening=opening,noise=noise,seed=seed,occlusion=occlusion,
                 one_sensor=one_sensor,span=span)
    height = pose[2,3] + (-.6 + penetration*1.2)
    if opening == "OPEN":
        height = pose[2,3] + (-.45+penetration*.9)
    u = np.linspace(-3,3,count_side)
    xx,yy = np.meshgrid(u,u)
    z = np.full(xx.size,height)
    if rough:
        z += .025*xx.ravel()+.02*np.sin(yy.ravel()*4)
    # Visibility: lower Grab surface is actually occluded by the material.
    grab = grab[grab["z"] >= height-.02]
    material = np.zeros(xx.size,dtype=CANONICAL)
    material["x"],material["y"],material["z"] = xx.ravel(),yy.ravel(),z
    material["timestamp"] = stamp
    material["sensor_id"] = 0 if one_sensor else np.arange(len(material))%2
    points = np.empty(len(grab)+len(material),dtype=CANONICAL)
    points[:len(grab)],points[len(grab):] = grab,material
    return points


class R1TrackingTests(unittest.TestCase):
    def test_epoch_and_validity_do_not_refresh_on_publication(self):
        self.assertEqual((10.,.7),tuple(round(v,5) for v in epoch_diagnostic(10.,10.7)))
        for reason,valid in (("INPUT_TIMEOUT","STALE"),("MODEL_NOT_CONFIGURED","NOT_CONFIGURED"),
                             ("WEAK_OBSERVABILITY","DEGRADED"),("unclassified","UNKNOWN")):
            self.assertEqual(valid,validity_for_reason(reason))
        for state in ("BENCH","CANDIDATE","NOT_CONFIGURED"):
            self.assertFalse(field_extrinsic_eligible(state,"id","PHYSICAL","PHYSICAL"))
        self.assertTrue(field_extrinsic_eligible("FIELD_VALIDATED","id","PHYSICAL","PHYSICAL"))

    def test_clearance_has_no_fake_negative_penetration(self):
        boundary=PitBoundaryModel(pit(),"map")
        self.assertEqual(0,boundary.signed_clearance([[3,0],[5,0],[5,1],[3,1]]))
        self.assertEqual(0,boundary.signed_clearance([[5,0],[6,0],[6,1],[5,1]]))
        self.assertEqual(0,boundary.signed_clearance([[3,0],[4,0],[4,1],[3,1]]))
        self.assertEqual(1,boundary.signed_clearance([[2,0],[3,0],[3,1],[2,1]]))

    def test_translated_model_support_and_normalized_observability(self):
        c=fixture();pose=np.eye(4);pose[2,3]=2
        model=GrabGeometryModel(c["model"]);reg=GrabModelRegistration(model,c["policy"])
        original=reg.register(scene(c["model"],pose,10),pose,"CLOSED")
        shifted=copy.deepcopy(c);offset=np.array([2.,-3.,1.])
        shape=shifted["model"]["CLOSED"]
        for k in ("sampled_surface","safety_vertices","bottom_reference_xyz","top_reference_xyz","suspension_reference_xyz"):
            shape[k]=(np.array(shape[k])+offset).tolist()
        seed=pose.copy();seed[:3,3]-=offset
        translated=GrabModelRegistration(GrabGeometryModel(shifted["model"]),c["policy"]).register(
            scene(c["model"],pose,10),seed,"CLOSED")
        self.assertEqual(original.sectors,translated.sectors)
        self.assertAlmostEqual(original.condition,translated.condition,delta=original.condition*.1)
        from chili_crane_slam.extrinsic_solver import skew
        basis=np.eye(6);basis[:3,3:]=-translated.pose[:3,:3]@skew(offset)
        centered=basis@translated.covariance@basis.T
        np.testing.assert_allclose(centered,original.covariance,rtol=.25,atol=1e-8)

    def test_core_transition_longer_than_timeout_and_missing_fallback(self):
        c=core_fixture();p=GrabPerceptionPipeline(c);pose=np.eye(4);pose[2,3]=2
        for i in range(3):
            p.process(scene(c["model"],pose,10+i*.1),10+i*.1,"CLOSED","SYNTHETIC",np.eye(6)*1e-6)
        for i in range(8):
            stamp=10.3+i*.1
            model=dict(c["model"],OPEN=c["model"]["TRACKING_CORE"])
            r=p.process(scene(model,pose,stamp,opening="TRANSITION",span=.02),stamp,
                        "TRANSITION","SYNTHETIC",np.eye(6)*1e-6)
            self.assertEqual("TRACKED",r["tracking"],r["reasons"])
            self.assertTrue(r["accepted"])
            self.assertFalse(r["valid_for_control"])
        c=fixture();p=GrabPerceptionPipeline(c)
        for i in range(3):
            p.process(scene(c["model"],pose,10+i*.1),10+i*.1,"CLOSED","SYNTHETIC",np.eye(6)*1e-6)
        for i in range(8):
            stamp=10.3+i*.1
            r=p.process(scene(c["model"],pose,stamp,opening="TRANSITION"),stamp,
                        "TRANSITION","SYNTHETIC",np.eye(6)*1e-6)
        self.assertEqual("LOST",r["tracking"])

    def test_io_opposite_geometry_conflict(self):
        for observed,io in (("CLOSED","OPEN"),("OPEN","CLOSED")):
            c=fixture();p=GrabPerceptionPipeline(c);pose=np.eye(4);pose[2,3]=2
            r=p.process(scene(c["model"],pose,10,opening=observed),10,io,"SYNTHETIC",np.eye(6)*1e-6)
            self.assertIn("GEOMETRY_IO_"+io+"_CONFLICT",r["reasons"])
            self.assertFalse(r["accepted"])

    def test_material_contact_flat_rough_open_closed_transition(self):
        for opening in ("OPEN","CLOSED","TRANSITION"):
            for rough in (False,True):
                c=core_fixture();c["policy"]["maximum_input_points"]=20000
                p=GrabPerceptionPipeline(c);pose=np.eye(4);pose[2,3]=2
                start_open="CLOSED" if opening=="TRANSITION" else opening
                for i in range(3):
                    p.process(scene(c["model"],pose,10+i*.1,opening=start_open,span=.02),10+i*.1,
                              start_open,"SYNTHETIC",np.eye(6)*1e-6)
                for i,depth in enumerate((-.1,0,.25,.5)):
                    stamp=10.3+i*.1
                    points=connected_material(c,pose,stamp,opening,depth,rough)
                    r=p.process(points,stamp,opening,"SYNTHETIC",np.eye(6)*1e-6)
                    self.assertEqual("TRACKED",r["tracking"],(opening,rough,depth,r["reasons"]))


if __name__=="__main__":
    unittest.main()
