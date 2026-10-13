#!/usr/bin/env python3
"""Seeded sequence and dense-contact acceptance. SYNTHETIC evidence, not field probability."""
import argparse
import copy
import json
from pathlib import Path
import sys
import time
import tracemalloc
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"tests_static"))
from test_phase2a_r1 import (core_fixture, connected_material, scene,
    GrabPerceptionPipeline, PitBoundaryModel, pit, LatestFrameSlot, CANONICAL)
from test_operational_geofence import geofence_fixture, OperationalGeofence


def verify_geometry(result, pipeline, truth, stamp, counts):
    counts["state_timestamp_violation"] += int(result["state_epoch"] != stamp)
    if result["accepted"]:
        counts["wrong_reacquisition"] += int(np.linalg.norm(pipeline.tracker.pose[:3,3]-truth[:3,3]) > .15)
    env=result["envelope"]
    if env is None:
        return
    from chili_crane_slam.calibration_geometry import transform
    lowest=float(transform(pipeline.model.safety(result["opening"]),truth)[:,2].min())
    counts["bottom_bound_violation"] += int(env.bottom_lower_bound > lowest+1e-6)
    clearance=PitBoundaryModel(pit(),"map").clearance(env)
    from shapely.geometry import Polygon
    ground_truth=Polygon(transform(pipeline.model.safety(result["opening"]),truth)[:,:2]).convex_hull
    boundary=PitBoundaryModel(pit(),"map")
    true_inside=boundary.polygon.covers(ground_truth)
    true_distance=ground_truth.distance(boundary.polygon.boundary) if true_inside else 0.
    counts["optimistic_clearance"] += int(clearance["clearance_defined"] and
        (not true_inside or clearance["conservative_clearance_m"] > true_distance+1e-5))


def perturb_extrinsic(points, bias):
    # Actual coherent common-frame error, not just a changed covariance number.
    xyz=np.column_stack([points[k] for k in ("x","y","z")]).astype(float)
    xyz=xyz@bias[:3,:3].T+bias[:3,3]
    for axis,key in enumerate(("x","y","z")):
        points[key]=xyz[:,axis]
    return points


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path)
    parser.add_argument("--sequences",type=int,default=200)
    args=parser.parse_args()
    if not 200 <= args.sequences <= 1000:
        raise ValueError("Acceptance requires 200..1000 sequences")
    rng=np.random.default_rng(20261013)
    counts={k:0 for k in ("false_TRACKED","wrong_reacquisition","transition_tracking_loss",
        "optimistic_clearance","bottom_bound_violation","state_timestamp_violation",
        "geofence_false_safe","directional_limit_violation")}
    coverage={k:0 for k in ("material","transition","dropout","single_sensor","io_conflict",
        "multiple_candidates","wall_clutter","global_reacquisition","extrinsic_perturbation",
        "axis_to_map_mismatch", "geofence_state_sequence")}
    timings=[]
    failures=[]
    for sequence in range(args.sequences):
        c=core_fixture();c["policy"]["maximum_input_points"]=200000
        p=GrabPerceptionPipeline(c)
        truth=np.eye(4);truth[:3,3]=[rng.uniform(-.15,.15),rng.uniform(-.15,.15),2.]
        truth[:3,:3]=Rotation.from_euler("xyz",[rng.uniform(-.03,.03),rng.uniform(-.03,.03),
            rng.choice([0.,np.pi])+rng.uniform(-.05,.05)]).as_matrix()
        ext=np.eye(6)*rng.uniform(1e-6,1e-5)
        bias_rng=np.random.default_rng(20261013+sequence)
        bias=np.eye(4)
        bias[:3,3]=bias_rng.uniform(-.0007,.0007,3)
        bias[:3,:3]=Rotation.from_rotvec(bias_rng.uniform(-.0002,.0002,3)).as_matrix()
        coverage["extrinsic_perturbation"]+=1
        opening="OPEN" if sequence%2 else "CLOSED"
        for frame in range(3):
            stamp=10+frame*.1
            r=p.process(perturb_extrinsic(scene(c["model"],truth,stamp,opening=opening,noise=.001,span=.02),bias),stamp,
                        opening,"SYNTHETIC",ext)
        assert r["tracking"]=="TRACKED", (sequence,r["reasons"])
        kind=sequence%10
        for frame in range(8 if kind==4 else 4):
            stamp=10.3+frame*.1
            target=truth.copy()
            target[0,3]+=.005*frame
            target[:3,:3]=truth[:3,:3]@Rotation.from_euler("y",.003*frame).as_matrix()
            op=opening
            if kind in (0,1,2,3):
                coverage["material"]+=1
                points=connected_material(c,target,stamp,op,[-.1,0.,.25,.5][frame],kind%2==1,
                    noise=.001,seed=sequence*10+frame,one_sensor=kind==3,count_side=71)
            elif kind==4:
                op="TRANSITION";coverage["transition"]+=1
                model=dict(c["model"],OPEN=c["model"]["TRACKING_CORE"])
                points=scene(model,target,stamp,opening=op,noise=.001,span=.02)
            elif kind==5:
                coverage["dropout"]+=1
                points=np.zeros(100,dtype=CANONICAL);points["x"]=np.linspace(4,5,100)
                points["z"]=1;points["timestamp"]=stamp
            elif kind==6:
                coverage["single_sensor"]+=1
                points=scene(c["model"],target,stamp,opening=op,noise=.002,occlusion=.2,
                    outliers=40,one_sensor=True,span=.03,velocity=[.05,0,0],seed=sequence+frame)
            elif kind==7:
                coverage["io_conflict"]+=1
                op="CLOSED" if opening=="OPEN" else "OPEN"
                points=scene(c["model"],target,stamp,opening=opening,span=.02)
            elif kind==8:
                coverage["wall_clutter"]+=1
                points=scene(c["model"],target,stamp,opening=op,outliers=150,span=.04,seed=sequence+frame)
            else:
                coverage["multiple_candidates"]+=1
                other=target.copy();other[0,3]+=4
                a=scene(c["model"],target,stamp,opening=op,span=.02)
                b=scene(c["model"],other,stamp,opening=op,span=.02)
                points=np.empty(len(a)+len(b),dtype=CANONICAL);points[:len(a)],points[len(a):]=a,b
            r=p.process(perturb_extrinsic(points,bias),stamp,op,"SYNTHETIC",ext)
            r["opening"]=op
            timings.append(r["total_seconds"])
            if kind==4:
                counts["transition_tracking_loss"]+=int(r["tracking"]!="TRACKED" or not r["accepted"])
                if r["tracking"]!="TRACKED" or not r["accepted"]:
                    failures.append(dict(sequence=sequence,frame=frame,tracking=r["tracking"],reasons=r["reasons"]))
            if kind==7:
                assert not r["accepted"], (sequence,r["reasons"])
            if kind==5:
                counts["false_TRACKED"]+=int(r["accepted"] or r["tracking"]=="TRACKED")
            verify_geometry(r,p,target,stamp,counts)
        # True global LOST -> multi-hypothesis reacquisition at a NEW truth pose.
        if sequence%20==0:
            coverage["global_reacquisition"]+=1
            p.tracker.predict(12.);p.tracker.missing(12.)
            target=truth.copy();target[0,3]+=1.
            for frame in range(3):
                stamp=12.1+frame*.1
                r=p.process(perturb_extrinsic(scene(c["model"],target,stamp,opening=opening,span=.02),bias),stamp,
                            opening,"SYNTHETIC",ext)
            counts["wrong_reacquisition"]+=int(r["tracking"]!="TRACKED" or
                np.linalg.norm(p.tracker.pose[:3,3]-target[:3,3])>.15)
        # Independent axis/limit and map/calibration fuzz, never a pose correction.
        gc,s,x,y,d=geofence_fixture()
        g=OperationalGeofence(gc,s)
        for step in range(3):
            coverage["geofence_state_sequence"]+=1
            xs=dict(x,position_m=float(rng.uniform(-1.5,1.5)))
            ys=dict(y,position_m=float(rng.uniform(-1.5,1.5)))
            if step==1:
                xs["evidence_age_sec"]=.3
            sr=g.evaluate(xs,ys,d)
            counts["geofence_false_safe"]+=int(step==1 and sr["automatic_eligibility"])
        if sequence%13==0:
            coverage["axis_to_map_mismatch"]+=1
            invalid=copy.deepcopy(gc);invalid["axis_to_map"]["x_calibration_id"]="different"
            try:
                OperationalGeofence(invalid,s)
            except ValueError:
                pass
            else:
                counts["geofence_false_safe"]+=1
        bad=sequence%10
        if bad==0: y.update(left_limit=True,right_limit=True)
        elif bad==1: y.update(left_limit=True)
        elif bad==2: y.update(position_m=-2.2)
        elif bad==3: x.update(evidence_age_sec=.3)
        elif bad==4: x.update(homed=False)
        elif bad==5: x.update(calibration_id="wrong")
        elif bad==6: d.update(map_id="wrong")
        elif bad==7: d.update(wall_collision_risk=True)
        elif bad==8: y.update(position_m=-2.2,left_limit=True)
        else: x.update(position_m=2.36)
        gr=g.evaluate(x,y,d)
        if bad<8:
            counts["geofence_false_safe"]+=int(gr["automatic_eligibility"])
        counts["directional_limit_violation"]+=int((y["left_limit"] and gr["y_negative"])
                                                  or (y["right_limit"] and gr["y_positive"]))
    # Adversarial pure material and wall: no initial Grab ground truth exists.
    for wall in (False,True):
        c=core_fixture();c["policy"]["maximum_input_points"]=200000
        p=GrabPerceptionPipeline(c);pose=np.eye(4);pose[2,3]=2
        for frame in range(4):
            stamp=20+frame*.1
            points=connected_material(c,pose,stamp,count_side=81)[-6561:].copy()
            if wall:
                points["z"]=points["y"]+3;points["y"]=0
            r=p.process(points,stamp,"CLOSED","SYNTHETIC",np.eye(6)*1e-6)
            counts["false_TRACKED"]+=int(r["tracking"]=="TRACKED")
    # Dense near maximum budget, latest-wins and retained Python allocation bounds.
    c=core_fixture();c["policy"]["maximum_input_points"]=200000
    p=GrabPerceptionPipeline(c);pose=np.eye(4);pose[2,3]=2
    for frame in range(3):
        stamp=30+frame*.1
        p.process(scene(c["model"],pose,stamp,span=.02),stamp,"CLOSED","SYNTHETIC",np.eye(6)*1e-6)
    tracemalloc.start();dense_times=[];retained=[];dense_count=0
    for frame in range(8):
        stamp=30.3+frame*.1
        points=connected_material(c,pose,stamp,penetration=.25,rough=True,count_side=444)
        # Bounded isolated clusters/wall clutter at ROI fringe, away from true Grab.
        clutter=np.zeros(1000,dtype=CANONICAL)
        clutter["x"]=2.3;clutter["y"]=np.linspace(-2,2,len(clutter));clutter["z"]=1.5;clutter["timestamp"]=stamp
        combined=np.empty(len(points)+len(clutter),dtype=CANONICAL)
        combined[:len(points)],combined[len(points):]=points,clutter
        dense_count=len(combined)
        slot=LatestFrameSlot()
        for _ in range(20):slot.put(combined)
        assert slot.dropped==19 and slot.take() is combined and slot.take() is None
        r=p.process(combined,stamp,"CLOSED","SYNTHETIC",np.eye(6)*1e-6)
        assert r["tracking"]=="TRACKED", r["reasons"]
        dense_times.append(r["total_seconds"])
        retained.append(tracemalloc.get_traced_memory()[0])
    current,peak=tracemalloc.get_traced_memory();tracemalloc.stop()
    assert dense_count<=200000 and dense_count>=190000
    assert retained[-1]-retained[1]<10*1024*1024, retained
    report=dict(status="PASS_SYNTHETIC_NOT_FIELD" if not any(counts.values()) else "FAIL",
        seed=20261013,sequences=args.sequences,coverage=coverage,violations=counts,
        failure_samples=failures[:50],
        performance_ms=dict(P50=float(np.percentile(timings,50)*1000),P95=float(np.percentile(timings,95)*1000),MAX=float(max(timings)*1000)),
        dense=dict(input_points=dense_count,frames=8,P50_ms=float(np.percentile(dense_times,50)*1000),
            P95_ms=float(np.percentile(dense_times,95)*1000),MAX_ms=float(max(dense_times)*1000),
            python_peak_bytes=peak,retained_growth_bytes=retained[-1]-retained[1],
            memory_scope="Python tracemalloc, not native heap or FIELD capacity guarantee"),
        valid_for_control=False,PHYSICAL_ACTUATION_STATUS="NOT_RUN")
    if args.output:
        if args.output.resolve().is_relative_to(ROOT) and ".local-data" not in args.output.parts:
            raise ValueError("Evidence must be outside versioned source")
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps(report,ensure_ascii=False,allow_nan=False))
    assert not any(counts.values()), counts


if __name__=="__main__":
    main()
