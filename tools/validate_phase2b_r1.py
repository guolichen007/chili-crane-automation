#!/usr/bin/env python3
"""Bounded synthetic R1 counterexamples and stage-separated, non-SLA timing."""
import argparse
import copy
from dataclasses import asdict,replace
import json
from pathlib import Path
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/chili_crane_perception/src'),str(ROOT/'src/chili_crane_control/src')]
from chili_crane_perception.phase2b_synthetic import fixture,scene,ingest
from chili_crane_perception.surface_mask import PitSurfaceBuilder
from chili_crane_perception.surface_payload import sparse_cells,expand_sparse
from chili_crane_control.geofence import OperationalGeofence
from chili_crane_control.grasp_planning import GraspPlanner

STAGES=('transform','grab_mask','pit_filtering','cell_ingest','temporal_fusion','height_bounds','slope_roughness',
        'snapshot_serialization','dds_payload_preparation','esdf_reject','exact_candidate_geometry','ranking','proposal_serialization')


def stats(values):
    return dict(P50_MS=float(np.percentile(values,50)),P95_MS=float(np.percentile(values,95)),MAX_MS=max(values))


def validate(repetitions=8):
    c,s,gc,ctx,grab=fixture(ROOT/'config/safety/pit_operational_geofence.template.yaml')
    counters={name:0 for name in ('SPATIAL_JITTER_FALSE_RESET_COUNT','STALE_GRAB_MASK_ACCEPTED_COUNT',
        'TRAVERSAL_ORDER_BIAS_COUNT','EMPTY_PIT_PROPOSAL_COUNT','OLD_GENERATION_RECOMPUTE_COUNT')}
    b=PitSurfaceBuilder(c,s);p,sensors=scene(b.grid,'flat')
    for i in range(4):
        a=.0002*i;t=np.eye(4);t[:3,:3]=[[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]];t[2,3]=a
        active=replace(ctx,transform=tuple(t.ravel()),uncertainty_m=.01+.001*i,motion_evidence_id='motion-'+str(i))
        ok=ingest(b,p,sensors,active,grab,10+i*.04,i+1)
        counters['SPATIAL_JITTER_FALSE_RESET_COUNT']+=int(not ok or len(b.grid.frames)!=i+1)
    snap=b.grid.snapshot(10.13);planner=GraspPlanner(c,OperationalGeofence(gc,s))
    first=planner.plan(snap,10.13,snap.context,snap.generation)
    before=planner.compute_count
    for i in range(10):
        planner.plan(snap,10.14+i*.01,snap.context,snap.generation)
    counters['OLD_GENERATION_RECOMPUTE_COUNT']+=planner.compute_count-before
    assert first.measurement_epoch==snap.window_end
    frame=dict(epoch=10.,mid_epoch=10.,source_counter=1,frame_id=ctx.source_frame,span_sec=.02,source_type='SYNTHETIC',calibration_id=ctx.calibration_id)
    old=PitSurfaceBuilder(c,s)
    counters['STALE_GRAB_MASK_ACCEPTED_COUNT']+=int(old.ingest(p,sensors,frame,ctx,replace(grab,epoch=9.9),10.01))
    assert old.grid.reason=='GRAB_MASK_TIME_ALIGNMENT_REJECTED'
    mc=copy.deepcopy(c);mc['maximum_candidates']=4;mc['kernel']['capture_depth_m']=2.
    mc['planning']['maximum_slope_rad']=1.5
    mound=PitSurfaceBuilder(mc,s);p,sensors=scene(mound.grid,'flat')
    p[:,2]=.3+.6*np.exp(-((p[:,0]-1.1)**2+(p[:,1]-1.1)**2))
    for i in range(2):assert ingest(mound,p,sensors,ctx,grab,10+i*.05,i+1)
    snap=mound.grid.snapshot(10.06);ids=[]
    for cells in (snap.cells,tuple(reversed(snap.cells)),tuple(np.random.default_rng(37).permutation(snap.cells))):
        result=GraspPlanner(mc,OperationalGeofence(gc,s)).plan(replace(snap,cells=cells),10.06,snap.context,snap.generation)
        assert result.selected is not None and min(result.selected.target_xy)>.5
        ids.append(result.selected.candidate_id)
    counters['TRAVERSAL_ORDER_BIAS_COUNT']+=int(len(set(ids))!=1)
    for z in (0.,.02,.04):
        empty=PitSurfaceBuilder(c,s);p,sensors=scene(empty.grid,'flat');p[:,2]=z
        for i in range(2):assert ingest(empty,p,sensors,ctx,grab,10+i*.05,i+1)
        snap=empty.grid.snapshot(10.06)
        result=GraspPlanner(c,OperationalGeofence(gc,s)).plan(snap,10.06,snap.context,snap.generation)
        counters['EMPTY_PIT_PROPOSAL_COUNT']+=int(result.selected is not None)
    try:
        from chili_crane_perception.surface_transport import snapshot_to_message,proposal_to_message
        from rclpy.serialization import serialize_message
        cdr=True
    except ImportError:
        cdr=False
    workloads={};capacity=[];maximum_bytes=0
    for workload,resolution in (('FAST_REJECT',.25),('FULL_VALID_PROPOSAL',.25),('DENSE_SURFACE',.125)):
        totals=[];stages={name:[] for name in STAGES};measurements=[]
        for repeat in range(repetitions):
            cfg=copy.deepcopy(c);cfg['resolution_m']=resolution
            builder=PitSurfaceBuilder(cfg,s);planner=GraspPlanner(cfg,OperationalGeofence(gc,s))
            points,sensors=scene(builder.grid,'flat')
            begin=time.perf_counter()
            if workload=='FAST_REJECT':
                builder.ingest(points,sensors,frame,ctx,replace(grab,epoch=9.9),10.01)
            else:
                for i in range(2):assert ingest(builder,points,sensors,ctx,grab,10+i*.05,i+1)
            snap=builder.grid.snapshot(10.06)
            result=planner.plan(snap,10.06,snap.context,snap.generation)
            assert (result.selected is not None)==(workload!='FAST_REJECT')
            times={**builder.grid.stage_ms,**planner.stage_ms}
            tick=time.perf_counter()
            if cdr:
                message=snapshot_to_message(snap,10.06,2)
            else:
                # Native JSON diagnostic model, explicitly NOT DDS/CDR evidence.
                message=json.dumps(dict(width=snap.width,height=snap.height,origin=snap.origin,
                    context=asdict(snap.context) if snap.context else None,cells=[asdict(v) for v in sparse_cells(snap)]))
            times['snapshot_serialization']=(time.perf_counter()-tick)*1000
            tick=time.perf_counter()
            payload=serialize_message(message) if cdr else message.encode('utf-8')
            times['dds_payload_preparation']=(time.perf_counter()-tick)*1000 if cdr else 0.
            tick=time.perf_counter()
            if cdr:
                proposal=proposal_to_message(result,message,.01)
                serialize_message(proposal)
            else:json.dumps(asdict(result))
            times['proposal_serialization']=(time.perf_counter()-tick)*1000
            totals.append((time.perf_counter()-begin)*1000)
            maximum_bytes=max(maximum_bytes,len(payload))
            for name in STAGES:stages[name].append(times.get(name,0.))
            measurements.append(dict(input_points=len(points),active_cells=snap.pit_active_cell_count,
                observed_cells=snap.observed_cell_count,serialized_cells=len(sparse_cells(snap)),
                serialized_bytes=len(payload),**planner.metrics))
        workloads[workload]=dict(total=stats(totals),stages={name:stats(values) for name,values in stages.items()},
            measurements=measurements,dds_payload_preparation_status='PASS_CDR' if cdr else 'NOT_RUN_NO_ROS',
            setup_excluded='configuration, active-mask precompute, input fixture; cold planner layout included')
    for res in (.5,.25,.125):
        cfg=copy.deepcopy(c);cfg['resolution_m']=res
        builder=PitSurfaceBuilder(cfg,s);p,sensors=scene(builder.grid,'partial')
        for i in range(2):assert ingest(builder,p,sensors,ctx,grab,10+i*.05,i+1)
        snap=builder.grid.snapshot(10.06);sparse=sparse_cells(snap)
        expanded=expand_sparse(sparse,snap.width,snap.height,cfg['maximum_grid_cells'])
        assert len(sparse)<=snap.width*snap.height//2
        assert sum(v.observed for v in expanded)==len(sparse)
        n=len(serialize_message(snapshot_to_message(snap,10.06,2))) if cdr else len(json.dumps([asdict(v) for v in sparse]).encode())
        if cdr:assert n<=16384+192*len(sparse)
        capacity.append(dict(grid_cells=snap.width*snap.height,configured_maximum=cfg['maximum_grid_cells'],
            serialized_cells=len(sparse),serialized_bytes=n,unknown_implicit=True))
    assert not any(counters.values()),counters
    return dict(status='PASS_SYNTHETIC_NOT_FIELD',counters=counters,workloads=workloads,capacity=capacity,
        repetitions=repetitions,max_planning_snapshot_bytes=maximum_bytes,
        payload_encoding='ROS2_CDR' if cdr else 'NATIVE_JSON_DIAGNOSTIC_NOT_DDS',
        physical_output_enabled=False,automatic_control_enabled=False,camera_control_authority=False,valid_for_control=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path);parser.add_argument('--repetitions',type=int,default=8)
    args=parser.parse_args()
    if not 2<=args.repetitions<=64:parser.error('repetitions must be bounded 2..64')
    result=validate(args.repetitions);output=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
    if args.output:args.output.write_text(output,encoding='utf-8')
    print(output)
