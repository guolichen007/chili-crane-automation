#!/usr/bin/env python3
"""Deterministic Phase2B finite counterexamples, NOT field or payload validation."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import time
import numpy as np
from shapely.geometry import Point,Polygon
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/chili_crane_perception/src'),str(ROOT/'src/chili_crane_control/src')]
from chili_crane_perception.phase2b_synthetic import fixture,scene,ingest,SCENARIOS
from chili_crane_perception.surface_mask import PitSurfaceBuilder
from chili_crane_perception.surface_disturbance import SurfaceDisturbanceTracker,SurfaceGraspEvent
from chili_crane_control.geofence import OperationalGeofence
from chili_crane_control.grasp_planning import GraspPlanner


def validate(sequences=64):
    counts={k:0 for k in ('UNKNOWN_AS_OBSERVED_COUNT','INTERPOLATED_CELL_CONTROL_ELIGIBLE_COUNT',
        'FALSE_SAFE_WALL_CANDIDATE_COUNT','GEOFENCE_ESCAPE_COUNT','STALE_SURFACE_PROPOSAL_COUNT',
        'PROVENANCE_MISMATCH_PROPOSAL_COUNT','GRAB_POINT_AS_MATERIAL_COUNT','DISTURBED_OLD_SURFACE_REUSED_COUNT')}
    rng=np.random.default_rng(20261013);durations=[];results=[];failures=[]
    coverage=dict(moving_grab=0,single_sensor=0,dropout=0,recovery=0,disturbance=0,generation_checks=0)
    c,s,gc,ctx,grab=fixture(ROOT/'config/safety/pit_operational_geofence.template.yaml')
    for name in SCENARIOS:
        sc=json.loads(json.dumps(s))
        if name=='concave': sc['pits'][0]['polygon_map_m']=[[-2,-2],[2,-2],[2,2],[.5,2],[.5,.5],[-.5,.5],[-.5,2],[-2,2]]
        b=PitSurfaceBuilder(c,sc);planner=GraspPlanner(c,OperationalGeofence(gc,sc));tracker=SurfaceDisturbanceTracker(c)
        p,sensors=scene(b.grid,name,rng)
        for i in range(3): ingest(b,p,sensors,ctx,grab,10+i*.05,i+1)
        snap=b.grid.snapshot(10.11)
        now=10.11
        expected=snap.context
        if name=='stale_surface' or name=='dropout': now=12.
        if name=='stale_transform': expected=replace(ctx,measurement_epoch=5.)
        if name=='map_mismatch': expected=replace(ctx,map_id='different')
        if name in ('disturbance','reobservation'):
            event=SurfaceGraspEvent('synthetic-event',1,10.12,'SYNTHETIC',ctx.map_id,ctx.semantic_version,ctx.pit_id,
                ctx.transform_id,c['kernel']['kernel_id'],((-.75,-.75),(.75,-.75),(.75,.75),(-.75,.75)))
            tracker.apply(b.grid,event,10.13)
            disturbed=b.grid.snapshot(10.13)
            for cell in disturbed.cells:
                if b.grid.cell_polygon(cell.index).intersects(Polygon(event.footprint_xy)) and cell.observed:
                    counts['DISTURBED_OLD_SURFACE_REUSED_COUNT']+=1
            if name=='reobservation':
                for i in range(3): ingest(b,p,sensors,ctx,grab,10.2+i*.05,4+i)
            snap=b.grid.snapshot(10.31);now=10.31;expected=snap.context
        start=time.perf_counter();proposal=planner.plan(snap,now,expected,snap.generation,tracker);durations.append((time.perf_counter()-start)*1000)
        if name in ('dropout','stale_surface') and proposal.selected: counts['STALE_SURFACE_PROPOSAL_COUNT']+=1
        if name in ('stale_transform','map_mismatch') and proposal.selected: counts['PROVENANCE_MISMATCH_PROPOSAL_COUNT']+=1
        for cell in snap.cells:
            if cell.observed and (cell.sample_count < c['minimum_cell_samples'] or cell.sensor_support_mask==0):
                counts['UNKNOWN_AS_OBSERVED_COUNT']+=1
            xy=b.grid.center(cell.index)
            if name in ('grab_occlusion','grab_connected') and abs(xy[0])<.4 and abs(xy[1])<.4 and cell.observed:
                counts['GRAB_POINT_AS_MATERIAL_COUNT']+=1
        for candidate in proposal.candidates:
            if candidate.accepted:
                fp=Polygon(candidate.footprint_xy)
                if not b.grid.polygon.covers(fp) or b.grid.polygon.boundary.distance(fp)<.2-1e-8:
                    counts['FALSE_SAFE_WALL_CANDIDATE_COUNT']+=1
                if not planner.geofence.safe_region.covers(Point(candidate.target_xy)):
                    counts['GEOFENCE_ESCAPE_COUNT']+=1
                if candidate.unknown_area_fraction>1e-8:
                    counts['UNKNOWN_AS_OBSERVED_COUNT']+=1
            if candidate.valid_for_control: counts['INTERPOLATED_CELL_CONTROL_ELIGIBLE_COUNT']+=1
        results.append(dict(scene=name,observed=sum(cell.observed for cell in snap.cells),selected=proposal.selected is not None,
                            reason=proposal.reason,candidates=len(proposal.candidates)))
    # Each deterministic sequence changes pile height, occlusion, support, disturbance and generation.
    for seq in range(sequences):
        b=PitSurfaceBuilder(c,s);planner=GraspPlanner(c,OperationalGeofence(gc,s));tracker=SurfaceDisturbanceTracker(c)
        previous=None
        for step in range(7):
            name=('single_mound','single_mound','grab_connected','sensor0','dropout','flat','multiple_mounds')[step]
            epoch=20+seq*2+step*.08
            p,sensors=scene(b.grid,name,rng);p[:,2]=np.maximum(.05,p[:,2]-.08*step)
            active_grab=replace(grab,footprint_xy=tuple((x+.5,y) for x,y in grab.footprint_xy)) if step in (2,3) else grab
            if step==2:
                active_grab=replace(active_grab,motion_stable=False)
                coverage['moving_grab']+=1
            if step==3:coverage['single_sensor']+=1
            if step==4:coverage['dropout']+=1
            if step in (5,6):coverage['recovery']+=1
            start=time.perf_counter()
            if name=='dropout': b.grid.clear('SYNTHETIC_DROPOUT')
            else: ingest(b,p,sensors,ctx,active_grab,epoch,step+1)
            if step==3 and b.grid.context is not None:
                event=SurfaceGraspEvent(f'event-{seq}',seq+1,epoch+.015,'SYNTHETIC',ctx.map_id,ctx.semantic_version,
                    ctx.pit_id,ctx.transform_id,c['kernel']['kernel_id'],((.5,.5),(1,.5),(1,1),(.5,1)))
                tracker.apply(b.grid,event,epoch+.02)
                coverage['disturbance']+=1
            snapshot=b.grid.snapshot(epoch+.025)
            proposal=planner.plan(snapshot,epoch+.025,snapshot.context,snapshot.generation,tracker)
            if previous and previous.context:
                coverage['generation_checks']+=1
                if planner.plan(previous,epoch+.025,previous.context,snapshot.generation).selected:
                    counts['DISTURBED_OLD_SURFACE_REUSED_COUNT']+=1;failures.append([seq,step,'old-generation'])
            if name=='dropout' and proposal.selected:
                counts['STALE_SURFACE_PROPOSAL_COUNT']+=1;failures.append([seq,step,'dropout'])
            if step==2 and (proposal.selected or any(cell.observed for cell in snapshot.cells)):
                counts['GRAB_POINT_AS_MATERIAL_COUNT']+=1;failures.append([seq,step,'moving-grab-fused'])
            for candidate in proposal.candidates:
                if not candidate.accepted:continue
                footprint=Polygon(candidate.footprint_xy)
                if not b.grid.polygon.covers(footprint) or b.grid.polygon.boundary.distance(footprint)<.2-1e-8:
                    counts['FALSE_SAFE_WALL_CANDIDATE_COUNT']+=1;failures.append([seq,step,'wall'])
                if not planner.geofence.safe_region.covers(Point(candidate.target_xy)) or not planner.geofence.track_inside(np.array(candidate.target_xy)):
                    counts['GEOFENCE_ESCAPE_COUNT']+=1;failures.append([seq,step,'geofence'])
                if candidate.unknown_area_fraction>1e-8:
                    counts['UNKNOWN_AS_OBSERVED_COUNT']+=1;failures.append([seq,step,'unknown-area'])
                if candidate.valid_for_control:
                    counts['INTERPOLATED_CELL_CONTROL_ELIGIBLE_COUNT']+=1;failures.append([seq,step,'control'])
            previous=snapshot
            durations.append((time.perf_counter()-start)*1000)
    result=dict(scope='FINITE_SYNTHETIC_NOT_FIELD',status='PASS' if not any(counts.values()) else 'FAIL',seed=20261013,
        sequences=sequences,sequence_frames=sequences*7,scenarios=results,sequence_coverage=coverage,counts=counts,failure_samples=failures,
        performance_ms=dict(P50=float(np.percentile(durations,50)),P95=float(np.percentile(durations,95)),MAX=max(durations)),
        valid_for_control=False,PIT_SURFACE_FIELD_STATUS='NOT_RUN',GRASP_PROPOSAL_FIELD_STATUS='NOT_RUN',
        MATERIAL_VOLUME_PROXY_FIELD_STATUS='NOT_RUN',PHYSICAL_ACTUATION_STATUS='NOT_RUN')
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--sequences',type=int,default=64)
    a=p.parse_args()
    if not 1 <= a.sequences <= 256: p.error('bounded sequence count required')
    result=validate(a.sequences)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
