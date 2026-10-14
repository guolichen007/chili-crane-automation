"""Generated ROS message adapters. Measurement epochs are never arrival times."""
from dataclasses import asdict
import json
import time
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Point32, Polygon
from chili_crane_msgs.msg import (EvidenceMetadata,TargetBinding as BindingMessage,ContactEvidence,SurfaceInteractionEvidence,
    ClosureEvidence,PayloadEvidence,LiftEvidence,UnloadEvidence,GraspInteractionState,SurfaceRefreshEvidence)
from .interaction_contracts import Evidence,TargetBinding
from .grasp_interaction import RefreshedSurface

SOURCE={1:'PHYSICAL',2:'SIMULATED',3:'REPLAY',4:'SYNTHETIC'}


def ns(stamp):return stamp.sec*1_000_000_000+stamp.nanosec


def stamp(value):
    return Time(sec=int(value)//1_000_000_000,nanosec=int(value)%1_000_000_000)


def polygon(xy):return Polygon(points=[Point32(x=float(x),y=float(y),z=0.) for x,y in xy])


def evidence_from_metadata(m):
    if m.header.stamp!=m.measurement_stamp:raise ValueError('EVIDENCE_HEADER_MEASUREMENT_MISMATCH')
    return Evidence(ns(m.measurement_stamp),int(m.source_counter),SOURCE.get(m.source_type,'UNKNOWN'),
        m.calibration_id,m.config_version,m.header.frame_id,int(m.validity),float(m.evidence_age_sec),ns(m.receive_stamp),m.reason)


def observation_evidence(m,residence=0.):
    if (m.evidence.header!=m.header or m.evidence.validity!=m.validity
            or getattr(m,'evidence_age_sec',m.evidence.evidence_age_sec)!=m.evidence.evidence_age_sec):
        raise ValueError('NORMALIZED_OBSERVATION_EVIDENCE_CONFLICT')
    e=evidence_from_metadata(m.evidence)
    from dataclasses import replace
    return replace(e,reported_age=max(e.reported_age,residence))


def evidence_to_message(e):
    m=EvidenceMetadata(validity=e.validity,reason=e.reason,source_counter=e.counter,
        source_type={v:k for k,v in SOURCE.items()}.get(e.source_type,0),calibration_id=e.calibration_id,
        config_version=e.config_version,evidence_age_sec=e.reported_age)
    m.header.frame_id=e.frame_id;m.header.stamp=stamp(e.epoch_ns)
    m.measurement_stamp=stamp(e.epoch_ns);m.receive_stamp=stamp(e.receive_ns)
    return m


IDENTITY_FIELDS=('run_id','task_id','cycle_id','cycle_index','binding_id','binding_schema_version','binding_digest_sha256',
    'proposal_id','candidate_id','map_id','semantic_version','pit_id','surface_snapshot_id','surface_session_id','surface_generation',
    'transform_id','spatial_calibration_id','geofence_id','geofence_version','geofence_config_sha256','kernel_id',
    'surface_nominal_m','surface_lower_m','surface_upper_m','material_volume_proxy_lower_bound_m3','binding_source_counter','valid_for_control')


def binding_to_message(b):
    m=BindingMessage(validity=4,reason='TARGET_BOUND_IMMUTABLE_SHADOW')
    for name in IDENTITY_FIELDS:setattr(m,name,getattr(b,name))
    m.target_suspension_map=Point(x=b.target_suspension_map[0],y=b.target_suspension_map[1],z=b.target_suspension_map[2])
    m.capture_footprint=polygon(b.capture_footprint);m.binding_stamp=stamp(b.binding_epoch_ns)
    m.evidence=evidence_to_message(b.evidence);m.header.frame_id=b.evidence.frame_id;m.header.stamp=stamp(b.binding_epoch_ns)
    return m


def binding_from_message(m):
    e=evidence_from_metadata(m.evidence)
    if (m.validity!=4 or m.reason!='TARGET_BOUND_IMMUTABLE_SHADOW' or m.header.stamp!=m.binding_stamp
            or m.header.frame_id!=e.frame_id or any(p.z!=0. for p in m.capture_footprint.points)):
        raise ValueError('BINDING_ENVELOPE_CONFLICT')
    values={name:getattr(m,name) for name in IDENTITY_FIELDS}
    values.update(binding_epoch_ns=ns(m.binding_stamp),evidence=e,
        target_suspension_map=(m.target_suspension_map.x,m.target_suspension_map.y,m.target_suspension_map.z),
        capture_footprint=tuple((p.x,p.y) for p in m.capture_footprint.points))
    return TargetBinding(**values)


def binding_from_proposal(proposal,run_id,task_id,cycle_id,cycle_index,now):
    if proposal.validity not in (3,4) or proposal.valid_for_control or not proposal.selected_candidate_id:
        raise ValueError('NO_VALID_SHADOW_PROPOSAL')
    selected=[c for c in proposal.candidates if c.candidate_id==proposal.selected_candidate_id]
    if len(selected)!=1 or not selected[0].hard_gates_passed or selected[0].rejection_reasons or selected[0].valid_for_control:
        raise ValueError('SELECTED_CANDIDATE_NOT_HARD_GATE_ELIGIBLE')
    c=selected[0];e=observation_evidence(proposal)
    return TargetBinding(run_id,task_id,cycle_id,int(cycle_index),cycle_id+':binding:'+proposal.surface_snapshot_id,
        proposal.proposal_id,c.candidate_id,proposal.map_id,proposal.semantic_version,proposal.pit_id,
        proposal.surface_snapshot_id,proposal.surface_session_id,int(proposal.surface_generation),proposal.transform_id,
        proposal.spatial_calibration_id,proposal.geofence_id,proposal.geofence_version,proposal.geofence_config_sha256,
        proposal.kernel_id,(c.target_suspension_map.x,c.target_suspension_map.y,c.target_suspension_map.z),
        tuple((p.x,p.y) for p in c.footprint.points),c.surface_nominal_m,c.surface_lower_m,c.surface_upper_m,
        c.material_volume_proxy_lower_bound_m3,int(round(now*1e9)),int(e.counter),e)


def common(m,b,e,reason):
    m.header.frame_id=e.frame_id;m.header.stamp=stamp(e.epoch_ns)
    m.validity=3 if e.validity in (3,4) else e.validity;m.reason=reason
    from dataclasses import replace
    m.evidence=evidence_to_message(replace(e,validity=m.validity));m.valid_for_control=False
    for name in ('run_id','task_id','cycle_id','binding_id','binding_digest_sha256'):setattr(m,name,getattr(b,name))
    return m


def result_to_message(result,b,e,config):
    if result.reason!='SHADOW_EVIDENCE_NOT_CONTROL':
        from dataclasses import replace
        e=replace(e,validity=2 if 'STALE' in result.reason else 1)
    m=common(GraspInteractionState(),b,e,result.reason)
    m.task_cycle_state=result.phase;m.abort_required=result.abort_required
    m.next_cycle_surface_ready=result.next_cycle_surface_ready;m.event_types=list(result.events)
    m.load_features_json=json.dumps(asdict(result.load_features),sort_keys=True,allow_nan=False)
    m.gate_diagnostics_json=json.dumps(result.diagnostics,sort_keys=True,allow_nan=False)
    contact=common(ContactEvidence(),b,e,result.contact['state'])
    contact.state=getattr(contact,result.contact['state'])
    for target,key in (('gap_lower_m','gap_lower'),('gap_upper_m','gap_upper'),('descending_verified','descending'),
                       ('geometry_overlap','overlap'),('load_change_persistent','load_change'),('persistence_sec','persistence_sec')):
        if key in result.contact:setattr(contact,target,result.contact[key])
    inter=common(SurfaceInteractionEvidence(),b,e,result.interaction['state'])
    inter.state=getattr(inter,result.interaction['state']);inter.interaction_id=result.interaction.get('interaction_id','')
    inter.surface_invalidation_required=result.interaction.get('invalidate',False)
    for name in ('map_id','semantic_version','pit_id','surface_session_id','spatial_calibration_id'):setattr(inter,name,getattr(b,name))
    inter.bound_surface_snapshot_id=b.surface_snapshot_id;inter.bound_surface_generation=b.surface_generation
    inter.capture_footprint=polygon(b.capture_footprint)
    inter.swept_footprints=[polygon(fp) for fp in result.interaction.get('swept',())]
    inter.collapse_margin_m=config.get('collapse_margin_m') or 0.
    inter.interaction_stamp=stamp(int(round(result.interaction.get('epoch',0.)*1e9)))
    inter.interaction_source_counter=result.interaction.get('source_counter',0)
    closure=common(ClosureEvidence(),b,e,result.closure['state']);closure.state=getattr(closure,result.closure['state'])
    closure.open_limit=result.closure.get('open_limit',False);closure.closed_limit=result.closure.get('closed_limit',False)
    closure.lidar_opening_state=result.closure.get('opening_state','UNKNOWN')
    closure.close_started_stamp=stamp(int(round(result.closure.get('close_started',0.)*1e9)))
    payload=common(PayloadEvidence(),b,e,result.payload['state']);payload.state=getattr(payload,result.payload['state'])
    f=result.load_features
    for target,value in (('suspended_empty_baseline',f.empty_baseline),('robust_scale',f.robust_scale),('stable_delta',f.delta),
            ('normalized_delta',f.normalized_delta),('load_slope_per_sec',f.slope),('settling_sec',f.settling_sec),
            ('vertical_velocity_mps',result.payload.get('velocity',0.)),('vertical_acceleration_mps2',result.payload.get('acceleration',0.))):
        setattr(payload,target,float(value))
    payload.calibration_confirmed_for_source=f.calibration_confirmed_for_source
    lift=common(LiftEvidence(),b,e,'SHADOW_LIFT_EVIDENCE')
    for name in ('clear_of_bound_surface','clear_of_updated_surface','clear_of_material_surface','clear_of_pit',
                 'safe_lift_height','pit_exit_confirmed'):setattr(lift,name,bool(result.lift.get(name,False)))
    lift.pit_exit_stamp=stamp(result.lift.get('pit_exit_epoch_ns',0))
    lift.pit_exit_source_counter=result.lift.get('pit_exit_source_counter',0)
    lift.surface_refresh_ready=bool(result.lift.get('refresh_ready',False))
    lift.updated_clear_state=result.lift.get('updated_clear_state','UNKNOWN')
    for target,key in (('observed_rise_m','observed_rise'),('bound_clearance_m','bound_clearance'),
                       ('updated_clearance_m','updated_clearance'),('pit_clearance_m','pit_clearance')):
        setattr(lift,target,float(result.lift.get(key,0.)))
    lift.refreshed_snapshot_id=result.lift.get('refreshed_snapshot_id','')
    lift.refreshed_surface_generation=result.lift.get('refreshed_generation',0)
    unload=common(UnloadEvidence(),b,e,result.unload['state']);unload.state=getattr(unload,result.unload['state'])
    for name in ('station_id','in_safe_zone','cart_present','aligned','not_full','not_overflow','open_confirmed',
                 'loaded_baseline','unload_delta'):
        if name in result.unload:setattr(unload,name,result.unload[name])
    m.contact,m.interaction,m.closure,m.payload,m.lift,m.unload=contact,inter,closure,payload,lift,unload
    return m


def refresh_to_message(r,lifecycle):
    b=lifecycle.binding;m=common(SurfaceRefreshEvidence(),b,r.evidence,'POST_INTERACTION_REAL_OBSERVATIONS' if r.ready else 'LOCAL_SURFACE_UNKNOWN')
    m.interaction_id=lifecycle.interaction_id;m.surface_session_id=r.surface_session_id;m.snapshot_id=r.snapshot_id
    m.surface_generation=r.surface_generation;m.affected_cell_count=len(lifecycle.affected)
    m.qualified_cell_count=lifecycle.qualified_cell_count;m.invalidation_acknowledged=bool(lifecycle.interaction_id)
    m.refreshed_cell_count=r.refreshed_cell_count;m.unknown_cell_count=r.unknown_cell_count
    m.observed_fraction=r.observed_fraction;m.generation_ready=r.generation_ready
    m.observation_enabled=lifecycle.observation_after is not None;m.refresh_ready=r.ready
    m.observation_after_epoch=lifecycle.observation_after or 0.
    m.surface_lower_m,m.surface_nominal_m,m.surface_upper_m=r.lower,r.nominal,r.upper
    m.binding_lineage=list(r.lineage)
    return m


def refresh_from_message(m):
    if (m.valid_for_control or not m.invalidation_acknowledged or not m.observation_enabled
            or m.affected_cell_count==0 or m.refreshed_cell_count!=m.qualified_cell_count
            or m.refreshed_cell_count+m.unknown_cell_count!=m.affected_cell_count
            or not 0<=m.observed_fraction<=1
            or abs(m.observed_fraction-m.refreshed_cell_count/m.affected_cell_count)>1e-9
            or m.refresh_ready and (not m.generation_ready or m.refreshed_cell_count==0)):
        raise ValueError('REFRESH_LOCAL_QUALITY_NOT_READY')
    return RefreshedSurface(observation_evidence(m),m.binding_id,m.binding_digest_sha256,m.snapshot_id,
        int(m.surface_generation),m.surface_session_id,m.surface_upper_m,m.surface_lower_m,m.surface_nominal_m,
        bool(m.refresh_ready),tuple(m.binding_lineage),int(m.affected_cell_count),int(m.refreshed_cell_count),
        int(m.unknown_cell_count),float(m.observed_fraction),bool(m.generation_ready))
