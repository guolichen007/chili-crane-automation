"""ROS typed adapters only; algorithm classes remain usable without ROS."""
import copy
import math
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32,Polygon
from chili_crane_msgs.msg import (PlanningSpatialContext as ContextMessage, PitSurfaceStatus,
    PitSurfaceSnapshot as SnapshotMessage,PitSurfaceCell as CellMessage,GraspCandidate as CandidateMessage,
    GraspProposal as ProposalMessage)
from .pit_surface import PlanningSpatialContext,SurfaceCell,SurfaceSnapshot
from .surface_payload import ENCODING,sparse_cells,expand_sparse
from .surface_session import snapshot_identity,valid_session_id

SOURCE_IDS={'PHYSICAL':1,'SIMULATED':2,'REPLAY':3,'SYNTHETIC':4}
VALIDITY={'UNKNOWN':0,'NOT_CONFIGURED':1,'STALE':2,'DEGRADED':3,'VALID':4}


def seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def stamp(value):
    ns=round(value*1e9)
    return Time(sec=ns//1000000000,nanosec=ns%1000000000)


def polygon(xy):
    return Polygon(points=[Point32(x=float(x),y=float(y),z=0.) for x,y in xy])


def context_from_message(m):
    if (m.validity not in (3,4) or m.header.frame_id != m.source_frame or m.evidence.validity not in (3,4)
            or m.evidence.source_type != SOURCE_IDS.get(m.source_type)
            or m.evidence.calibration_id != m.calibration_id
            or m.evidence.measurement_stamp != m.header.stamp):
        raise ValueError('SPATIAL_TYPED_EVIDENCE_MISMATCH')
    return PlanningSpatialContext(m.source_frame,m.target_frame,m.map_id,m.semantic_version,m.pit_id,
        m.transform_id,m.calibration_id,m.transform_state,m.source_type,seconds(m.header.stamp),m.uncertainty_m,
        tuple(m.transform_row_major),tuple(m.sensor_calibration_ids),m.platform_motion_stable,m.motion_evidence_id,
        m.field_extrinsic_id,m.axis_to_map_id,m.crane_pose_evidence_id)


def context_to_message(ctx):
    m=ContextMessage()
    if ctx is None:
        return m
    m.header.frame_id,m.header.stamp=ctx.source_frame,stamp(ctx.measurement_epoch)
    m.validity,m.reason=4,'EXPLICIT_SPATIAL_CONTEXT'
    for key in ('source_frame','target_frame','map_id','semantic_version','pit_id','transform_id','calibration_id',
                'transform_state','source_type','uncertainty_m','platform_motion_stable','motion_evidence_id',
                'field_extrinsic_id','axis_to_map_id','crane_pose_evidence_id'):
        setattr(m,key,getattr(ctx,key))
    m.transform_row_major=list(ctx.transform)
    m.sensor_calibration_ids=list(ctx.sensor_calibration_ids)
    m.evidence.header=copy.deepcopy(m.header)
    m.evidence.measurement_stamp=copy.deepcopy(m.header.stamp)
    m.evidence.source_type=SOURCE_IDS[ctx.source_type]
    m.evidence.calibration_id=ctx.calibration_id
    m.evidence.validity=4
    return m


def snapshot_to_message(snapshot,now,counter):
    m=SnapshotMessage();s=m.status
    s.header.frame_id=snapshot.context.target_frame if snapshot.context else 'map'
    s.header.stamp=stamp(snapshot.window_end)
    s.validity=VALIDITY[snapshot.validity]
    s.reason=snapshot.reason
    m.validity,m.reason=s.validity,s.reason
    s.snapshot_id,s.surface_generation=snapshot.snapshot_id,snapshot.generation
    m.surface_session_id=s.surface_session_id=snapshot.surface_session_id
    s.config_sha256=snapshot.config_sha256
    s.last_disturbance_id,s.disturbance_counter=snapshot.last_disturbance_id,snapshot.disturbance_counter
    s.resolution_m=snapshot.resolution
    s.window_start,s.window_end=stamp(snapshot.window_start),stamp(snapshot.window_end)
    s.total_cell_count=snapshot.pit_active_cell_count
    observed=[c for c in snapshot.cells if c.observed]
    s.valid_cell_count=len(observed)
    s.coverage=len(observed)/s.total_cell_count if s.total_cell_count else 0.
    for key in ('pit_active_cell_count','observed_cell_count','support_qualified_cell_count','dual_supported_cell_count'):
        setattr(s,key,getattr(snapshot,key))
    s.robust_min_z_m=min((c.z_lower_bound for c in observed),default=math.nan)
    s.robust_max_z_m=max((c.z_upper_bound for c in observed),default=math.nan)
    s.evidence_age_sec=max(0.,now-snapshot.window_end) if snapshot.window_end > 0 and now >= snapshot.window_end else 1e9
    s.valid_for_control=False
    s.spatial_eligible=snapshot.context is not None and snapshot.validity != 'STALE'
    m.spatial_context=context_to_message(snapshot.context)
    if snapshot.context:
        ctx=snapshot.context
        s.map_id,s.semantic_version,s.pit_id=ctx.map_id,ctx.semantic_version,ctx.pit_id
        s.transform_id,s.spatial_calibration_id=ctx.transform_id,ctx.calibration_id
    s.evidence.header=copy.deepcopy(s.header)
    s.evidence.measurement_stamp=copy.deepcopy(s.header.stamp)
    s.evidence.source_counter=max(0,counter)
    s.evidence.evidence_age_sec=s.evidence_age_sec
    s.evidence.source_type=SOURCE_IDS.get(snapshot.context.source_type,0) if snapshot.context else 0
    s.evidence.calibration_id=s.spatial_calibration_id
    s.evidence.config_version=snapshot.config_sha256
    s.evidence.validity,s.evidence.reason=s.validity,s.reason
    m.origin_x_m,m.origin_y_m=snapshot.origin
    m.width,m.height=snapshot.width,snapshot.height
    m.cell_encoding=ENCODING
    for cell in sparse_cells(snapshot):
        item=CellMessage()
        for key in ('index','observed','sample_count','frame_count','sensor_support_mask','z_median','z_lower_bound',
                    'z_upper_bound','neighborhood_coverage','neighborhood_support'):
            setattr(item,key,getattr(cell,key))
        item.last_observed_epoch=stamp(cell.last_observed_epoch)
        item.validity=VALIDITY[cell.validity]
        item.reason='OBSERVED_ONLY' if cell.observed else 'NO_RETURN_IS_UNKNOWN'
        item.roughness_m,item.slope_rad,item.height_uncertainty_m=cell.roughness,cell.slope,cell.height_uncertainty
        m.cells.append(item)
    return m


def snapshot_from_message(m,config):
    s=m.status
    if (m.validity != s.validity or m.reason != s.reason or s.valid_for_control
            or not valid_session_id(s.surface_session_id) or m.surface_session_id != s.surface_session_id
            or s.evidence.measurement_stamp != s.header.stamp or s.window_end != s.header.stamp
            or s.evidence.evidence_age_sec != s.evidence_age_sec or not math.isfinite(s.evidence_age_sec)
            or not 0 <= s.evidence_age_sec <= config['surface_ttl_sec'] or not s.spatial_eligible
            or m.cell_encoding != ENCODING or not 0 < m.width*m.height <= config['maximum_grid_cells']
            or len(m.cells) > m.width*m.height or len(m.cells) != s.observed_cell_count
            or s.total_cell_count != s.pit_active_cell_count
            or not 0 <= s.dual_supported_cell_count <= s.support_qualified_cell_count <= s.observed_cell_count <= s.pit_active_cell_count <= m.width*m.height):
        raise ValueError('SURFACE_TYPED_IDENTITY_OR_FRESHNESS_REJECTED')
    ctx=context_from_message(m.spatial_context)
    if s.snapshot_id != snapshot_identity(s.surface_session_id,s.surface_generation,s.config_sha256,ctx.lineage_key):
        raise ValueError('SURFACE_SESSION_SNAPSHOT_ID_MISMATCH')
    if (s.map_id != ctx.map_id or s.semantic_version != ctx.semantic_version or s.pit_id != ctx.pit_id
            or s.transform_id != ctx.transform_id or s.spatial_calibration_id != ctx.calibration_id
            or s.header.frame_id != ctx.target_frame or s.evidence.source_type != SOURCE_IDS[ctx.source_type]
            or s.evidence.calibration_id != ctx.calibration_id):
        raise ValueError('SURFACE_SPATIAL_IDENTITY_MISMATCH')
    sparse=tuple(SurfaceCell(c.index,c.observed,c.sample_count,c.frame_count,c.sensor_support_mask,
        seconds(c.last_observed_epoch),c.z_median,c.z_lower_bound,c.z_upper_bound,c.roughness_m,c.slope_rad,
        c.height_uncertainty_m,c.neighborhood_coverage,c.neighborhood_support,
        {v:k for k,v in VALIDITY.items()}.get(c.validity,'UNKNOWN')) for c in m.cells)
    cells=expand_sparse(sparse,m.width,m.height,config['maximum_grid_cells'])
    return SurfaceSnapshot(s.snapshot_id,s.surface_generation,seconds(s.window_start),seconds(s.window_end),ctx,cells,
        (m.origin_x_m,m.origin_y_m),s.resolution_m,m.width,m.height,{v:k for k,v in VALIDITY.items()}.get(s.validity,'UNKNOWN'),
        s.reason,s.config_sha256,False,s.last_disturbance_id,s.disturbance_counter,
        s.pit_active_cell_count,s.observed_cell_count,s.support_qualified_cell_count,s.dual_supported_cell_count,s.surface_session_id)


def proposal_to_message(proposal,surface_message,age):
    m=ProposalMessage()
    s=surface_message.status
    m.header=copy.deepcopy(s.header)
    m.validity=3 if proposal.selected else 2 if 'STALE' in proposal.reason else 0
    m.reason,m.proposal_id=proposal.reason,proposal.proposal_id
    m.selected_candidate_id=proposal.selected.candidate_id if proposal.selected else ''
    m.surface_snapshot_id,m.surface_generation=proposal.snapshot_id,proposal.generation
    if proposal.surface_session_id != s.surface_session_id:
        raise ValueError('PROPOSAL_SURFACE_SESSION_MISMATCH')
    m.surface_session_id=proposal.surface_session_id
    for key in ('map_id','semantic_version','pit_id','transform_id','spatial_calibration_id'):
        setattr(m,key,getattr(s,key))
    provenance=proposal.geofence_provenance
    m.geofence_id,m.geofence_version=provenance['geofence_id'],provenance['geofence_version']
    m.geofence_config_sha256=provenance['config_sha256']
    m.kernel_id=proposal.kernel_id
    m.ranking_order=['volume_lower_anchored_epsilon_bands','observed_coverage_desc','recent_grab_penalty_asc',
                     'roughness_asc','slope_asc','wall_clearance_desc','candidate_id_asc']
    m.search_complete,m.global_optimum_proven=proposal.search_complete,proposal.global_optimum_proven
    m.ranked_candidate_ids=list(proposal.ranked_candidate_ids)
    m.valid_for_control=False
    m.evidence=copy.deepcopy(s.evidence)
    m.evidence.evidence_age_sec=age
    m.evidence.validity,m.evidence.reason=m.validity,m.reason
    for candidate in proposal.candidates:
        c=CandidateMessage(candidate_id=candidate.candidate_id,hard_gates_passed=candidate.accepted,
            rejection_reasons=list(candidate.rejection_reasons),valid_for_control=False)
        c.validity=3 if candidate.accepted else 0
        c.reason='SHADOW_HARD_GATES_PASSED' if candidate.accepted else '|'.join(candidate.rejection_reasons)
        c.target_suspension_map.x,c.target_suspension_map.y=map(float,candidate.target_xy)
        c.footprint=polygon(candidate.footprint_xy)
        for key in ('observed_area_fraction','unknown_area_fraction','surface_nominal_m','surface_lower_m','surface_upper_m',
                    'roughness_m','slope_rad','material_volume_proxy_m3','material_volume_proxy_lower_bound_m3','wall_clearance_m','recent_grab_penalty'):
            setattr(c,key,float(getattr(candidate,key)))
        m.candidates.append(c)
    return m
