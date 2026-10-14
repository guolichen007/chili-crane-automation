"""Evidence maturity for the EXISTING TaskCycles. Never an action executor."""
from dataclasses import dataclass, field
import math
from shapely.geometry import Polygon
from .cycle import CycleState
from .interaction_contracts import Evidence, TargetBinding, BindingLedger
from .load_features import LoadFeatureEstimator, VerticalMotionEstimator, configured_number


@dataclass(frozen=True)
class GrabObservation:
    evidence: Evidence
    bottom_lower: float
    bottom_upper: float
    top_upper: float
    z: float
    footprint: tuple
    opening_state: str
    extrinsic_id: str
    tracked: bool = True
    measurement_accepted: bool = True
    motion_stable: bool = False
    fault: bool = False


@dataclass(frozen=True)
class HoistObservation:
    evidence: Evidence
    motion: str
    fault: bool = False


@dataclass(frozen=True)
class IoObservation:
    evidence: Evidence
    open_limit: bool
    closed_limit: bool
    limits_known: bool = True
    fault: bool = False
    running: bool = False


@dataclass(frozen=True)
class RefreshedSurface:
    evidence: Evidence
    binding_id: str
    binding_digest: str
    snapshot_id: str
    surface_generation: int
    surface_session_id: str
    upper: float
    lower: float
    nominal: float
    ready: bool
    lineage: tuple


@dataclass(frozen=True)
class CartObservation:
    evidence: Evidence
    station_id: str
    present: bool
    aligned: bool
    full: bool
    overflow: bool


@dataclass(frozen=True)
class ZoneObservation:
    evidence: Evidence
    bounds: tuple
    geometry_version: str
    verified: bool


@dataclass(frozen=True)
class InteractionResult:
    phase: str
    reason: str
    abort_required: bool
    contact: dict
    interaction: dict
    closure: dict
    payload: dict
    lift: dict
    unload: dict
    load_features: object
    next_cycle_surface_ready: bool
    events: tuple = ()
    valid_for_control: bool = False


class Persistence:
    def __init__(self,maximum_gap):
        self.maximum_gap=maximum_gap
        self.started,self.last=None,None

    def observe(self,condition,epoch):
        if self.last is not None and epoch<self.last:
            self.started=None;return 0.
        if not condition:
            self.started=None;self.last=epoch;return 0.
        if self.last is None or epoch-self.last>self.maximum_gap:self.started=None
        if self.started is None:self.started=epoch
        self.last=epoch
        return epoch-self.started


class GraspInteractionEngine:
    def __init__(self,config,semantic=None):
        self.config=dict(config)
        self.semantic=semantic
        if any(config.get(k) is not False for k in ('physical_output_enabled','automatic_control_enabled',
                'camera_control_authority','valid_for_control')):
            raise ValueError('PHASE2C_SHADOW_FLAGS_REQUIRED')
        self.ledger=BindingLedger(config['maximum_bindings'])
        self.load=LoadFeatureEstimator(config['load'])
        self.motion=VerticalMotionEstimator(config['motion'])
        self.binding=None;self.last_phase=None;self.previous_interaction_generation=None
        self.events=[]
        self.reset_cycle()

    def reset_cycle(self):
        gap=self.config.get('maximum_sample_gap_sec')
        gap=gap if type(gap) in (int,float) and gap>0 else 0.
        self.persistence={name:Persistence(gap) for name in ('contact','penetration','closure','unload')}
        self.watermarks={};self.committed=False;self.commit_epoch=0.;self.commit_counter=0
        self.swept=[];self.close_started=None;self.raise_started=None;self.lower_started=None
        self.lower_min_z=None;self.loaded_baseline=None;self.last_payload='PAYLOAD_UNKNOWN'
        self.once_events=set()
        self.last_refresh=None
        for name in ('contact','closure','payload','unload'):setattr(self,'_last_'+name,None)
        self.motion=VerticalMotionEstimator(self.config['motion'])

    def bind(self,binding,now):
        if not binding.evidence.fresh(now,self.config.get('evidence_ttl_sec') or 0.,self.config.get('source_type')):
            raise ValueError('BINDING_INPUT_STALE_OR_NOT_CONFIGURED')
        if not 0 <= now-binding.binding_epoch_ns/1e9 <= self.config['evidence_ttl_sec']:
            raise ValueError('BINDING_EVENT_TIME_INVALID')
        if self.binding is not None and self.last_phase not in (CycleState.COMPLETE.name,CycleState.ABORTED.name):
            raise ValueError('ACTIVE_CYCLE_CANNOT_REBIND')
        if self.binding is not None and self.committed and (
                binding.surface_session_id!=self.binding.surface_session_id
                or binding.surface_generation<=self.binding.surface_generation
                or self.last_refresh is None or not self.last_refresh.ready
                or not self.last_refresh.evidence.fresh(now,self.config['evidence_ttl_sec'],self.config['source_type'])
                or binding.surface_generation<self.last_refresh.surface_generation):
            raise ValueError('NEXT_CYCLE_REQUIRES_POST_INTERACTION_NEW_SURFACE')
        self.binding=self.ledger.bind(binding);self.reset_cycle();self.last_phase=CycleState.PLAN_PICK.name
        self.events.append('TARGET_BOUND')
        return binding

    def _fresh(self,observation,now,geometry=False):
        c=self.config
        if observation is None or not configured_number(c,'evidence_ttl_sec',True):return False
        if not observation.evidence.fresh(now,c['evidence_ttl_sec'],c.get('source_type')):return False
        if getattr(observation,'fault',False):return False
        if geometry:
            if (not observation.tracked or not observation.measurement_accepted
                    or observation.evidence.frame_id!=self.binding.evidence.frame_id
                    or observation.extrinsic_id!=c.get('geometry_calibration_id')
                    or not all(math.isfinite(v) for v in (observation.bottom_lower,observation.bottom_upper,
                                                         observation.top_upper,observation.z))
                    or not observation.bottom_lower<=observation.bottom_upper<=observation.top_upper):return False
            polygon=Polygon(observation.footprint)
            if not polygon.is_valid or polygon.is_empty:return False
        return True

    def _pit_elevation(self):
        b,c,s=self.binding,self.config,self.semantic
        if s is not None:
            if (s.get('schema_version')!=1 or s.get('config_state')!='VALID' or s.get('map_id')!=b.map_id
                    or s.get('semantic_version')!=b.semantic_version or s.get('frame_id')!=b.evidence.frame_id):return None
            pits=[p for p in s.get('pits',[]) if p.get('pit_id')==b.pit_id]
            value=pits[0].get('wall_top_z_m') if len(pits)==1 else None
        elif c.get('source_type')!='PHYSICAL' and c.get('config_state')=='SYNTHETIC_NON_PRODUCTION':
            value=c.get('pit_rim_elevation_m')  # Explicit non-production isolated fixture only.
        else:return None
        return float(value) if type(value) in (int,float) and math.isfinite(value) else None

    def _ordered(self,key,observation,payload):
        e=observation.evidence;previous=self.watermarks.get(key)
        stamp=(e.epoch_ns,e.counter)
        if previous is not None:
            old,old_payload=previous
            if stamp==old:return payload==old_payload
            if stamp[0]<=old[0] or stamp[1]<=old[1]:return False
        self.watermarks[key]=(stamp,payload)
        return True

    def _empty(self,phase,reason,abort=False):
        return InteractionResult(phase,reason,abort,dict(state='CONTACT_UNKNOWN'),
            dict(state='INTERACTION_COMMITTED' if self.committed else 'NO_INTERACTION',
                 interaction_id=(self.binding.binding_id+':interaction') if self.binding else '',
                 epoch=self.commit_epoch,source_counter=self.commit_counter,invalidate=self.committed,
                 swept=tuple(self.swept)),dict(state='CLOSURE_UNKNOWN'),dict(state='PAYLOAD_UNKNOWN'),
            dict(clear_of_bound_surface=False,clear_of_updated_surface=False,clear_of_material_surface=False,
                 clear_of_pit=False,safe_lift_height=False,refresh_ready=False),dict(state='UNLOAD_UNKNOWN'),
            self.load.features,False,(),False)

    def evaluate(self,phase,grab,hoist,io,load,now,lineage=None,refresh=None,cart=None,zone=None):
        phase=phase.name if isinstance(phase,CycleState) else phase
        if phase not in CycleState.__members__:return self._empty(str(phase),'CYCLE_STATE_UNKNOWN',True)
        self.last_phase=phase
        start=len(self.events);c,b=self.config,self.binding
        if b is None:return self._empty(phase,'TARGET_BINDING_MISSING')
        if lineage is not None:
            try:self.ledger.check_lineage(b,lineage)
            except ValueError:return self._empty(phase,'BINDING_LINEAGE_CONFLICT_ABORT_REQUIRED',True)
        if phase=='ABORTED':
            if 'CYCLE_ABORTED' not in self.events[start:]:self.events.append('CYCLE_ABORTED')
            return self._empty(phase,'CYCLE_ABORTED',True)
        required=('maximum_evidence_skew_sec','contact_persistence_sec','penetration_depth_m','penetration_persistence_sec',
                  'collapse_margin_m','bound_clearance_margin_m','updated_clearance_margin_m','minimum_observed_rise_m',
                  'maximum_stable_velocity_mps','maximum_stable_acceleration_mps2','payload_settling_sec',
                  'closure_persistence_sec','unload_persistence_sec')
        if not all(configured_number(c,k,True) for k in required):return self._empty(phase,'INTERACTION_PARAMETERS_NOT_CONFIGURED')
        if not self._fresh(grab,now,True) or not self._fresh(hoist,now):return self._empty(phase,'GRAB_OR_HOIST_STALE_UNKNOWN')
        if (not self._ordered('grab',grab,(grab.bottom_lower,grab.bottom_upper,grab.z,grab.opening_state,grab.footprint))
                or not self._ordered('hoist',hoist,(hoist.motion,))):
            return self._empty(phase,'SOURCE_CHRONOLOGY_CONFLICT',True)
        if abs(grab.evidence.epoch-hoist.evidence.epoch)>c['maximum_evidence_skew_sec']:
            return self._empty(phase,'GRAB_HOIST_TIME_SKEW')
        epoch=grab.evidence.epoch
        motion=self.motion.observe(grab.evidence,grab.z)
        stopped=hoist.motion=='STOPPED'
        empty_eligible=(stopped and grab.motion_stable and not self.committed
                        and phase not in ('LOWER','CLOSE','VERIFY_LOAD','RAISE'))
        features=self.load.observe(load,now,empty_eligible) if load is not None else self.load.current(now)
        io_ok=self._fresh(io,now) and io.limits_known and abs(io.evidence.epoch-epoch)<=c['maximum_evidence_skew_sec']
        if io_ok and not self._ordered('io',io,(io.open_limit,io.closed_limit)):
            return self._empty(phase,'IO_CHRONOLOGY_CONFLICT',True)
        xy_overlap=Polygon(grab.footprint).intersects(Polygon(b.capture_footprint))
        gap_lower=grab.bottom_lower-b.surface_upper_m;gap_upper=grab.bottom_upper-b.surface_lower_m
        overlap=xy_overlap and gap_lower<=0<=gap_upper
        intersection=xy_overlap and gap_lower<=0 and grab.top_upper>=b.surface_lower_m
        descending=phase=='LOWER' and hoist.motion=='DOWN' and motion.valid and motion.velocity<0
        if phase=='LOWER':
            if self.lower_started is None:self.lower_started=epoch
            self.lower_min_z=grab.z if self.lower_min_z is None else min(self.lower_min_z,grab.z)
        load_common=(features.fresh and features.baseline_valid and features.change_latched
                     and abs(features.measurement_epoch-epoch)<=c['maximum_evidence_skew_sec'])
        contact_change=(load_common and features.calibration_confirmed_for_source
                        and abs(features.delta)>=c['load']['contact_abs_threshold'])
        duration=self.persistence['contact'].observe(descending and overlap and contact_change,epoch)
        contact='CONTACT_POSSIBLE' if overlap or intersection else 'CONTACT_UNKNOWN'
        if descending and overlap and contact_change and duration>=c['contact_persistence_sec']:contact='CONTACT_CONFIRMED'
        if overlap and hoist.motion not in ('DOWN','STOPPED','UP'):contact='CONTACT_CONFLICT'
        closure='CLOSURE_UNKNOWN'
        if phase=='CLOSE' and self.close_started is None:
            self.close_started=epoch;self.events.append('CLOSE_STARTED')
        if io_ok:
            if io.open_limit and io.closed_limit or (io.closed_limit and grab.opening_state=='OPEN') or (
                    io.open_limit and grab.opening_state=='CLOSED'):
                closure='CLOSURE_CONFLICT'
            elif io.closed_limit and grab.opening_state=='CLOSED':
                eligible=self.close_started is not None and epoch>=self.close_started and io.evidence.epoch>=self.close_started
                duration_closed=self.persistence['closure'].observe(eligible,epoch)
                closure='CLOSURE_CONFIRMED' if eligible and duration_closed>=c['closure_persistence_sec'] else 'CLOSURE_CANDIDATE'
            else:self.persistence['closure'].observe(False,epoch)
        if intersection and phase in ('LOWER','CLOSE','VERIFY_LOAD','RAISE'):
            footprint=tuple(tuple(v) for v in grab.footprint)
            if footprint not in self.swept:
                if len(self.swept)>=c['maximum_swept_footprints']:
                    return self._empty(phase,'SWEPT_FOOTPRINT_CAPACITY_ABORT_REQUIRED',True)
                self.swept.append(footprint)
        penetration=descending and intersection and grab.bottom_upper<=b.surface_lower_m-c['penetration_depth_m']
        penetration_sec=self.persistence['penetration'].observe(penetration,epoch)
        actual_closure=(io_ok and (closure=='CLOSURE_CONFIRMED' or grab.opening_state=='TRANSITION' and io.running))
        reliable=(penetration and penetration_sec>=c['penetration_persistence_sec'] or contact=='CONTACT_CONFIRMED'
                  or intersection and phase=='CLOSE' and self.lower_started is not None and actual_closure)
        if reliable and not self.committed:
            self.committed=True;self.commit_epoch=epoch;self.commit_counter=grab.evidence.counter
            self.events.extend(('INTERACTION_COMMITTED','SURFACE_INVALIDATION_REQUIRED'))
        interaction='INTERACTION_COMMITTED' if self.committed else 'INTERACTION_POSSIBLE' if intersection else 'NO_INTERACTION'
        if phase=='RAISE' and self.raise_started is None:
            self.raise_started=epoch;self.events.append('RAISE_STARTED')
        rise=grab.z-self.lower_min_z if self.lower_min_z is not None else 0.
        rise_order=(self.raise_started is not None and self.close_started is not None
                    and self.raise_started>=self.close_started>= (self.lower_started or math.inf)
                    and rise>=c['minimum_observed_rise_m'])
        bound_clear=(self.committed and rise_order and hoist.motion in ('UP','STOPPED')
                     and grab.bottom_lower-b.surface_upper_m>=c['bound_clearance_margin_m'])
        refresh_ok=(refresh is not None and refresh.ready and refresh.binding_id==b.binding_id
                    and refresh.binding_digest==b.binding_digest_sha256 and refresh.lineage==b.lineage
                    and refresh.surface_session_id==b.surface_session_id and refresh.surface_generation>b.surface_generation
                    and refresh.evidence.epoch>self.commit_epoch
                    and refresh.evidence.calibration_id==b.spatial_calibration_id
                    and refresh.evidence.frame_id==b.evidence.frame_id
                    and refresh.evidence.fresh(now,c['evidence_ttl_sec'],c['source_type'])
                    and all(math.isfinite(v) for v in (refresh.lower,refresh.nominal,refresh.upper))
                    and refresh.lower<=refresh.nominal<=refresh.upper)
        updated_clear=(bound_clear and refresh_ok and grab.bottom_lower-refresh.upper>=c['updated_clearance_margin_m'])
        self.last_refresh=refresh if refresh_ok else None
        pit_state='FIELD_VALIDATED' if c['source_type']=='PHYSICAL' else 'SYNTHETIC_VALIDATED'
        pit_elevation=self._pit_elevation()
        pit_config=(c.get('pit_geometry_state')==pit_state and pit_elevation is not None
                    and configured_number(c,'pit_clearance_margin_m',True) and configured_number(c,'safe_lift_height_m',True))
        pit_clear=bool(pit_config and updated_clear and grab.bottom_lower-pit_elevation>=c['pit_clearance_margin_m'])
        safe_height=bool(pit_clear and grab.bottom_lower>=c['safe_lift_height_m'])
        settled=(motion.valid and grab.motion_stable and stopped
                 and abs(motion.velocity)<=c['maximum_stable_velocity_mps']
                 and abs(motion.acceleration)<=c['maximum_stable_acceleration_mps2']
                 and features.fresh and features.settling_sec>=c['payload_settling_sec'])
        payload='PAYLOAD_UNKNOWN'
        if closure=='CLOSURE_CONFLICT':payload='PAYLOAD_CONFLICT'
        elif self.committed and closure=='CLOSURE_CONFIRMED' and rise_order and features.baseline_valid and features.fresh:
            payload='PAYLOAD_CANDIDATE'
            calibrated=features.calibration_confirmed_for_source and configured_number(c['load'],'payload_normalized_threshold',True)
            if settled and updated_clear and calibrated:
                delta=features.delta*c['load']['load_polarity']
                loaded=delta>=c['load']['payload_abs_threshold'] and features.normalized_delta*c['load']['load_polarity']>=c['load']['payload_normalized_threshold']
                payload='PAYLOAD_CONFIRMED' if loaded else 'PAYLOAD_NOT_CONFIRMED'
                if loaded:self.loaded_baseline=features.short_median
        # Payload is historical evidence after opening; unloading evaluates its
        # loaded baseline separately rather than replacing it with an empty signal.
        if payload in ('PAYLOAD_CONFIRMED','PAYLOAD_NOT_CONFIRMED'):self.last_payload=payload
        unload='UNLOAD_UNKNOWN';zone_ok=False;cart_ok=False;open_ok=False;unload_delta=0.
        if (self._fresh(cart,now) and self._fresh(zone,now) and zone.verified and len(zone.bounds)==6
                and all(math.isfinite(v) for v in zone.bounds)
                and all(zone.bounds[i]<zone.bounds[i+1] for i in (0,2,4))
                and zone.geometry_version not in ('','NOT_CONFIGURED','UNKNOWN')
                and cart.station_id not in ('','NOT_CONFIGURED','UNKNOWN')):
            x0,x1,y0,y1,z0,z1=zone.bounds
            polygon=Polygon(grab.footprint)
            from shapely.geometry import box
            zone_ok=(zone.evidence.frame_id==b.evidence.frame_id and box(x0,y0,x1,y1).covers(polygon)
                     and z0<=grab.bottom_lower<=grab.top_upper<=z1)
            cart_ok=cart.present and cart.aligned and not cart.full and not cart.overflow
            if cart.full or cart.overflow:unload='UNLOAD_CONFLICT'
            elif zone_ok and cart_ok and pit_clear and self.loaded_baseline is not None:
                unload='UNLOAD_READY'
                open_ok=io_ok and io.open_limit and not io.closed_limit and grab.opening_state=='OPEN'
                if (phase in ('OPEN_GRAB','VERIFY_EMPTY') and open_ok and settled and features.calibration_confirmed_for_source
                        and configured_number(c['load'],'unload_abs_threshold',True)):
                    unload_delta=(self.loaded_baseline-features.short_median)*c['load']['load_polarity']
                    duration_unload=self.persistence['unload'].observe(unload_delta>=c['load']['unload_abs_threshold'],epoch)
                    if duration_unload>=c['unload_persistence_sec']:unload='UNLOAD_CONFIRMED'
                if closure=='CLOSURE_CONFLICT':unload='UNLOAD_CONFLICT'
        for name,value in (('contact',contact),('closure',closure),('payload',payload),('unload',unload)):
            old=getattr(self,'_last_'+name,None)
            if value!=old and value not in ('CONTACT_UNKNOWN','CLOSURE_UNKNOWN','PAYLOAD_UNKNOWN','UNLOAD_UNKNOWN'):
                self.events.append(value)
            setattr(self,'_last_'+name,value)
        for event,value in (('CLEAR_OF_BOUND_SURFACE',bound_clear),('SURFACE_REFRESH_READY',refresh_ok)):
            if value and event not in self.once_events:self.events.append(event);self.once_events.add(event)
        emitted=tuple(self.events[start:])
        self.events=self.events[-c['maximum_events']:]
        return InteractionResult(phase,'SHADOW_EVIDENCE_NOT_CONTROL',False,
            dict(state=contact,gap_lower=gap_lower,gap_upper=gap_upper,descending=descending,overlap=overlap,
                 load_change=contact_change,persistence_sec=duration),
            dict(state=interaction,interaction_id=b.binding_id+':interaction',epoch=self.commit_epoch,
                 source_counter=self.commit_counter,invalidate=self.committed,swept=tuple(self.swept)),
            dict(state=closure,open_limit=io.open_limit if io_ok else False,closed_limit=io.closed_limit if io_ok else False,
                 opening_state=grab.opening_state,close_started=self.close_started or 0.),
            dict(state=payload,velocity=motion.velocity,acceleration=motion.acceleration,**{'rise':rise}),
            dict(clear_of_bound_surface=bound_clear,clear_of_updated_surface=updated_clear,clear_of_material_surface=updated_clear,
                 clear_of_pit=pit_clear,safe_lift_height=safe_height,refresh_ready=refresh_ok,observed_rise=rise,
                 bound_clearance=grab.bottom_lower-b.surface_upper_m,updated_clearance=grab.bottom_lower-refresh.upper if refresh_ok else 0.,
                 pit_clearance=grab.bottom_lower-pit_elevation if pit_config else 0.,
                 refreshed_snapshot_id=refresh.snapshot_id if refresh_ok else '',refreshed_generation=refresh.surface_generation if refresh_ok else 0),
            dict(state=unload,in_safe_zone=zone_ok,cart_present=cart.present if cart is not None else False,
                 aligned=cart.aligned if cart is not None else False,not_full=not cart.full if cart is not None else False,
                 not_overflow=not cart.overflow if cart is not None else False,open_confirmed=open_ok,
                 station_id=cart.station_id if cart is not None else '',loaded_baseline=self.loaded_baseline or 0.,unload_delta=unload_delta),
            features,bool(not self.committed or refresh_ok),emitted)


def evidence_allows_advance(cycles,result):
    """Shadow admission only. The original FSM still owns state transitions."""
    if result.abort_required or result.phase!=cycles.state.name:return False
    state=cycles.state
    if state==CycleState.LOWER:return result.interaction['state']=='INTERACTION_COMMITTED'
    if state==CycleState.CLOSE:return result.closure['state']=='CLOSURE_CONFIRMED'
    if state==CycleState.VERIFY_LOAD:
        return result.closure['state']=='CLOSURE_CONFIRMED' and result.interaction['state']=='INTERACTION_COMMITTED'
    if state==CycleState.RAISE:
        return (result.payload['state']=='PAYLOAD_CONFIRMED' and result.lift['safe_lift_height']
                and result.lift['clear_of_updated_surface'])
    if state==CycleState.VERIFY_EMPTY:return result.unload['state']=='UNLOAD_CONFIRMED' and result.next_cycle_surface_ready
    # Other states still require their existing owner readiness/positioning policy.
    return False
