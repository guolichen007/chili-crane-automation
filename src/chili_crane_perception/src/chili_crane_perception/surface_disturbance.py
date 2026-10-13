"""Synthetic/replay disturbance lifecycle only; no physical execution subscriber."""
from collections import deque
from dataclasses import dataclass
import math
from shapely.geometry import Polygon
from .pit_surface import identity,number


@dataclass(frozen=True)
class SurfaceGraspEvent:
    event_id: str
    source_counter: int
    epoch: float
    source_type: str
    map_id: str
    semantic_version: str
    pit_id: str
    transform_id: str
    kernel_id: str
    footprint_xy: tuple


class SurfaceDisturbanceTracker:
    def __init__(self, config):
        p=config["planning"]
        self.dilation=number(p.get("disturbance_dilation_m"),"DISTURBANCE_DILATION")
        self.radius=number(p.get("recent_grab_radius_m"),"RECENT_GRAB_RADIUS",True)
        self.ttl=number(p.get("recent_grab_ttl_sec"),"RECENT_GRAB_TTL",True)
        capacity=p.get("maximum_recent_events")
        if type(capacity) is not int or not 1 <= capacity <= 256:
            raise ValueError("RECENT_EVENT_CAPACITY_NOT_CONFIGURED")
        self.events=deque(maxlen=capacity)
        self.counter,self.epoch=-1,0.
        self.kernel_id=config["kernel"]["kernel_id"]

    def apply(self, grid, event, now):
        ctx=grid.context
        if (ctx is None or event.source_type not in ("SYNTHETIC","REPLAY") or event.source_type != ctx.source_type
                or event.map_id != ctx.map_id or event.semantic_version != ctx.semantic_version
                or event.pit_id != ctx.pit_id or event.transform_id != ctx.transform_id or event.kernel_id != self.kernel_id):
            raise ValueError("DISTURBANCE_PROVENANCE_REJECTED")
        if (not identity(event.event_id) or type(event.source_counter) is not int or event.source_counter <= self.counter
                or not math.isfinite(event.epoch) or event.epoch <= self.epoch or not 0 <= now-event.epoch <= self.ttl):
            raise ValueError("DISTURBANCE_STALE_DUPLICATE_OR_FUTURE")
        footprint=Polygon(event.footprint_xy)
        if not footprint.is_valid or footprint.is_empty or footprint.area <= 0:
            raise ValueError("DISTURBANCE_FOOTPRINT_INVALID")
        disturbed=footprint.buffer(self.dilation/math.cos(math.pi/64),resolution=16)
        grid.invalidate(disturbed,event.epoch)
        grid.last_disturbance_id,grid.disturbance_counter=event.event_id,event.source_counter
        self.events.append((event.epoch,disturbed,ctx.key))
        self.counter,self.epoch=event.source_counter,event.epoch
        self.expire(now)

    def expire(self, now):
        while self.events and (now-self.events[0][0] > self.ttl or now < self.events[0][0]):
            self.events.popleft()

    def penalty(self, footprint, now, context_key=None):
        self.expire(now)
        penalty=0.
        for epoch,disturbed,key in self.events:
            if context_key is not None and key != context_key:
                continue
            distance=disturbed.distance(footprint)
            penalty=max(penalty,max(0.,1-distance/self.radius)*max(0.,1-(now-epoch)/self.ttl))
        return penalty
