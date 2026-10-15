"""R2 whole-window lifetime around the unchanged Phase2B builder."""
from chili_crane_control.snapshot_cycle import SnapshotCycleGate
from .surface_mask import PitSurfaceBuilder


class CycleSnapshotWindow:
    def __init__(self,config,semantic,session):
        self.config,self.semantic,self.session=config,semantic,session
        self.gate=SnapshotCycleGate(config['source_type'],config['frame_ttl_sec'])
        self.builder=PitSurfaceBuilder(config,semantic,session)
        self.frozen=None

    def observe(self,cycle,now):
        reset=self.gate.observe(cycle,now)
        if reset:
            generation=self.builder.grid.generation+1
            self.builder=PitSurfaceBuilder(self.config,self.semantic,self.session)
            self.builder.grid.generation=generation
            self.frozen=None
        return reset

    def ingest(self,xyz,sensors,frame,context,grab,now):
        if not self.gate.scanning(now):return False
        if int(round(frame['epoch']*1e9))<=self.gate.scan_start_ns:return False
        return self.builder.ingest(xyz,sensors,frame,context,grab,now)

    def snapshot(self,now):
        if self.gate.scanning(now):
            self.frozen=self.builder.grid.snapshot(now)
            self.gate.snapshot_ready=self.frozen.context is not None and self.frozen.validity!='STALE'
        return self.frozen
