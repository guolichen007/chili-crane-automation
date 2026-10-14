"""Producer lifecycle admission only, independent of surface/ranking algorithms."""
import re


def valid_session_id(value):
    return isinstance(value,str) and re.fullmatch(r'[0-9a-f]{32}',value) is not None


def snapshot_identity(session_id,generation,config_sha,lineage):
    return f'{session_id}:{config_sha[:12]}:{generation}:{lineage[:12]}'


class SurfaceSessionGate:
    """Never evict retired IDs to silently re-admit old producer data.

    Capacity exhaustion is fail-closed and requires a deliberate consumer
    restart. It is an implementation resource bound, not a FIELD threshold.
    """
    def __init__(self):
        self.session_id=None
        self.retired=set()
        self.generation_floor=-1
        self.seen_generation=-1
        self.source_counter=-1
        self.exhausted=False

    def begin(self,session_id):
        if not valid_session_id(session_id):
            return 'INVALID_SESSION'
        if self.exhausted:
            return 'SESSION_HISTORY_CAPACITY_EXCEEDED'
        if session_id in self.retired:
            return 'RETIRED_SESSION_REPLAY'
        if session_id == self.session_id:
            return 'SAME_SESSION'
        if self.session_id is not None:
            if len(self.retired) >= 128:
                self.exhausted=True
                return 'SESSION_HISTORY_CAPACITY_EXCEEDED'
            self.retired.add(self.session_id)
        self.session_id=session_id
        self.generation_floor,self.source_counter=-1,-1
        self.seen_generation=-1
        return 'NEW_SESSION'

    def note(self,generation):
        # Even a STALE/invalid higher generation is an invalidation barrier;
        # never fall back to an older valid snapshot after observing it.
        if type(generation) is not int or generation < 0 or generation <= self.seen_generation:
            return False
        self.seen_generation=generation
        return True

    def accept(self,generation,counter):
        if (self.exhausted or self.session_id is None or type(generation) is not int
                or type(counter) is not int or generation < 0 or counter < 0):
            return False
        if generation <= self.generation_floor or counter < self.source_counter:
            return False
        self.generation_floor,self.source_counter=generation,counter
        return True
