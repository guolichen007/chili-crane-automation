"""Acceptance fixture scheduling: metadata cannot advance without its cloud."""
from collections import deque
import math


def _stamp(s):return (s.sec,s.nanosec)


class AtomicLidarPublisher:
    def __init__(self,publishers,interval_sec=.2):
        if not math.isfinite(interval_sec) or interval_sec<=0:raise ValueError('INVALID_FIXTURE_LIDAR_INTERVAL')
        self.publishers=publishers;self.interval=interval_sec;self.next_due=0.
        self.counts=dict.fromkeys(('cloud','meta','context'),0)
        self.records=deque(maxlen=64);self.last_counter=-1;self.last_stamp=(-1,-1)

    def publish_due(self,monotonic,factory):
        if monotonic<self.next_due:return False
        cloud,meta,context=factory()
        stamps=(_stamp(cloud.header.stamp),_stamp(meta.header.stamp),_stamp(meta.measurement_stamp),
                _stamp(context.header.stamp),_stamp(context.evidence.header.stamp),_stamp(context.evidence.measurement_stamp))
        if (len(set(stamps))!=1 or meta.source_counter!=context.evidence.source_counter
                or meta.source_counter<=self.last_counter or stamps[0]<=self.last_stamp
                or meta.header.frame_id!=cloud.header.frame_id or context.header.frame_id!=cloud.header.frame_id):
            raise ValueError('LIDAR_FRAME_BUNDLE_IDENTITY_CONFLICT')
        # No event-loop yield between these three publications.
        for key,message in (('meta',meta),('context',context),('cloud',cloud)):
            self.publishers[key].publish(message);self.counts[key]+=1
        self.last_counter=meta.source_counter;self.last_stamp=stamps[0]
        self.records.append(dict(counter=meta.source_counter,stamp=stamps[0]))
        self.next_due=monotonic+self.interval
        return True
