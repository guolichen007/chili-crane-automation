"""Bounded original observations for asynchronous LiDAR MID_SCAN masking."""
from collections import OrderedDict


class GrabMaskPairs:
    def __init__(self,capacity=32):
        self.capacity=capacity
        self.samples={key:OrderedDict() for key in ('grab','clearance')}

    def add(self,kind,message,arrival):
        stamp=message.header.stamp
        key=(stamp.sec*1_000_000_000+stamp.nanosec,message.evidence.source_counter,message.header.frame_id)
        samples=self.samples[kind]
        samples[key]=(message,arrival)
        while len(samples)>self.capacity:samples.popitem(last=False)

    def closest(self,mid_epoch,maximum_skew):
        mid_ns=int(round(mid_epoch*1e9))
        keys=self.samples['grab'].keys() & self.samples['clearance'].keys()
        keys=[key for key in keys if abs(key[0]-mid_ns)<=maximum_skew*1e9]
        if not keys:raise ValueError('GRAB_MASK_TIME_ALIGNMENT_REJECTED')
        key=min(keys,key=lambda k:(abs(k[0]-mid_ns),-k[0]))
        return self.samples['grab'][key],self.samples['clearance'][key]
