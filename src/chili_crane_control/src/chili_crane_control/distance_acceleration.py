"""Grid distance is a rejection accelerator ONLY; never a geometry acceptance."""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from shapely.geometry import box


class RejectionDistanceField:
    def __init__(self,snapshot,polygon):
        self.snapshot=snapshot
        r=snapshot.resolution
        inside=np.zeros((snapshot.height,snapshot.width),dtype=bool)
        for row in range(snapshot.height):
            for col in range(snapshot.width):
                x=snapshot.origin[0]+col*r
                y=snapshot.origin[1]+row*r
                inside[row,col]=polygon.covers(box(x,y,x+r,y+r))
        self.distance=distance_transform_edt(np.pad(inside,1,constant_values=False))[1:-1,1:-1]*r

    def definitely_reject(self,xy,required_clearance):
        s=self.snapshot
        col=math.floor((xy[0]-s.origin[0])/s.resolution)
        row=math.floor((xy[1]-s.origin[1])/s.resolution)
        if not 0 <= col < s.width or not 0 <= row < s.height:
            return True
        # Approximation may reject a safe candidate; it cannot accept any candidate.
        return self.distance[row,col]+math.sqrt(2)*s.resolution < required_clearance
