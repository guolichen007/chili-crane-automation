"""Phase2B-local scan anchors; never reinterpret or alter upstream headers."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class ScanEpochs:
    header_epoch: float
    start_epoch: float
    end_epoch: float
    mid_epoch: float

    @property
    def span_sec(self):
        return self.end_epoch-self.start_epoch


def scan_epochs(header_epoch, timestamps, sensor_ids):
    times, sensors = np.asarray(timestamps,dtype=float),np.asarray(sensor_ids)
    if (times.ndim != 1 or not times.size or sensors.shape != times.shape
            or not np.isfinite(times).all() or (times <= 0).any()
            or not np.isin(sensors,[0,1]).all()):
        raise ValueError('POINT_TIMESTAMPS_INVALID')
    # The unchanged merger concatenates two scans. Global ordering can restart
    # at the second scan; regression is checked within each sensor's true order.
    for sensor in (0,1):
        if (np.diff(times[sensors==sensor]) < 0).any():
            raise ValueError('POINT_TIMESTAMP_REGRESSION')
    start,end = float(times.min()),float(times.max())
    if not math.isfinite(header_epoch) or not start <= header_epoch <= end:
        raise ValueError('HEADER_OUTSIDE_POINT_TIME_RANGE')
    return ScanEpochs(float(header_epoch),start,end,start+(end-start)/2)
