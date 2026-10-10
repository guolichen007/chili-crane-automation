"""Bounded consume-once synchronizer and explicitly calibrated vectorized merger."""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np
from .pointcloud import CANONICAL, validate_points


@dataclass(frozen=True)
class Cloud:
    stamp: float
    received_monotonic: float
    frame_id: str
    points: np.ndarray
    source_type: int = 0
    received_source_time: float | None = None


class DualLidarHealth:
    def __init__(self, stale_timeout):
        if not math.isfinite(stale_timeout) or stale_timeout <= 0:
            raise ValueError("invalid diagnostic stale timeout")
        self.stale = stale_timeout
        self.latest = {}
        self.arrivals = {"204": deque(maxlen=100), "205": deque(maxlen=100)}
        self.invalid = {"204": 0, "205": 0}

    def accepted(self, sensor, cloud):
        self.latest[sensor] = cloud
        self.arrivals[sensor].append(cloud.received_monotonic)

    def status(self, sensor, source_now, monotonic_now):
        cloud = self.latest.get(sensor)
        if cloud is None:
            return {"online": False, "fresh": False, "age_sec": None, "source_age_sec": None, "hz": 0.0}
        age, source_age = monotonic_now - cloud.received_monotonic, source_now - cloud.stamp
        arrivals = [v for v in self.arrivals[sensor] if monotonic_now - v <= self.stale]
        duration = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0
        return {"online": 0 <= age <= self.stale,
                "fresh": 0 <= age <= self.stale and 0 <= source_age <= self.stale,
                "age_sec": age, "source_age_sec": source_age,
                "hz": (len(arrivals) - 1) / duration if duration > 0 else 0.0}


class DualLidarSynchronizer:
    def __init__(self, maximum_pair_delta, stale_timeout, maximum_queue_size=16):
        if (any(type(v) not in (float, int) or not math.isfinite(v) or v <= 0
                for v in (maximum_pair_delta, stale_timeout))
                or maximum_pair_delta > stale_timeout
                or type(maximum_queue_size) is not int or not 1 <= maximum_queue_size <= 128):
            raise ValueError("sync timing/queue NOT_CONFIGURED")
        self.delta, self.stale, self.capacity = maximum_pair_delta, stale_timeout, maximum_queue_size
        self.queues = {"204": deque(), "205": deque()}
        self.last_stamp = {"204": 0.0, "205": 0.0}
        self.dropped = {"204": 0, "205": 0}
        self.paired_count = 0
        self.pair_delta = None
        self.last_pair = None

    def push(self, sensor, cloud, source_now, monotonic_now):
        if sensor not in self.queues:
            raise ValueError("unknown sensor")
        validate_points(cloud.points)
        if (not cloud.frame_id or cloud.frame_id == "NOT_CONFIGURED"
                or any(type(v) not in (int, float) or not math.isfinite(v)
                       for v in (cloud.stamp, source_now, monotonic_now, cloud.received_monotonic))
                or cloud.stamp <= self.last_stamp[sensor]
                or not 0 <= source_now - cloud.stamp <= self.stale
                or not 0 <= monotonic_now - cloud.received_monotonic <= self.stale):
            self.dropped[sensor] += 1
            return []
        self.last_stamp[sensor] = cloud.stamp
        queue = self.queues[sensor]
        if len(queue) == self.capacity:
            queue.popleft()
            self.dropped[sensor] += 1
        queue.append(cloud)
        self.expire(source_now, monotonic_now)
        pairs = []
        a, b = self.queues["204"], self.queues["205"]
        while a and b:
            delta = a[0].stamp - b[0].stamp
            if abs(delta) <= self.delta + 1e-9:
                first, second = a.popleft(), b.popleft()
                self.paired_count += 1
                self.pair_delta = abs(delta)
                self.last_pair = (first, second)
                pairs.append((first, second))
            elif delta < 0:
                a.popleft()
                self.dropped["204"] += 1
            else:
                b.popleft()
                self.dropped["205"] += 1
        return pairs

    def expire(self, source_now, monotonic_now):
        for sensor, queue in self.queues.items():
            while queue and (not 0 <= source_now - queue[0].stamp <= self.stale
                    or not 0 <= monotonic_now - queue[0].received_monotonic <= self.stale):
                queue.popleft()
                self.dropped[sensor] += 1


@dataclass(frozen=True)
class Extrinsic:
    source_frame: str
    target_frame: str
    calibration_id: str
    rotation: np.ndarray
    translation: np.ndarray

    @classmethod
    def from_config(cls, config):
        if config.get("config_state") != "VALID" or not config.get("calibration_id") or config["calibration_id"] == "NOT_CONFIGURED":
            raise ValueError("extrinsic NOT_CONFIGURED")
        source, target = config.get("source_frame"), config.get("target_frame")
        if any(not isinstance(x, str) or x in {"", "NOT_CONFIGURED"} for x in (source, target)):
            raise ValueError("extrinsic frame missing")
        rotation = np.asarray(config.get("rotation_matrix"), dtype=float)
        translation = np.asarray(config.get("translation_m"), dtype=float)
        if (rotation.shape != (3, 3) or translation.shape != (3,)
                or not np.isfinite(rotation).all() or not np.isfinite(translation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
            raise ValueError("invalid rigid extrinsic")
        rotation.setflags(write=False)
        translation.setflags(write=False)
        return cls(source, target, config["calibration_id"], rotation, translation)

    def transform(self, cloud):
        if cloud.frame_id != self.source_frame:
            raise ValueError("cloud/extrinsic frame mismatch")
        result = cloud.points.copy()
        xyz = np.column_stack([result[n] for n in ("x", "y", "z")])
        xyz = xyz @ self.rotation.T + self.translation
        for i, name in enumerate(("x", "y", "z")):
            result[name] = xyz[:, i]
        validate_points(result)
        return result


class DualLidarMerger:
    def __init__(self, a, b):
        if a.target_frame != b.target_frame:
            raise ValueError("extrinsics must share base frame")
        self.a, self.b = a, b

    def merge(self, pair):
        first, second = pair
        if first.source_type != second.source_type or first.source_type not in (1, 2, 3, 4):
            raise ValueError("mixed/unknown pointcloud provenance")
        # NumPy otherwise repacks structured fields, silently dropping canonical padding.
        points = np.concatenate((self.a.transform(first), self.b.transform(second)), dtype=CANONICAL)
        validate_points(points)
        return Cloud(max(first.stamp, second.stamp), max(first.received_monotonic, second.received_monotonic),
                     self.a.target_frame, points, first.source_type,
                     max(first.received_source_time, second.received_source_time)
                     if first.received_source_time is not None and second.received_source_time is not None else None)
