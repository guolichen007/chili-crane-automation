"""World-translation/right-orientation 12-state ESKF and measured swing stability."""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation
from chili_crane_slam.extrinsic_solver import skew
from .geometry import covariance, positive
from .registration import equivalent_pose


@dataclass(frozen=True)
class SuspensionPrior:
    point: np.ndarray
    stamp: float
    evidence_id: str

    def fresh(self, now, ttl):
        return (np.asarray(self.point).shape == (3,) and np.isfinite(self.point).all()
                and self.evidence_id not in ("", "NOT_CONFIGURED")
                and 0 < self.stamp <= now and now - self.stamp <= ttl)


class GrabStateEstimator:
    def __init__(self, policy):
        self.policy = policy
        self.pose = None
        self.velocity = np.zeros(3)
        self.omega = np.zeros(3)
        self.covariance = np.zeros((12, 12))
        self.stamp = None
        self.last_measurement = None
        self.confirmations = 0
        self.tracking, self.source = "LOST", "NONE"
        self.innovation = 0.0

    def predict(self, stamp):
        positive(stamp, "measurement_stamp")
        if self.stamp is not None and stamp <= self.stamp:
            raise ValueError("DUPLICATE_OR_REGRESSING_MEASUREMENT_TIME")
        dt = 0 if self.stamp is None else stamp - self.stamp
        self.stamp = stamp
        if self.pose is None:
            return dt
        if dt > self.policy["maximum_prediction_dt_sec"]:
            self.missing(stamp, force_lost=True)
            return dt
        self.pose[:3, 3] += self.velocity * dt
        angle = self.omega * dt
        self.pose[:3, :3] = self.pose[:3, :3] @ Rotation.from_rotvec(angle).as_matrix()
        f = np.eye(12)
        f[:3, 6:9] = np.eye(3) * dt
        f[3:6, 3:6] = Rotation.from_rotvec(-angle).as_matrix()
        theta, a = np.linalg.norm(angle), skew(angle)
        jr = (np.eye(3) - a / 2 + a @ a / 6 if theta < 1e-8 else
              np.eye(3) - (1 - math.cos(theta)) / theta**2 * a
              + (theta - math.sin(theta)) / theta**3 * a @ a)
        f[3:6, 9:12] = jr * dt
        q = np.zeros((12, 12))
        for start, vel, noise in ((0, 6, self.policy["acceleration_noise"]),
                                   (3, 9, self.policy["angular_acceleration_noise"])):
            q[start:start+3, start:start+3] = np.eye(3) * noise**2 * dt**3 / 3
            q[start:start+3, vel:vel+3] = q[vel:vel+3, start:start+3] = np.eye(3) * noise**2 * dt**2 / 2
            q[vel:vel+3, vel:vel+3] = np.eye(3) * noise**2 * dt
        self.covariance = covariance(f @ self.covariance @ f.T + q, 12)
        self.source = "PREDICTED"
        return dt

    def update(self, pose, measurement_covariance, model):
        r = covariance(measurement_covariance, 6)
        if self.pose is None:
            self.pose = pose.copy()
            self.covariance = np.zeros((12,12))  # No old pose/twist cross terms across LOST reinitialization.
            self.covariance[:6, :6] = r
            self.covariance[6:, 6:] = np.eye(6) * self.policy["initial_twist_sigma"]**2
            self.innovation = 0.0
        else:
            original = pose
            pose = equivalent_pose(pose, self.pose, model.period)
            basis = np.eye(6)
            basis[3:,3:] = pose[:3,:3].T @ original[:3,:3]
            r = basis @ r @ basis.T
            residual = np.r_[pose[:3, 3] - self.pose[:3, 3],
                Rotation.from_matrix(self.pose[:3, :3].T @ pose[:3, :3]).as_rotvec()]
            self.innovation = float(np.linalg.norm(residual[:3]))
            h = np.zeros((6, 12))
            h[:, :6] = np.eye(6)
            s = h @ self.covariance @ h.T + r
            gain = np.linalg.solve(s, h @ self.covariance).T
            error = gain @ residual
            self.pose[:3, 3] += error[:3]
            self.pose[:3, :3] = self.pose[:3, :3] @ Rotation.from_rotvec(error[3:6]).as_matrix()
            self.velocity += error[6:9]
            self.omega += error[9:12]
            identity = np.eye(12) - gain @ h
            # Joseph update and right-error reset preserve PSD and orientation convention.
            p = identity @ self.covariance @ identity.T + gain @ r @ gain.T
            reset = np.eye(12)
            reset[3:6, 3:6] -= skew(error[3:6]) / 2
            self.covariance = covariance(reset @ p @ reset.T, 12)
        self.confirmations += 1
        self.last_measurement = self.stamp
        self.tracking = "TRACKED" if self.confirmations >= self.policy["confirmation_frames"] else "CANDIDATE"
        self.source = "FILTERED" if self.tracking == "TRACKED" else "MEASURED"

    def missing(self, stamp, force_lost=False):
        self.confirmations = 0
        if (force_lost or self.last_measurement is None
                or stamp - self.last_measurement > self.policy["lost_timeout_sec"]):
            self.pose = None
            self.velocity[:] = 0
            self.omega[:] = 0
            self.tracking, self.source = "LOST", "NONE"
        else:
            self.tracking, self.source = "LOST_HOLD", "PREDICTED"


class SwingStability:
    def __init__(self, policy):
        self.policy = policy
        self.window = deque(maxlen=128)
        self.previous = None
        self.stable = False

    def update(self, state, suspension, attachment):
        if state.pose is None or suspension is None or state.source == "PREDICTED":
            self.window.clear()
            self.previous, self.stable = None, False
            return dict(valid=False, motion_stable=False)
        point = state.pose[:3, :3] @ attachment + state.pose[:3, 3]
        delta = point - suspension.point
        drop, horizontal = -float(delta[2]), float(np.linalg.norm(delta[:2]))
        if drop <= 0:
            self.window.clear()
            self.previous, self.stable = None, False
            return dict(valid=False, motion_stable=False)
        angle = math.atan2(horizontal, drop)
        azimuth = math.atan2(float(delta[1]), float(delta[0]))
        rate = 0.0
        if self.previous is not None:
            previous_stamp, previous_angle = self.previous
            rate = (angle - previous_angle) / (state.stamp - previous_stamp)
        self.previous = (state.stamp, angle)
        metrics = np.array([np.linalg.norm(state.velocity[:2]), angle, abs(rate), state.innovation])
        self.window.append((state.stamp, metrics))
        duration = self.policy["minimum_stable_duration_sec"]
        while len(self.window) > 1 and state.stamp - self.window[1][0] >= duration:
            self.window.popleft()
        limits = np.asarray(self.policy["stable_exit"] if self.stable else self.policy["stable_enter"])
        aggregate = np.max([v for _, v in self.window], axis=0)
        aggregate[3] = np.sqrt(np.mean([v[3]**2 for _, v in self.window]))
        sufficient = state.stamp - self.window[0][0] >= duration
        self.stable = bool(sufficient and np.all(aggregate <= limits) and state.tracking == "TRACKED")
        return dict(valid=True, horizontal_displacement_m=horizontal, vertical_drop_m=drop,
            swing_angle_rad=angle, swing_azimuth_rad=azimuth, swing_angular_velocity_radps=rate,
            lateral_velocity_mps=float(metrics[0]), innovation_rms_m=float(aggregate[3]),
            motion_stable=self.stable)
