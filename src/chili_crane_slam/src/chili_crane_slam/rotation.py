"""Rotation matrix to quaternion for publishing only validated calibration TF."""
import math


def quaternion(rotation):
    r = rotation
    trace = float(r[0, 0] + r[1, 1] + r[2, 2])
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        return ((r[2,1] - r[1,2]) / s, (r[0,2] - r[2,0]) / s,
                (r[1,0] - r[0,1]) / s, .25 * s)
    i = max(range(3), key=lambda n: r[n, n])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1 + r[i,i] - r[j,j] - r[k,k]) * 2
    q = [0.0] * 4
    q[i], q[j], q[k], q[3] = .25 * s, (r[j,i] + r[i,j]) / s, (r[k,i] + r[i,k]) / s, (r[k,j] - r[j,k]) / s
    return tuple(q)
