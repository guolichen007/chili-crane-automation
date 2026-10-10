"""PointCloud2 boundary: padding/endian-aware normalization to canonical XYZIRT."""
import math
import numpy as np

CANONICAL = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "sensor_id", "timestamp"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "u1", "<f8"],
    "offsets": [0, 4, 8, 12, 16, 18, 24], "itemsize": 32})
FORMATS = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
REQUIRED = ("x", "y", "z", "intensity", "timestamp")


def validate_points(points):
    if points.dtype != CANONICAL or len(points) == 0:
        raise ValueError("empty/noncanonical cloud")
    for field in REQUIRED:
        if not np.isfinite(points[field]).all():
            raise ValueError("nonfinite " + field)
    if (points["timestamp"] <= 0).any():
        raise ValueError("point timestamp must be explicit positive seconds")


def normalize_cloud(data, fields, point_step, row_step, width, height, bigendian=False, sensor_id=0):
    if (any(type(v) is not int or v <= 0 for v in (point_step, row_step, width, height))
            or width * height > 2_000_000 or row_step < width * point_step
            or len(data) != height * row_step):
        raise ValueError("invalid cloud dimensions/row padding/buffer")
    names, formats, offsets = [], [], []
    for field in fields:
        name, offset, datatype, count = field
        if name in names or datatype not in FORMATS or count != 1:
            raise ValueError("duplicate/unsupported point field")
        dtype = np.dtype((">" if bigendian else "<") + FORMATS[datatype])
        if type(offset) is not int or not 0 <= offset <= point_step - dtype.itemsize:
            raise ValueError("invalid field offset")
        if any(offset < previous + np.dtype(fmt).itemsize and previous < offset + dtype.itemsize
               for previous, fmt in zip(offsets, formats)):
            raise ValueError("overlapping point fields")
        names.append(name)
        formats.append(dtype)
        offsets.append(offset)
    if any(name not in names for name in REQUIRED):
        raise ValueError("required XYZIT field missing; no timestamp fabricated")
    input_type = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": point_step})
    source = np.ndarray((height, width), dtype=input_type, buffer=data,
                        strides=(row_step, point_step)).reshape(-1)
    result = np.zeros(len(source), dtype=CANONICAL)
    for name in REQUIRED:
        result[name] = source[name]
    if "ring" in names:
        if (source["ring"] < 0).any() or (source["ring"] > 65535).any():
            raise ValueError("invalid ring")
        result["ring"] = source["ring"]
    else:
        result["ring"] = 65535  # Explicit unavailable sentinel, not ring zero.
    result["sensor_id"] = sensor_id
    validate_points(result)
    return result


def crop(points, bounds):
    if (not isinstance(bounds, (list, tuple)) or len(bounds) != 6
            or any(type(v) not in (float, int) or not math.isfinite(v) for v in bounds)
            or any(bounds[i] >= bounds[i + 1] for i in (0, 2, 4))):
        raise ValueError("ROI bounds NOT_CONFIGURED")
    mask = np.ones(len(points), dtype=bool)
    for i, name in enumerate(("x", "y", "z")):
        mask &= (points[name] >= bounds[2 * i]) & (points[name] <= bounds[2 * i + 1])
    return points[mask].copy()
