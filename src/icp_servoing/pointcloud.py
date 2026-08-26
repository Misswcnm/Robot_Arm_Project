"""Point cloud processing."""
from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2


_POINT_FIELD_DTYPES = {
    1: 'i1',   # INT8
    2: 'u1',   # UINT8
    3: 'i2',   # INT16
    4: 'u2',   # UINT16
    5: 'i4',   # INT32
    6: 'u4',   # UINT32
    7: 'f4',   # FLOAT32
    8: 'f8',   # FLOAT64
}


def _xyz_dtype(msg: PointCloud2) -> np.dtype:
    """Build a zero-copy XYZ view without depending on sensor_msgs_py."""
    fields = {field.name: field for field in msg.fields}
    missing = [name for name in ('x', 'y', 'z') if name not in fields]
    if missing:
        raise ValueError(
            'PointCloud2 is missing fields: %s' % ', '.join(missing))
    if int(msg.point_step) <= 0:
        raise ValueError('PointCloud2 point_step must be positive')

    endian = '>' if bool(msg.is_bigendian) else '<'
    formats = []
    offsets = []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        code = _POINT_FIELD_DTYPES.get(int(field.datatype))
        if code is None:
            raise ValueError(
                'PointCloud2 field %s has unsupported datatype %s' %
                (name, field.datatype))
        if int(getattr(field, 'count', 1)) < 1:
            raise ValueError('PointCloud2 field %s has invalid count' % name)
        dtype = np.dtype(endian + code)
        offset = int(field.offset)
        if offset < 0 or offset + dtype.itemsize > int(msg.point_step):
            raise ValueError(
                'PointCloud2 field %s exceeds point_step' % name)
        formats.append(dtype)
        offsets.append(offset)
    return np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': formats,
        'offsets': offsets,
        'itemsize': int(msg.point_step),
    })


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Convert organized/unorganized PointCloud2 XYZ fields to float64.

    ROS 2 Foxy does not always ship the optional ``sensor_msgs_py`` module.
    Parsing the standard PointCloud2 byte layout directly also avoids the
    Python-per-point loop used by the compatibility helper.
    """
    width = int(msg.width)
    height = int(msg.height)
    if width <= 0 or height <= 0:
        return np.empty((0, 3), dtype=np.float64)
    point_step = int(msg.point_step)
    row_bytes = width * point_step
    row_step = int(msg.row_step) if int(msg.row_step) > 0 else row_bytes
    if row_step < row_bytes:
        raise ValueError('PointCloud2 row_step is smaller than one data row')
    required = (height - 1) * row_step + row_bytes
    if len(msg.data) < required:
        raise ValueError(
            'PointCloud2 data is truncated: %d < %d bytes' %
            (len(msg.data), required))

    dtype = _xyz_dtype(msg)
    data = memoryview(msg.data)
    rows = []
    for row in range(height):
        start = row * row_step
        view = np.ndarray(
            shape=(width,), dtype=dtype,
            buffer=data[start:start + row_bytes])
        rows.append(np.column_stack((view['x'], view['y'], view['z'])))
    points = np.concatenate(rows, axis=0).astype(np.float64, copy=False)
    return points[np.all(np.isfinite(points), axis=1)]


def voxel_down(pts: np.ndarray, vs: float = 0.005) -> np.ndarray:
    """体素下采样, vs(m)"""
    if len(pts) == 0 or vs <= 0:
        return pts
    idx = np.floor(pts / vs).astype(np.int64)
    _, u = np.unique(idx, axis=0, return_index=True)
    return pts[u]


def fuse_frames(frames: list[np.ndarray], vs: float = 0.002) -> np.ndarray:
    """多帧点云融合: 合并+下采样"""
    if not frames:
        return np.empty((0, 3))
    merged = np.vstack(frames)
    return voxel_down(merged, vs)
