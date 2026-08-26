import struct
from types import SimpleNamespace

import numpy as np

from icp_servoing.pointcloud import cloud_to_xyz


def field(name, offset, datatype=7):
    return SimpleNamespace(
        name=name, offset=offset, datatype=datatype, count=1)


def cloud(data, width, height=1, point_step=16, row_step=None,
          bigendian=False):
    return SimpleNamespace(
        fields=[field('x', 0), field('y', 4), field('z', 8)],
        width=width,
        height=height,
        point_step=point_step,
        row_step=row_step or width * point_step,
        is_bigendian=bigendian,
        data=data,
    )


def test_cloud_to_xyz_without_sensor_msgs_py_filters_nan():
    data = b''.join([
        struct.pack('<fffI', 1.0, 2.0, 3.0, 0),
        struct.pack('<fffI', float('nan'), 5.0, 6.0, 0),
        struct.pack('<fffI', -1.0, -2.0, -3.0, 0),
    ])
    result = cloud_to_xyz(cloud(data, width=3))
    assert result.dtype == np.float64
    assert np.allclose(result, [[1.0, 2.0, 3.0], [-1.0, -2.0, -3.0]])


def test_cloud_to_xyz_honours_organized_row_padding():
    first = struct.pack('<fffI', 1.0, 2.0, 3.0, 0)
    second = struct.pack('<fffI', 4.0, 5.0, 6.0, 0)
    result = cloud_to_xyz(cloud(
        first + b'padding' + second + b'padding',
        width=1, height=2, row_step=23))
    assert np.allclose(result, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])


def test_cloud_to_xyz_honours_big_endian_fields():
    data = struct.pack('>fffI', 1.25, -2.5, 3.75, 0)
    result = cloud_to_xyz(cloud(data, width=1, bigendian=True))
    assert np.allclose(result, [[1.25, -2.5, 3.75]])
