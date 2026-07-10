import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(xyz, quat_xyzw=None, rpy_deg=None):
    transform = np.eye(4, dtype=float)
    transform[:3, 3] = np.asarray(xyz, dtype=float)
    if quat_xyzw is not None:
        transform[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    elif rpy_deg is not None:
        transform[:3, :3] = Rotation.from_euler(
            'xyz', rpy_deg, degrees=True).as_matrix()
    return transform


def load_handeye(path):
    path = Path(path).expanduser()
    with path.open(encoding='utf-8') as stream:
        data = json.load(stream)
    transform = np.asarray(data['X_camera_in_tool']['matrix'], dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f'无效手眼矩阵: {path}')
    return transform


def load_tcp_offset(path):
    """Load tool-origin to physical-tip translation in tool coordinates."""
    path = Path(path).expanduser()
    with path.open(encoding='utf-8') as stream:
        data = json.load(stream)
    offset = np.asarray(data['result']['tcp_offset_tool_mm'], dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError(f'无效TCP标定: {path}')
    return offset


def tool_pose_matrix(pose):
    """CR5 TCP pose: mm + XYZ intrinsic Euler degrees."""
    return pose_matrix(pose[:3], rpy_deg=pose[3:6])


def matrix_to_tool_pose(transform):
    rpy = Rotation.from_matrix(transform[:3, :3]).as_euler(
        'xyz', degrees=True)
    return [*transform[:3, 3].tolist(), *rpy.tolist()]
