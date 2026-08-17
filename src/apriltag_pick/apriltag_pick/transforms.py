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


def load_handeye_document(path):
    """Resolve the active-handeye pointer and return (result_path, JSON)."""
    current = Path(path).expanduser()
    if not current.is_absolute():
        current = current.resolve()
    visited = set()
    for _ in range(4):
        current = current.resolve()
        if current in visited:
            raise ValueError(f'手眼标定指针循环引用: {current}')
        visited.add(current)
        with current.open(encoding='utf-8') as stream:
            data = json.load(stream)
        if isinstance(data, str):
            target = Path(data).expanduser()
            current = target if target.is_absolute() else current.parent / target
            continue
        if not isinstance(data, dict):
            raise ValueError(f'无效手眼标定入口: {current}')
        return current, data
    raise ValueError(f'手眼标定指针层级过深: {path}')


def load_handeye(path):
    path, data = load_handeye_document(path)
    record = data.get('T_flange_camera', data.get('X_camera_in_tool'))
    if record is None:
        raise ValueError(f'缺少 T_flange_camera: {path}')
    transform = np.asarray(record['matrix'], dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f'无效 T_flange_camera: {path}')
    return transform


def load_tcp_offset(path):
    """Load flange-origin to physical-tip translation in flange coordinates."""
    current = Path(path).expanduser()
    if not current.is_absolute():
        current = current.resolve()
    visited = set()
    for _ in range(4):
        current = current.resolve()
        if current in visited:
            raise ValueError(f'TCP标定指针循环引用: {current}')
        visited.add(current)
        with current.open(encoding='utf-8') as stream:
            data = json.load(stream)
        if isinstance(data, str):
            target = Path(data).expanduser()
            current = target if target.is_absolute() else current.parent / target
            continue
        if not isinstance(data, dict):
            raise ValueError(f'无效TCP标定入口: {current}')
        break
    else:
        raise ValueError(f'TCP标定指针层级过深: {path}')
    result = data['result']
    record = result.get(
        'tcp_offset_flange_mm', result.get('tcp_offset_tool_mm'))
    if record is None:
        raise ValueError(f'缺少 tcp_offset_flange_mm: {current}')
    offset = np.asarray(record, dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError(f'无效TCP标定: {current}')
    return offset


def tool_pose_matrix(pose):
    """CR5 GetPose in User0, i.e. T_base_flange: mm + XYZ Euler deg."""
    return pose_matrix(pose[:3], rpy_deg=pose[3:6])


def matrix_to_tool_pose(transform):
    rpy = Rotation.from_matrix(transform[:3, :3]).as_euler(
        'xyz', degrees=True)
    return [*transform[:3, 3].tolist(), *rpy.tolist()]
