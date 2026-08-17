"""Load hand-eye calibration matrix."""
import json, os
from pathlib import Path
import numpy as np


def _load_document(path):
    current = Path(path).expanduser().resolve()
    visited = set()
    for _ in range(4):
        if current in visited:
            raise ValueError(f'手眼标定指针循环引用: {current}')
        visited.add(current)
        with current.open(encoding='utf-8') as stream:
            data = json.load(stream)
        if isinstance(data, str):
            target = Path(data).expanduser()
            current = (target if target.is_absolute()
                       else current.parent / target).resolve()
            continue
        if isinstance(data, dict):
            return current, data
        raise ValueError(f'无效手眼标定入口: {current}')
    raise ValueError(f'手眼标定指针层级过深: {path}')


def load_X(path: str = None) -> np.ndarray:
    """加载 T_flange_camera (4x4)。旧字段仅作兼容读取。"""
    if path is None:
        path = os.path.expanduser(
            '~/Robot_Arm_Project/scripts/active_handeye_calibration.json')
    if not os.path.exists(path):
        raise FileNotFoundError(f'标定文件不存在: {path}')
    result_path, d = _load_document(path)
    record = d.get('T_flange_camera', d.get('X_camera_in_tool'))
    if record is None:
        raise ValueError(f'缺少 T_flange_camera: {result_path}')
    X = np.array(record['matrix'])
    print(f'📂 {result_path}\n   T_flange_camera xyz={record["xyz_mm"]}  rpy={record["rpy_deg"]}')
    return X
