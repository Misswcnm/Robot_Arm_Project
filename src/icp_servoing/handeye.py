"""Load hand-eye calibration matrix."""
import json, os
import numpy as np


def load_X(path: str = None) -> np.ndarray:
    """加载 T_camera_in_tool (4x4)"""
    if path is None:
        path = os.path.expanduser('~/Robot_Arm_Project/scripts/handeye_chessboard_result.json')
    if not os.path.exists(path):
        raise FileNotFoundError(f'标定文件不存在: {path}')
    with open(path) as f:
        d = json.load(f)
    X = np.array(d['X_camera_in_tool']['matrix'])
    print(f'📂 {path}\n   xyz={d["X_camera_in_tool"]["xyz_mm"]}  rpy={d["X_camera_in_tool"]["rpy_deg"]}')
    return X
