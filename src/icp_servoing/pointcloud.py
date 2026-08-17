"""Point cloud processing."""
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    pts = []
    for p in pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True):
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            pts.append([x, y, z])
    return np.array(pts, dtype=np.float64) if pts else np.empty((0, 3))


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
