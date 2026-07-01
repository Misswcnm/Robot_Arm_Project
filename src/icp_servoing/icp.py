"""Multi-scale ICP (point-to-point, Kabsch-Umeyama)."""
import numpy as np
from scipy.spatial import KDTree


def icp_step(src: np.ndarray, tgt: np.ndarray, dmax: float = 0.05):
    tree = KDTree(tgt)
    dist, idx = tree.query(src, distance_upper_bound=dmax)
    mask = dist < dmax
    if mask.sum() < 10:
        return None, None, float('inf'), 0
    s, d = src[mask], tgt[idx[mask]]
    cs, cd = s.mean(0), d.mean(0)
    H = (s - cs).T @ (d - cd)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2] *= -1
        R = Vt.T @ U.T
    t = cd - R @ cs
    err = np.sqrt(np.mean(np.sum(((R @ s.T).T + t - d) ** 2, axis=1)))
    return R, t, err, int(mask.sum())


def icp_align(src: np.ndarray, tgt: np.ndarray,
              init_guess: np.ndarray = None,
              max_iter: int = 40, dmax: float = 0.05, tol: float = 1e-6):
    """
    ICP from src to tgt, with optional initial guess.
    返回: (T_4x4, info) where p_tgt ≈ T[:3,:3] @ p_src + T[:3,3]
    """
    R_acc, t_acc = np.eye(3), np.zeros(3)
    if init_guess is not None:
        R_acc, t_acc = init_guess[:3, :3], init_guess[:3, 3]
        cur = (R_acc @ src.T).T + t_acc
    else:
        cur = src.copy()

    for i in range(max_iter):
        Ri, ti, err, n = icp_step(cur, tgt, dmax)
        if Ri is None:
            T = np.eye(4)
            T[:3, :3] = R_acc
            T[:3, 3] = t_acc
            return T, dict(rmse=float('inf'), iters=i, converged=False, inliers=0, overlap=0.0)
        t_acc = Ri @ t_acc + ti
        R_acc = Ri @ R_acc
        cur = (Ri @ cur.T).T + ti
        if i >= 5 and err < tol:
            break

    T = np.eye(4)
    T[:3, :3] = R_acc
    T[:3, 3] = t_acc

    # Compute final overlap stats
    tree = KDTree(tgt)
    cur = (R_acc @ src.T).T + t_acc
    dist, _ = tree.query(cur, distance_upper_bound=dmax)
    inliers = int((dist < dmax).sum())
    overlap = inliers / len(cur) if len(cur) > 0 else 0

    return T, dict(rmse=float(err), iters=i + 1,
                   converged=(err < 0.05),
                   inliers=inliers,
                   overlap=float(overlap),
                   n_src=len(src), n_tgt=len(tgt))


def multi_scale_icp(src: np.ndarray, tgt: np.ndarray,
                    init_guess: np.ndarray = None,
                    scales: list = None) -> tuple:
    """
    多尺度ICP:
      scale 0: 20mm voxel, dmax=100mm  (粗)
      scale 1: 10mm voxel, dmax=50mm   (中)
      scale 2: 5mm voxel,  dmax=25mm   (精)
    返回: (T_4x4, info_dict)
    """
    if scales is None:
        from icp_servoing.pointcloud import voxel_down
        scales = [
            (0.020, 0.100),
            (0.010, 0.050),
            (0.005, 0.025),
        ]

    T_cur = init_guess.copy() if init_guess is not None else np.eye(4)
    all_info = []

    for vs, dmax in scales:
        # Downsample
        n_vox = int(1.0 / vs)
        idx_src = np.floor(src / vs).astype(np.int64)
        _, u_src = np.unique(idx_src, axis=0, return_index=True)
        idx_tgt = np.floor(tgt / vs).astype(np.int64)
        _, u_tgt = np.unique(idx_tgt, axis=0, return_index=True)

        T_new, info = icp_align(src[u_src], tgt[u_tgt],
                                init_guess=T_cur, max_iter=30, dmax=dmax)
        T_cur = T_new
        info['scale_vs_mm'] = vs * 1000
        info['scale_dmax_mm'] = dmax * 1000
        all_info.append(info)

    info = {
        'rmse': all_info[-1]['rmse'] if all_info else float('inf'),
        'overlap': all_info[-1]['overlap'] if all_info else 0,
        'inliers': all_info[-1]['inliers'] if all_info else 0,
        'converged': all_info[-1]['converged'] if all_info else False,
        'scales': all_info,
    }
    return T_cur, info
