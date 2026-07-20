"""
ICP Visual Servoing — 闭环补偿核心逻辑
======================================
公式链:
  T_cam = pyramid_icp(src, ref, T_init)  (current→template, camera frame)
  T_delta = X @ T_cam @ X⁻¹              (camera→tool frame)
  T_correction = inv(T_delta)             (取消误差的方向)
  T_target = T_cur @ T_correction         (100%全量补偿)
  → MovJ(x,y,z,rx,ry,rz)                 (控制器笛卡尔点到点)
    失败/未到位时 → Jacobian IK → JointMovJ 兜底
"""
import time, rclpy
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from icp_servoing.robot import CR5Robot
from icp_servoing.pointcloud import voxel_down


class VisualServo:
    def __init__(self, robot: CR5Robot, X: np.ndarray,
                 speed: int = 15):
        self.robot = robot
        self.X = X
        self.X_inv = np.linalg.inv(X)

        # Template pyramid (pre-built for speed)
        self._ref_pyramid = None
        self._p2plane_ref = None
        self._T_base_tool_ref = None
        self._T_base_camera_ref = None
        self._last_tool = None
        self._step_count = 0
        self._total_dp = 0.0; self._total_dr = 0.0
        self.final_icp_trans_thresh_mm = 2.0
        self.final_icp_rot_thresh_deg = 0.2
        self.final_stable_frames = 2
        self._stable_count = 0
        self.point_to_plane_trigger_mm = 10.0
        self.point_to_plane_dmax = 0.010
        self.point_to_plane_iters = 6
        self.point_to_plane_max_points = 80000
        self._cartesian_movj_disabled = False

    # ── Quality gate ──
    @staticmethod
    def quality_ok(info: dict, T_icp: np.ndarray,
                   max_rmse: float = 0.060,
                   min_overlap: float = 0.03,
                   min_inliers: int = 500,
                   max_translation: float = 0.500,
                   max_rotation_deg: float = 60.0) -> bool:
        """宽松门控 — 大偏移下ICP overlap会很低, 只要>500匹配点且|t|<500mm就允许补偿"""
        if info.get('inliers', 0) < min_inliers:
            return False
        if info.get('rmse', 99) > max_rmse:
            return False
        t_norm = np.linalg.norm(T_icp[:3, 3])
        r_deg = np.degrees(np.linalg.norm(
            Rot.from_matrix(T_icp[:3, :3]).as_rotvec()))
        if t_norm > max_translation:
            return False
        if r_deg > max_rotation_deg:
            return False
        return True

    @staticmethod
    def converged(T_icp: np.ndarray,
                  trans_thresh: float = 2.0,
                  rot_thresh_deg: float = 0.2) -> bool:
        """判断视觉剩余校正是否已收敛: T_icp ≈ I"""
        t_norm = np.linalg.norm(T_icp[:3, 3])
        r_angle = np.degrees(
            np.linalg.norm(Rot.from_matrix(T_icp[:3, :3]).as_rotvec()))
        return t_norm < trans_thresh and r_angle < rot_thresh_deg

    @staticmethod
    def pose_error(T_cur: np.ndarray, T_ref: np.ndarray) -> tuple[float, float]:
        """Tool当前位姿相对模板位姿的误差: 返回(mm, deg)."""
        T_err = np.linalg.inv(T_ref) @ T_cur
        t_err = np.linalg.norm(T_err[:3, 3])
        r_err = np.degrees(
            np.linalg.norm(Rot.from_matrix(T_err[:3, :3]).as_rotvec()))
        return float(t_err), float(r_err)

    @staticmethod
    def _estimate_normals(pts: np.ndarray, tree, k: int = 16) -> np.ndarray:
        """用局部PCA估计模板点法向, 供point-to-plane精配准使用."""
        if len(pts) == 0:
            return np.empty((0, 3), dtype=np.float64)
        kk = min(k, len(pts))
        _, idx = tree.query(pts, k=kk, workers=-1)
        if kk == 1:
            return np.tile(np.array([0.0, 0.0, 1.0]), (len(pts), 1))
        normals = np.zeros_like(pts)
        for i, neigh_idx in enumerate(idx):
            q = pts[np.atleast_1d(neigh_idx)]
            q = q - q.mean(axis=0)
            _, _, vh = np.linalg.svd(q, full_matrices=False)
            n = vh[-1]
            normals[i] = n / (np.linalg.norm(n) + 1e-12)
        return normals

    @staticmethod
    def _stride_sample(pts: np.ndarray, max_points: int) -> np.ndarray:
        """确定性等步长抽样, 避免point-to-plane处理过多重复点."""
        if len(pts) <= max_points:
            return pts
        idx = np.linspace(0, len(pts) - 1, max_points, dtype=np.int64)
        return pts[idx]

    @staticmethod
    def _point_to_plane_refine(src: np.ndarray, tgt: np.ndarray, tree,
                               normals: np.ndarray, T_init: np.ndarray,
                               dmax: float = 0.012,
                               max_iter: int = 6) -> tuple[np.ndarray, dict]:
        """小残差下的point-to-plane ICP微调, 单位沿用点云(m)."""
        T = T_init.copy()
        last_rmse = float('inf')
        inliers = 0
        overlap = 0.0

        for _ in range(max_iter):
            src_tf = (T[:3, :3] @ src.T).T + T[:3, 3]
            dist, idx = tree.query(src_tf, distance_upper_bound=dmax, workers=-1)
            mask = dist < dmax
            inliers = int(mask.sum())
            overlap = inliers / len(src) if len(src) else 0.0
            if inliers < 30:
                break

            p = src_tf[mask]
            q = tgt[idx[mask]]
            n = normals[idx[mask]]
            residual = np.sum((p - q) * n, axis=1)

            keep = np.abs(residual) < max(dmax, 2.5 * np.median(np.abs(residual)) + 1e-6)
            if int(keep.sum()) < 30:
                break
            p, n, residual = p[keep], n[keep], residual[keep]
            inliers = int(keep.sum())
            overlap = inliers / len(src) if len(src) else 0.0

            A = np.hstack([np.cross(p, n), n])
            try:
                delta, *_ = np.linalg.lstsq(A, -residual, rcond=1e-4)
            except np.linalg.LinAlgError:
                break

            rotvec = delta[:3]
            t_step = delta[3:]
            rot_norm = np.linalg.norm(rotvec)
            t_norm = np.linalg.norm(t_step)
            if rot_norm > np.radians(0.5):
                rotvec *= np.radians(0.5) / rot_norm
            if t_norm > 0.003:
                t_step *= 0.003 / t_norm

            R_step = Rot.from_rotvec(rotvec).as_matrix()
            T[:3, :3] = R_step @ T[:3, :3]
            T[:3, 3] = R_step @ T[:3, 3] + t_step

            rmse = float(np.sqrt(np.mean(residual ** 2)))
            if abs(last_rmse - rmse) < 1e-5:
                last_rmse = rmse
                break
            last_rmse = rmse

        src_tf = (T[:3, :3] @ src.T).T + T[:3, 3]
        dist, idx = tree.query(src_tf, distance_upper_bound=dmax, workers=-1)
        mask = dist < dmax
        inliers = int(mask.sum())
        overlap = inliers / len(src) if len(src) else 0.0
        if inliers >= 30:
            p = src_tf[mask]
            q = tgt[idx[mask]]
            n = normals[idx[mask]]
            residual = np.sum((p - q) * n, axis=1)
            last_rmse = float(np.sqrt(np.mean(residual ** 2)))

        return T, {'rmse': last_rmse, 'inliers': inliers, 'overlap': overlap}

    # ── Template: pre-build pyramid with KDTree ──
    def record_template(self, n_frames: int = 5) -> bool:
        """多帧融合→5mm压缩→预建3层KDTree pyramid"""
        print(f'🎯 录制模板 ({n_frames} 帧)...')
        pc_node = getattr(self, '_pc_node', None)
        frames = []
        for i in range(n_frames):
            pc = self._capture_pc(vs=0.005)
            if pc is not None:
                frames.append(pc)
                print(f'  [{i+1}/{n_frames}] {len(pc)} 点')
        if len(frames) < 2:
            print('❌ 点云帧不足'); return False

        # Fuse + compress to 5mm (target ~100k points)
        merged = np.vstack(frames)
        ref_5mm = voxel_down(merged, 0.005)
        print(f'  融合{len(frames)}帧→{len(ref_5mm)}点(5mm压缩)')

        # Build pyramid: 20mm, 10mm, 5mm (each with pre-built cKDTree)
        from scipy.spatial import cKDTree
        self._ref_pyramid = []
        for vs in [0.020, 0.010, 0.005]:
            pts = voxel_down(ref_5mm if vs < 0.01 else merged, vs)
            tree = cKDTree(pts)
            self._ref_pyramid.append((pts, tree, vs))
            print(f'  L{len(self._ref_pyramid)-1}: {len(pts)}pts vs={vs*1000:.0f}mm')

        p2_pts = self._stride_sample(ref_5mm, self.point_to_plane_max_points)
        p2_tree = cKDTree(p2_pts)
        t_normals = time.perf_counter()
        p2_normals = self._estimate_normals(p2_pts, p2_tree)
        self._p2plane_ref = (p2_pts, p2_tree, p2_normals)
        print(f'  L3(p2plane): {len(p2_pts)}pts normals={((time.perf_counter()-t_normals)*1000):.0f}ms')

        self._T_base_tool_ref = self._get_tool_matrix()
        if self._T_base_tool_ref is None:
            print('❌ GetPose 无有效位姿'); return False
        self._T_base_camera_ref = self._T_base_tool_ref @ self.X

        self._step_count = 0  # reset for new template
        self._stable_count = 0
        t = self._T_base_tool_ref[:3, 3]
        print(f'✅ 模板OK  xyz=[{t[0]:.0f} {t[1]:.0f} {t[2]:.0f}]')
        return True

    def _get_tool_matrix(self) -> np.ndarray | None:
        """GetPose → 4x4 SE3. 跳变>200mm时拒绝, 用上次有效值."""
        tool = self.robot.get_tool()
        if tool is None:
            return None
        x, y, z = tool[:3]
        # Jump detection
        if self._last_tool is not None:
            d = np.linalg.norm(np.array([x,y,z]) - np.array(self._last_tool[:3]))
            if d > 200:
                print(f'  ⚠ GetPose跳变{d:.0f}mm, 用上次值')
                x, y, z = self._last_tool[:3]
                tool = self._last_tool
        self._last_tool = tool
        R = Rot.from_euler('xyz', [tool[3], tool[4], tool[5]], degrees=True).as_matrix()
        T = np.eye(4); T[:3,:3] = R; T[:3,3] = [x, y, z]
        return T

    def _capture_pc(self, vs: float = 0.005) -> np.ndarray | None:
        """等停稳后的新点云帧"""
        pc_node = getattr(self, '_pc_node', None)
        if pc_node is None:
            return None
        pc_node.wait_fresh()  # ← 确保是新帧
        pc = pc_node.latest_pc
        if pc is not None and len(pc) > 500:
            return voxel_down(pc, vs)
        return None

    # ── One iteration of ICP + compensation ──
    def step(self) -> dict:
        """
        一次闭环: 采点云 → ICP → 质量门控 → 补偿移动.
        返回: {'ok', 'T_icp', 'rmse', 'overlap', 'delta_tool_mm', 'converged'}
        """
        out = {'ok': False, 'converged': False}

        if self._ref_pyramid is None:
            out['error'] = '未录制模板 (按 r)'
            print(f'  ❌ {out["error"]}')
            return out

        # 1. 当前点云 + 初值
        pnow = self._capture_pc()
        if pnow is None:
            out['error'] = '点云失败'
            return out

        T_cur = self._get_tool_matrix()
        if T_cur is None:
            out['error'] = 'GetPose失败'
            return out

        T_base_cam_cur = T_cur @ self.X
        T_init_mm = np.linalg.inv(self._T_base_camera_ref) @ T_base_cam_cur

        # ICP内部m, Link层mm: T_init平移÷1000
        T_init_icp = T_init_mm.copy()
        T_init_icp[:3, 3] /= 1000.0

        print(f'  T_init |t|={np.linalg.norm(T_init_mm[:3,3]):.0f}mm  '
              f'T_cur=[{T_cur[0,3]:.0f} {T_cur[1,3]:.0f} {T_cur[2,3]:.0f}]  '
              f'T_ref=[{self._T_base_tool_ref[0,3]:.0f} {self._T_base_tool_ref[1,3]:.0f} {self._T_base_tool_ref[2,3]:.0f}]')
        pose_t_err, pose_r_err = self.pose_error(T_cur, self._T_base_tool_ref)
        out['pose_error_mm'] = round(pose_t_err, 1)
        out['pose_error_deg'] = round(pose_r_err, 2)

        # 2. Pyramid ICP (pre-built cKDTree, no rebuild)
        from scipy.spatial import cKDTree
        icp_t0 = time.perf_counter()
        T_acc = T_init_icp.copy()
        scales_info = []
        dmax_list = [0.100, 0.050, 0.025]

        fine_src_ds = None
        for li, level in enumerate(self._ref_pyramid):
            level_t0 = time.perf_counter()
            tgt_pts, tree, vs = level
            dmax = dmax_list[li] if li < len(dmax_list) else 0.025
            # Downsample current scan to match this level
            idx_src = np.floor(pnow / vs).astype(np.int64)
            _, u = np.unique(idx_src, axis=0, return_index=True)
            src_ds = pnow[u]
            if li == len(self._ref_pyramid) - 1:
                fine_src_ds = self._stride_sample(src_ds, self.point_to_plane_max_points)

            # Pre-apply accumulated transform
            src_tf = (T_acc[:3,:3] @ src_ds.T).T + T_acc[:3,3]

            # Use tree.query directly (pre-built, fast)
            dist, idx = tree.query(src_tf, distance_upper_bound=dmax, workers=-1)
            mask = dist < dmax
            if mask.sum() < 10:
                scales_info.append({'rmse': float('inf'), 'inliers': 0, 'overlap': 0,
                                    'scale_vs_mm': vs*1000, 'scale_dmax_mm': dmax*1000,
                                    'time_ms': (time.perf_counter() - level_t0) * 1000.0})
                continue

            s, d = src_tf[mask], tgt_pts[idx[mask]]
            cs, cd = s.mean(0), d.mean(0)
            H = (s-cs).T @ (d-cd)
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0: Vt[2]*=-1; R = Vt.T @ U.T
            t_step = cd - R @ cs
            err = np.sqrt(np.mean(np.sum(((R@s.T).T+t_step-d)**2, axis=1)))

            T_acc[:3,:3] = R @ T_acc[:3,:3]
            T_acc[:3,3] = R @ T_acc[:3,3] + t_step
            inl = int(mask.sum())
            scales_info.append({'rmse': float(err), 'inliers': inl,
                                'overlap': inl/len(src_ds),
                                'scale_vs_mm': vs*1000, 'scale_dmax_mm': dmax*1000,
                                'time_ms': (time.perf_counter() - level_t0) * 1000.0})

        p2p_t_norm_mm = np.linalg.norm(T_acc[:3, 3]) * 1000.0
        if (fine_src_ds is not None and self._p2plane_ref is not None and
                p2p_t_norm_mm <= self.point_to_plane_trigger_mm):
            p2l_t0 = time.perf_counter()
            fine_tgt_pts, fine_tree, fine_normals = self._p2plane_ref
            T_p2l, p2l_info = self._point_to_plane_refine(
                fine_src_ds, fine_tgt_pts, fine_tree, fine_normals, T_acc,
                dmax=self.point_to_plane_dmax,
                max_iter=self.point_to_plane_iters)
            if p2l_info['inliers'] >= 30 and np.isfinite(p2l_info['rmse']):
                T_acc = T_p2l
                p2l_info['scale_vs_mm'] = 5.0
                p2l_info['scale_dmax_mm'] = self.point_to_plane_dmax * 1000
                p2l_info['method'] = 'p2plane'
                p2l_info['time_ms'] = (time.perf_counter() - p2l_t0) * 1000.0
                scales_info.append(p2l_info)

        # Convert back to mm
        icp_time_ms = (time.perf_counter() - icp_t0) * 1000.0
        T_icp_mm = T_acc.copy()
        T_icp_mm[:3, 3] *= 1000.0

        out['T_icp'] = T_icp_mm
        out['rmse'] = scales_info[-1]['rmse'] * 1000 if scales_info else 999
        out['overlap'] = scales_info[-1]['overlap'] if scales_info else 0
        out['inliers'] = scales_info[-1]['inliers'] if scales_info else 0
        out['icp_time_ms'] = round(icp_time_ms, 1)

        for si, s in enumerate(scales_info):
            method = f'({s["method"]})' if s.get('method') else ''
            print(f'    L{si}{method}: {s["scale_vs_mm"]:.0f}mm/{s["scale_dmax_mm"]:.0f}mm '
                  f'RMSE={s["rmse"]*1000:.1f}mm inl={s["inliers"]} overl={s["overlap"]:.2f} '
                  f't={s.get("time_ms", 0):.1f}ms')

        if out['inliers'] < 10:
            out['error'] = f'ICP匹配点不足 ({out["inliers"]}) |T_init|={np.linalg.norm(T_init_mm[:3,3]):.0f}mm'
            print(f'  ❌ {out["error"]}')
            return out

        t_norm = np.linalg.norm(T_icp_mm[:3, 3])
        r_deg = np.degrees(np.linalg.norm(Rot.from_matrix(T_icp_mm[:3,:3]).as_rotvec()))
        out['icp_delta_mm'] = round(t_norm, 1)
        out['icp_rot_deg'] = round(r_deg, 2)
        print(f'  ICP: RMSE={out["rmse"]:.1f}mm  overl={out["overlap"]:.2f}  '
              f'|t|={t_norm:.1f}mm  |r|={r_deg:.2f}°  '
              f'time={icp_time_ms:.1f}ms')
        print(f'  Tool误差: |T_cur-T_ref|={pose_t_err:.1f}mm  |r|={pose_r_err:.2f}°')

        # 3. Quality gate
        info = {'rmse': scales_info[-1]['rmse'] if scales_info else 99,
                'overlap': out['overlap'], 'inliers': out['inliers']}
        if not self.quality_ok(info, T_acc):
            out['error'] = (f'质量门控拒绝: RMSE={out["rmse"]:.1f}mm '
                            f'overl={info["overlap"]:.2f} '
                            f'|t|={t_norm:.1f}mm |r|={r_deg:.1f}°')
            print(f'  ❌ {out["error"]}')
            self._stable_count = 0
            return out

        # 4. 收敛判断: 只使用ICP估计的剩余校正量。
        # Tool误差只用于实验观测, 不能参与闭环判定。
        icp_converged = self.converged(
            T_icp_mm,
            self.final_icp_trans_thresh_mm,
            self.final_icp_rot_thresh_deg)
        if icp_converged:
            self._stable_count += 1
            if self._stable_count < self.final_stable_frames:
                out['ok'] = True
                print(f'  ↳ ICP残差达标, 稳定确认 '
                      f'{self._stable_count}/{self.final_stable_frames}: '
                      f'{t_norm:.1f}mm/{r_deg:.2f}°')
                return out
            out['ok'] = True
            out['converged'] = True
            print(f'  ✅ 已收敛: ICP残差={t_norm:.1f}mm/{r_deg:.2f}°  '
                  f'Tool误差(仅监控)={pose_t_err:.1f}mm/{pose_r_err:.2f}°')
            return out
        self._stable_count = 0

        # 5. 100% 全量补偿，不做末端比例缩放或单步限幅
        self._step_count += 1
        mode = '全量'

        #    T_delta = X @ T_icp_mm @ X⁻¹
        T_delta = self.X @ T_icp_mm @ self.X_inv
        T_correction = np.linalg.inv(T_delta)

        rotvec_full = Rot.from_matrix(T_correction[:3, :3]).as_rotvec()
        T_target = T_cur @ T_correction
        dp = [T_target[i,3]-T_cur[i,3] for i in range(3)]
        out['delta_mm'] = [round(v,1) for v in dp]

        pre_move = self.robot.get_tool()
        print(f'  {mode}补偿 100% (step{self._step_count}, 不限幅): '
              f'Δp=[{dp[0]:.1f} {dp[1]:.1f} {dp[2]:.1f}]mm  优先MovJ(pose)')
        attempted_cartesian_movj = False
        if self._cartesian_movj_disabled:
            print('  ↳ 本次闭环已检测到MovJ(pose)触发控制器ERROR, 直接使用Jacobian兜底')
        else:
            attempted_cartesian_movj = True

        if attempted_cartesian_movj:
            movj_status = self.robot.movj_pose_status(
                T_target, label=f'{mode}补偿/MovJ')
            if movj_status == 'arrived':
                after = self.robot.get_tool()
                if after and pre_move:
                    dp_real = np.linalg.norm(np.array(after[:3])-np.array(pre_move[:3]))
                    Ra = Rot.from_euler('xyz',after[3:6],degrees=True).as_matrix()
                    Rb = Rot.from_euler('xyz',pre_move[3:6],degrees=True).as_matrix()
                    dr = np.degrees(np.linalg.norm(Rot.from_matrix(Ra@Rb.T).as_rotvec()))
                    self._total_dp += dp_real; self._total_dr += dr
                    print(f'  → MovJ {dp_real/10:.1f}cm  {dr:.1f}°')
                out['ok'] = True; return out
            if movj_status == 'queued':
                print('  ↳ MovJ已入队但未确认完成, 等下一轮ICP重新判断')
                out['ok'] = True; return out

        if attempted_cartesian_movj:
            print('  ↳ MovJ(pose)发送失败, 回退Jacobian关节补偿')
        mode_after_movj = self.robot.get_robot_mode()
        if mode_after_movj in (9, 11):
            mode_name = 'ERROR' if mode_after_movj == 9 else 'COLLISION'
            self._cartesian_movj_disabled = True
            print(f'  ↳ 控制器处于{mode_name}(mode={mode_after_movj}), '
                  f'本次闭环后续禁用MovJ(pose), 先恢复安全状态再执行Jacobian兜底')
            if not self.robot.recover():
                out['ok'] = False; out['error'] = f'{mode_name}后恢复失败'; return out

        j_now = self.robot.get_joints()
        if not j_now:
            self.robot.recover()
            out['ok'] = False; out['error'] = 'GetAngle失败'; return out

        jac_dp = np.asarray(dp, dtype=float)
        drot_base = T_cur[:3, :3] @ rotvec_full
        cart = np.hstack([jac_dp, drot_base])
        J = _compute_jacobian(j_now)
        try:
            dtheta = np.degrees(np.linalg.pinv(J, rcond=1e-3) @ cart)
        except Exception:
            self.robot.recover()
            out['ok'] = False; out['error'] = 'Jacobian奇异'; return out

        target_j = [j_now[i] + dtheta[i] for i in range(6)]
        print('  Jacobian补偿(100%, 不限幅): '
              f'Δp=[{jac_dp[0]:.1f} {jac_dp[1]:.1f} {jac_dp[2]:.1f}]mm '
              f'Δθ=[{dtheta[0]:+.2f} {dtheta[1]:+.2f} {dtheta[2]:+.2f} '
              f'{dtheta[3]:+.2f} {dtheta[4]:+.2f} {dtheta[5]:+.2f}]°')
        if self.robot.movj(target_j):
            after = self.robot.get_tool()
            if after and pre_move:
                dp_real = np.linalg.norm(np.array(after[:3])-np.array(pre_move[:3]))
                Ra = Rot.from_euler('xyz', after[3:6], degrees=True).as_matrix()
                Rb = Rot.from_euler('xyz', pre_move[3:6], degrees=True).as_matrix()
                dr = np.degrees(np.linalg.norm(Rot.from_matrix(Ra @ Rb.T).as_rotvec()))
                self._total_dp += dp_real; self._total_dr += dr
                print(f'  → JointMovJ {dp_real/10:.1f}cm  {dr:.1f}°')
            out['ok'] = True; return out

        self.robot.recover()
        out['ok'] = False; out['error'] = 'MovJ(pose)失败且JointMovJ兜底失败(已恢复)'
        print(f'  ❌ {out["error"]}')
        return out

    def align(self, max_iters: int = 15) -> bool:
        print(f'\n{"="*55}\n  闭环对齐 (最多{max_iters}次)\n{"="*55}')
        self._step_count = 0
        self._stable_count = 0
        self._total_dp = 0.0; self._total_dr = 0.0
        self._cartesian_movj_disabled = False
        i = 0
        while i < max_iters:
            i += 1
            print(f'\n  [{i}/{max_iters}]')
            result = self.step()
            if result.get('converged'):
                pose_msg = ''
                if 'pose_error_mm' in result:
                    pose_msg = (f'  T_cur-T_ref误差 {result["pose_error_mm"]:.1f}mm'
                                f'  {result.get("pose_error_deg", 0):.2f}°')
                print(f'\n✅ 闭环收敛 ({i}次)  共补偿 {self._total_dp/10:.1f}cm  '
                      f'{self._total_dr:.1f}°{pose_msg}')
                return True
            if result['ok']:
                continue  # 成功, 下一轮
            # 失败: 直接重试(从当前位姿重新ICP)
            print(f'  ↻ 本轮失败后重试 (从当前位姿)')
        print(f'\n⚠ 达最大迭代次数({max_iters})  共补偿 {self._total_dp/10:.1f}cm  {self._total_dr:.1f}°')
        return False


# ── Jacobian (MovJ(pose)失败时的小误差JointMovJ兜底) ──
def _compute_jacobian(jd, eps=0.0005):
    import math as _m

    def _rx(a):
        c, s = _m.cos(a), _m.sin(a)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    def _ry(a):
        c, s = _m.cos(a), _m.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    def _rz(a):
        c, s = _m.cos(a), _m.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def _rz4(a):
        c, s = _m.cos(a), _m.sin(a)
        return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def _urdf_T(xyz, rpy):
        x, y, z = xyz
        r, p, yw = rpy
        R = _rz(yw) @ _ry(p) @ _rx(r)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def _fk(T_acc, j_rad):
        T = T_acc.copy()
        T = T @ _urdf_T([0, 0, 0.147], [0, 0, 0]) @ _rz4(j_rad[0])
        T = T @ _urdf_T([0, 0, 0], [_m.pi / 2, _m.pi / 2, 0]) @ _rz4(j_rad[1])
        T = T @ _urdf_T([-0.427, -0.000349, 0], [0, 0, 0]) @ _rz4(j_rad[2])
        T = T @ _urdf_T([-0.357, 0.000349, 0.141], [0, 0, -_m.pi / 2]) @ _rz4(j_rad[3])
        T = T @ _urdf_T([0, -0.116, 0], [_m.pi / 2, 0, 0]) @ _rz4(j_rad[4])
        T = T @ _urdf_T([0, 0.105, 0], [-_m.pi / 2, 0, 0]) @ _rz4(j_rad[5])
        T[:3, 3] *= 1000
        return T

    jr = np.radians(jd)
    T0 = _fk(np.eye(4), jr)
    p0 = T0[:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        jp = jr.copy()
        jp[i] += eps
        T1 = _fk(np.eye(4), jp)
        J[0:3, i] = (T1[:3, 3] - p0) / eps
        dR = T1[:3, :3] @ T0[:3, :3].T
        J[3:6, i] = Rot.from_matrix(dR).as_rotvec() / eps
    return J
