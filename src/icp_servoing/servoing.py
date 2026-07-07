"""
ICP Visual Servoing — 闭环补偿核心逻辑
======================================
公式链:
  T_cam = pyramid_icp(src, ref, T_init)  (current→template, camera frame)
  T_delta = X @ T_cam @ X⁻¹              (camera→tool frame)
  T_correction = inv(T_delta)             (取消误差的方向)
  T_target = T_cur @ T_correction_partial (70%部分补偿)
  → Jacobian IK → JointMovJ              (CR5 MovL不可靠)
"""
import time, rclpy
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from icp_servoing.robot import CR5Robot
from icp_servoing.pointcloud import voxel_down


class VisualServo:
    def __init__(self, robot: CR5Robot, X: np.ndarray,
                 speed: int = 15, compensation_ratio: float = 0.7):
        self.robot = robot
        self.X = X
        self.X_inv = np.linalg.inv(X)
        self.comp_ratio = compensation_ratio

        # Template pyramid (pre-built for speed)
        self._ref_pyramid = None
        self._T_base_tool_ref = None
        self._T_base_camera_ref = None
        self._last_tool = None
        self._step_count = 0
        self._total_dp = 0.0; self._total_dr = 0.0
        self.final_icp_trans_thresh_mm = 5.0
        self.final_icp_rot_thresh_deg = 0.5

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
                  trans_thresh: float = 5.0,
                  rot_thresh_deg: float = 0.5) -> bool:
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

        self._T_base_tool_ref = self._get_tool_matrix()
        if self._T_base_tool_ref is None:
            print('❌ ToolVectorActual 无数据'); return False
        self._T_base_camera_ref = self._T_base_tool_ref @ self.X

        self._step_count = 0  # reset for new template
        t = self._T_base_tool_ref[:3, 3]
        print(f'✅ 模板OK  xyz=[{t[0]:.0f} {t[1]:.0f} {t[2]:.0f}]')
        return True

    def _get_tool_matrix(self) -> np.ndarray | None:
        """ToolVectorActual → 4x4 SE3. 跳变>200mm时拒绝, 用上次有效值."""
        tool = self.robot.get_tool()
        if tool is None:
            return None
        x, y, z = tool[:3]
        # Jump detection
        if self._last_tool is not None:
            d = np.linalg.norm(np.array([x,y,z]) - np.array(self._last_tool[:3]))
            if d > 200:
                print(f'  ⚠ ToolVectorActual跳变{d:.0f}mm, 用上次值')
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
            out['error'] = 'ToolVectorActual失败'
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
        T_acc = T_init_icp.copy()
        scales_info = []
        dmax_list = [0.100, 0.050, 0.025]

        for li, (tgt_pts, tree, vs) in enumerate(self._ref_pyramid):
            dmax = dmax_list[li] if li < len(dmax_list) else 0.025
            # Downsample current scan to match this level
            idx_src = np.floor(pnow / vs).astype(np.int64)
            _, u = np.unique(idx_src, axis=0, return_index=True)
            src_ds = pnow[u]

            # Pre-apply accumulated transform
            src_tf = (T_acc[:3,:3] @ src_ds.T).T + T_acc[:3,3]

            # Use tree.query directly (pre-built, fast)
            dist, idx = tree.query(src_tf, distance_upper_bound=dmax, workers=-1)
            mask = dist < dmax
            if mask.sum() < 10:
                scales_info.append({'rmse': float('inf'), 'inliers': 0, 'overlap': 0})
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
                                'scale_vs_mm': vs*1000, 'scale_dmax_mm': dmax*1000})

        # Convert back to mm
        T_icp_mm = T_acc.copy()
        T_icp_mm[:3, 3] *= 1000.0

        out['T_icp'] = T_icp_mm
        out['rmse'] = scales_info[-1]['rmse'] * 1000 if scales_info else 999
        out['overlap'] = scales_info[-1]['overlap'] if scales_info else 0
        out['inliers'] = scales_info[-1]['inliers'] if scales_info else 0

        for si, s in enumerate(scales_info):
            print(f'    L{si}: {s["scale_vs_mm"]:.0f}mm/{s["scale_dmax_mm"]:.0f}mm '
                  f'RMSE={s["rmse"]*1000:.1f}mm inl={s["inliers"]} overl={s["overlap"]:.2f}')

        if out['inliers'] < 10:
            out['error'] = f'ICP匹配点不足 ({out["inliers"]}) |T_init|={np.linalg.norm(T_init_mm[:3,3]):.0f}mm'
            print(f'  ❌ {out["error"]}')
            return out

        t_norm = np.linalg.norm(T_icp_mm[:3, 3])
        r_deg = np.degrees(np.linalg.norm(Rot.from_matrix(T_icp_mm[:3,:3]).as_rotvec()))
        out['icp_delta_mm'] = round(t_norm, 1)
        out['icp_rot_deg'] = round(r_deg, 2)
        print(f'  ICP: RMSE={out["rmse"]:.1f}mm  overl={out["overlap"]:.2f}  '
              f'|t|={t_norm:.1f}mm  |r|={r_deg:.2f}°')
        print(f'  Tool误差: |T_cur-T_ref|={pose_t_err:.1f}mm  |r|={pose_r_err:.2f}°')

        # 3. Quality gate
        info = {'rmse': scales_info[-1]['rmse'] if scales_info else 99,
                'overlap': out['overlap'], 'inliers': out['inliers']}
        if not self.quality_ok(info, T_acc):
            out['error'] = (f'质量门控拒绝: RMSE={out["rmse"]:.1f}mm '
                            f'overl={info["overlap"]:.2f} '
                            f'|t|={t_norm:.1f}mm |r|={r_deg:.1f}°')
            print(f'  ❌ {out["error"]}')
            return out

        # 4. 收敛判断: 只使用ICP估计的剩余校正量。
        # Tool误差只用于实验观测, 不能参与闭环判定。
        icp_converged = self.converged(
            T_icp_mm,
            self.final_icp_trans_thresh_mm,
            self.final_icp_rot_thresh_deg)
        if icp_converged:
            out['ok'] = True
            out['converged'] = True
            print(f'  ✅ 已收敛: ICP残差={t_norm:.1f}mm/{r_deg:.2f}°  '
                  f'Tool误差(仅监控)={pose_t_err:.1f}mm/{pose_r_err:.2f}°')
            return out

        # 5. 全量补偿 (单步限幅30mm兜底)
        self._step_count += 1
        ratio = 1.0

        #    T_delta = X @ T_icp_mm @ X⁻¹
        T_delta = self.X @ T_icp_mm @ self.X_inv
        T_correction = np.linalg.inv(T_delta)

        # 提取完整修正量 (平移+旋转)
        t_full = T_correction[:3, 3]
        R_full = T_correction[:3, :3]

        # 单步限幅: |t| ≤ 30mm
        t_norm_corr = np.linalg.norm(t_full)
        if t_norm_corr > 30:
            t_full *= 30.0 / t_norm_corr

        t_partial = t_full * ratio
        R_partial = Rot.from_matrix(R_full)
        rotvec_partial = R_partial.as_rotvec() * ratio
        R_partial = Rot.from_rotvec(rotvec_partial).as_matrix()

        # JointMovJ: 完整6DOF (CR5 MovL已移除)
        T_corr_p = np.eye(4)
        T_corr_p[:3,:3] = R_partial
        T_corr_p[:3,3] = t_partial
        T_target = T_cur @ T_corr_p
        dp = [T_target[i,3]-T_cur[i,3] for i in range(3)]
        out['delta_mm'] = [round(v,1) for v in dp]

        j_now = self.robot.get_joints()
        if not j_now:
            self.robot.recover()
            out['ok'] = False; out['error'] = 'GetAngle失败'; return out
        drot_base = T_cur[:3,:3] @ rotvec_partial
        cart = np.hstack([dp, drot_base])
        J = _compute_jacobian(j_now)
        try: dtheta = np.degrees(np.linalg.pinv(J,rcond=1e-3) @ cart)
        except:
            self.robot.recover()
            out['ok'] = False; out['error'] = 'Jacobian奇异'; return out
        target_j = [j_now[i]+dtheta[i] for i in range(6)]
        pre_move = self.robot.get_tool()
        print(f'  补偿 {ratio*100:.0f}% (step{self._step_count}): '
              f'Δp=[{dp[0]:.1f} {dp[1]:.1f} {dp[2]:.1f}]mm '
              f'Δθ=[{dtheta[0]:+.2f} {dtheta[1]:+.2f} {dtheta[2]:+.2f} '
              f'{dtheta[3]:+.2f} {dtheta[4]:+.2f} {dtheta[5]:+.2f}]°')
        if self.robot.movj(target_j):
            # 实际移动量 (从ToolVectorActual)
            after = self.robot.get_tool()
            if after and pre_move:
                dp = np.linalg.norm(np.array(after[:3])-np.array(pre_move[:3]))
                Ra = Rot.from_euler('xyz',after[3:6],degrees=True).as_matrix()
                Rb = Rot.from_euler('xyz',pre_move[3:6],degrees=True).as_matrix()
                dr = np.degrees(np.linalg.norm(Rot.from_matrix(Ra@Rb.T).as_rotvec()))
                self._total_dp += dp; self._total_dr += dr
                print(f'  → {dp/10:.1f}cm  {dr:.1f}°')
            out['ok'] = True; return out

        self.robot.recover()
        out['ok'] = False; out['error'] = 'JointMovJ失败(已恢复)'
        return out

    def align(self, max_iters: int = 15) -> bool:
        print(f'\n{"="*55}\n  闭环对齐 (最多{max_iters}次)\n{"="*55}')
        self._step_count = 0
        self._total_dp = 0.0; self._total_dr = 0.0
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
            # 失败: 已recover, 直接重试(从当前位姿重新ICP)
            print(f'  ↻ 恢复后重试 (从当前位姿)')
        print(f'\n⚠ 达最大迭代次数({max_iters})  共补偿 {self._total_dp/10:.1f}cm  {self._total_dr:.1f}°')
        return False


# ── Jacobian (小误差JointMovJ用) ──
def _compute_jacobian(jd, eps=0.0005):
    import math as _m
    def _rx(a): c,s=_m.cos(a),_m.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    def _ry(a): c,s=_m.cos(a),_m.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    def _rz(a): c,s=_m.cos(a),_m.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
    def _rz4(a): c,s=_m.cos(a),_m.sin(a); return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])
    def _urdf_T(xyz,rpy):
        x,y,z=xyz; r,p,yw=rpy; R=_rz(yw)@_ry(p)@_rx(r)
        T=np.eye(4); T[:3,:3]=R; T[:3,3]=[x,y,z]; return T
    def _fk(T_acc,j_rad):
        T=T_acc.copy()
        T=T@_urdf_T([0,0,0.147],[0,0,0])@_rz4(j_rad[0])
        T=T@_urdf_T([0,0,0],[_m.pi/2,_m.pi/2,0])@_rz4(j_rad[1])
        T=T@_urdf_T([-0.427,-0.000349,0],[0,0,0])@_rz4(j_rad[2])
        T=T@_urdf_T([-0.357,0.000349,0.141],[0,0,-_m.pi/2])@_rz4(j_rad[3])
        T=T@_urdf_T([0,-0.116,0],[_m.pi/2,0,0])@_rz4(j_rad[4])
        T=T@_urdf_T([0,0.105,0],[-_m.pi/2,0,0])@_rz4(j_rad[5])
        T[:3,3]*=1000; return T
    jr=np.radians(jd); T0=_fk(np.eye(4),jr); p0=T0[:3,3]; J=np.zeros((6,6))
    for i in range(6):
        jp=jr.copy(); jp[i]+=eps; T1=_fk(np.eye(4),jp)
        J[0:3,i]=(T1[:3,3]-p0)/eps
        dR=T1[:3,:3]@T0[:3,:3].T; J[3:6,i]=Rot.from_matrix(dR).as_rotvec()/eps
    return J
