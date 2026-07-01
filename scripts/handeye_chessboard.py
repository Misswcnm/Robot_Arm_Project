#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼标定 — 棋盘格法 (ToolVectorActual + 质量过滤)
==================================================
- 全部使用 ToolVectorActual 真实 TCP 位姿 (不依赖 FK)
- 严格棋盘格检测质量检查 (重投影误差/完整性/距离)
- 标定前数据质量报告

用法:
  ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
  ros2 launch realsense2_camera rs_launch.py
  python3 scripts/handeye_chessboard.py
"""

import json, time, os, math, sys
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import (EnableRobot, DisableRobot, ClearError,
                                MovJ, SpeedFactor, GetAngle, SetCollisionLevel, RobotMode)


# ═══════════════════════════════════════════════
# 参数
# ═══════════════════════════════════════════════
CHESS_COLS, CHESS_ROWS = 8, 11     # 内角点 (列×行)
SQUARE_MM  = 20.0                  # 格子边长 mm
SPEED      = 15

# 质量阈值
MAX_REPROJ_ERR = 0.5    # 最大重投影误差 (像素)
MIN_BOARD_DIST = 150    # 棋盘格最近距离 (mm)
MAX_BOARD_DIST = 2000   # 棋盘格最远距离 (mm)

TOPIC_IMAGE       = '/camera/camera/color/image_raw'
TOPIC_INFO        = '/camera/camera/color/camera_info'
TOPIC_TOOL        = '/dobot_msgs_v4/msg/ToolVectorActual'

OUTPUT_FILE = os.path.expanduser('~/Robot_Arm_Project/scripts/handeye_chessboard_result.json')


# ═══════════════════════════════════════════════
# SE(3)
# ═══════════════════════════════════════════════
def pose_to_matrix(xyzrpy):
    """CR5 ToolVectorActual: rx,ry,rz = XYZ intrinsic Euler"""
    x,y,z,rx,ry,rz=xyzrpy
    R=Rot.from_euler('xyz',[rx,ry,rz],degrees=True).as_matrix()
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=[x,y,z]; return T

def inv_se3(T):
    R,t=T[:3,:3],T[:3,3]; Ti=np.eye(4); Ti[:3,:3]=R.T; Ti[:3,3]=-R.T@t
    return Ti

def matrix_to_pose(T):
    t=T[:3,3]; r=Rot.from_matrix(T[:3,:3]).as_euler('zyx',degrees=True)
    return [t[0],t[1],t[2],r[2],r[1],r[0]]


# ═══════════════════════════════════════════════
# AX=XB
# ═══════════════════════════════════════════════
def solve_ax_xb(A_list, B_list):
    # Rotation
    M=np.zeros((3,3))
    for A,B in zip(A_list,B_list):
        ra,rb=Rot.from_matrix(A[:3,:3]),Rot.from_matrix(B[:3,:3])
        if ra.magnitude()<1e-12 or rb.magnitude()<1e-12: continue
        M+=np.outer(ra.as_rotvec(),rb.as_rotvec())
    U,S,Vt=np.linalg.svd(M)
    R_X=U@np.diag([1.,1.,np.linalg.det(U@Vt)])@Vt
    # Translation
    K_rows,g_rows=[],[]
    for A,B in zip(A_list,B_list):
        K_rows.append(A[:3,:3]-np.eye(3)); g_rows.append(R_X@B[:3,3]-A[:3,3])
    K=np.vstack(K_rows); g=np.hstack(g_rows)
    t_X=np.linalg.lstsq(K,g,rcond=None)[0]
    X=np.eye(4); X[:3,:3]=R_X; X[:3,3]=t_X
    return X


# ═══════════════════════════════════════════════
# 棋盘格质量检查
# ═══════════════════════════════════════════════
def check_board_quality(objp, corners, K, D):
    """返回 (ok, reproj_err, board_dist, message)"""
    n_corners = CHESS_COLS * CHESS_ROWS
    if len(corners) != n_corners:
        return False, 999, 0, f'角点不全 ({len(corners)}/{n_corners})'

    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return False, 999, 0, 'solvePnP失败'

    # 重投影误差
    proj,_ = cv2.projectPoints(objp, rvec, tvec, K, D)
    errors = np.linalg.norm(corners.reshape(-1,2) - proj.reshape(-1,2), axis=1)
    rmse = np.sqrt(np.mean(errors**2))
    max_err = errors.max()

    if rmse > MAX_REPROJ_ERR:
        return False, rmse, 0, f'重投影误差过大 RMS={rmse:.3f}px max={max_err:.3f}px'

    # 距离
    dist = np.linalg.norm(tvec)
    if dist < MIN_BOARD_DIST or dist > MAX_BOARD_DIST:
        return False, rmse, dist, f'距离异常 {dist:.0f}mm (范围{MIN_BOARD_DIST}-{MAX_BOARD_DIST})'

    return True, rmse, dist, 'OK'


# ═══════════════════════════════════════════════
# 位姿运动质量检查
# ═══════════════════════════════════════════════
def check_motion_quality(T_rob_i, T_rob_j, T_cam_i, T_cam_j):
    """检查一对位姿的运动是否足够用于标定"""
    # 机器人运动量
    dt_rob = np.linalg.norm(T_rob_j[:3,3] - T_rob_i[:3,3])
    dR_rob = Rot.from_matrix(T_rob_j[:3,:3] @ T_rob_i[:3,:3].T).magnitude()

    # 相机运动量
    dt_cam = np.linalg.norm(T_cam_j[:3,3] - T_cam_i[:3,3])
    dR_cam = Rot.from_matrix(T_cam_j[:3,:3] @ T_cam_i[:3,:3].T).magnitude()

    issues = []
    if dt_rob < 3 and np.degrees(dR_rob) < 0.5:
        issues.append(f'机器人几乎未动 (Δ={dt_rob:.1f}mm, {np.degrees(dR_rob):.1f}°)')
    if dt_cam < 3 and np.degrees(dR_cam) < 0.5:
        issues.append(f'相机几乎未动 (Δ={dt_cam:.1f}mm, {np.degrees(dR_cam):.1f}°)')

    return len(issues)==0, issues


# ═══════════════════════════════════════════════
# ROS2 Node
# ═══════════════════════════════════════════════
class HandEyeCalib(Node):
    def __init__(self):
        super().__init__('handeye_chessboard')
        self.ClearError   = self.create_client(ClearError,   '/dobot_bringup_ros2/srv/ClearError')
        self.DisableRobot = self.create_client(DisableRobot, '/dobot_bringup_ros2/srv/DisableRobot')
        self.EnableRobot  = self.create_client(EnableRobot,  '/dobot_bringup_ros2/srv/EnableRobot')
        self.MovJ         = self.create_client(MovJ,         '/dobot_bringup_ros2/srv/MovJ')
        self.SpeedFactor  = self.create_client(SpeedFactor,  '/dobot_bringup_ros2/srv/SpeedFactor')
        self.GetAngle     = self.create_client(GetAngle,     '/dobot_bringup_ros2/srv/GetAngle')
        self.SetCollision = self.create_client(SetCollisionLevel,'/dobot_bringup_ros2/srv/SetCollisionLevel')
        self.RobotMode    = self.create_client(RobotMode,'/dobot_bringup_ros2/srv/RobotMode')

        for n,c in [('EnableRobot',self.EnableRobot),('MovJ',self.MovJ)]:
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'等待 {n}...')
        self.get_logger().info('✅ Robot ready')

        self._img = None; self._K = None; self._D = None
        self._sub_img = self.create_subscription(Image, TOPIC_IMAGE, self._cb_img, 10)
        self._sub_info = self.create_subscription(CameraInfo, TOPIC_INFO, self._cb_info, 10)

        self._tool = None  # [x,y,z,rx,ry,rz] from ToolVectorActual
        self._sub_tool = self.create_subscription(
            ToolVectorActual, TOPIC_TOOL,
            lambda m: setattr(self,'_tool',[m.x,m.y,m.z,m.rx,m.ry,m.rz]), 10)

    def _cb_img(self, msg):
        try:
            h,w=msg.height,msg.width; data=np.frombuffer(msg.data,dtype=np.uint8)
            if msg.encoding=='rgb8':
                img=data.reshape(h,msg.step)[:,:w*3].reshape(h,w,3)
                img=cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
            elif msg.encoding=='bgr8':
                img=data.reshape(h,msg.step)[:,:w*3].reshape(h,w,3)
            else: return
            self._img=img
        except: pass

    def _cb_info(self, msg):
        if self._K is None: self._K=np.array(msg.k).reshape(3,3).copy(); self._D=np.array(msg.d).copy()

    def _call(self, c, req, timeout=10.0):
        fut=c.call_async(req); t0=time.time()
        while not fut.done() and time.time()-t0<timeout: rclpy.spin_once(self,timeout_sec=0.05)
        if not fut.done(): fut.cancel(); return False,'timeout'
        try: return True,fut.result()
        except Exception as e: return False,str(e)

    def init_robot(self):
        self._call(self.ClearError,ClearError.Request())
        self._call(self.DisableRobot,DisableRobot.Request())
        self._call(self.EnableRobot,EnableRobot.Request(),timeout=10.0)
        for _ in range(15): rclpy.spin_once(self,timeout_sec=0.2)
        s=SpeedFactor.Request(); s.ratio=SPEED; self._call(self.SpeedFactor,s)
        c=SetCollisionLevel.Request(); c.level=5; self._call(self.SetCollision,c)
        self.get_logger().info(f'✅ Init done  speed={SPEED}%  collision=Lv.5')

    def get_tool_pose(self):
        """获取 ToolVectorActual 真实 TCP 位姿 [x,y,z,rx,ry,rz] mm,deg"""
        for _ in range(30):
            rclpy.spin_once(self,timeout_sec=0.1)
            if self._tool is not None:
                x,y,z = self._tool[:3]
                if abs(x)>0.5 or abs(y)>0.5 or abs(z)>0.5:
                    return list(self._tool)
        return None

    def movj(self, joints):
        req=MovJ.Request(); req.mode=True
        req.a,req.b,req.c=float(joints[0]),float(joints[1]),float(joints[2])
        req.d,req.e,req.f=float(joints[3]),float(joints[4]),float(joints[5])
        req.param_value=['user=0','tool=0']
        ok,r=self._call(self.MovJ,req,timeout=15.0)
        return ok and r.res==0

    def wait_robot_stop(self, max_wait=15.0):
        """RobotMode: RUNNING(7)→ENABLE(5), 处理快速运动漏检"""
        t0 = time.time()
        seen_running = False
        idle_since = None
        # 先等0.3s确保JointMovJ已开始执行
        time.sleep(0.3)
        while time.time()-t0 < max_wait:
            ok, r = self._call(self.RobotMode, RobotMode.Request(), timeout=2.0)
            if ok:
                try:
                    mode = int(r.robot_return.strip('{}'))
                    if mode == 7:
                        seen_running = True
                        idle_since = None
                    elif mode == 5:
                        if seen_running:
                            time.sleep(0.5); return True  # 7→5 确认停止
                        if idle_since is None:
                            idle_since = time.time()
                        elif time.time()-idle_since > 4.0:
                            return True  # 4秒持续5=可能运动太快已结束
                    elif mode == 9:
                        print(' ⚠ ERROR!'); return False
                except: pass
            time.sleep(0.15)
        return False

    def capture_and_detect(self):
        """拍照 + 棋盘格检测 + 质量检查 → (T_board_in_cam, quality_info) 或 None"""
        self._img = None
        for _ in range(20): rclpy.spin_once(self,timeout_sec=0.1)

        if self._img is None or self._K is None: return None

        gray = cv2.cvtColor(self._img, cv2.COLOR_BGR2GRAY) if len(self._img.shape)==3 else self._img
        ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS, CHESS_ROWS),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)
        if not ret:
            # 不用 FAST_CHECK 再试一次
            ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS, CHESS_ROWS),
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ret: return None

        criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)

        objp = np.zeros((CHESS_COLS*CHESS_ROWS,3), np.float32)
        objp[:,:2] = np.mgrid[0:CHESS_COLS, 0:CHESS_ROWS].T.reshape(-1,2) * SQUARE_MM

        ok_q, rmse, dist, msg = check_board_quality(objp, corners, self._K, self._D)
        if not ok_q:
            return {'error': msg, 'reproj_rmse': rmse, 'dist': dist}

        ok, rvec, tvec = cv2.solvePnP(objp, corners, self._K, self._D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok: return {'error': 'solvePnP failed'}

        R,_ = cv2.Rodrigues(rvec)
        T = np.eye(4); T[:3,:3]=R; T[:3,3]=tvec.flatten()
        return {'T': T, 'reproj_rmse': rmse, 'dist': dist, 'corners': corners}


# ═══════════════════════════════════════════════
# Joint offsets for calibration poses
# ═══════════════════════════════════════════════
def generate_poses(j0, n=16):
    """小幅多角度: 前几个位姿移动不大(保证棋盘格可见), 逐步增大旋转"""
    D = [
        [  0,  0,  0,   0,  0,  0],   #  0: 基准
        [  6,  0,  0,   0,  0,  0],   #  1: J1小
        [ -6,  0,  0,   0,  0,  0],   #  2
        [  0,  4,  0,   0,  0,  0],   #  3: J2小
        [  0, -4,  0,   0,  0,  0],   #  4
        [ 12,  0,  0,   0,  0,  0],   #  5: J1中
        [-12,  0,  0,   0,  0,  0],   #  6
        [  0,  0,  0,   6,  0,  0],   #  7: J4小
        [  0,  0,  0,  -6,  0,  0],   #  8
        [  0,  0,  0,   0,  5,  0],   #  9: J5小
        [  0,  0,  0,   0, -5,  0],   # 10
        [  0,  0,  0,   0,  0,  5],   # 11: J6小
        [  0,  0,  0,   0,  0, -5],   # 12
        [ 10,  0,  0,   0,  5,  0],   # 13: 混合1
        [-10,  0,  0,   0, -5,  0],   # 14: 混合2
        [  0,  6,  0,   6,  0,  4],   # 15: 混合3
    ]
    return [[j0[i]+D[k][i] for i in range(6)] for k in range(min(n,len(D)))]


# ═══════════════════════════════════════════════
def main():
    rclpy.init()
    node = HandEyeCalib()

    print('\n'+'='*60)
    print('  手眼标定 — 棋盘格法 (ToolVectorActual + 质量过滤)')
    print(f'  棋盘格: {CHESS_COLS}×{CHESS_ROWS}  格边={SQUARE_MM}mm')
    print('='*60)

    # Wait camera
    print('等待相机内参...')
    for _ in range(50): rclpy.spin_once(node,timeout_sec=0.1)
    if node._K is None: print('❌ 无CameraInfo'); node.destroy_node(); rclpy.shutdown(); return
    print(f'📷 K: fx={node._K[0,0]:.0f} fy={node._K[1,1]:.0f}')

    # Init robot
    node.init_robot()

    # Check ToolVectorActual
    tool = node.get_tool_pose()
    if tool is None:
        print('❌ ToolVectorActual 无数据！确认驱动已启动且连接机械臂WiFi')
        node.destroy_node(); rclpy.shutdown(); return
    print(f'📍 ToolVectorActual: xyz=[{tool[0]:.1f} {tool[1]:.1f} {tool[2]:.1f}] '
          f'rpy=[{tool[3]:.2f} {tool[4]:.2f} {tool[5]:.2f}]')

    # Check board visible
    res = node.capture_and_detect()
    if res and 'T' in res:
        print(f'✅ 棋盘格可见  距离={res["dist"]:.0f}mm  重投影RMS={res["reproj_rmse"]:.3f}px')
    elif res:
        print(f'⚠️  棋盘格质量问题: {res["error"]}')
    else:
        print('⚠️  未检测到棋盘格')

    # Generate poses
    j0 = [0]*6
    # Read current joint angles for reference
    req=GetAngle.Request()
    ok,r=node._call(node.GetAngle,req)
    if ok:
        try: j0=[float(v) for v in r.robot_return.strip('{}').split(',')]
        except: pass
    targets = generate_poses(j0, n=12)
    print(f'\n{len(targets)} 位姿  按 Enter 开始...')
    input()

    # ── Collect ──
    results = []  # [{T_robot, T_cam, tool_pose, quality}]
    img_dir = os.path.expanduser('~/Robot_Arm_Project/scripts/calib_middle_image')
    os.makedirs(img_dir, exist_ok=True)

    for idx, tj in enumerate(targets):
        delta = [tj[i]-j0[i] for i in range(6)]
        tag = f'{idx+1:02d}'
        print(f'\n▶ [{tag}/{len(targets)}] ΔJ=[{delta[0]:+.0f} {delta[1]:+.0f} '
              f'{delta[2]:+.0f} {delta[3]:+.0f} {delta[4]:+.0f} {delta[5]:+.0f}]')

        if idx > 0:
            if not node.movj(tj): print('  ⚠ 移动失败'); continue
            # 等待机械臂完全停止
            print('  ⏳ 等待停止...', end='', flush=True)
            stopped = node.wait_robot_stop(max_wait=10.0)
            if stopped:
                print(' 已停止')
            else:
                print(' ⚠ 超时(仍继续)')

        # ── ToolVectorActual (必须) ──
        tool = node.get_tool_pose()
        if tool is None:
            print('  ❌ ToolVectorActual 无数据!')
            cv2.imwrite(f'{img_dir}/{tag}_reject_notool.jpg', node._img)
            continue
        T_robot = pose_to_matrix(tool)

        # ── 棋盘格检测 ──
        res = node.capture_and_detect()
        if res is None:
            print('  ❌ 未检测到棋盘格')
            if node._img is not None:
                cv2.imwrite(f'{img_dir}/{tag}_reject_noboard.jpg', node._img)
            continue
        if 'T' not in res:
            print(f'  ❌ 质量不合格: {res["error"]}')
            if node._img is not None:
                vis = node._img.copy()
                cv2.drawChessboardCorners(vis, (CHESS_COLS,CHESS_ROWS), res.get('corners'), True)
                cv2.imwrite(f'{img_dir}/{tag}_reject_quality.jpg', vis)
            continue

        quality = {'reproj_rmse': res['reproj_rmse'], 'dist': res['dist']}
        results.append({'T_robot': T_robot, 'T_cam': res['T'],
                        'tool_pose': tool, 'quality': quality,
                        'joint_idx': idx})

        # Save accepted image with detected corners
        vis = node._img.copy()
        cv2.drawChessboardCorners(vis, (CHESS_COLS,CHESS_ROWS), res['corners'], True)
        cv2.imwrite(f'{img_dir}/{tag}_ok.jpg', vis)

        print(f'  ✅ [{len(results)}]  dist={res["dist"]:.0f}mm  '
              f'reproj={res["reproj_rmse"]:.3f}px  '
              f'xyz=[{tool[0]:.0f} {tool[1]:.0f} {tool[2]:.0f}]')

    # Return home
    node.movj(targets[0])

    if len(results) < 5:
        print(f'\n❌ 有效帧不足 ({len(results)})，需≥5'); node.destroy_node(); rclpy.shutdown(); return

    # ── Quality report ──
    print(f'\n{"="*60}')
    print(f'📊 数据质量报告 ({len(results)} 帧)')
    print('='*60)
    reprojs = [r['quality']['reproj_rmse'] for r in results]
    dists = [r['quality']['dist'] for r in results]
    print(f'  重投影误差: min={min(reprojs):.3f} max={max(reprojs):.3f} avg={np.mean(reprojs):.3f}px')
    print(f'  棋盘距离:   min={min(dists):.0f} max={max(dists):.0f} avg={np.mean(dists):.0f}mm')

    # ── Build A,B pairs with motion quality check ──
    print(f'\n{"="*60}')
    print(f'  构造运动对...')
    print('='*60)

    A_list, B_list = [], []
    skipped_motion, skipped_total = 0, 0
    for i in range(len(results)):
        for j in range(i+1, len(results)):
            skipped_total += 1
            ok_m, issues = check_motion_quality(
                results[i]['T_robot'], results[j]['T_robot'],
                results[i]['T_cam'], results[j]['T_cam'])
            if not ok_m:
                skipped_motion += 1
                continue

            # 推导: G_j⁻¹ @ G_i @ X = X @ C_j @ C_i⁻¹
            # → A = G_j⁻¹ @ G_i (机器人运动)  B = C_j @ C_i⁻¹ (相机运动)
            A = inv_se3(results[j]['T_robot']) @ results[i]['T_robot']   # G_j⁻¹ @ G_i
            B = results[j]['T_cam'] @ inv_se3(results[i]['T_cam'])       # C_j @ C_i⁻¹
            A_list.append(A); B_list.append(B)

    print(f'  有效: {len(A_list)}  跳过(运动不足): {skipped_motion}  总计: {len(A_list)+skipped_motion}')
    if len(A_list) < 8:
        print(f'  ❌ 有效对不足 ({len(A_list)}), 需≥8'); node.destroy_node(); rclpy.shutdown(); return

    # ── Solve ──
    print(f'\n{"="*60}')
    print(f'  求解 AX=XB...')
    print('='*60)
    X = solve_ax_xb(A_list, B_list)
    X_pose = matrix_to_pose(X)

    print(f'\n 【手眼矩阵 X = camera_in_tool】')
    print(f'  平移(mm):  [{X_pose[0]:.3f} {X_pose[1]:.3f} {X_pose[2]:.3f}]')
    print(f'  旋转(deg): [{X_pose[3]:.4f} {X_pose[4]:.4f} {X_pose[5]:.4f}]')

    # ── Self-check: 标定数据内部一致性 ──
    print(f'\n{"="*60}')
    print(f'  Self-check: T_board_in_world 一致性')
    print('='*60)
    bw_positions = []
    for r in results:
        T_bw = r['T_robot'] @ X @ r['T_cam']
        bw_positions.append(T_bw[:3,3])
    bw = np.array(bw_positions)
    bw_mean = bw.mean(axis=0)
    bw_errs = np.linalg.norm(bw - bw_mean, axis=1)
    print(f'  均值 xyz=[{bw_mean[0]:.0f} {bw_mean[1]:.0f} {bw_mean[2]:.0f}] mm')
    print(f'  σ=[{bw[:,0].std():.1f} {bw[:,1].std():.1f} {bw[:,2].std():.1f}] mm')
    print(f'  RMS={np.sqrt(np.mean(bw_errs**2)):.1f} mm')
    if np.sqrt(np.mean(bw_errs**2)) < 10:
        print(f'  ✅ 内部一致性良好')
    else:
        print(f'  ⚠️  内部一致性差 → 可能: 移动未停止 / 棋盘格移动 / ToolVectorActual不准')

    # Save (含原始数据, 供后续诊断)
    raw_frames = []
    for r in results:
        T_cam_list = [[float(r['T_cam'][i,j]) for j in range(4)] for i in range(4)]
        raw_frames.append({
            'tool_xyzrpy': [float(v) for v in r['tool_pose']],
            'T_cam_4x4': T_cam_list,
            'quality': {'reproj_rmse': float(r['quality']['reproj_rmse']),
                        'dist': float(r['quality']['dist'])},
        })
    out = {
        'method': 'chessboard+ToolVectorActual+quality_filter',
        'chessboard': {'size':[CHESS_COLS,CHESS_ROWS], 'square_mm':SQUARE_MM},
        'num_frames': len(results), 'num_pairs': len(A_list),
        'quality': {'reproj_rmse_stats': [float(min(reprojs)),float(max(reprojs)),float(np.mean(reprojs))],
                    'dist_stats': [float(min(dists)),float(max(dists)),float(np.mean(dists))]},
        'X_camera_in_tool': {
            'xyz_mm': [round(v,4) for v in X_pose[:3]],
            'rpy_deg': [round(v,6) for v in X_pose[3:6]],
            'matrix': X.tolist(),
        },
        'raw_frames': raw_frames,
    }
    with open(OUTPUT_FILE,'w') as f: json.dump(out,f,indent=2,ensure_ascii=False)
    print(f'\n📁 {OUTPUT_FILE}')

    node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
