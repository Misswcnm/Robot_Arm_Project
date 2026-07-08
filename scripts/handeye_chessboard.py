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
from datetime import datetime
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import (EnableRobot, DisableRobot, ClearError,
                                MovJ, SpeedFactor, GetAngle, SetCollisionLevel,
                                RobotMode, StartDrag, StopDrag)


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

OUTPUT_ROOT = os.path.expanduser('~/Robot_Arm_Project/scripts/handeye_calib_runs')


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
        self.StartDrag    = self.create_client(StartDrag, '/dobot_bringup_ros2/srv/StartDrag')
        self.StopDrag     = self.create_client(StopDrag,  '/dobot_bringup_ros2/srv/StopDrag')

        for n,c in [('EnableRobot',self.EnableRobot),('StartDrag',self.StartDrag),('StopDrag',self.StopDrag)]:
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

    def start_drag(self):
        ok, r = self._call(self.StartDrag, StartDrag.Request(), timeout=5.0)
        return ok and getattr(r, 'res', -1) == 0

    def stop_drag(self):
        ok, r = self._call(self.StopDrag, StopDrag.Request(), timeout=5.0)
        return ok and getattr(r, 'res', -1) == 0

    def capture_and_detect(self):
        """拍照 + 棋盘格检测 + 质量检查 → (T_board_in_cam, quality_info) 或 None"""
        self._img = None
        for _ in range(20): rclpy.spin_once(self,timeout_sec=0.1)

        if self._img is None or self._K is None: return None
        img = self._img.copy()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape)==3 else img
        ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS, CHESS_ROWS),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)
        if not ret:
            # 不用 FAST_CHECK 再试一次
            ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS, CHESS_ROWS),
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ret: return {'error': '未检测到棋盘格', 'image': img}

        criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)

        objp = np.zeros((CHESS_COLS*CHESS_ROWS,3), np.float32)
        objp[:,:2] = np.mgrid[0:CHESS_COLS, 0:CHESS_ROWS].T.reshape(-1,2) * SQUARE_MM

        ok_q, rmse, dist, msg = check_board_quality(objp, corners, self._K, self._D)
        if not ok_q:
            return {'error': msg, 'reproj_rmse': rmse, 'dist': dist,
                    'corners': corners, 'image': img}

        ok, rvec, tvec = cv2.solvePnP(objp, corners, self._K, self._D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok: return {'error': 'solvePnP failed', 'corners': corners, 'image': img}

        R,_ = cv2.Rodrigues(rvec)
        T = np.eye(4); T[:3,:3]=R; T[:3,3]=tvec.flatten()
        return {'T': T, 'reproj_rmse': rmse, 'dist': dist,
                'corners': corners, 'image': img}


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
def create_session_dir():
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(OUTPUT_ROOT, f'handeye_{stamp}')
    img_dir = os.path.join(out_dir, 'images')
    os.makedirs(img_dir, exist_ok=False)
    return out_dir, img_dir


def draw_and_save_sample(img_dir, tag, image, corners=None, ok=True):
    if image is None:
        return None
    vis = image.copy()
    if corners is not None:
        cv2.drawChessboardCorners(vis, (CHESS_COLS, CHESS_ROWS), corners, ok)
    suffix = 'ok' if ok else 'reject'
    path = os.path.join(img_dir, f'{tag}_{suffix}.jpg')
    cv2.imwrite(path, vis)
    return path


def compute_and_save(results, out_dir):
    if len(results) < 5:
        print(f'\n❌ 有效帧不足 ({len(results)})，需≥5')
        return False

    print(f'\n{"="*60}')
    print(f'📊 数据质量报告 ({len(results)} 帧)')
    print('='*60)
    reprojs = [r['quality']['reproj_rmse'] for r in results]
    dists = [r['quality']['dist'] for r in results]
    print(f'  重投影误差: min={min(reprojs):.3f} max={max(reprojs):.3f} avg={np.mean(reprojs):.3f}px')
    print(f'  棋盘距离:   min={min(dists):.0f} max={max(dists):.0f} avg={np.mean(dists):.0f}mm')

    print(f'\n{"="*60}')
    print('  构造运动对...')
    print('='*60)
    A_list, B_list = [], []
    skipped_motion = 0
    for i in range(len(results)):
        for j in range(i + 1, len(results)):
            ok_m, _ = check_motion_quality(
                results[i]['T_robot'], results[j]['T_robot'],
                results[i]['T_cam'], results[j]['T_cam'])
            if not ok_m:
                skipped_motion += 1
                continue

            A = inv_se3(results[j]['T_robot']) @ results[i]['T_robot']
            B = results[j]['T_cam'] @ inv_se3(results[i]['T_cam'])
            A_list.append(A); B_list.append(B)

    print(f'  有效: {len(A_list)}  跳过(运动不足): {skipped_motion}  总计: {len(A_list)+skipped_motion}')
    if len(A_list) < 8:
        print(f'  ❌ 有效对不足 ({len(A_list)}), 需≥8')
        return False

    print(f'\n{"="*60}')
    print('  求解 AX=XB...')
    print('='*60)
    X = solve_ax_xb(A_list, B_list)
    X_pose = matrix_to_pose(X)

    print('\n 【手眼矩阵 X = camera_in_tool】')
    print(f'  平移(mm):  [{X_pose[0]:.3f} {X_pose[1]:.3f} {X_pose[2]:.3f}]')
    print(f'  旋转(deg): [{X_pose[3]:.4f} {X_pose[4]:.4f} {X_pose[5]:.4f}]')

    print(f'\n{"="*60}')
    print('  Self-check: T_board_in_world 一致性')
    print('='*60)
    bw_positions = []
    for r in results:
        T_bw = r['T_robot'] @ X @ r['T_cam']
        bw_positions.append(T_bw[:3, 3])
    bw = np.array(bw_positions)
    bw_mean = bw.mean(axis=0)
    bw_errs = np.linalg.norm(bw - bw_mean, axis=1)
    bw_rms = float(np.sqrt(np.mean(bw_errs ** 2)))
    print(f'  均值 xyz=[{bw_mean[0]:.0f} {bw_mean[1]:.0f} {bw_mean[2]:.0f}] mm')
    print(f'  σ=[{bw[:,0].std():.1f} {bw[:,1].std():.1f} {bw[:,2].std():.1f}] mm')
    print(f'  RMS={bw_rms:.1f} mm')
    if bw_rms < 10:
        print('  ✅ 内部一致性良好')
    else:
        print('  ⚠️  内部一致性差 → 可能: 拖拽后未稳定 / 棋盘格移动 / ToolVectorActual不准')

    raw_frames = []
    for r in results:
        T_cam_list = [[float(r['T_cam'][i, j]) for j in range(4)] for i in range(4)]
        T_robot_list = [[float(r['T_robot'][i, j]) for j in range(4)] for i in range(4)]
        raw_frames.append({
            'index': int(r['index']),
            'tool_xyzrpy': [float(v) for v in r['tool_pose']],
            'T_robot_4x4': T_robot_list,
            'T_cam_4x4': T_cam_list,
            'image_path': r.get('image_path'),
            'quality': {'reproj_rmse': float(r['quality']['reproj_rmse']),
                        'dist': float(r['quality']['dist'])},
        })

    out = {
        'method': 'interactive_drag_chessboard+ToolVectorActual+quality_filter',
        'created_at': datetime.now().isoformat(timespec='seconds'),
        'output_dir': out_dir,
        'chessboard': {'size': [CHESS_COLS, CHESS_ROWS], 'square_mm': SQUARE_MM},
        'topics': {'image': TOPIC_IMAGE, 'camera_info': TOPIC_INFO, 'tool': TOPIC_TOOL},
        'num_frames': len(results),
        'num_pairs': len(A_list),
        'quality': {
            'reproj_rmse_stats': [float(min(reprojs)), float(max(reprojs)), float(np.mean(reprojs))],
            'dist_stats': [float(min(dists)), float(max(dists)), float(np.mean(dists))],
            'board_world_rms_mm': bw_rms,
        },
        'X_camera_in_tool': {
            'xyz_mm': [round(v, 4) for v in X_pose[:3]],
            'rpy_deg': [round(v, 6) for v in X_pose[3:6]],
            'matrix': X.tolist(),
        },
        'raw_frames': raw_frames,
    }

    output_file = os.path.join(out_dir, 'handeye_chessboard_result.json')
    with open(output_file, 'w') as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(f'\n📁 标定结果: {output_file}')
    print('   旧的 scripts/handeye_chessboard_result.json 未覆盖')
    return True


def print_interactive_help():
    print('\n交互命令:')
    print('  d  进入拖拽模式 StartDrag')
    print('  e  退出拖拽模式 StopDrag 并重新使能')
    print('  s  采样当前 ToolVectorActual + 当前图像')
    print('  u  删除上一条有效采样')
    print('  c  根据当前采样计算手眼并保存到本次新文件夹')
    print('  h  显示帮助')
    print('  q  退出')


def main():
    rclpy.init()
    node = HandEyeCalib()
    out_dir, img_dir = create_session_dir()

    print('\n' + '=' * 60)
    print('  手眼标定 — 拖拽采样棋盘格法')
    print(f'  棋盘格: {CHESS_COLS}×{CHESS_ROWS}  格边={SQUARE_MM}mm')
    print(f'  输出目录: {out_dir}')
    print('=' * 60)

    print('等待相机内参...')
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node._K is None:
        print('❌ 无CameraInfo')
        node.destroy_node(); rclpy.shutdown(); return
    print(f'📷 K: fx={node._K[0,0]:.0f} fy={node._K[1,1]:.0f}')

    node.init_robot()
    tool = node.get_tool_pose()
    if tool is None:
        print('❌ ToolVectorActual 无数据！确认驱动已启动且连接机械臂WiFi')
        node.destroy_node(); rclpy.shutdown(); return
    print(f'📍 ToolVectorActual: xyz=[{tool[0]:.1f} {tool[1]:.1f} {tool[2]:.1f}] '
          f'rpy=[{tool[3]:.2f} {tool[4]:.2f} {tool[5]:.2f}]')

    res = node.capture_and_detect()
    if res and 'T' in res:
        print(f'✅ 当前棋盘格可见  距离={res["dist"]:.0f}mm  重投影RMS={res["reproj_rmse"]:.3f}px')
    elif res:
        print(f'⚠️  当前棋盘格未通过: {res["error"]}')
    else:
        print('⚠️  当前未收到图像')

    results = []
    rejected_count = 0
    print_interactive_help()

    try:
        while rclpy.ok():
            cmd = input(f'\n[{len(results)}帧] 命令(d/e/s/u/c/h/q)> ').strip().lower()
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=0.02)

            if cmd == 'd':
                if node.start_drag():
                    print('🖐 已进入拖拽模式，请拖到一个新姿态后按 s 采样')
                else:
                    print('❌ StartDrag 失败')

            elif cmd == 'e':
                if node.stop_drag():
                    print('✅ 已退出拖拽')
                else:
                    print('⚠ StopDrag 失败或机器人未处于拖拽')
                node._call(node.EnableRobot, EnableRobot.Request(), timeout=10.0)

            elif cmd == 's':
                idx = len(results) + 1
                tag = f'{idx:02d}'
                tool = node.get_tool_pose()
                if tool is None:
                    print('  ❌ ToolVectorActual 无数据，未采样')
                    continue
                T_robot = pose_to_matrix(tool)
                res = node.capture_and_detect()
                if res is None or 'T' not in res:
                    rejected_count += 1
                    reason = res['error'] if res else '无图像'
                    reject_tag = f'reject_{rejected_count:02d}'
                    if res and 'image' in res:
                        draw_and_save_sample(img_dir, reject_tag, res['image'], res.get('corners'), ok=False)
                    print(f'  ❌ 采样拒绝: {reason}')
                    continue

                image_path = draw_and_save_sample(img_dir, tag, res['image'], res['corners'], ok=True)
                quality = {'reproj_rmse': res['reproj_rmse'], 'dist': res['dist']}
                results.append({
                    'index': idx,
                    'T_robot': T_robot,
                    'T_cam': res['T'],
                    'tool_pose': tool,
                    'quality': quality,
                    'image_path': image_path,
                })
                print(f'  ✅ [{len(results)}] dist={res["dist"]:.0f}mm '
                      f'reproj={res["reproj_rmse"]:.3f}px '
                      f'xyz=[{tool[0]:.0f} {tool[1]:.0f} {tool[2]:.0f}]')

            elif cmd == 'u':
                if not results:
                    print('  没有可删除的采样')
                else:
                    r = results.pop()
                    print(f'  ↩ 已删除第 {r["index"]} 条有效采样')

            elif cmd == 'c':
                if compute_and_save(results, out_dir):
                    print('✅ 计算完成，可以按 q 退出，或继续采样后再次 c 重新计算到同一新文件夹')

            elif cmd == 'h' or cmd == '':
                print_interactive_help()

            elif cmd == 'q':
                break

            else:
                print('未知命令，按 h 查看帮助')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
