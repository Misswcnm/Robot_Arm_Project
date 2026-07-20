#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼标定 — 棋盘格法 (GetPose + 质量过滤)
==================================================
- 全部使用 GetPose() 读取真实法兰位姿 (不依赖 FK)
- 严格棋盘格检测质量检查 (重投影误差/完整性/距离)
- 标定前数据质量报告
- 独立验证组不参与求解，报告棋盘格世界位姿的平移/旋转误差

用法:
  ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
  ros2 launch realsense2_camera rs_launch.py
  python3 scripts/handeye_chessboard.py
  python3 scripts/handeye_chessboard.py --verify [标定结果.json]
"""

import argparse, json, time, os, math, sys
from datetime import datetime
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from dobot_msgs_v4.srv import (EnableRobot, DisableRobot, ClearError,
                                MovJ, SpeedFactor, GetAngle, GetPose, SetCollisionLevel,
                                RobotMode, StartDrag, StopDrag, Tool, User)


# ═══════════════════════════════════════════════
# 参数
# ═══════════════════════════════════════════════
CHESS_COLS, CHESS_ROWS = 8, 11     # 内角点 (列×行)
SQUARE_MM  = 20.0                  # 格子边长 mm
SPEED      = 15
POSE_SYNC_MAX_TRANS_MM = 0.8
POSE_SYNC_MAX_ROT_DEG = 0.15

# 质量阈值
MAX_REPROJ_ERR = 0.5    # 最大重投影误差 (像素)
MIN_BOARD_DIST = 150    # 棋盘格最近距离 (mm)
MAX_BOARD_DIST = 2000   # 棋盘格最远距离 (mm)

TOPIC_IMAGE       = '/camera/camera/color/image_raw'
TOPIC_INFO        = '/camera/camera/color/camera_info'
GET_POSE_SERVICE  = '/dobot_bringup_ros2/srv/GetPose'
OUTPUT_ROOT = os.path.expanduser('~/Robot_Arm_Project/scripts/handeye_calib_runs')
ACTIVE_CALIBRATION = os.path.expanduser(
    '~/Robot_Arm_Project/scripts/active_handeye_calibration.json')


# ═══════════════════════════════════════════════
# SE(3)
# ═══════════════════════════════════════════════
def pose_to_matrix(xyzrpy):
    """CR5 GetPose: rx,ry,rz = XYZ intrinsic Euler"""
    x,y,z,rx,ry,rz=xyzrpy
    R=Rot.from_euler('xyz',[rx,ry,rz],degrees=True).as_matrix()
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=[x,y,z]; return T

def inv_se3(T):
    R,t=T[:3,:3],T[:3,3]; Ti=np.eye(4); Ti[:3,:3]=R.T; Ti[:3,3]=-R.T@t
    return Ti

def matrix_to_pose(T):
    t=T[:3,3]; r=Rot.from_matrix(T[:3,:3]).as_euler('xyz',degrees=True)
    return [t[0],t[1],t[2],r[0],r[1],r[2]]

def xyzrpy_delta(a, b):
    """Distance between two CR5 xyz/rpy poses."""
    pa = np.asarray(a[:3], dtype=float)
    pb = np.asarray(b[:3], dtype=float)
    ra = Rot.from_euler('xyz', a[3:6], degrees=True)
    rb = Rot.from_euler('xyz', b[3:6], degrees=True)
    return (
        float(np.linalg.norm(pb - pa)),
        float(np.degrees((rb * ra.inv()).magnitude())),
    )


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
        self.GetPose      = self.create_client(GetPose,      '/dobot_bringup_ros2/srv/GetPose')
        self.SetCollision = self.create_client(SetCollisionLevel,'/dobot_bringup_ros2/srv/SetCollisionLevel')
        self.RobotMode    = self.create_client(RobotMode,'/dobot_bringup_ros2/srv/RobotMode')
        self.StartDrag    = self.create_client(StartDrag, '/dobot_bringup_ros2/srv/StartDrag')
        self.StopDrag     = self.create_client(StopDrag,  '/dobot_bringup_ros2/srv/StopDrag')
        self.User         = self.create_client(User, '/dobot_bringup_ros2/srv/User')
        self.Tool         = self.create_client(Tool, '/dobot_bringup_ros2/srv/Tool')

        for n,c in [('EnableRobot',self.EnableRobot),('StartDrag',self.StartDrag),('StopDrag',self.StopDrag)]:
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'等待 {n}...')
        self.get_logger().info('✅ Robot ready')

        self._img = None
        self._img_stamp_ns = None
        self._img_wall_time = None
        self._K = None; self._D = None
        self._sub_img = self.create_subscription(Image, TOPIC_IMAGE, self._cb_img, 10)
        self._sub_info = self.create_subscription(CameraInfo, TOPIC_INFO, self._cb_info, 10)

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
            self._img_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self._img_wall_time = time.time()
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
        if not self.enable_safe():
            raise RuntimeError('EnableRobot/速度/碰撞/User0 初始化失败')
        self.get_logger().info(f'✅ Init done  speed={SPEED}%  collision=Lv.5')

    def enable_safe(self):
        ok_enable, res_enable = self._call(self.EnableRobot, EnableRobot.Request(), timeout=10.0)
        if not ok_enable or getattr(res_enable, 'res', -1) != 0:
            print(f'  ⚠ EnableRobot失败: {getattr(res_enable, "res", res_enable)}')
            return False
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        s = SpeedFactor.Request(); s.ratio = SPEED
        c = SetCollisionLevel.Request(); c.level = 5
        ok_speed, res_speed = self._call(self.SpeedFactor, s, timeout=3.0)
        ok_collision, res_collision = self._call(self.SetCollision, c, timeout=3.0)
        if not ok_speed or getattr(res_speed, 'res', -1) != 0:
            print(f'  ⚠ SpeedFactor({SPEED})失败: {getattr(res_speed, "res", res_speed)}')
            return False
        if not ok_collision or getattr(res_collision, 'res', -1) != 0:
            print(f'  ⚠ SetCollisionLevel(5)失败: {getattr(res_collision, "res", res_collision)}')
            return False
        if not self.use_base_frame():
            print('  ⚠ User(0) 基座坐标系设置失败')
            return False
        return True

    def use_base_frame(self):
        """Select User0 so GetPose returns the flange pose in the base frame."""
        user = User.Request(); user.index = 0
        ok_user, res_user = self._call(self.User, user, timeout=3.0)
        return ok_user and getattr(res_user, 'res', -1) == 0

    def get_flange_pose(self):
        """获取 T_base_flange [x,y,z,rx,ry,rz] mm,deg."""
        if not self.use_base_frame():
            print('  ⚠ User(0)设置失败，拒绝记录法兰位姿')
            return None
        ok, r = self._call(self.GetPose, GetPose.Request(), timeout=3.0)
        if ok and getattr(r, 'res', -1) == 0:
            try:
                vals = [float(v) for v in r.robot_return.strip('{}').split(',')]
                if len(vals) == 6 and np.linalg.norm(vals[:3]) > 1.0:
                    return vals
                print(f'  ⚠ GetPose返回无效位姿: {vals}')
            except Exception:
                pass
        return None

    def get_mode(self):
        ok, r = self._call(self.RobotMode, RobotMode.Request(), timeout=1.5)
        if not ok or getattr(r, 'res', -1) != 0:
            return None
        try:
            return int(r.robot_return.strip('{}'))
        except Exception:
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
        self.use_base_frame()
        ok, r = self._call(self.StartDrag, StartDrag.Request(), timeout=5.0)
        return ok and getattr(r, 'res', -1) == 0

    def stop_drag(self):
        ok, r = self._call(self.StopDrag, StopDrag.Request(), timeout=5.0)
        if not ok or getattr(r, 'res', -1) != 0:
            return False
        return self.enable_safe()

    def capture_and_detect(self):
        """拍照 + 棋盘格检测 + 质量检查 → (T_board_in_cam, quality_info) 或 None"""
        last_stamp = self._img_stamp_ns
        deadline = time.time() + 2.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._img is not None and self._img_stamp_ns != last_stamp:
                break

        if self._img is None or self._K is None: return None
        if self._img_stamp_ns == last_stamp:
            return {'error': '2秒内没有收到新图像'}
        img = self._img.copy()
        stamp_ns = self._img_stamp_ns

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape)==3 else img
        ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS, CHESS_ROWS),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)
        if not ret:
            # 不用 FAST_CHECK 再试一次
            ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS, CHESS_ROWS),
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ret: return {'error': '未检测到棋盘格', 'image': img, 'image_stamp_ns': stamp_ns}

        criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)

        objp = np.zeros((CHESS_COLS*CHESS_ROWS,3), np.float32)
        objp[:,:2] = np.mgrid[0:CHESS_COLS, 0:CHESS_ROWS].T.reshape(-1,2) * SQUARE_MM

        ok_q, rmse, dist, msg = check_board_quality(objp, corners, self._K, self._D)
        if not ok_q:
            return {'error': msg, 'reproj_rmse': rmse, 'dist': dist,
                    'corners': corners, 'image': img, 'image_stamp_ns': stamp_ns}

        ok, rvec, tvec = cv2.solvePnP(objp, corners, self._K, self._D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return {'error': 'solvePnP failed', 'corners': corners, 'image': img,
                    'image_stamp_ns': stamp_ns}

        R,_ = cv2.Rodrigues(rvec)
        T = np.eye(4); T[:3,:3]=R; T[:3,3]=tvec.flatten()
        return {'T': T, 'reproj_rmse': rmse, 'dist': dist,
                'corners': corners, 'image': img, 'image_stamp_ns': stamp_ns}

    def capture_synchronized_sample(self):
        """Bracket a fresh camera frame with GetPose before/after and reject motion."""
        pose_before = self.get_flange_pose()
        if pose_before is None:
            return None, 'GetPose(before) 无有效位姿'
        result = self.capture_and_detect()
        pose_after = self.get_flange_pose()
        if pose_after is None:
            return None, 'GetPose(after) 无有效位姿'

        trans, rot = xyzrpy_delta(pose_before, pose_after)
        if trans > POSE_SYNC_MAX_TRANS_MM or rot > POSE_SYNC_MAX_ROT_DEG:
            return {
                'pose_before': pose_before,
                'pose_after': pose_after,
                'pose_delta_mm': trans,
                'pose_delta_deg': rot,
                'image_result': result,
            }, (f'采样期间机械臂仍在动: Δpose={trans:.2f}mm/{rot:.3f}° '
                f'(阈值{POSE_SYNC_MAX_TRANS_MM:.1f}mm/{POSE_SYNC_MAX_ROT_DEG:.2f}°)')
        if result is None or 'T' not in result:
            reason = result.get('error', '无图像') if result else '无图像'
            return {'image_result': result, 'pose_before': pose_before,
                    'pose_after': pose_after}, reason
        result['pose_before'] = pose_before
        result['pose_after'] = pose_after
        result['pose_delta_mm'] = trans
        result['pose_delta_deg'] = rot
        return result, None


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


def rotation_mean(rotations):
    """SO(3) mean used as the fixed-board reference orientation."""
    return Rot.from_matrix(np.asarray(rotations)).mean().as_matrix()


def pose_error(T_actual, T_reference):
    """Return translation and rotation distance between two SE(3) poses."""
    trans_mm = float(np.linalg.norm(T_actual[:3, 3] - T_reference[:3, 3]))
    dR = T_reference[:3, :3].T @ T_actual[:3, :3]
    rot_deg = float(np.degrees(Rot.from_matrix(dR).magnitude()))
    return trans_mm, rot_deg


def mean_transform(transforms):
    """Estimate one consensus SE(3) pose from repeated observations."""
    if not transforms:
        raise ValueError('无法对空位姿组求均值')
    reference = np.eye(4)
    reference[:3, 3] = np.mean(
        [transform[:3, 3] for transform in transforms], axis=0)
    reference[:3, :3] = rotation_mean(
        [transform[:3, :3] for transform in transforms])
    return reference


def validation_report(samples, X, title='多帧手眼验证'):
    """Report fixed-board consistency without using its calibration-time pose."""
    board_poses = [sample['T_robot'] @ X @ sample['T_cam']
                   for sample in samples]
    reference = mean_transform(board_poses)
    translations = []
    rotations = []

    print('\n' + '=' * 72)
    print(f'  {title}（共识棋盘位姿由当前 {len(samples)} 帧重算）')
    reference_pose = matrix_to_pose(reference)
    print(f'  本次棋盘共识(base): xyz={np.round(reference_pose[:3], 2).tolist()} mm')
    print('  注意: X=T_flange_camera 保持不变；这里只重算本次棋盘位姿。')
    print('-' * 72)
    frame_reports = []
    for index, (sample, board_pose) in enumerate(zip(samples, board_poses), 1):
        delta = board_pose[:3, 3] - reference[:3, 3]
        trans_mm, rot_deg = pose_error(board_pose, reference)
        translations.append(trans_mm)
        rotations.append(rot_deg)
        quality = sample.get('quality', {})
        reproj = quality.get('reproj_rmse', float('nan'))
        sync_mm = quality.get('pose_delta_mm', float('nan'))
        sync_deg = quality.get('pose_delta_deg', float('nan'))
        print(
            f'  V{index:02d}: Δxyz=[{delta[0]:+.2f}, {delta[1]:+.2f}, '
            f'{delta[2]:+.2f}]mm  |Δ|={trans_mm:.2f}mm  '
            f'旋转={rot_deg:.3f}°  PnP={reproj:.3f}px  '
            f'同步={sync_mm:.2f}mm/{sync_deg:.3f}°')
        frame_reports.append({
            'T_board_in_base_4x4': board_pose.tolist(),
            'delta_xyz_mm': delta.tolist(),
            'translation_error_mm': trans_mm,
            'rotation_error_deg': rot_deg,
        })

    trans_rms = float(np.sqrt(np.mean(np.square(translations))))
    rot_rms = float(np.sqrt(np.mean(np.square(rotations))))
    print('-' * 72)
    print(f'  总平移误差: RMS={trans_rms:.2f}mm  '
          f'平均={np.mean(translations):.2f}mm  最大={max(translations):.2f}mm')
    print(f'  总旋转误差: RMS={rot_rms:.3f}°  '
          f'平均={np.mean(rotations):.3f}°  最大={max(rotations):.3f}°')
    if trans_rms <= 5.0 and rot_rms <= 1.0:
        print('  ✅ 多姿态一致性通过（≤5mm / ≤1°）')
    else:
        print('  ⚠ 多姿态一致性超限；检查手眼外参、采样同步和棋盘是否在组内移动')
    print('=' * 72)
    return {
        'reference_T_board_in_base_4x4': reference.tolist(),
        'translation_rms_mm': trans_rms,
        'translation_mean_mm': float(np.mean(translations)),
        'translation_max_mm': float(max(translations)),
        'rotation_rms_deg': rot_rms,
        'rotation_mean_deg': float(np.mean(rotations)),
        'rotation_max_deg': float(max(rotations)),
        'frames': frame_reports,
    }


def load_verification_calibration(path):
    """Load handeye; verification builds a new board reference from its frames."""
    path = os.path.abspath(os.path.expanduser(path))
    visited = set()
    for _ in range(4):
        path = os.path.realpath(path)
        if path in visited:
            raise ValueError(f'手眼标定指针循环引用: {path}')
        visited.add(path)
        with open(path, encoding='utf-8') as stream:
            data = json.load(stream)
        if isinstance(data, str):
            target = os.path.expanduser(data)
            path = (target if os.path.isabs(target)
                    else os.path.join(os.path.dirname(path), target))
            continue
        if not isinstance(data, dict):
            raise ValueError(f'无效手眼标定入口: {path}')
        break
    else:
        raise ValueError('手眼标定指针层级过深')
    record = data.get('T_flange_camera', data.get('X_camera_in_tool'))
    if record is None:
        raise ValueError('标定文件缺少 T_flange_camera')
    X = np.asarray(record['matrix'], dtype=float)
    if X.shape != (4, 4) or not np.isfinite(X).all():
        raise ValueError('标定文件中的 T_flange_camera.matrix 无效')
    frames = data.get('coordinate_frames', {})
    frame_verified = (frames.get('user_index') == 0 and
                      ('flange' in str(frames.get('robot_pose', '')).lower() or
                       frames.get('tool_index') == 0))
    return path, X, frame_verified


def capture_verification_sample(node):
    """Capture one synchronized validation sample in solver-compatible form."""
    result, reason = node.capture_synchronized_sample()
    if reason:
        print(f'  ❌ 当前帧无法验证: {reason}')
        return None
    return {
        'T_robot': pose_to_matrix(result['pose_after']),
        'T_cam': result['T'],
        'flange_pose': result['pose_after'],
        'quality': {
            'reproj_rmse': result['reproj_rmse'],
            'dist': result['dist'],
            'pose_delta_mm': result['pose_delta_mm'],
            'pose_delta_deg': result['pose_delta_deg'],
        },
    }


def run_instant_verification(node, calibration_path):
    path, X, frame_verified = load_verification_calibration(calibration_path)
    samples = []
    print('\n' + '=' * 64)
    print('  手眼标定多帧验证模式（不修改标定文件）')
    print(f'  标定文件: {path}')
    print('  棋盘可以放到任意位置，但本验证组采集期间必须保持不动。')
    print('  每帧必须换一个明显不同的机械臂姿态，至少采集3帧。')
    if not frame_verified:
        print('  ⚠ 此旧文件未声明 T_base_flange/User0，结果仅供诊断')
    print('  v/Enter=采集  p=重算并报告  r=清空  d=拖拽  e=退出拖拽  q=退出')
    print('=' * 64)
    while rclpy.ok():
        command = input(f'\n验证[{len(samples)}帧]命令(v/p/r/d/e/q)> ').strip().lower()
        if command in ('', 'v'):
            sample = capture_verification_sample(node)
            if sample is None:
                continue
            duplicate = None
            for index, previous in enumerate(samples, 1):
                moved, issues = check_motion_quality(
                    previous['T_robot'], sample['T_robot'],
                    previous['T_cam'], sample['T_cam'])
                if not moved:
                    duplicate = (index, issues)
                    break
            if duplicate is not None:
                index, issues = duplicate
                print(f'  ❌ 与 V{index:02d} 姿态变化不足，未计入验证组: '
                      + '; '.join(issues))
                continue
            samples.append(sample)
            pose = sample['flange_pose']
            quality = sample['quality']
            print(f'  ✅ 已记录 V{len(samples):02d}: '
                  f'flange xyz={np.round(pose[:3], 1).tolist()}mm  '
                  f'PnP={quality["reproj_rmse"]:.3f}px')
            if len(samples) < 3:
                print(f'  还需至少 {3 - len(samples)} 帧；移动到不同姿态后继续采集。')
            else:
                validation_report(samples, X)
        elif command == 'p':
            if len(samples) < 3:
                print(f'  ❌ 当前只有{len(samples)}帧，至少需要3帧')
            else:
                validation_report(samples, X)
        elif command == 'r':
            samples.clear()
            print('  ↩ 已清空当前验证组，可以移动棋盘后重新采集')
        elif command == 'd':
            print('  ✅ 已进入拖拽' if node.start_drag() else '  ❌ StartDrag失败')
        elif command == 'e':
            if node.stop_drag():
                print('  ✅ 已退出拖拽')
            else:
                print('  ⚠ StopDrag失败或当前不在拖拽模式')
        elif command == 'q':
            return
        else:
            print('  v/Enter=采集  p=报告  r=清空  d=拖拽  e=退出拖拽  q=退出')


def print_pose(label, pose, mode=None):
    if pose is None:
        print(f'  {label}: GetPose失败  mode={mode}')
        return
    print(f'  {label}: mode={mode} xyz=[{pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f}] '
          f'rpy=[{pose[3]:.3f} {pose[4]:.3f} {pose[5]:.3f}]')


def diagnose_pose_modes(node):
    print('\n' + '=' * 64)
    print('  GetPose 模式诊断')
    print('  目的: 比较 ENABLE / DRAG / 退出DRAG后，User0下法兰坐标是否跳变')
    print('  注意: 测试期间不要移动机械臂，除非提示进入拖拽后你主动拖动')
    print('=' * 64)

    def read(label):
        node.use_base_frame()
        mode = node.get_mode()
        pose = node.get_flange_pose()
        print_pose(label, pose, mode)
        return pose

    p0 = read('A enable初始')
    time.sleep(0.3)
    p1 = read('B enable复读')
    if p0 and p1:
        dt, dr = xyzrpy_delta(p0, p1)
        print(f'    A->B 静止复读差: {dt:.3f}mm / {dr:.4f}°')

    input('\n按 Enter 进入拖拽；进入后先不要移动，直接再按 Enter 读取拖拽坐标...')
    if not node.start_drag():
        print('  ❌ StartDrag失败')
        return
    p2 = read('C drag未移动')
    if p1 and p2:
        dt, dr = xyzrpy_delta(p1, p2)
        print(f'    B->C 仅模式切换差: {dt:.3f}mm / {dr:.4f}°')

    input('\n现在可以拖到新位置，停稳后按 Enter 读取拖拽坐标...')
    p3 = read('D drag拖动后')

    input('\n保持不动，按 Enter 退出拖拽并重新使能...')
    if not node.stop_drag():
        print('  ❌ StopDrag/Enable恢复失败')
        return
    time.sleep(0.3)
    p4 = read('E enable恢复后')
    if p3 and p4:
        dt, dr = xyzrpy_delta(p3, p4)
        print(f'    D->E 退出拖拽坐标差: {dt:.3f}mm / {dr:.4f}°')

    print('\n判读:')
    print('  B->C 大: StartDrag改变了控制器当前坐标系/工具状态，或GetPose在拖拽态不可信。')
    print('  D->E 大: StopDrag/Enable后User坐标系没有恢复到User0。')
    print('  A->B小、B->C小、D->E小: 坐标模式没问题，第一帧错误就是图像/位姿不同步。')


def serialize_frame(r):
    return {
        'index': int(r['index']),
        'flange_xyzrpy': [float(v) for v in r['flange_pose']],
        'T_robot_4x4': np.asarray(r['T_robot']).tolist(),
        'T_cam_4x4': np.asarray(r['T_cam']).tolist(),
        'image_path': r.get('image_path'),
        'quality': {
            'reproj_rmse': float(r['quality']['reproj_rmse']),
            'dist': float(r['quality']['dist']),
            'pose_delta_mm': float(r['quality'].get('pose_delta_mm', 0.0)),
            'pose_delta_deg': float(r['quality'].get('pose_delta_deg', 0.0)),
        },
    }


def compute_and_save(results, validation_results, out_dir):
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

    print('\n 【手眼矩阵 X = T_flange_camera】')
    print(f'  平移(mm):  [{X_pose[0]:.3f} {X_pose[1]:.3f} {X_pose[2]:.3f}]')
    print(f'  旋转(deg): [{X_pose[3]:.4f} {X_pose[4]:.4f} {X_pose[5]:.4f}]')

    print(f'\n{"="*60}')
    print('  Self-check: T_board_in_world 一致性')
    print('='*60)
    board_world_poses = []
    for r in results:
        T_bw = r['T_robot'] @ X @ r['T_cam']
        board_world_poses.append(T_bw)
    bw = np.array([T[:3, 3] for T in board_world_poses])
    bw_mean = bw.mean(axis=0)
    bw_rot_mean = rotation_mean([T[:3, :3] for T in board_world_poses])
    T_board_reference = np.eye(4)
    T_board_reference[:3, :3] = bw_rot_mean
    T_board_reference[:3, 3] = bw_mean
    bw_errs = np.linalg.norm(bw - bw_mean, axis=1)
    bw_rms = float(np.sqrt(np.mean(bw_errs ** 2)))
    print(f'  均值 xyz=[{bw_mean[0]:.0f} {bw_mean[1]:.0f} {bw_mean[2]:.0f}] mm')
    print(f'  σ=[{bw[:,0].std():.1f} {bw[:,1].std():.1f} {bw[:,2].std():.1f}] mm')
    print(f'  RMS={bw_rms:.1f} mm')
    if bw_rms < 10:
        print('  ✅ 内部一致性良好')
    else:
        print('  ⚠️  内部一致性差 → 可能: 拖拽后未稳定 / 棋盘格移动 / GetPose不准')

    if len(validation_results) >= 3:
        validation_summary = validation_report(
            validation_results, X, title='独立验证组')
        validation_frames = []
        for raw, report in zip(validation_results,
                               validation_summary['frames']):
            frame = serialize_frame(raw)
            frame.update(report)
            validation_frames.append(frame)
        val_trans_rms = validation_summary['translation_rms_mm']
        val_rot_rms = validation_summary['rotation_rms_deg']
    else:
        validation_summary = None
        validation_frames = [serialize_frame(r) for r in validation_results]
        val_trans_rms = None
        val_rot_rms = None
        print(f'\n  ⚠️ 独立验证组只有 {len(validation_results)} 帧；至少3帧才计算误差')

    raw_frames = [serialize_frame(r) for r in results]

    out = {
        'method': 'interactive_drag_chessboard+GetPose+quality_filter',
        'created_at': datetime.now().isoformat(timespec='seconds'),
        'output_dir': out_dir,
        'chessboard': {'size': [CHESS_COLS, CHESS_ROWS], 'square_mm': SQUARE_MM},
        'topics': {'image': TOPIC_IMAGE, 'camera_info': TOPIC_INFO,
                   'flange_pose_service': GET_POSE_SERVICE},
        'coordinate_frames': {
            'user_index': 0,
            'robot_pose': 'T_base_flange_from_GetPose_in_user0',
            'camera_extrinsic': 'T_flange_camera',
            'chain': 'T_base_target = T_base_flange * T_flange_camera * T_camera_target',
        },
        'num_frames': len(results),
        'num_pairs': len(A_list),
        'quality': {
            'reproj_rmse_stats': [float(min(reprojs)), float(max(reprojs)), float(np.mean(reprojs))],
            'dist_stats': [float(min(dists)), float(max(dists)), float(np.mean(dists))],
            'board_world_rms_mm': bw_rms,
        },
        'validation': {
            'num_frames': len(validation_results),
            'method': 'current_group_consensus_fixed_board',
            'reference_T_board_in_base_4x4': (
                validation_summary['reference_T_board_in_base_4x4']
                if validation_summary else None),
            'translation_rms_mm': val_trans_rms,
            'rotation_rms_deg': val_rot_rms,
            'translation_mean_mm': (
                validation_summary['translation_mean_mm']
                if validation_summary else None),
            'translation_max_mm': (
                validation_summary['translation_max_mm']
                if validation_summary else None),
            'rotation_mean_deg': (
                validation_summary['rotation_mean_deg']
                if validation_summary else None),
            'rotation_max_deg': (
                validation_summary['rotation_max_deg']
                if validation_summary else None),
            'frames': validation_frames,
        },
        'T_flange_camera': {
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
    print('   如需统一启用，只修改 scripts/active_handeye_calibration.json 中的路径。')
    return True


def print_interactive_help():
    print('\n交互命令:')
    print('  d  进入拖拽模式 StartDrag')
    print('  e  退出拖拽模式 StopDrag 并重新使能')
    print('  s  采样当前 GetPose + 当前图像')
    print('  v  采集独立验证组（不参与手眼求解）')
    print('  u  删除上一条有效采样')
    print('  x  删除上一条验证采样')
    print('  c  根据当前采样计算手眼并保存到本次新文件夹')
    print('  h  显示帮助')
    print('  q  退出')


def main():
    parser = argparse.ArgumentParser(description='CR5 棋盘格手眼标定与多帧验证')
    parser.add_argument(
        '--verify', nargs='?', const=ACTIVE_CALIBRATION, metavar='RESULT_JSON',
        help='加载手眼结果并进入至少3帧的多姿态验证；省略路径时使用 active 标定')
    parser.add_argument(
        '--pose-diagnose', action='store_true',
        help='诊断 ENABLE/DRAG/退出DRAG 后 User0 下法兰 GetPose 是否跳变')
    args = parser.parse_args()

    rclpy.init()
    node = HandEyeCalib()

    if args.verify or args.pose_diagnose:
        try:
            print('等待相机内参...')
            for _ in range(50):
                rclpy.spin_once(node, timeout_sec=0.1)
            if args.verify and node._K is None:
                raise RuntimeError('无CameraInfo')
            node.init_robot()
            if args.pose_diagnose:
                diagnose_pose_modes(node)
            else:
                run_instant_verification(node, args.verify)
        except (OSError, KeyError, TypeError, ValueError, RuntimeError) as exc:
            print(f'❌ 无法启动诊断/验证: {exc}')
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

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
    res, reason = node.capture_synchronized_sample()
    if res and not reason and 'T' in res:
        tool = res['pose_after']
        print(f'📍 GetPose: xyz=[{tool[0]:.1f} {tool[1]:.1f} {tool[2]:.1f}] '
              f'rpy=[{tool[3]:.2f} {tool[4]:.2f} {tool[5]:.2f}]')
        print(f'✅ 当前棋盘格可见  距离={res["dist"]:.0f}mm  重投影RMS={res["reproj_rmse"]:.3f}px')
    else:
        print(f'⚠️  当前同步预检未通过: {reason or "无图像"}')

    results = []
    validation_results = []
    rejected_count = 0
    print_interactive_help()

    try:
        while rclpy.ok():
            cmd = input(f'\n[标定{len(results)}帧/验证{len(validation_results)}帧] '
                        '命令(d/e/s/v/u/x/c/h/q)> ').strip().lower()
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

            elif cmd in ('s', 'v'):
                is_validation = cmd == 'v'
                target = validation_results if is_validation else results
                idx = len(target) + 1
                tag = f'validation_{idx:02d}' if is_validation else f'calibration_{idx:02d}'
                res, reason = node.capture_synchronized_sample()
                if reason:
                    rejected_count += 1
                    reject_tag = f'reject_{rejected_count:02d}'
                    image_result = res.get('image_result') if isinstance(res, dict) else res
                    if image_result and 'image' in image_result:
                        draw_and_save_sample(img_dir, reject_tag, image_result['image'],
                                             image_result.get('corners'), ok=False)
                    print(f'  ❌ 采样拒绝: {reason}')
                    continue

                tool = res['pose_after']
                T_robot = pose_to_matrix(tool)
                image_path = draw_and_save_sample(img_dir, tag, res['image'], res['corners'], ok=True)
                quality = {
                    'reproj_rmse': res['reproj_rmse'],
                    'dist': res['dist'],
                    'pose_delta_mm': res['pose_delta_mm'],
                    'pose_delta_deg': res['pose_delta_deg'],
                }
                target.append({
                    'index': idx,
                    'T_robot': T_robot,
                    'T_cam': res['T'],
                    'flange_pose': tool,
                    'quality': quality,
                    'image_path': image_path,
                })
                kind = '验证' if is_validation else '标定'
                print(f'  ✅ {kind}[{len(target)}] dist={res["dist"]:.0f}mm '
                      f'reproj={res["reproj_rmse"]:.3f}px '
                      f'xyz=[{tool[0]:.0f} {tool[1]:.0f} {tool[2]:.0f}] '
                      f'同步Δ={res["pose_delta_mm"]:.2f}mm/{res["pose_delta_deg"]:.3f}°')

            elif cmd == 'u':
                if not results:
                    print('  没有可删除的采样')
                else:
                    r = results.pop()
                    print(f'  ↩ 已删除第 {r["index"]} 条有效采样')

            elif cmd == 'x':
                if not validation_results:
                    print('  没有可删除的验证采样')
                else:
                    r = validation_results.pop()
                    print(f'  ↩ 已删除第 {r["index"]} 条验证采样')

            elif cmd == 'c':
                if compute_and_save(results, validation_results, out_dir):
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
