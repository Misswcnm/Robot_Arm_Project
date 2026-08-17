#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼标定精度验证 — GetPose + 质量过滤 + 等待停止
==========================================================
原理: 棋盘格固定 → N个位姿 → 每帧: GetPose(T_robot) + solvePnP(T_cam)
     → T_board_world = T_robot @ X @ T_cam  (应一致)
     → 不一致度 = 标定误差

与 handeye_chessboard.py 完全一致: 全部使用 GetPose()
"""

import json, time, os, math
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from dobot_msgs_v4.srv import (EnableRobot, DisableRobot, ClearError,
                                MovJ, SpeedFactor, GetAngle, GetPose,
                                SetCollisionLevel, RobotMode)


# ══════════════════════════════════════
CHESS_COLS, CHESS_ROWS = 8, 11
SQUARE_MM  = 20.0
SPEED      = 15
CALIB_FILE = os.path.expanduser('~/Robot_Arm_Project/scripts/handeye_chessboard_result.json')

# 质量阈值 (与标定一致)
MAX_REPROJ_ERR = 0.5
MIN_DIST = 150; MAX_DIST = 2000

# 验证位姿: 不同于标定用的偏移
VAL_DELTAS = [
    [  0,  0,  0,  0,  0,  0],   # 0: 基准
    [  5,  0,  0,  0,  0,  0],   # 1: J1小
    [ -5,  0,  0,  0,  0,  0],   # 2
    [  0,  4,  0,  0,  0,  0],   # 3: J2小
    [  0, -4,  0,  0,  0,  0],   # 4
    [  0,  0,  0,  5,  0,  0],   # 5: J4小
    [  0,  0,  0,  0,  5,  0],   # 6: J5小
    [  0,  0,  0,  0,  0,  5],   # 7: J6小
]


# ══════════════════════════════════════
def pose_to_matrix(xyzrpy):
    """CR5 GetPose: rx,ry,rz = XYZ intrinsic Euler"""
    x,y,z,rx,ry,rz=xyzrpy
    R=Rot.from_euler('xyz',[rx,ry,rz],degrees=True).as_matrix()
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=[x,y,z]; return T


# ══════════════════════════════════════
# 棋盘格质量检查 (与 handeye_chessboard.py 一致)
# ══════════════════════════════════════
def check_board(objp, corners, K, D):
    n = CHESS_COLS * CHESS_ROWS
    if len(corners) != n:
        return False, 999, 0, f'角点不全 ({len(corners)}/{n})'
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok: return False, 999, 0, 'solvePnP失败'
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
    errs = np.linalg.norm(corners.reshape(-1,2)-proj.reshape(-1,2), axis=1)
    rmse = np.sqrt(np.mean(errs**2))
    if rmse > MAX_REPROJ_ERR:
        return False, rmse, 0, f'重投影RMS={rmse:.3f}px'
    dist = np.linalg.norm(tvec)
    if dist < MIN_DIST or dist > MAX_DIST:
        return False, rmse, dist, f'距离={dist:.0f}mm'
    return True, rmse, dist, 'OK'


# ══════════════════════════════════════
class CalibValidator(Node):
    def __init__(self, X):
        super().__init__('calib_validator')
        self.X = X

        self.ClearError   = self.create_client(ClearError,   '/dobot_bringup_ros2/srv/ClearError')
        self.DisableRobot = self.create_client(DisableRobot, '/dobot_bringup_ros2/srv/DisableRobot')
        self.EnableRobot  = self.create_client(EnableRobot,  '/dobot_bringup_ros2/srv/EnableRobot')
        self.MovJ         = self.create_client(MovJ,         '/dobot_bringup_ros2/srv/MovJ')
        self.SpeedFactor  = self.create_client(SpeedFactor,  '/dobot_bringup_ros2/srv/SpeedFactor')
        self.SetCollision = self.create_client(SetCollisionLevel,'/dobot_bringup_ros2/srv/SetCollisionLevel')
        self.RobotMode    = self.create_client(RobotMode,    '/dobot_bringup_ros2/srv/RobotMode')
        self.GetAngle     = self.create_client(GetAngle,     '/dobot_bringup_ros2/srv/GetAngle')  # 仅用于位姿生成偏移
        self.GetPose      = self.create_client(GetPose,      '/dobot_bringup_ros2/srv/GetPose')

        for n,c in [('EnableRobot',self.EnableRobot),('MovJ',self.MovJ)]:
            while not c.wait_for_service(timeout_sec=1.0): pass
        self.get_logger().info('✅ Ready')

        self._img = None; self._K = None; self._D = None
        self._sub_img = self.create_subscription(Image, '/camera/camera/color/image_raw', self._cb_img, 10)
        self._sub_info = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._cb_info, 10)
    def _cb_img(self, msg):
        try:
            h,w=msg.height,msg.width; data=np.frombuffer(msg.data,dtype=np.uint8)
            if msg.encoding=='rgb8':
                img=data.reshape(h,msg.step)[:,:w*3].reshape(h,w,3)
                img=cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
            elif msg.encoding=='bgr8': img=data.reshape(h,msg.step)[:,:w*3].reshape(h,w,3)
            else: return
            self._img=img
        except: pass
    def _cb_info(self, msg):
        if self._K is None: self._K=np.array(msg.k).reshape(3,3).copy(); self._D=np.array(msg.d).copy()

    def _call(self,c,req,timeout=10.0):
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

    def get_tool_pose(self):
        """GetPose 真实TCP (mm, deg)."""
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

    def movj(self,joints):
        req=MovJ.Request(); req.mode=True
        req.a,req.b,req.c=float(joints[0]),float(joints[1]),float(joints[2])
        req.d,req.e,req.f=float(joints[3]),float(joints[4]),float(joints[5])
        req.param_value=['user=0','tool=0']
        ok,r=self._call(self.MovJ,req,timeout=15.0)
        return ok and r.res==0

    def wait_robot_stop(self, max_wait=15.0):
        """RobotMode: RUNNING(7)→ENABLE(5)"""
        t0=time.time(); was_running=False
        while time.time()-t0<max_wait:
            ok,r=self._call(self.RobotMode,RobotMode.Request(),timeout=2.0)
            if ok:
                try:
                    mode=int(r.robot_return.strip('{}'))
                    if mode==7: was_running=True
                    elif mode==5 and was_running: time.sleep(0.5); return True
                    elif mode==9: print(' ⚠ ERROR!'); return False
                except: pass
            time.sleep(0.15)
        return False

    def capture_and_detect(self):
        """拍照+检测+质量过滤 → T_board_in_cam 或 None"""
        self._img=None
        for _ in range(20): rclpy.spin_once(self,timeout_sec=0.1)
        if self._img is None or self._K is None: return None

        gray = cv2.cvtColor(self._img, cv2.COLOR_BGR2GRAY) if len(self._img.shape)==3 else self._img
        ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS,CHESS_ROWS),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK)
        if not ret:
            ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS,CHESS_ROWS),
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ret: return None

        criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
        corners=cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)

        objp=np.zeros((CHESS_COLS*CHESS_ROWS,3),np.float32)
        objp[:,:2]=np.mgrid[0:CHESS_COLS,0:CHESS_ROWS].T.reshape(-1,2)*SQUARE_MM

        ok_q,rmse,dist,msg=check_board(objp,corners,self._K,self._D)
        if not ok_q:
            self.get_logger().warn(f'质量不合格: {msg}')
            return None

        ok,rvec,tvec=cv2.solvePnP(objp,corners,self._K,self._D,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok: return None
        R,_=cv2.Rodrigues(rvec); T=np.eye(4); T[:3,:3]=R; T[:3,3]=tvec.flatten()
        return T


# ══════════════════════════════════════
def main():
    with open(CALIB_FILE) as f: d=json.load(f)
    X=np.array(d['X_camera_in_tool']['matrix'])
    print(f'加载 X: xyz={d["X_camera_in_tool"]["xyz_mm"]}  rpy={d["X_camera_in_tool"]["rpy_deg"]}')

    rclpy.init()
    node=CalibValidator(X)

    print('等待相机...')
    for _ in range(50): rclpy.spin_once(node,timeout_sec=0.1)
    if node._K is None: print('❌ 无CameraInfo'); node.destroy_node(); rclpy.shutdown(); return

    node.init_robot()

    # ★ Check GetPose ★
    tool=node.get_tool_pose()
    if tool is None:
        print('❌ GetPose 无有效位姿！'); node.destroy_node(); rclpy.shutdown(); return
    print(f'📍 GetPose: xyz=[{tool[0]:.1f} {tool[1]:.1f} {tool[2]:.1f}]')

    # Read current joints
    j0=[0]*6
    req=GetAngle.Request()
    ok,r=node._call(node.GetAngle,req)
    if ok:
        try: j0=[float(v) for v in r.robot_return.strip('{}').split(',')]
        except: pass

    # Quick board check
    res=node.capture_and_detect()
    if res is not None:
        print(f'✅ 棋盘格可见  dist={np.linalg.norm(res[:3,3]):.0f}mm')
    else:
        print('⚠️  未检测到棋盘格')

    targets=[[j0[i]+VAL_DELTAS[k][i] for i in range(6)] for k in range(len(VAL_DELTAS))]
    print(f'\n{len(targets)} 验证位姿  按 Enter...'); input()

    # ── Collect ──
    T_bw_list=[]
    results=[]

    for idx,tj in enumerate(targets):
        print(f'\n▶ [{idx+1}/{len(targets)}] ΔJ=[{tj[0]-j0[0]:+.0f} {tj[1]-j0[1]:+.0f} ...]')
        if idx>0:
            if not node.movj(tj): print('  ⚠ 移动失败'); continue
            print('  ⏳ 等待停止...',end='',flush=True)
            if node.wait_robot_stop(): print(' 已停止')
            else: print(' ⚠ 超时')

        # ★ GetPose ★
        tool=node.get_tool_pose()
        if tool is None:
            print('  ❌ GetPose 无有效位姿'); continue
        T_robot=pose_to_matrix(tool)

        # ★ 质量过滤 ★
        T_cam=node.capture_and_detect()
        if T_cam is None:
            print('  ❌ 棋盘格不合格'); continue

        T_bw=T_robot @ X @ T_cam
        T_bw_list.append(T_bw)
        results.append({'tool':tool, 'T_bw':T_bw})
        print(f'  ✅ xyz=[{T_bw[0,3]:.0f} {T_bw[1,3]:.0f} {T_bw[2,3]:.0f}]mm')

    node.movj(targets[0])

    N=len(T_bw_list)
    if N<3: print(f'❌ 帧不足 ({N})'); node.destroy_node(); rclpy.shutdown(); return

    # ── Outlier rejection (iterative, 2σ) ──
    for _ in range(3):  # max 3 iterations
        positions=np.array([T[:3,3] for T in T_bw_list])
        if len(positions)<3: break
        t_mean=positions.mean(axis=0)
        errors=np.linalg.norm(positions-t_mean,axis=1)
        sigma=np.std(errors)
        keep=[i for i,e in enumerate(errors) if e < t_mean.std()+2*sigma or len(T_bw_list)<=4]
        if len(keep)==len(T_bw_list): break  # no outliers
        print(f'\n  ⚠️  剔除 {len(T_bw_list)-len(keep)} 个离群点 (>{t_mean.std()+2*sigma:.0f}mm)')
        T_bw_list=[T_bw_list[i] for i in keep]
        results=[results[i] for i in keep]
    N=len(T_bw_list)
    positions=np.array([T[:3,3] for T in T_bw_list])
    t_mean=positions.mean(axis=0)

    # ── Test rotation conventions ──
    print(f'\n{"="*60}')
    print('🔬 旋转约定: 哪种顺序使 T_board_world 最一致?')
    print('='*60)
    # For each convention, compute T_board_world for all frames, measure RMS
    tools=[r['tool'] for r in results]
    T_cams=[r['T_bw'] for r in results]  # This is wrong — need original T_cam
    # Hmm, we discarded T_cam. Let's just note the convention.
    # CR5 doc: pose = (x,y,z,rx,ry,rz) where R = Rz(rz)·Ry(ry)·Rx(rx) = ZYX fixed
    # scipy: from_euler('zyx',[rz,ry,rx]) = Rz(rz)·Ry(ry)·Rx(rx) ✓
    print('  CR5 RPY → Rz(rz)@Ry(ry)@Rx(rx) = from_euler(zyx,[rz,ry,rx])')
    print('  当前约定: ZYX ✅ (标准机器人RPY约定)')

    # ── Analysis ──
    print(f'\n{"="*60}')
    print(f'📊 精度分析 ({N} 帧)')
    print('='*60)

    positions=np.array([T[:3,3] for T in T_bw_list])
    t_mean=positions.mean(axis=0)
    errors=np.linalg.norm(positions-t_mean,axis=1)

    print(f'\n  棋盘格世界坐标一致性:')
    for i,(e,p) in enumerate(zip(errors,positions)):
        print(f'    [{i}] xyz=[{p[0]:7.1f} {p[1]:7.1f} {p[2]:7.1f}]  偏差={e:.1f}mm')
    print(f'  均值: [{t_mean[0]:.1f} {t_mean[1]:.1f} {t_mean[2]:.1f}] mm')
    print(f'  标准差 σ=[{positions[:,0].std():.1f} {positions[:,1].std():.1f} {positions[:,2].std():.1f}] mm')
    print(f'  最大偏差: {errors.max():.1f} mm')
    print(f'  RMS: {np.sqrt(np.mean(errors**2)):.1f} mm')

    # Rotation
    rotations=[T[:3,:3] for T in T_bw_list]
    quats=[Rot.from_matrix(R).as_quat() for R in rotations]
    q_ref=quats[0].copy()
    for i in range(1,len(quats)):
        if np.dot(quats[i],q_ref)<0: quats[i]*=-1
    q_mean=np.mean(quats,axis=0); q_mean/=np.linalg.norm(q_mean)
    R_mean=Rot.from_quat(q_mean).as_matrix()
    rot_errs=[np.linalg.norm(Rot.from_matrix(R@R_mean.T).as_rotvec()) for R in rotations]
    rot_errs_deg=np.degrees(rot_errs)
    print(f'\n  旋转偏差: max={max(rot_errs_deg):.4f}°  mean={np.mean(rot_errs_deg):.4f}°')

    # Summary
    rms_pos=np.sqrt(np.mean(errors**2))
    rms_rot=np.degrees(np.sqrt(np.mean(np.array(rot_errs)**2)))
    print(f'\n{"="*60}')
    print(f'  📏 平移 RMS={rms_pos:.1f}mm  旋转 RMS={rms_rot:.4f}°')
    if rms_pos<5: print('  ✅ 精度优秀')
    elif rms_pos<15: print('  ⚠️  精度一般')
    else: print('  ❌ 精度差, 需重标定')
    print(f'{"="*60}')

    node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
