"""CR5 robot interface: init, movj, GetPose, wait_stop."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from dobot_msgs_v4.srv import (EnableRobot, DisableRobot, ClearError,
                                MovJ, SpeedFactor, GetAngle, GetPose, GetErrorID,
                                SetCollisionLevel, RobotMode, StartDrag, StopDrag)


class CR5Robot:
    """CR5 机械臂控制 (不继承Node, 接收外部node)"""
    def __init__(self, node: Node, speed: int = 15):
        self._node = node
        self._speed = speed
        self._logger = node.get_logger()

        self.ClearError   = node.create_client(ClearError,   '/dobot_bringup_ros2/srv/ClearError')
        self.DisableRobot = node.create_client(DisableRobot, '/dobot_bringup_ros2/srv/DisableRobot')
        self.EnableRobot  = node.create_client(EnableRobot,  '/dobot_bringup_ros2/srv/EnableRobot')
        self.MovJ         = node.create_client(MovJ,         '/dobot_bringup_ros2/srv/MovJ')
        self.SpeedFactor  = node.create_client(SpeedFactor,  '/dobot_bringup_ros2/srv/SpeedFactor')
        self.SetCollision = node.create_client(SetCollisionLevel, '/dobot_bringup_ros2/srv/SetCollisionLevel')
        self.RobotMode    = node.create_client(RobotMode,    '/dobot_bringup_ros2/srv/RobotMode')
        self.GetErrorID   = node.create_client(GetErrorID,   '/dobot_bringup_ros2/srv/GetErrorID')
        self.GetAngle     = node.create_client(GetAngle,     '/dobot_bringup_ros2/srv/GetAngle')
        self.GetPose      = node.create_client(GetPose,      '/dobot_bringup_ros2/srv/GetPose')
        self.StartDrag    = node.create_client(StartDrag,    '/dobot_bringup_ros2/srv/StartDrag')
        self.StopDrag     = node.create_client(StopDrag,     '/dobot_bringup_ros2/srv/StopDrag')

        for n, c in [('EnableRobot', self.EnableRobot), ('MovJ', self.MovJ)]:
            while not c.wait_for_service(timeout_sec=1.0):
                self._logger.info(f'等待 {n}...')

        self._last_getpose_warn = 0.0

    def _call(self, client, req, timeout=10.0):
        fut = client.call_async(req)
        # spin_until_future_complete: ROS2标准方式, 比手写spin_once循环高效
        rclpy.spin_until_future_complete(self._node, fut, timeout_sec=timeout)
        if not fut.done():
            fut.cancel()
            return False, 'timeout'
        try:
            return True, fut.result()
        except Exception as e:
            return False, str(e)

    # ── Init (全部指令连续发送, recv后立即下一步) ──
    def init(self):
        self._call(self.ClearError, ClearError.Request())
        self._call(self.DisableRobot, DisableRobot.Request())
        self._call(self.EnableRobot, EnableRobot.Request(), timeout=10.0)  # ~4s
        rclpy.spin_once(self._node, timeout_sec=0.1)  # 让topic到位
        self.apply_motion_safety()
        self._logger.info(f'✅ CR5 ready  speed={self._speed}%  collision=Lv.5')

    def apply_motion_safety(self) -> bool:
        """每次EnableRobot后都重新设置速度和碰撞等级，避免恢复默认高速。"""
        s = SpeedFactor.Request()
        s.ratio = self._speed
        ok_speed, speed_resp = self._call(self.SpeedFactor, s)
        c = SetCollisionLevel.Request()
        c.level = 5
        ok_collision, collision_resp = self._call(self.SetCollision, c)
        if not ok_speed or getattr(speed_resp, 'res', 0) != 0:
            self._logger.error(
                f'SpeedFactor设置失败: ok={ok_speed} '
                f'res={getattr(speed_resp, "res", None)}')
            return False
        if not ok_collision or getattr(collision_resp, 'res', 0) != 0:
            self._logger.error(
                f'SetCollisionLevel设置失败: ok={ok_collision} '
                f'res={getattr(collision_resp, "res", None)}')
            return False
        self._logger.info(f'运动安全参数已设置: speed={self._speed}% collision=Lv.5')
        return True

    # ── GetPose ──
    @staticmethod
    def _valid_tool_pose(tool) -> bool:
        if tool is None or len(tool) < 6:
            return False
        vals = np.asarray(tool[:6], dtype=float)
        if not np.all(np.isfinite(vals)):
            return False
        return np.linalg.norm(vals[:3]) > 1.0

    def get_tool(self, warn: bool = True) -> list | None:
        """真实TCP [x,y,z,rx,ry,rz] mm,deg. 统一使用GetPose()."""
        now = time.time()
        if warn and now - self._last_getpose_warn > 2.0:
            self._logger.info('GetPose()读取当前TCP位姿')
            self._last_getpose_warn = now
        if not self.GetPose.wait_for_service(timeout_sec=1.0):
            return None
        ok, response = self._call(self.GetPose, GetPose.Request(), timeout=3.0)
        if ok and response is not None and response.res == 0:
            try:
                s = response.robot_return.strip('{}')
                vals = [float(v) for v in s.split(',')]
                if len(vals) == 6 and self._valid_tool_pose(vals):
                    return vals
                if warn:
                    self._logger.warn(f'GetPose返回无效全零位姿: {vals}')
            except Exception:
                pass
        elif warn:
            self._logger.warn(
                f'GetPose调用失败: ok={ok} res={getattr(response, "res", None)}')
        return None

    # ── Joint angles ──
    def get_joints(self) -> list | None:
        ok, r = self._call(self.GetAngle, GetAngle.Request())
        if ok:
            try:
                return [float(v) for v in r.robot_return.strip('{}').split(',')]
            except:
                pass
        return None

    @staticmethod
    def matrix_to_pose(T: np.ndarray) -> list:
        """4x4工具位姿矩阵 -> Dobot xyz/rpy(mm,deg)."""
        rpy = Rot.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
        return [float(T[0, 3]), float(T[1, 3]), float(T[2, 3]),
                float(rpy[0]), float(rpy[1]), float(rpy[2])]

    def movj_pose(self, target, label: str = 'MovJ') -> bool:
        """直接使用控制器笛卡尔MovJ，并等待队列真正启动和到位."""
        return self.movj_pose_status(target, label=label) == 'arrived'

    def movj_pose_status(self, target, label: str = 'MovJ') -> str:
        """返回 arrived/queued/failed: queued表示已入队但未确认到位."""
        pose = self.matrix_to_pose(target) if isinstance(target, np.ndarray) else list(target)
        pre_move = self.get_tool()
        self._logger.info(
            f'{label}: 直接发送MovJ(pose) pose=[{pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f} '
            f'{pose[3]:.3f} {pose[4]:.3f} {pose[5]:.3f}]')
        req = MovJ.Request()
        req.mode = False
        req.a, req.b, req.c = float(pose[0]), float(pose[1]), float(pose[2])
        req.d, req.e, req.f = float(pose[3]), float(pose[4]), float(pose[5])
        req.param_value = []
        ok, r = self._call(self.MovJ, req, timeout=20.0)
        if not ok or r.res != 0:
            self._logger.warn(f'{label}: MovJ(pose)失败 ok={ok} res={getattr(r, "res", None)} ret={getattr(r, "robot_return", r)}')
            return 'failed'
        arrived, pos_err, rot_err = self.wait_pose_arrival(
            pose, start_pose=pre_move, timeout=12.0)
        if not arrived:
            self._logger.warn(
                f'{label}: MovJ已入队但未确认到位, 不抢跑Jacobian '
                f'(目标残差={pos_err:.1f}mm/{rot_err:.2f}°)')
            return 'queued'
        self._logger.info(
            f'{label}: MovJ(pose)到位 '
            f'(目标残差={pos_err:.1f}mm/{rot_err:.2f}°)')
        return 'arrived'

    @staticmethod
    def _pose_residual(actual, target) -> tuple[float, float]:
        pos_err = float(np.linalg.norm(
            np.asarray(actual[:3], dtype=float) -
            np.asarray(target[:3], dtype=float)))
        Ra = Rot.from_euler('xyz', actual[3:6], degrees=True)
        Rt = Rot.from_euler('xyz', target[3:6], degrees=True)
        rot_err = float(np.degrees((Rt * Ra.inv()).magnitude()))
        return pos_err, rot_err

    def wait_pose_arrival(self, target, start_pose=None, timeout: float = 12.0,
                          pos_tol: float = 1.0,
                          rot_tol: float = 0.15,
                          feedback_timeout: float = 8.0,
                          poll_interval: float = 0.5) -> tuple[bool, float, float]:
        """MovJ返回0仅代表入队；用GetPose轮询目标残差确认到位。"""
        started = start_pose is None
        stable_at_target = 0
        pos_err = float('inf')
        rot_err = float('inf')
        t0 = time.time()

        while time.time() - t0 < timeout:
            current = self.get_tool(warn=False)
            if current is None:
                if time.time() - t0 > feedback_timeout:
                    self._logger.warn('MovJ后GetPose无有效反馈, 快速切换Jacobian')
                    return False, pos_err, rot_err
                time.sleep(poll_interval)
                continue

            if not started:
                moved_mm, moved_deg = self._pose_residual(current, start_pose)
                started = moved_mm > 0.2 or moved_deg > 0.03

            pos_err, rot_err = self._pose_residual(current, target)
            if pos_err <= pos_tol and rot_err <= rot_tol:
                stable_at_target += 1
                if stable_at_target >= 3:
                    return True, pos_err, rot_err
            else:
                stable_at_target = 0
            time.sleep(poll_interval)

        current = self.get_tool(warn=False)
        if current:
            pos_err, rot_err = self._pose_residual(current, target)
        self._logger.warn(f'MovJ等待超时: 是否启动={started}')
        return False, pos_err, rot_err

    def get_robot_mode(self) -> int | None:
        ok, response = self._call(
            self.RobotMode, RobotMode.Request(), timeout=1.0)
        if not ok or response is None or response.res != 0:
            return None
        try:
            return int(response.robot_return.strip('{}'))
        except (TypeError, ValueError):
            return None

    def get_error_id(self) -> str:
        ok, response = self._call(
            self.GetErrorID, GetErrorID.Request(), timeout=1.0)
        if not ok or response is None:
            return '<查询失败>'
        return response.robot_return.strip()

    def movj(self, joints: list) -> bool:
        pre_move = self.get_tool()
        req = MovJ.Request(); req.mode = True
        req.a, req.b, req.c = float(joints[0]), float(joints[1]), float(joints[2])
        req.d, req.e, req.f = float(joints[3]), float(joints[4]), float(joints[5])
        req.param_value = ['user=0', 'tool=0']
        ok, r = self._call(self.MovJ, req, timeout=15.0)
        if not ok or r.res != 0:
            return False
        self.wait_tool_stable()
        cur = self.get_tool()
        if pre_move and cur:
            d = np.linalg.norm(np.array(cur[:3]) - np.array(pre_move[:3]))
            if d < 0.5:
                self._logger.warn(f'JointMovJ未执行(Δ={d:.1f}mm) → 恢复')
                self.recover()
                return False
        return True


    def wait_tool_stable(self, timeout: float = 5.0,
                          stable_required: int = 5,
                          min_wait: float = 0.15,
                          feedback_timeout: float = 3.5,
                          poll_interval: float = 0.25) -> bool:
        """GetPose稳定检测: 连续stable_required次 xyz<0.3mm 且 rpy<0.05deg."""
        time.sleep(min_wait)  # 确保运动已启动
        prev_xyz = None; prev_rpy = None
        stable = 0
        t0 = time.time()
        while time.time() - t0 < timeout:
            current = self.get_tool(warn=False)
            if current is None:
                if time.time() - t0 > feedback_timeout:
                    self._logger.warn('等待停稳时GetPose无有效反馈')
                    return False
                time.sleep(poll_interval)
                continue

            cur_xyz = np.array(current[:3])
            cur_rpy = np.array(current[3:6])
            if prev_xyz is not None:
                d_xyz = np.linalg.norm(cur_xyz - prev_xyz)
                d_rpy = np.linalg.norm(cur_rpy - prev_rpy)
                if d_xyz < 0.3 and d_rpy < 0.05:
                    stable += 1
                    if stable >= stable_required:
                        return True
                else:
                    stable = 0  # 动了, 重置
            prev_xyz = cur_xyz
            prev_rpy = cur_rpy
            time.sleep(poll_interval)
        return True  # 超时默认已停

    def start_drag(self) -> bool:
        ok, r = self._call(self.StartDrag, StartDrag.Request(), timeout=5.0)
        return ok and r.res == 0

    def stop_drag(self) -> bool:
        ok, r = self._call(self.StopDrag, StopDrag.Request(), timeout=5.0)
        return ok and r.res == 0

    def enable(self) -> bool:
        """退出拖拽后重新使能机器人，并强制恢复速度/碰撞等级。"""
        ok, r = self._call(self.EnableRobot, EnableRobot.Request(), timeout=10.0)
        if not ok:
            self._logger.error(f'EnableRobot 调用失败: {r}')
            return False
        if r.res != 0:
            self._logger.error(f'EnableRobot 返回异常 res={r.res}')
            return False
        rclpy.spin_once(self._node, timeout_sec=0.1)
        return self.apply_motion_safety()

    def recover(self) -> bool:
        """错误恢复: ClearError→EnableRobot→SpeedFactor→SetCollisionLevel."""
        self._logger.error('ERROR! 执行ClearError+EnableRobot恢复序列...')
        self._call(self.ClearError, ClearError.Request())
        ok, _ = self._call(self.EnableRobot, EnableRobot.Request(), timeout=10.0)
        rclpy.spin_once(self._node, timeout_sec=0.1)
        if not ok:
            self._logger.error('EnableRobot恢复失败')
            return False
        if not self.apply_motion_safety():
            return False
        self._logger.info('✅ 恢复完成')
        return True
