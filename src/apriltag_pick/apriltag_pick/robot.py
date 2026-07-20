import re
import threading
import time

import numpy as np
import rclpy
from dobot_msgs_v4.srv import (
    ClearError, DisableRobot, EnableRobot, GetErrorID, GetPose, MovJ,
    RobotMode, SetCollisionLevel, SpeedFactor, StartDrag, StopDrag,
    Tool, ToolDOInstant, User)

from .transforms import matrix_to_tool_pose


class CR5Robot:
    """Small CR5 adapter following the controller quirks recorded in CLAUDE.md."""

    PREFIX = '/dobot_bringup_ros2/srv'
    MODE_DISABLED = 4
    MODE_ENABLED = 5
    MODE_BACKDRIVE = 6
    MODE_RUNNING = 7
    MODE_ERROR = 9
    MODE_COLLISION = 11

    def __init__(self, node, speed=15):
        self.node = node
        self.log = node.get_logger()
        self.speed = int(speed)
        self.last_getpose_warn = 0.0
        self.dragging = False
        self.clear = node.create_client(ClearError, f'{self.PREFIX}/ClearError')
        self.disable = node.create_client(DisableRobot, f'{self.PREFIX}/DisableRobot')
        self.enable = node.create_client(EnableRobot, f'{self.PREFIX}/EnableRobot')
        self.movj = node.create_client(MovJ, f'{self.PREFIX}/MovJ')
        self.speed_factor = node.create_client(SpeedFactor, f'{self.PREFIX}/SpeedFactor')
        self.collision = node.create_client(
            SetCollisionLevel, f'{self.PREFIX}/SetCollisionLevel')
        self.mode = node.create_client(RobotMode, f'{self.PREFIX}/RobotMode')
        self.errors = node.create_client(GetErrorID, f'{self.PREFIX}/GetErrorID')
        self.start_drag_client = node.create_client(
            StartDrag, f'{self.PREFIX}/StartDrag')
        self.stop_drag_client = node.create_client(
            StopDrag, f'{self.PREFIX}/StopDrag')
        self.tool_do = node.create_client(
            ToolDOInstant, f'{self.PREFIX}/ToolDOInstant')
        self.get_pose_client = node.create_client(GetPose, f'{self.PREFIX}/GetPose')
        self.user_frame = node.create_client(User, f'{self.PREFIX}/User')
        self.tool_frame = node.create_client(Tool, f'{self.PREFIX}/Tool')

    @staticmethod
    def valid_tool_pose(tool):
        if tool is None or len(tool) < 6:
            return False
        values = np.asarray(tool[:6], dtype=float)
        if not np.all(np.isfinite(values)):
            return False
        return np.linalg.norm(values[:3]) > 1.0

    def call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        completed.wait(timeout)
        return future.result() if future.done() else None

    def initialize(self):
        """Never PowerOn: this CR5 requires this exact safe sequence."""
        self.call(self.clear, ClearError.Request())
        self.call(self.disable, DisableRobot.Request())
        response = self.call(self.enable, EnableRobot.Request(), 12.0)
        if response is None or response.res != 0:
            raise RuntimeError('EnableRobot 失败')
        if not self.apply_motion_safety():
            raise RuntimeError('速度/碰撞等级设置失败')
        if not self.use_base_tool0():
            raise RuntimeError('User(0)/Tool(0) 坐标系设置失败')
        if not self.wait_mode(self.MODE_ENABLED, timeout=5.0):
            raise RuntimeError(f'初始化后状态异常: RobotMode={self.get_mode()}，期望5')
        self.dragging = False
        self.log.info(f'CR5 ready: speed={self.speed}%, collision=5')

    def use_base_tool0(self):
        """Select User0/Tool0 for commands whose target is a flange pose."""
        user = User.Request()
        user.index = 0
        tool = Tool.Request()
        tool.index = 0
        user_response = self.call(self.user_frame, user, 3.0)
        tool_response = self.call(self.tool_frame, tool, 3.0)
        ok = (user_response is not None and user_response.res == 0 and
              tool_response is not None and tool_response.res == 0)
        if not ok:
            self.log.error(
                'User(0)/Tool(0)设置失败: user_res=%s tool_res=%s' % (
                    getattr(user_response, 'res', None),
                    getattr(tool_response, 'res', None)))
        return ok

    def use_base_frame(self):
        """Select User0 only; GetPose itself measures the flange, not a TCP."""
        user = User.Request()
        user.index = 0
        response = self.call(self.user_frame, user, 3.0)
        ok = response is not None and response.res == 0
        if not ok:
            self.log.error('User(0)设置失败: res=%s' % getattr(response, 'res', None))
        return ok

    def apply_motion_safety(self):
        """Every EnableRobot must be followed by speed/collision settings."""
        speed = SpeedFactor.Request()
        speed.ratio = self.speed
        speed_resp = self.call(self.speed_factor, speed)
        collision = SetCollisionLevel.Request()
        collision.level = 5
        collision_resp = self.call(self.collision, collision)
        if speed_resp is None or speed_resp.res != 0:
            self.log.error(
                f'SpeedFactor设置失败: res={getattr(speed_resp, "res", None)}')
            return False
        if collision_resp is None or collision_resp.res != 0:
            self.log.error(
                f'SetCollisionLevel设置失败: res={getattr(collision_resp, "res", None)}')
            return False
        self.log.info(f'运动安全参数已设置: speed={self.speed}%, collision=5')
        return True

    def get_tool(self, warn=True):
        now = time.monotonic()
        if warn and now - self.last_getpose_warn > 2.0:
            self.log.info('GetPose()读取当前法兰位姿（以当前User原点表达）')
            self.last_getpose_warn = now
        response = self.call(self.get_pose_client, GetPose.Request(), 3.0)
        if response is not None and response.res == 0:
            try:
                values = [float(x) for x in response.robot_return.strip('{}').split(',')]
                if len(values) == 6 and self.valid_tool_pose(values):
                    return values
                if warn:
                    self.log.warn(f'GetPose返回无效全零位姿: {values}')
            except ValueError:
                pass
        return None

    def get_mode(self):
        response = self.call(self.mode, RobotMode.Request(), 1.0)
        if response is None or response.res != 0:
            return None
        try:
            return int(response.robot_return.strip('{}'))
        except (AttributeError, TypeError, ValueError):
            return None

    @staticmethod
    def _alarm_codes(response):
        """Flatten Dobot's [[controller], [J1] ... [J6]] response."""
        if response is None:
            return []
        raw = getattr(response, 'robot_return', '')
        return [int(value) for value in re.findall(r'-?\d+', raw)]

    def wait_mode(self, expected, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.get_mode() == expected:
                return True
            time.sleep(0.2)
        return False

    def move_pose(self, transform, timeout=15.0):
        mode = self.get_mode()
        if self.dragging or mode == self.MODE_BACKDRIVE:
            self.log.error('拒绝MovJ：机械臂仍在拖拽模式，请先按2正常退出拖拽')
            return False
        if mode != self.MODE_ENABLED:
            self.log.error(
                f'拒绝MovJ：RobotMode={mode}，只有ENABLED空闲态(mode=5)允许运动')
            return False
        pose = matrix_to_tool_pose(transform)
        self.log.info(
            'MovJ target xyz=[%.1f, %.1f, %.1f] mm, '
            'rpy=[%.1f, %.1f, %.1f] deg' % tuple(pose))
        request = MovJ.Request()
        request.mode = False
        request.a, request.b, request.c = map(float, pose[:3])
        request.d, request.e, request.f = map(float, pose[3:])
        request.param_value = []  # old CR5 rejects named a/v/cp and pose={}
        response = self.call(self.movj, request, 20.0)
        if response is None or response.res != 0:
            self.log.error(f'MovJ失败: {getattr(response, "res", None)}')
            return False

        deadline = time.monotonic() + timeout
        stable = 0
        while time.monotonic() < deadline:
            current = self.get_tool()
            mode = self.get_mode()
            if mode in (9, 11):
                alarm = self.call(self.errors, GetErrorID.Request(), 1.0)
                codes = self._alarm_codes(alarm)
                hint = '（17=逆运动学无解，请调整目标姿态或预抓取点）' if 17 in codes else ''
                self.log.error(
                    f'机械臂异常 mode={mode}, alarm_codes={codes}{hint}')
                return False
            if current:
                position_error = np.linalg.norm(
                    np.asarray(current[:3]) - np.asarray(pose[:3]))
                # Euler only used as arrival guard; position remains primary.
                rotation_error = np.linalg.norm(
                    (np.asarray(current[3:]) - np.asarray(pose[3:]) + 180) % 360 - 180)
                stable = stable + 1 if position_error < 1.5 and rotation_error < 1.0 else 0
                if stable >= 2:
                    return True
        self.log.error('MovJ等待到位超时')
        return False

    def set_gripper(self, closed, index=1, active_high=True):
        request = ToolDOInstant.Request()
        request.index = int(index)
        request.status = int(bool(closed) == bool(active_high))
        response = self.call(self.tool_do, request, 3.0)
        return response is not None and response.res == 0

    def start_drag(self):
        mode = self.get_mode()
        if self.dragging or mode == self.MODE_BACKDRIVE:
            self.log.warn('机械臂已经在拖拽模式')
            self.dragging = True
            return True
        if mode != self.MODE_ENABLED:
            self.log.error(
                f'不能进入拖拽：RobotMode={mode}，期望ENABLED(mode=5)')
            return False
        response = self.call(self.start_drag_client, StartDrag.Request(), 5.0)
        if response is None or response.res != 0:
            return False
        if not self.wait_mode(self.MODE_BACKDRIVE, timeout=3.0):
            self.log.error(
                f'StartDrag返回成功但状态未进入BACKDRIVE: mode={self.get_mode()}')
            return False
        self.dragging = True
        return True

    def stop_drag(self):
        mode = self.get_mode()
        if not self.dragging and mode != self.MODE_BACKDRIVE:
            self.log.warn(f'当前不在拖拽模式(mode={mode})，仅执行重新使能')
        response = self.call(self.stop_drag_client, StopDrag.Request(), 5.0)
        if response is None or response.res != 0:
            self.log.error(
                f'StopDrag失败: res={getattr(response, "res", None)}')
            return False
        self.dragging = False
        # StopDrag 后控制器回到未使能态；必须重新 Enable 并确认 mode=5。
        enabled = self.call(self.enable, EnableRobot.Request(), 12.0)
        if enabled is None or enabled.res != 0:
            self.log.error(
                f'退出拖拽后EnableRobot失败: res={getattr(enabled, "res", None)}')
            return False
        if not self.apply_motion_safety():
            return False
        if not self.use_base_tool0():
            return False
        if not self.wait_mode(self.MODE_ENABLED, timeout=5.0):
            self.log.error(
                f'EnableRobot后状态异常: mode={self.get_mode()}，期望5')
            return False
        self.log.info('状态切换完成: BACKDRIVE → ENABLED(mode=5)')
        return True
