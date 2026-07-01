"""CR5 robot interface: init, movj, ToolVectorActual, wait_stop."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import (EnableRobot, DisableRobot, ClearError,
                                MovJ, MovL, SpeedFactor, GetAngle, GetPose,
                                SetCollisionLevel, RobotMode)


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
        self.MovL         = node.create_client(MovL,         '/dobot_bringup_ros2/srv/MovL')
        self.SpeedFactor  = node.create_client(SpeedFactor,  '/dobot_bringup_ros2/srv/SpeedFactor')
        self.SetCollision = node.create_client(SetCollisionLevel, '/dobot_bringup_ros2/srv/SetCollisionLevel')
        self.RobotMode    = node.create_client(RobotMode,    '/dobot_bringup_ros2/srv/RobotMode')
        self.GetAngle     = node.create_client(GetAngle,     '/dobot_bringup_ros2/srv/GetAngle')

        for n, c in [('EnableRobot', self.EnableRobot), ('MovJ', self.MovJ)]:
            while not c.wait_for_service(timeout_sec=1.0):
                self._logger.info(f'等待 {n}...')

        self._tool = None
        node.create_subscription(
            ToolVectorActual, '/dobot_msgs_v4/msg/ToolVectorActual',
            lambda m: setattr(self, '_tool', [m.x, m.y, m.z, m.rx, m.ry, m.rz]), 10)

    def _call(self, client, req, timeout=10.0):
        fut = client.call_async(req)
        t0 = time.time()
        # Fast spin: check future every 10ms, skip pointcloud callbacks
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self._node, timeout_sec=0.01)
        if not fut.done():
            fut.cancel()
            return False, 'timeout'
        try:
            return True, fut.result()
        except Exception as e:
            return False, str(e)

    def check_moved(self, pre_tool, min_delta: float = 0.5) -> bool:
        """验证移动. 未动→完整recover (Clear→Disable→Enable→Speed→Collision)"""
        self.wait_stop()
        cur = self.get_tool()
        if cur is None or pre_tool is None: return True
        d = np.linalg.norm(np.array(cur[:3]) - np.array(pre_tool[:3]))
        if d < min_delta:
            self._logger.warn(f'未移动 Δ={d:.1f}mm → 完整恢复')
            self.recover()
            return False
        return True

    # ── Init (全部指令连续发送, recv后立即下一步) ──
    def init(self):
        self._call(self.ClearError, ClearError.Request())
        self._call(self.DisableRobot, DisableRobot.Request())
        self._call(self.EnableRobot, EnableRobot.Request(), timeout=10.0)  # ~4s
        rclpy.spin_once(self._node, timeout_sec=0.1)  # 让topic到位
        s = SpeedFactor.Request(); s.ratio = self._speed; self._call(self.SpeedFactor, s)
        c = SetCollisionLevel.Request(); c.level = 5; self._call(self.SetCollision, c)
        self._logger.info(f'✅ CR5 ready  speed={self._speed}%  collision=Lv.5')

    # ── ToolVectorActual + GetPose fallback ──
    def get_tool(self) -> list | None:
        """真实TCP [x,y,z,rx,ry,rz] mm,deg. 优先ToolVectorActual, 回退GetPose"""
        # 1. Try ToolVectorActual topic (few spins)
        for _ in range(5):
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if self._tool is not None:
                x, y, z = self._tool[:3]
                if abs(x) > 0.5 or abs(y) > 0.5 or abs(z) > 0.5:
                    return list(self._tool)
        # 2. Fallback: GetPose() 不带参数
        self._logger.warn('ToolVectorActual无数据, GetPose()...')
        gp = self._node.create_client(GetPose, '/dobot_bringup_ros2/srv/GetPose')
        if not gp.wait_for_service(timeout_sec=1.0):
            return None
        # Try without params first (older CR5 firmware)
        req = GetPose.Request()
        fut = gp.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time()-t0 < 3.0:
            rclpy.spin_once(self._node, timeout_sec=0.05)
        if fut.done() and fut.result().res == 0:
            try:
                s = fut.result().robot_return.strip('{}')
                vals = [float(v) for v in s.split(',')]
                if len(vals) == 6: return vals
            except: pass
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

    # ── Joint move ──
    def movj(self, joints: list) -> bool:
        req = MovJ.Request(); req.mode = True
        req.a, req.b, req.c = float(joints[0]), float(joints[1]), float(joints[2])
        req.d, req.e, req.f = float(joints[3]), float(joints[4]), float(joints[5])
        req.param_value = ['user=0', 'tool=0']
        ok, r = self._call(self.MovJ, req, timeout=15.0)
        return ok and r.res == 0

    # ── Cartesian move (MovL 直接控制末端) ──
    def movl(self, pose: list) -> bool:
        """pose=[x,y,z,rx,ry,rz] mm,deg (基座帧笛卡尔坐标)"""
        req = MovL.Request(); req.mode = False  # False=笛卡尔
        req.a, req.b, req.c = float(pose[0]), float(pose[1]), float(pose[2])
        req.d, req.e, req.f = float(pose[3]), float(pose[4]), float(pose[5])
        req.param_value = ['user=0', 'tool=0']
        ok, r = self._call(self.MovL, req, timeout=15.0)
        if ok and r.res == 0:
            return True
        self._logger.warn(f'MovL failed: res={r.res if ok else r}')
        return False

    # ── Wait stop (ToolVectorActual连续3次变化<0.3mm & <0.05°) ──
    def wait_stop(self, timeout: float = 5.0) -> bool:
        """轮询ToolVectorActual, 连续3帧稳定→已停止"""
        prev = None; stable = 0
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._tool is None or abs(self._tool[0]) < 0.5:
                time.sleep(0.05)
                continue
            cur = np.array(self._tool[:3])
            if prev is not None:
                d = np.linalg.norm(cur - prev)
                if d < 0.3:
                    stable += 1
                    if stable >= 3:
                        return True
                else:
                    stable = 0
            prev = cur
        return True  # 超时也认为停了

    def check_error(self) -> bool:
        """快速检查ERROR, 是则清错(不完整恢复)"""
        ok, r = self._call(self.RobotMode, RobotMode.Request(), timeout=1.0)
        if ok:
            try:
                if int(r.robot_return.strip('{}')) == 9:
                    self._call(self.ClearError, ClearError.Request())
                    return True
            except: pass
        return False

    def recover(self) -> bool:
        """完整恢复: ClearError→DisableRobot→EnableRobot→SpeedFactor→SetCollisionLevel"""
        self._logger.error('ERROR! 执行完整恢复序列...')
        self._call(self.ClearError, ClearError.Request())
        self._call(self.DisableRobot, DisableRobot.Request())
        ok, _ = self._call(self.EnableRobot, EnableRobot.Request(), timeout=10.0)
        rclpy.spin_once(self._node, timeout_sec=0.1)
        s = SpeedFactor.Request(); s.ratio = self._speed; self._call(self.SpeedFactor, s)
        c = SetCollisionLevel.Request(); c.level = 5; self._call(self.SetCollision, c)
        self._logger.info('✅ 恢复完成')
        return True
