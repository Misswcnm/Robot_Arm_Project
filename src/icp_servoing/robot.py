"""CR5 robot interface: init, movj, ToolVectorActual, wait_stop."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import (EnableRobot, DisableRobot, ClearError,
                                MovJ, SpeedFactor, GetAngle, GetPose,
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
        self.SpeedFactor  = node.create_client(SpeedFactor,  '/dobot_bringup_ros2/srv/SpeedFactor')
        self.SetCollision = node.create_client(SetCollisionLevel, '/dobot_bringup_ros2/srv/SetCollisionLevel')
        self.RobotMode    = node.create_client(RobotMode,    '/dobot_bringup_ros2/srv/RobotMode')
        self.GetAngle     = node.create_client(GetAngle,     '/dobot_bringup_ros2/srv/GetAngle')

        for n, c in [('EnableRobot', self.EnableRobot), ('MovJ', self.MovJ)]:
            while not c.wait_for_service(timeout_sec=1.0):
                self._logger.info(f'等待 {n}...')

        self._tool = None
        self._tool_seq = -1
        def _cb(msg):
            self._tool = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]
            self._tool_seq += 1  # 每次新数据递增
        node.create_subscription(ToolVectorActual, '/dobot_msgs_v4/msg/ToolVectorActual', _cb, 10)

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
        s = SpeedFactor.Request(); s.ratio = self._speed; self._call(self.SpeedFactor, s)
        c = SetCollisionLevel.Request(); c.level = 5; self._call(self.SetCollision, c)
        self._logger.info(f'✅ CR5 ready  speed={self._speed}%  collision=Lv.5')

    # ── ToolVectorActual + GetPose fallback ──
    def get_tool(self) -> list | None:
        """真实TCP [x,y,z,rx,ry,rz] mm,deg. 优先ToolVectorActual, 回退GetPose"""
        for _ in range(3):
            rclpy.spin_once(self._node, timeout_sec=0.02)
            if self._tool is not None and abs(self._tool[0]) > 0.5:
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
                          min_wait: float = 0.15) -> bool:
        """
        ToolVectorActual seq-based 稳定检测.
        只比较不同seq的新帧, 连续stable_required次满足:
          xyz变化<0.3mm 且 rpy变化<0.05°
        """
        time.sleep(min_wait)  # 确保运动已启动
        prev_xyz = None; prev_rpy = None
        last_seq = self._tool_seq - 1
        stable = 0
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self._node, timeout_sec=0.03)
            if self._tool is None or abs(self._tool[0]) < 0.5:
                continue
            if self._tool_seq <= last_seq:
                continue  # 不是新数据, 跳过
            last_seq = self._tool_seq

            cur_xyz = np.array(self._tool[:3])
            cur_rpy = np.array(self._tool[3:6])
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
        return True  # 超时默认已停

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
