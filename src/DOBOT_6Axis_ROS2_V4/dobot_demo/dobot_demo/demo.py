#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot CR5 Demo — 修复版
- 跳过 PowerOn（CR5 会因此下电）
- MovJ/MovL 使用位置参数格式（driver 已改成新格式）
"""

import sys
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import *
import time


class AdderClient(Node):
    def __init__(self, name):
        super().__init__(name)
        self.EnableRobot_l = self.create_client(EnableRobot, '/dobot_bringup_ros2/srv/EnableRobot')
        self.DisableRobot_l = self.create_client(DisableRobot, '/dobot_bringup_ros2/srv/DisableRobot')
        self.ClearError_l = self.create_client(ClearError, '/dobot_bringup_ros2/srv/ClearError')
        self.MovJ_l = self.create_client(MovJ, '/dobot_bringup_ros2/srv/MovJ')
        self.SpeedFactor_l = self.create_client(SpeedFactor, '/dobot_bringup_ros2/srv/SpeedFactor')
        self.MovL_l = self.create_client(MovL, '/dobot_bringup_ros2/srv/MovL')
        self.DO_l = self.create_client(DO, '/dobot_bringup_ros2/srv/DO')
        while not self.EnableRobot_l.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def initialization(self):  # 初始化（CR5: 不调 PowerOn！）
        # 清错
        response = self.ClearError_l.call_async(ClearError.Request())
        self.spin_until_future_complete(response)
        self.get_logger().info(f'ClearError: {response.result()}')

        # 下使能（确保干净状态）
        response = self.DisableRobot_l.call_async(DisableRobot.Request())
        self.spin_until_future_complete(response)
        self.get_logger().info(f'DisableRobot: {response.result()}')

        # 使能（CR5: EnableRobot 直接使能到 mode=5，耗时 ~4s）
        response = self.EnableRobot_l.call_async(EnableRobot.Request())
        self.spin_until_future_complete(response)
        self.get_logger().info(f'EnableRobot: {response.result()}')

        # ⚠️ CR5 千万不要调 PowerOn！会让机器人下电！

        # 设速度
        spe = SpeedFactor.Request()
        spe.ratio = 10
        response = self.SpeedFactor_l.call_async(spe)
        self.spin_until_future_complete(response)
        self.get_logger().info(f'SpeedFactor: {response.result()}')

        self.get_logger().info('✅ 初始化完成')

    def point(self, Move, X_j1, Y_j2, Z_j3, RX_j4, RY_j5, RZ_j6):  # 运动指令
        if Move == "MovJ":
            P1 = MovJ.Request()
            P1.mode = True
            P1.a = float(X_j1)
            P1.b = float(Y_j2)
            P1.c = float(Z_j3)
            P1.d = float(RX_j4)
            P1.e = float(RY_j5)
            P1.f = float(RZ_j6)
            P1.param_value = ['user=0', 'tool=0']  # CR5 需要 user/tool 参数
            response = self.MovJ_l.call_async(P1)
            self.spin_until_future_complete(response)
            self.get_logger().info(f'MovJ: res={response.result().res}')
        elif Move == "MovL":
            P1 = MovL.Request()
            P1.mode = False  # False=笛卡尔位姿
            P1.a = float(X_j1)
            P1.b = float(Y_j2)
            P1.c = float(Z_j3)
            P1.d = float(RX_j4)
            P1.e = float(RY_j5)
            P1.f = float(RZ_j6)
            P1.param_value = ['user=0', 'tool=0']
            response = self.MovL_l.call_async(P1)
            self.spin_until_future_complete(response)
            self.get_logger().info(f'MovL: res={response.result().res}')
        else:
            print("无该指令")

    def DO(self, index, status):  # IO 控制夹爪/气泵
        DO_V = DO.Request()
        DO_V.index = index
        DO_V.status = status
        response = self.DO_l.call_async(DO_V)
        self.spin_until_future_complete(response)
        self.get_logger().info(f'DO: res={response.result().res}')


def main(args=None):
    rclpy.init(args=args)
    node = AdderClient("dobot_demo")

    # 先初始化
    node.initialization()

    # 测试 MovJ
    node.point("MovJ", 50, -8, 0, 0, 0, 0)
    node.point("MovJ", 0, -8, 0, 0, 0, 0)
    time.sleep(3)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
