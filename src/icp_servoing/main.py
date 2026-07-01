#!/usr/bin/env python3
"""ICP Visual Servoing — entry point.
Usage:
  cd ~/Robot_Arm_Project
  PYTHONPATH=src python3 src/icp_servoing/main.py
"""
import sys, os, json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from src.icp_servoing.pointcloud import cloud_to_xyz, voxel_down
from src.icp_servoing.icp import multi_scale_icp
from src.icp_servoing.robot import CR5Robot
from src.icp_servoing.handeye import load_X
from src.icp_servoing.servoing import VisualServo

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class ServoNode(Node):
    def __init__(self):
        super().__init__('icp_servoing')
        self._pc = None
        self._sub = self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self._cb_pc, 10)

    def _cb_pc(self, msg):
        self._pc = cloud_to_xyz(msg)


PERTURB = {
    '1': {'label': '小-', 'delta': [-5,  0,  0, -3,  0,  0]},
    '2': {'label': '中-', 'delta': [-10, -5,  0, -5, -3,  0]},
    '3': {'label': '大-', 'delta': [-15, -8,  0, -8, -5, -3]},
    '4': {'label': '小+', 'delta': [ 5,  0,  0,  3,  0,  0]},
    '5': {'label': '中+', 'delta': [ 10,  5,  0,  5,  3,  0]},
    '6': {'label': '大+', 'delta': [ 15,  8,  0,  8,  5,  3]},
}


def main():
    rclpy.init()
    node = ServoNode()
    X = load_X()
    robot = CR5Robot(node, speed=15)
    servo = VisualServo(robot, X, compensation_ratio=0.7)

    print('\n' + '=' * 55)
    print('  ICP Visual Servoing')
    print('  r=模板  m=单步ICP  a=闭环对齐  q=退出')
    print('  1/2/3=反向偏移  4/5/6=正向偏移  h=回home  H=记录home')
    print('=' * 55)

    robot.init()

    # Check camera (fast)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node._pc is not None and len(node._pc) > 500:
            print(f'✅ 相机OK ({len(node._pc)}点)')
            break
    else:
        print('⚠️  未收到点云! 确认相机启动')

    try:
        while rclpy.ok():
            c = input('\n🎯 ').strip().lower()
            if c == 'q':
                break
            elif c == 'r':
                servo.record_template(n_frames=8)
            elif c == 'm':
                servo.step()
                robot.wait_stop()
            elif c == 'a':
                servo.align(max_iters=10)
            elif c == 'h':
                path = os.path.expanduser('~/Robot_Arm_Project/scripts/home_pose.json')
                if os.path.exists(path):
                    with open(path) as f:
                        home_j = json.load(f)
                    print(f'▶ 回home: J=[{home_j[0]:.1f} {home_j[1]:.1f} ...]')
                    robot.movj(home_j)
                    robot.wait_stop()
                else:
                    print('⚠ home_pose.json 不存在, 按 H 记录')
            elif c == 'H':
                j = robot.get_joints()
                if j:
                    path = os.path.expanduser('~/Robot_Arm_Project/scripts/home_pose.json')
                    with open(path, 'w') as f:
                        json.dump(j, f)
                    print(f'✅ home已记录: J=[{j[0]:.1f} {j[1]:.1f} {j[2]:.1f} {j[3]:.1f} {j[4]:.1f} {j[5]:.1f}]')
            elif c in ('1', '2', '3', '4', '5', '6'):
                j = robot.get_joints()
                if j is None:
                    print('❌ GetAngle失败')
                    continue
                pb = PERTURB[c]
                dj = pb['delta']
                target = [j[i] + dj[i] for i in range(6)]
                print(f'  {pb["label"]}: ΔJ=[{dj[0]:+d} {dj[1]:+d} {dj[2]:+d} '
                      f'{dj[3]:+d} {dj[4]:+d} {dj[5]:+d}]°')
                if robot.movj(target) and robot.wait_stop():
                    new_j = robot.get_joints()
                    if new_j:
                        dd = [new_j[i]-j[i] for i in range(6)]
                        print(f'  实际ΔJ=[{dd[0]:+.2f} {dd[1]:+.2f} {dd[2]:+.2f} '
                              f'{dd[3]:+.2f} {dd[4]:+.2f} {dd[5]:+.2f}]°')
                else:
                    print('  ❌ 移动失败')
            else:
                print('r=模板 m=单步 a=对齐 1-3=反向 4-6=正向 q=退出')
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
