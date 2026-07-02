#!/usr/bin/env python3
"""ICP Visual Servoing — entry point.
RobotNode(服务+ToolVectorActual) + ServoNode(点云) 分离, 避免点云阻塞服务调用."""
import sys, os, json, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from icp_servoing.robot import CR5Robot
from icp_servoing.handeye import load_X
from icp_servoing.servoing import VisualServo
from icp_servoing.pointcloud import cloud_to_xyz

PERTURB = {
    '1': {'label': '小-', 'delta': [-5,  0,  0, -3,  0,  0]},
    '2': {'label': '中-', 'delta': [-10, -5,  0, -5, -3,  0]},
    '3': {'label': '大-', 'delta': [-15, -8,  0, -8, -5, -3]},
    '4': {'label': '小+', 'delta': [ 5,  0,  0,  3,  0,  0]},
    '5': {'label': '中+', 'delta': [ 10,  5,  0,  5,  3,  0]},
    '6': {'label': '大+', 'delta': [ 15,  8,  0,  8,  5,  3]},
}


class PointCloudNode(Node):
    """只订阅点云, 轻量回调: 保存最新numpy + seq"""
    def __init__(self):
        super().__init__('servo_pointcloud')
        self.latest_pc = None
        self._pc_seq = -1
        self._sub = self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self._cb, 10)
    def _cb(self, msg):
        self.latest_pc = cloud_to_xyz(msg)
        self._pc_seq += 1
    def wait_fresh(self, timeout: float = 2.0):
        """等新点云帧 (seq号大于当前)"""
        t0 = time.time()
        target = self._pc_seq + 1
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._pc_seq >= target:
                return True
        return False


def main():
    rclpy.init()

    # RobotNode: 只服务+ToolVectorActual, 不订阅点云 → _call不被点云阻塞
    robot_node = Node('cr5_robot_control')
    # PCNode: 只订阅点云, 轻量回调
    pc_node = PointCloudNode()

    X = load_X()
    robot = CR5Robot(robot_node, speed=15)
    servo = VisualServo(robot, X, compensation_ratio=0.7)
    servo._pc_node = pc_node  # 让 servo 能读到点云

    print('\n' + '=' * 55)
    print('  ICP Visual Servoing')
    print('  r=模板  m=单步  a=闭环对齐  1-6=偏移  h=home  H=记录home  q=退出')
    print('=' * 55)

    robot.init()

    # Check camera
    for _ in range(10):
        rclpy.spin_once(pc_node, timeout_sec=0.1)
        if pc_node.latest_pc is not None and len(pc_node.latest_pc) > 500:
            print(f'✅ 相机OK ({len(pc_node.latest_pc)}点)')
            break
    else:
        print('⚠️  未收到点云!')

    try:
        while rclpy.ok():
            c = input('\n🎯 ').strip().lower()
            if c == 'q': break
            elif c == 'r':
                servo.record_template(n_frames=5)
            elif c == 'm':
                servo.step()
            elif c == 'a':
                servo.align(max_iters=15)
            elif c == 'h':
                path = os.path.expanduser('~/Robot_Arm_Project/scripts/home_pose.json')
                if os.path.exists(path):
                    with open(path) as f: home_j = json.load(f)
                    print(f'▶ home: J=[{home_j[0]:.1f} {home_j[1]:.1f} ...]')
                    robot.movj(home_j)
                else:
                    print('⚠ scripts/home_pose.json 不存在')
            elif c == 'H':
                j = robot.get_joints()
                if j:
                    path = os.path.expanduser('~/Robot_Arm_Project/scripts/home_pose.json')
                    with open(path, 'w') as f: json.dump(j, f)
                    print(f'✅ home已记录: J=[{j[0]:.1f} {j[1]:.1f} {j[2]:.1f} {j[3]:.1f} {j[4]:.1f} {j[5]:.1f}]')
            elif c in PERTURB:
                p = PERTURB[c]
                j = robot.get_joints()
                if j:
                    target = [j[i] + p['delta'][i] for i in range(6)]
                    print(f'  {p["label"]}: ΔJ={p["delta"]}')
                    if robot.movj(target):
                        new_j = robot.get_joints()
                        if new_j:
                            dd = [new_j[i]-j[i] for i in range(6)]
                            print(f'  实际ΔJ=[{dd[0]:+.2f} {dd[1]:+.2f} {dd[2]:+.2f} '
                                  f'{dd[3]:+.2f} {dd[4]:+.2f} {dd[5]:+.2f}]°')
                    else:
                        print('  ❌ 移动失败')
            else:
                print('r=模板 m=单步 a=对齐 1-6=偏移 h=home q=退出')
    except KeyboardInterrupt: pass

    robot_node.destroy_node()
    pc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
