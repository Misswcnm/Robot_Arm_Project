#!/usr/bin/env python3
"""ICP 回正能力边界: 3档偏移×3轮, 连续3败=边界, 连续3成=晋级."""
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation as Rot

from icp_servoing.robot import CR5Robot
from icp_servoing.handeye import load_X
from icp_servoing.servoing import VisualServo
from icp_servoing.pointcloud import cloud_to_xyz

LEVELS = [
    {'name': '小', 'delta_j': [-5,  0,  0, -3,  0,  0]},
    {'name': '中', 'delta_j': [-10, -5,  0, -5, -3,  0]},
    {'name': '大', 'delta_j': [-15, -8,  0, -8, -5, -3]},
]
CONVERGE_OK = 10.0   # 残差<10mm=成功
TRIALS_PER = 3        # 每级测试次数

class TestNode(Node):
    def __init__(self):
        super().__init__('icp_range_test')
        self._pc = None
        self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self._cb, 10)
    def _cb(self, m): self._pc = cloud_to_xyz(m)


def main():
    rclpy.init()
    node = TestNode()
    X = load_X()
    robot = CR5Robot(node, speed=15)
    servo = VisualServo(robot, X, compensation_ratio=0.7)
    robot.init()

    print('\n' + '='*55)
    print('  ICP 回正边界测试 (3档×3轮)')
    print('='*55)

    # ── Template ──
    if not servo.record_template(n_frames=5):
        print('❌'); node.destroy_node(); rclpy.shutdown(); return
    P0 = robot.get_tool()
    J0 = robot.get_joints()
    print(f'基准: [{P0[0]:.0f} {P0[1]:.0f} {P0[2]:.0f}]')
    input('\n⚠ 按 Enter 开始...')

    results = {}
    for li, lv in enumerate(LEVELS):
        name = lv['name']
        print(f'\n{"="*55}\n  Level {li+1}: {name}偏移 {lv["delta_j"]}\n{"="*55}')
        ok_cnt, fail_cnt = 0, 0
        level_errs = []

        for ti in range(TRIALS_PER):
            # ── 回基准 ──
            robot.movj(J0)
            robot.wait_stop()

            # ── 关节偏移 ──
            j = robot.get_joints()
            if j is None: print('  ❌ GetAngle'); continue
            target = [j[i] + lv['delta_j'][i] for i in range(6)]
            robot.movj(target)
            robot.wait_stop()

            # ── ICP 回正 ──
            print(f'  [{ti+1}/{TRIALS_PER}] ICP回正...')
            converged = False
            for _ in range(8):
                r = servo.step()
                if not r.get('ok'): break
                robot.wait_stop()
                if r.get('converged'):
                    converged = True; break

            P = robot.get_tool()
            err = np.linalg.norm(np.array(P[:3]) - np.array(P0[:3])) if P else 999
            ok = err < CONVERGE_OK
            level_errs.append(err)

            tag = '✅' if ok else '❌'
            if ok: ok_cnt += 1; fail_cnt = 0
            else:  fail_cnt += 1; ok_cnt = 0
            print(f'    {tag} 残差={err:.1f}mm  (连续OK:{ok_cnt} 连续FAIL:{fail_cnt})')

        results[name] = {'errs': level_errs, 'mean': np.mean(level_errs),
                         'ok': ok_cnt, 'fail': fail_cnt}
        print(f'  → {name}: 均值{np.mean(level_errs):.1f}mm  OK:{ok_cnt}/{TRIALS_PER}')

    # ── Summary ──
    print(f'\n{"="*55}\n📊 汇总\n{"="*55}')
    for name, r in results.items():
        status = '✅' if r['ok'] >= TRIALS_PER else ('⚠' if r['ok'] > 0 else '❌')
        print(f'  {name}: {status} 均值={r["mean"]:.1f}mm ({r["ok"]}/{TRIALS_PER})')
    robot.movj(J0); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
