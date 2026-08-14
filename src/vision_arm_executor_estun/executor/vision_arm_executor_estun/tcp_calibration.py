"""Interactive ESTUN flange-to-tool-tip pivot calibration."""

import argparse
from datetime import datetime, timezone
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from vision_arm_executor.store import atomic_json

from .calibration_math import PoseSample, solve_pivot
from .robot import EstunRobot


DEFAULT_ROOT = os.path.expanduser(
    '~/Robot_Arm_Project/data/vision_arm_estun/calibration/tcp')


class EstunTcpCalibration(Node):
    def __init__(self, args):
        super().__init__('estun_tcp_calibration')
        self.args = args
        self.robot = EstunRobot(self, speed=args.robot_speed)
        self.samples = []
        self.solution = None
        self.run_dir = os.path.join(
            os.path.expanduser(args.output_root), 'runs',
            datetime.now().strftime('tcp_%Y%m%d_%H%M%S'))
        os.makedirs(self.run_dir, exist_ok=False)

    def current_pose(self):
        """Read and record exactly one controller pose on operator command."""
        pose = self.robot.get_tool(warn=False)
        if pose is None:
            raise RuntimeError('ESTUN flange pose unavailable')
        values = np.asarray(pose, dtype=float).reshape(-1)
        if values.size != 6 or not np.all(np.isfinite(values)):
            raise RuntimeError('ESTUN flange pose is invalid')
        return PoseSample(time.time(), *[float(value) for value in values])

    def record(self):
        sample = self.current_pose()
        self.samples.append(sample)
        self.solution = None
        return sample

    def solve(self):
        tcp, fixed, residuals = solve_pivot(self.samples)
        self.solution = {
            'tcp_offset_flange_mm': tcp.tolist(),
            'fixed_point_base_mm': fixed.tolist(),
            'residual_mean_mm': float(np.mean(residuals)),
            'residual_max_mm': float(np.max(residuals)),
            'residual_std_mm': float(np.std(residuals)),
            'sample_count': len(self.samples),
        }
        return self.solution

    def save(self):
        solution = self.solution or self.solve()
        payload = {
            'schema_version': 2,
            'robot_model': 'ESTUN',
            'created_at': datetime.now(timezone.utc).isoformat(),
            'pose_source': 'estun_codroid/get_pose',
            'coordinate_frames': {
                'robot_pose': 'T_base_flange_from_estun_codroid_get_pose',
                'tcp_offset': (
                    'translation_from_flange_origin_to_physical_tip'),
            },
            'method': 'pivot_calibration',
            'pose_convention': 'xyz mm, XYZ Euler deg',
            'samples': [sample.to_dict() for sample in self.samples],
            'result': solution,
        }
        result_path = os.path.join(self.run_dir, 'tcp_result.json')
        atomic_json(result_path, payload)
        if self.args.activate:
            active_path = os.path.join(
                os.path.expanduser(self.args.output_root), 'active.json')
            atomic_json(active_path, os.path.abspath(result_path))
            self.get_logger().info('activated TCP: %s' % active_path)
        return result_path


def parser():
    result = argparse.ArgumentParser(
        description='ESTUN TCP/tool-tip pivot calibration')
    result.add_argument('--output-root', default=DEFAULT_ROOT)
    result.add_argument('--activate', action='store_true')
    result.add_argument('--robot-speed', type=int, default=15)
    return result


def print_help():
    print('d=释放控制并进入手动拖动  e=恢复CPOS  r=记录当前姿态')
    print('u=撤销  l=列出  s=解算  w=保存  h=帮助  q=退出')


def main(args=None):
    options, ros_args = parser().parse_known_args(args)
    rclpy.init(args=ros_args)
    node = EstunTcpCalibration(options)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        print('让夹爪实际作业尖端始终接触同一个固定点，改变至少 4 个姿态。')
        print('推荐 12~20 个姿态，并覆盖多个方向。')
        print_help()
        while rclpy.ok():
            command = input('\nTCP[%d]> ' % len(node.samples)).strip().lower()
            if command == 'd':
                print('已释放控制，请在示教器进入手动拖动。'
                      if node.robot.start_drag() else '释放控制失败。')
            elif command == 'e':
                print('已恢复CPOS。' if node.robot.stop_drag() else '恢复失败。')
            elif command == 'r':
                try:
                    sample = node.record()
                    print('记录 #%d xyz=%s rpy=%s' % (
                        len(node.samples),
                        np.round(sample.values[:3], 3).tolist(),
                        np.round(sample.values[3:6], 3).tolist()))
                except RuntimeError as error:
                    print('未记录: %s' % error)
            elif command == 'u':
                if node.samples:
                    node.samples.pop()
                    node.solution = None
                    print('已撤销上一条。')
            elif command == 'l':
                for index, sample in enumerate(node.samples, 1):
                    print('%02d %s' % (
                        index, np.round(sample.values, 3).tolist()))
            elif command == 's':
                try:
                    solution = node.solve()
                    print('tcp_offset_flange_mm=%s' % np.round(
                        solution['tcp_offset_flange_mm'], 3).tolist())
                    print('residual mean/max=%.3f/%.3f mm' % (
                        solution['residual_mean_mm'],
                        solution['residual_max_mm']))
                except ValueError as error:
                    print('无法解算: %s' % error)
            elif command == 'w':
                try:
                    print('已保存: %s' % node.save())
                except ValueError as error:
                    print('无法保存: %s' % error)
            elif command == 'h' or not command:
                print_help()
            elif command == 'q':
                break
            else:
                print_help()
    finally:
        if node.robot.dragging:
            node.robot.stop_drag()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
