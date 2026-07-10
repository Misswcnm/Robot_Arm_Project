import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from .robot import CR5Robot
from .transforms import (
    load_handeye, load_tcp_offset, pose_matrix, tool_pose_matrix)


class AprilTagPickNode(Node):
    def __init__(self):
        super().__init__('apriltag_pick')
        p = self.declare_parameter
        self.tag_frame = p('tag_frame', 'tag36h11:0').value
        self.handeye_path = p(
            'handeye_path',
            '/home/ylx/Robot_Arm_Project/scripts/handeye_chessboard_result.json').value
        self.tcp_calibration_path = p(
            'tcp_calibration_path',
            '/home/ylx/Robot_Arm_Project/scripts/tcp_calibration/'
            'tcp_calibration_20260706_150548.json').value
        self.execute_enabled = bool(p('execute_enabled', False).value)
        self.max_detection_age = float(p('max_detection_age_sec', 1.0).value)
        self.sample_window = float(p('sample_window_sec', 2.0).value)
        self.samples = int(p('samples', 10).value)
        self.max_sample_spread_mm = float(p('max_sample_spread_mm', 8.0).value)
        self.pregrasp_height = float(p('pregrasp_height_mm', 80.0).value)
        self.lift_height = float(p('lift_height_mm', 100.0).value)
        self.min_base_z = float(p('min_base_z_mm', 20.0).value)
        self.max_reach = float(p('max_base_radius_mm', 850.0).value)
        self.gripper_enabled = bool(p('gripper.enabled', False).value)
        self.gripper_index = int(p('gripper.index', 1).value)
        self.gripper_active_high = bool(p('gripper.active_high', True).value)
        tag_tcp_xyz = p('tag_to_tcp.xyz_mm', [0.0, 0.0, 0.0]).value
        tag_tcp_rpy = p('tag_to_tcp.rpy_deg', [180.0, 0.0, 0.0]).value
        self.tag_to_tcp = pose_matrix(tag_tcp_xyz, rpy_deg=tag_tcp_rpy)
        self.camera_in_tool = load_handeye(self.handeye_path)
        self.tcp_offset_tool = load_tcp_offset(self.tcp_calibration_path)
        self.robot = CR5Robot(self, speed=int(p('robot_speed', 15).value))
        self.latest = None
        self.history = []
        self.lock = threading.Lock()
        self.create_subscription(TFMessage, '/tf', self._tf_callback, 20)
        self.get_logger().info(
            f'等待 {self.tag_frame}; execute_enabled={self.execute_enabled}')

    def _tf_callback(self, message):
        for item in message.transforms:
            if item.child_frame_id != self.tag_frame:
                continue
            t, q = item.transform.translation, item.transform.rotation
            transform = pose_matrix(
                [t.x * 1000.0, t.y * 1000.0, t.z * 1000.0],
                quat_xyzw=[q.x, q.y, q.z, q.w])
            stamp = item.header.stamp.sec + item.header.stamp.nanosec * 1e-9
            with self.lock:
                self.latest = (stamp, transform)
                self.history.append((stamp, transform))
                self.history = self.history[-max(30, self.samples * 3):]

    def locate(self):
        tool = self.robot.get_tool()
        if tool is None:
            raise RuntimeError('GetPose无有效位姿')
        now = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            history = list(self.history)
        if not history:
            raise RuntimeError('尚未检测到Tag')
        newest_stamp = history[-1][0]
        age = now - newest_stamp
        if age > self.max_detection_age:
            raise RuntimeError(
                f'Tag已过期: age={age:.2f}s > {self.max_detection_age:.2f}s')
        # One command performs one localization. Reuse up to N recent frames
        # for robustness, but never wait for a fixed frame count.
        recent = [
            transform for stamp, transform in history
            if newest_stamp - stamp <= self.sample_window
        ][-self.samples:]
        translations = np.asarray([item[:3, 3] for item in recent])
        center = np.median(translations, axis=0)
        spread = float(np.max(np.linalg.norm(translations - center, axis=1)))
        if spread > self.max_sample_spread_mm:
            raise RuntimeError(
                f'Tag检测抖动过大: {spread:.1f}mm > {self.max_sample_spread_mm:.1f}mm')

        # Robust translation median; use the pose nearest that median for rotation.
        camera_tag = min(recent, key=lambda item: np.linalg.norm(item[:3, 3] - center)).copy()
        camera_tag[:3, 3] = center
        base_tool = tool_pose_matrix(tool)
        base_tag = base_tool @ self.camera_in_tool @ camera_tag
        # The pivot calibration defines tip position, not tip orientation.
        # Keep the manually selected current tool orientation; only take the
        # desired contact position from Tag + configured positional offset.
        tag_tip = base_tag @ self.tag_to_tcp
        base_tip = base_tool.copy()
        base_tip[:3, 3] = tag_tip[:3, 3]
        base_tool_target = base_tip.copy()
        base_tool_target[:3, 3] -= (
            base_tool_target[:3, :3] @ self.tcp_offset_tool)
        xyz = base_tool_target[:3, 3]
        radius = float(np.linalg.norm(xyz[:2]))
        if xyz[2] < self.min_base_z or radius > self.max_reach:
            raise RuntimeError(
                f'目标越过安全边界: xyz={xyz.round(1).tolist()}, radius={radius:.1f}')
        self.get_logger().info(
            f'Tag base xyz={base_tag[:3, 3].round(1).tolist()} mm; '
            f'tip target={base_tip[:3, 3].round(1).tolist()} mm; '
            f'tool target={xyz.round(1).tolist()} mm; '
            f'frames={len(recent)}, spread={spread:.1f} mm')
        return base_tool_target

    def retract_pose(self, tool_target, distance_mm):
        """Retract the calibrated tip backward along the tool-to-tip axis."""
        axis_tool = self.tcp_offset_tool / np.linalg.norm(self.tcp_offset_tool)
        axis_base = tool_target[:3, :3] @ axis_tool
        retracted = tool_target.copy()
        retracted[:3, 3] -= axis_base * float(distance_mm)
        return retracted

    def pick(self):
        target = self.locate()
        if not self.execute_enabled:
            raise RuntimeError('当前是定位模式；将 execute_enabled 改为 true 才允许运动')
        pregrasp = self.retract_pose(target, self.pregrasp_height)
        if self.gripper_enabled and not self.robot.set_gripper(
                False, self.gripper_index, self.gripper_active_high):
            raise RuntimeError('夹爪打开失败')
        if not self.robot.move_pose(pregrasp):
            raise RuntimeError('预抓取位姿失败')
        # The base-frame target is locked from the first observation. An
        # eye-in-hand camera commonly loses/occludes the tag near contact, so
        # the contact move must not depend on a second detection.
        if not self.robot.move_pose(target):
            raise RuntimeError('抓取位姿失败')
        if self.gripper_enabled and not self.robot.set_gripper(
                True, self.gripper_index, self.gripper_active_high):
            raise RuntimeError('夹爪闭合失败')
        time.sleep(0.3)
        lift = self.retract_pose(target, self.lift_height)
        if not self.robot.move_pose(lift):
            raise RuntimeError('抬升失败')
        self.get_logger().info('抓取流程完成')


class CommandReader:
    """Read from the controlling terminal even when ros2 launch replaces stdin."""

    def __init__(self):
        self.terminal = None
        try:
            self.terminal = open('/dev/tty', 'r', encoding='utf-8')
        except OSError:
            # Direct `ros2 run` / python execution follows icp_servoing's input().
            pass

    def read(self, prompt):
        if self.terminal is None:
            return input(prompt)
        print(prompt, end='', flush=True)
        line = self.terminal.readline()
        if line == '':
            raise EOFError('终端输入已关闭')
        return line.rstrip('\n')

    def close(self):
        if self.terminal is not None:
            self.terminal.close()


def _console(node):
    reader = CommandReader()
    print('\nAprilTag Pick: l=定位  t=拖拽找视野  p=抓取  q=退出')
    try:
        while rclpy.ok():
            try:
                command = reader.read('apriltag> ').strip().lower()
                if command == 'l':
                    node.locate()
                elif command == 't':
                    if not node.robot.start_drag():
                        raise RuntimeError('进入拖拽模式失败')
                    print('拖拽中：移动机械臂寻找 Tag；l=查看定位，2=退出拖拽')
                    try:
                        while rclpy.ok():
                            drag_command = reader.read('drag> ').strip().lower()
                            if drag_command == 'l':
                                try:
                                    node.locate()
                                except Exception as exc:
                                    node.get_logger().error(str(exc))
                            elif drag_command == '2':
                                if not node.robot.stop_drag():
                                    node.get_logger().error(
                                        '退出拖拽或重新使能失败，仍停留在拖拽菜单；'
                                        '禁止发送运动指令')
                                    continue
                                print('已退出拖拽，机械臂已重新使能')
                                break
                            else:
                                print('l=查看定位，2=退出拖拽')
                    except (KeyboardInterrupt, EOFError):
                        node.robot.stop_drag()
                        raise
                elif command == 'p':
                    node.pick()
                elif command == 'q':
                    rclpy.shutdown()
                    return
                else:
                    print('l=定位  t=拖拽找视野  p=抓取  q=退出')
            except Exception as exc:
                node.get_logger().error(str(exc))
    finally:
        reader.close()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPickNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.robot.initialize()
        _console(node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
