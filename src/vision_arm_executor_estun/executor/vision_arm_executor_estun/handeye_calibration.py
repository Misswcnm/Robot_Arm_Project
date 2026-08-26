"""Interactive eye-in-hand chessboard calibration for ESTUN robots."""

import argparse
from datetime import datetime, timezone
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from apriltag_pick.camera_topics import camera_topic
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from vision_arm_executor.store import atomic_json

from .calibration_math import (
    handeye_consistency,
    matrix_to_pose,
    pose_delta,
    pose_to_matrix,
    solve_handeye,
)
from .robot import EstunRobot


DEFAULT_ROOT = os.path.expanduser(
    '~/Robot_Arm_Project/data/vision_arm_estun/calibration/handeye')


class EstunHandeyeCalibration(Node):
    def __init__(self, args):
        super().__init__('estun_handeye_calibration')
        self.args = args
        self.robot = EstunRobot(self, speed=args.robot_speed)
        self.image = None
        self.image_sequence = 0
        self.camera_matrix = None
        self.distortion = None
        self.samples = []
        self.run_dir = os.path.join(
            os.path.expanduser(args.output_root), 'runs',
            datetime.now().strftime('handeye_%Y%m%d_%H%M%S'))
        self.image_dir = os.path.join(self.run_dir, 'images')
        os.makedirs(self.image_dir, exist_ok=False)
        self.create_subscription(
            Image, args.image_topic, self._image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, args.camera_info_topic, self._camera_info,
            qos_profile_sensor_data)

    def _image(self, message):
        try:
            height = message.height
            width = message.width
            raw = np.frombuffer(message.data, dtype=np.uint8)
            if message.encoding == 'rgb8':
                image = raw.reshape(height, message.step)[
                    :, :width * 3].reshape(height, width, 3)
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif message.encoding == 'bgr8':
                image = raw.reshape(height, message.step)[
                    :, :width * 3].reshape(height, width, 3)
            else:
                return
            self.image = image.copy()
            self.image_sequence += 1
        except (ValueError, TypeError) as error:
            self.get_logger().warn('invalid camera image: %s' % error)

    def _camera_info(self, message):
        if self.camera_matrix is None:
            self.camera_matrix = np.asarray(
                message.k, dtype=float).reshape(3, 3)
            self.distortion = np.asarray(message.d, dtype=float)

    def wait_camera(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.image is not None and self.camera_matrix is not None:
                return True
            time.sleep(0.05)
        return False

    def wait_robot_pose(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.robot.get_tool(warn=False) is not None:
                return True
            time.sleep(0.1)
        return False

    def _fresh_image(self, timeout=2.0):
        sequence = self.image_sequence
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.image_sequence > sequence:
                return self.image.copy()
            time.sleep(0.02)
        return None

    def _detect_board(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        pattern = (self.args.chess_cols, self.args.chess_rows)
        found, corners = cv2.findChessboardCorners(
            gray, pattern,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK)
        if not found:
            found, corners = cv2.findChessboardCorners(
                gray, pattern,
                cv2.CALIB_CB_ADAPTIVE_THRESH |
                cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            raise RuntimeError('chessboard not detected')
        corners = cv2.cornerSubPix(
            gray, corners, (5, 5), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
             30, 0.001))
        object_points = np.zeros(
            (self.args.chess_cols * self.args.chess_rows, 3), np.float32)
        object_points[:, :2] = np.mgrid[
            0:self.args.chess_cols,
            0:self.args.chess_rows,
        ].T.reshape(-1, 2) * self.args.square_mm
        success, rotation_vector, translation = cv2.solvePnP(
            object_points, corners, self.camera_matrix, self.distortion,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            raise RuntimeError('solvePnP failed')
        projected, unused = cv2.projectPoints(
            object_points, rotation_vector, translation,
            self.camera_matrix, self.distortion)
        error = np.linalg.norm(
            corners.reshape(-1, 2) - projected.reshape(-1, 2), axis=1)
        reprojection = float(np.sqrt(np.mean(np.square(error))))
        distance = float(np.linalg.norm(translation))
        if reprojection > self.args.max_reprojection_px:
            raise RuntimeError(
                'reprojection error %.3f px exceeds %.3f px' %
                (reprojection, self.args.max_reprojection_px))
        if not self.args.min_board_distance_mm <= distance <= \
                self.args.max_board_distance_mm:
            raise RuntimeError('board distance %.1f mm is out of range' % distance)
        rotation, unused = cv2.Rodrigues(rotation_vector)
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation.reshape(3)
        return transform, corners, reprojection, distance

    def capture(self):
        before = self.robot.get_tool(warn=True)
        if before is None:
            raise RuntimeError('ESTUN flange pose unavailable before image')
        image = self._fresh_image()
        if image is None:
            raise RuntimeError('no fresh camera image in 2 seconds')
        camera_board, corners, reprojection, distance = \
            self._detect_board(image)
        after = self.robot.get_tool(warn=True)
        if after is None:
            raise RuntimeError('ESTUN flange pose unavailable after image')
        translation, rotation = pose_delta(before, after)
        if (translation > self.args.max_capture_motion_mm or
                rotation > self.args.max_capture_motion_deg):
            raise RuntimeError(
                'robot moved during capture: %.2f mm / %.3f deg' %
                (translation, rotation))
        index = len(self.samples) + 1
        visualization = image.copy()
        cv2.drawChessboardCorners(
            visualization,
            (self.args.chess_cols, self.args.chess_rows), corners, True)
        image_path = os.path.join(
            self.image_dir, 'calibration_%02d.jpg' % index)
        cv2.imwrite(image_path, visualization)
        sample = {
            'index': index,
            'flange_pose': [float(value) for value in after],
            'T_robot': pose_to_matrix(after),
            'T_camera_board': camera_board,
            'reprojection_rmse_px': reprojection,
            'board_distance_mm': distance,
            'capture_motion_mm': translation,
            'capture_motion_deg': rotation,
            'image_path': image_path,
        }
        self.samples.append(sample)
        return sample

    def solve_and_save(self):
        robot = [sample['T_robot'] for sample in self.samples]
        camera = [sample['T_camera_board'] for sample in self.samples]
        handeye, pair_count = solve_handeye(robot, camera)
        quality = handeye_consistency(robot, camera, handeye)
        pose = matrix_to_pose(handeye)
        payload = {
            'schema_version': 2,
            'robot_model': 'ESTUN',
            'method': 'interactive_drag_chessboard_AX_XB',
            'created_at': datetime.now(timezone.utc).isoformat(),
            'coordinate_frames': {
                'robot_pose': 'T_base_flange_from_estun_codroid_get_pose',
                'camera_extrinsic': 'T_flange_camera',
                'chain': (
                    'T_base_target = T_base_flange * '
                    'T_flange_camera * T_camera_target'),
            },
            'chessboard': {
                'size': [self.args.chess_cols, self.args.chess_rows],
                'square_mm': self.args.square_mm,
            },
            'num_frames': len(self.samples),
            'num_pairs': pair_count,
            'quality': quality,
            'T_flange_camera': {
                'xyz_mm': pose[:3],
                'rpy_deg': pose[3:6],
                'matrix': handeye.tolist(),
            },
            'raw_frames': [{
                'index': sample['index'],
                'flange_xyzrpy': sample['flange_pose'],
                'T_robot_4x4': sample['T_robot'].tolist(),
                'T_cam_4x4': sample['T_camera_board'].tolist(),
                'image_path': sample['image_path'],
                'quality': {
                    'reproj_rmse': sample['reprojection_rmse_px'],
                    'dist': sample['board_distance_mm'],
                    'pose_delta_mm': sample['capture_motion_mm'],
                    'pose_delta_deg': sample['capture_motion_deg'],
                },
            } for sample in self.samples],
        }
        result_path = os.path.join(self.run_dir, 'handeye_result.json')
        atomic_json(result_path, payload)
        if self.args.activate:
            if (quality['board_world_rms_mm'] >
                    self.args.max_board_world_rms_mm or
                    quality['board_world_rotation_rms_deg'] >
                    self.args.max_board_world_rotation_rms_deg):
                raise ValueError(
                    'quality gate failed; run saved but not activated: '
                    'translation RMS %.3f/%.3f mm, rotation RMS %.3f/%.3f deg' %
                    (quality['board_world_rms_mm'],
                     self.args.max_board_world_rms_mm,
                     quality['board_world_rotation_rms_deg'],
                     self.args.max_board_world_rotation_rms_deg))
            active_path = os.path.join(
                os.path.expanduser(self.args.output_root), 'active.json')
            atomic_json(active_path, os.path.abspath(result_path))
            self.get_logger().info('activated hand-eye: %s' % active_path)
        return result_path, payload


def parser():
    result = argparse.ArgumentParser(
        description='ESTUN eye-in-hand chessboard calibration')
    result.add_argument('--output-root', default=DEFAULT_ROOT)
    result.add_argument('--activate', action='store_true')
    result.add_argument('--image-topic',
                        default=camera_topic('color/image_raw'))
    result.add_argument('--camera-info-topic',
                        default=camera_topic('color/camera_info'))
    result.add_argument('--chess-cols', type=int, default=8)
    result.add_argument('--chess-rows', type=int, default=11)
    result.add_argument('--square-mm', type=float, default=20.0)
    result.add_argument('--robot-speed', type=int, default=15)
    result.add_argument('--max-reprojection-px', type=float, default=0.5)
    result.add_argument('--min-board-distance-mm', type=float, default=150.0)
    result.add_argument('--max-board-distance-mm', type=float, default=2000.0)
    result.add_argument('--max-capture-motion-mm', type=float, default=0.8)
    result.add_argument('--max-capture-motion-deg', type=float, default=0.15)
    result.add_argument('--max-board-world-rms-mm', type=float, default=10.0)
    result.add_argument(
        '--max-board-world-rotation-rms-deg', type=float, default=2.0)
    return result


def print_help():
    print('d=释放控制并进入手动拖动  e=恢复CPOS  s=采样  u=撤销')
    print('l=列出  c=计算并保存  h=帮助  q=退出')


def main(args=None):
    options, ros_args = parser().parse_known_args(args)
    rclpy.init(args=ros_args)
    node = EstunHandeyeCalibration(options)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        if not node.wait_camera():
            raise RuntimeError('camera image/CameraInfo unavailable')
        if not node.wait_robot_pose():
            raise RuntimeError('ESTUN Codroid get_pose has no valid flange pose')
        print('输出目录: %s' % node.run_dir)
        print('棋盘格在整个采样过程中必须固定，至少采集 5 个明显不同姿态。')
        print_help()
        while rclpy.ok():
            command = input('\n手眼[%d]> ' % len(node.samples)).strip().lower()
            if command == 'd':
                print('已释放控制，请在示教器进入手动拖动。'
                      if node.robot.start_drag() else '释放控制失败。')
            elif command == 'e':
                print('已恢复自动模式。' if node.robot.stop_drag() else '恢复失败。')
            elif command == 's':
                try:
                    sample = node.capture()
                    print('记录 #%d PnP=%.3fpx 距离=%.0fmm' % (
                        sample['index'], sample['reprojection_rmse_px'],
                        sample['board_distance_mm']))
                except RuntimeError as error:
                    print('未记录: %s' % error)
            elif command == 'u':
                if node.samples:
                    print('撤销 #%d' % node.samples.pop()['index'])
            elif command == 'l':
                for sample in node.samples:
                    print('%02d xyz=%s PnP=%.3fpx' % (
                        sample['index'],
                        np.round(sample['flange_pose'][:3], 2).tolist(),
                        sample['reprojection_rmse_px']))
            elif command == 'c':
                try:
                    path, payload = node.solve_and_save()
                    print('已保存: %s' % path)
                    print('T_flange_camera xyz=%s rpy=%s' % (
                        np.round(payload['T_flange_camera']['xyz_mm'], 3),
                        np.round(payload['T_flange_camera']['rpy_deg'], 3)))
                    print('一致性 RMS=%.3fmm' %
                          payload['quality']['board_world_rms_mm'])
                except ValueError as error:
                    print('无法解算: %s' % error)
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
