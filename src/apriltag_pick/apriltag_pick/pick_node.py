import json
import os
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from .robot import CR5Robot
from .transforms import (
    load_handeye, load_handeye_document, load_tcp_offset, pose_matrix,
    tool_pose_matrix)


class AprilTagPickNode(Node):
    def __init__(self):
        super().__init__('apriltag_pick')
        p = self.declare_parameter
        self.tag_frame = p('tag_frame', 'tag36h11:0').value
        self.tag_size_m = float(p('tag_size_m', 0.15).value)
        self.handeye_path = p(
            'handeye_path',
            '/home/ylx/Robot_Arm_Project/scripts/'
            'active_handeye_calibration.json').value
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
        self.min_base_z = float(p('min_base_z_mm', 20.0).value)
        self.max_reach = float(p('max_base_radius_mm', 850.0).value)
        self.gripper_enabled = bool(p('gripper.enabled', False).value)
        self.gripper_index = int(p('gripper.index', 1).value)
        self.gripper_active_high = bool(p('gripper.active_high', True).value)
        self.debug_output_dir = os.path.expanduser(p(
            'debug_output_dir',
            '~/Robot_Arm_Project/scripts/apriltag_validation_runs').value)
        tag_tcp_xyz = p('tag_to_tcp.xyz_mm', [0.0, 0.0, 0.0]).value
        tag_tcp_rpy = p('tag_to_tcp.rpy_deg', [180.0, 0.0, 0.0]).value
        self.tag_to_tcp = pose_matrix(tag_tcp_xyz, rpy_deg=tag_tcp_rpy)
        self.flange_from_camera = load_handeye(self.handeye_path)
        self.tcp_offset_flange = load_tcp_offset(self.tcp_calibration_path)
        handeye_frame_ok = self._uses_base_flange_handeye(self.handeye_path)
        tcp_frame_ok = self._uses_flange_tcp(self.tcp_calibration_path)
        invalid_frame_files = []
        if not handeye_frame_ok:
            invalid_frame_files.append(f'手眼: {self.handeye_path}')
        if not tcp_frame_ok:
            invalid_frame_files.append(f'TCP: {self.tcp_calibration_path}')
        if invalid_frame_files:
            self.execute_enabled = False
            self.get_logger().error(
                '以下标定文件的坐标契约不完整，已强制关闭自动运动：\n  - ' +
                '\n  - '.join(invalid_frame_files) +
                '\n手眼必须声明 T_base_flange(User0) 与 T_flange_camera；'
                'TCP 文件必须声明其法兰偏移约定。')
        self.robot = CR5Robot(self, speed=int(p('robot_speed', 15).value))
        self.latest = None
        self.history = []
        self.image_history = []
        self.detection_history = []
        self.camera_info = None
        self.last_localization = None
        self.localization_cache = []
        self.localization_index = 0
        self.manual_tag_center = None
        self.tag_diagnostic_samples = []
        self.tag_diagnostic_save_index = 0
        self.debug_save_index = 0
        self.lock = threading.Lock()
        self.create_subscription(TFMessage, '/tf', self._tf_callback, 20)
        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._image_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self._camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(
            AprilTagDetectionArray, '/detections', self._detection_callback, 10)
        self.get_logger().info(
            f'等待 {self.tag_frame}; execute_enabled={self.execute_enabled}')

    @staticmethod
    def _uses_flange_tcp(path):
        try:
            _, data = load_handeye_document(path)
            frames = data.get('coordinate_frames', {})
            contract = str(frames.get('tcp_offset', '')).lower()
            return (frames.get('user_index') == 0 and
                    ('flange' in contract or frames.get('tool_index') == 0))
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            return False

    @staticmethod
    def _uses_base_flange_handeye(path):
        try:
            _, data = load_handeye_document(path)
            frames = data.get('coordinate_frames', {})
            has_flange_contract = (
                frames.get('camera_extrinsic') == 'T_flange_camera' or
                'flange' in str(frames.get('robot_pose', '')).lower())
            legacy_tool0_flange = frames.get('tool_index') == 0
            return (frames.get('user_index') == 0 and
                    (has_flange_contract or legacy_tool0_flange))
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            return False

    @staticmethod
    def _error_text(delta):
        delta = np.asarray(delta, dtype=float)
        return (f'Δ=[{delta[0]:+.2f}, {delta[1]:+.2f}, {delta[2]:+.2f}]mm  '
                f'|Δ|={np.linalg.norm(delta):.2f}mm')

    def _physical_tip_position(self, base_flange):
        """Compute physical-tip position from T_base_flange and flange offset."""
        return (base_flange[:3, 3] +
                base_flange[:3, :3] @ self.tcp_offset_flange)

    def _tcp_position_variants(self, base_flange):
        """Return both TCP sign hypotheses alongside the raw GetPose origin."""
        offset_base = base_flange[:3, :3] @ self.tcp_offset_flange
        flange_origin = base_flange[:3, 3].copy()
        return {
            'flange_origin_getpose': flange_origin,
            'tip_plus': flange_origin + offset_base,
            'tip_minus': flange_origin - offset_base,
            'offset_base': offset_base,
        }

    @staticmethod
    def _rotation_distance_deg(first, second):
        relative = first[:3, :3].T @ second[:3, :3]
        cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
        return float(np.degrees(np.arccos(cosine)))

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

    @staticmethod
    def _message_stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def _image_callback(self, message):
        try:
            height, width = message.height, message.width
            data = np.frombuffer(message.data, dtype=np.uint8)
            if message.encoding in ('rgb8', 'bgr8'):
                image = data.reshape(height, message.step)[:, :width * 3]
                image = image.reshape(height, width, 3).copy()
                if message.encoding == 'rgb8':
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif message.encoding in ('rgba8', 'bgra8'):
                image = data.reshape(height, message.step)[:, :width * 4]
                image = image.reshape(height, width, 4).copy()
                code = (cv2.COLOR_RGBA2BGR if message.encoding == 'rgba8'
                        else cv2.COLOR_BGRA2BGR)
                image = cv2.cvtColor(image, code)
            elif message.encoding == 'mono8':
                image = data.reshape(height, message.step)[:, :width].copy()
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            else:
                return
        except (ValueError, cv2.error):
            return
        stamp = self._message_stamp(message)
        with self.lock:
            self.image_history.append((stamp, image, message.header.frame_id))
            self.image_history = self.image_history[-15:]

    def _camera_info_callback(self, message):
        with self.lock:
            self.camera_info = {
                'stamp': self._message_stamp(message),
                'frame_id': message.header.frame_id,
                'width': int(message.width),
                'height': int(message.height),
                'distortion_model': message.distortion_model,
                'd': [float(value) for value in message.d],
                'k': [float(value) for value in message.k],
                'p': [float(value) for value in message.p],
            }

    def _detection_callback(self, message):
        stamp = self._message_stamp(message)
        with self.lock:
            for detection in message.detections:
                self.detection_history.append((stamp, detection))
            self.detection_history = self.detection_history[-30:]

    def _target_tag_id(self):
        try:
            return int(self.tag_frame.rsplit(':', 1)[1])
        except (IndexError, ValueError):
            return None

    def _draw_pnp_3d(self, image, detected_corners, camera_info,
                     camera_tag):
        """Project axes/cube with the exact same-frame raw PnP pose."""
        if camera_info is None:
            raise ValueError('缺少CameraInfo')
        camera_matrix = np.asarray(
            camera_info['k'], dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(camera_info['d'], dtype=np.float64)
        rotation = np.asarray(camera_tag[:3, :3], dtype=np.float64)
        translation = np.asarray(
            camera_tag[:3, 3], dtype=np.float64).reshape(3, 1)
        rvec, _ = cv2.Rodrigues(rotation)
        size_mm = self.tag_size_m * 1000.0
        half = size_mm * 0.5
        height = size_mm * 0.5

        tag_corners_3d = np.asarray([
            [-half, -half, 0.0],
            [+half, -half, 0.0],
            [+half, +half, 0.0],
            [-half, +half, 0.0],
        ], dtype=np.float64)
        projected_corners, _ = cv2.projectPoints(
            tag_corners_3d, rvec, translation, camera_matrix, distortion)
        projected_corners = projected_corners.reshape(-1, 2)
        corner_errors = np.linalg.norm(
            projected_corners - detected_corners, axis=1)
        reprojection_rms = float(np.sqrt(np.mean(corner_errors ** 2)))

        axis_3d = np.asarray([
            [0.0, 0.0, 0.0],
            [half, 0.0, 0.0],
            [0.0, half, 0.0],
            [0.0, 0.0, height],
        ], dtype=np.float64)
        axis_2d, _ = cv2.projectPoints(
            axis_3d, rvec, translation, camera_matrix, distortion)
        axis_2d = np.rint(axis_2d.reshape(-1, 2)).astype(int)

        cube_3d = np.asarray([
            [-half, -half, 0.0], [+half, -half, 0.0],
            [+half, +half, 0.0], [-half, +half, 0.0],
            [-half, -half, height], [+half, -half, height],
            [+half, +half, height], [-half, +half, height],
        ], dtype=np.float64)
        cube_2d, _ = cv2.projectPoints(
            cube_3d, rvec, translation, camera_matrix, distortion)
        cube_2d_float = cube_2d.reshape(-1, 2)
        cube_2d = np.rint(cube_2d_float).astype(int)

        canvas = image.copy()
        # Cyan points are the PnP-projected tag corners; green is detection.
        for point in projected_corners:
            cv2.circle(canvas, tuple(np.rint(point).astype(int)), 5,
                       (255, 255, 0), 2, cv2.LINE_AA)
        for start, end in ((0, 1), (1, 2), (2, 3), (3, 0),
                           (4, 5), (5, 6), (6, 7), (7, 4),
                           (0, 4), (1, 5), (2, 6), (3, 7)):
            cv2.line(canvas, tuple(cube_2d[start]), tuple(cube_2d[end]),
                     (255, 0, 255), 2, cv2.LINE_AA)
        origin = tuple(axis_2d[0])
        for endpoint, colour, label in (
                (axis_2d[1], (0, 0, 255), 'X'),
                (axis_2d[2], (0, 255, 0), 'Y'),
                (axis_2d[3], (255, 0, 0), 'Z')):
            cv2.arrowedLine(canvas, origin, tuple(endpoint), colour, 4,
                            cv2.LINE_AA, tipLength=0.12)
            cv2.putText(canvas, label, tuple(endpoint + [6, -6]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2,
                        cv2.LINE_AA)
        cv2.putText(
            canvas, f'PnP reprojection RMS={reprojection_rms:.3f}px',
            (20, image.shape[0] - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
            (0, 255, 255), 2, cv2.LINE_AA)
        return canvas, {
            'tag_size_m': self.tag_size_m,
            'rvec': rvec.reshape(-1).tolist(),
            'tvec_mm': translation.reshape(-1).tolist(),
            'T_camera_tag_same_frame_4x4': camera_tag.tolist(),
            'detected_corners_px': detected_corners.tolist(),
            'projected_corners_px': projected_corners.tolist(),
            'corner_errors_px': corner_errors.tolist(),
            'reprojection_rms_px': reprojection_rms,
            'axis_points_px': axis_2d.tolist(),
            'cube_points_px': cube_2d_float.tolist(),
        }

    def save_detection_debug(self, localization_stamp):
        """Save the image, detected border/centre, pixels and camera model."""
        target_id = self._target_tag_id()
        with self.lock:
            detections = list(self.detection_history)
            images = list(self.image_history)
            poses = list(self.history)
            camera_info = dict(self.camera_info) if self.camera_info else None
        candidates = [
            item for item in detections
            if target_id is None or item[1].id == target_id
        ]
        if not candidates or not images or not poses:
            self.get_logger().warn('无法保存Tag验证图：尚无检测角点或彩色图像')
            return None
        detection_stamp, detection = min(
            candidates, key=lambda item: abs(item[0] - localization_stamp))
        image_stamp, image, frame_id = min(
            images, key=lambda item: abs(item[0] - detection_stamp))
        pose_stamp, camera_tag_same_frame = min(
            poses, key=lambda item: abs(item[0] - detection_stamp))
        detection_dt = abs(detection_stamp - localization_stamp)
        image_dt = abs(image_stamp - detection_stamp)
        pose_dt = abs(pose_stamp - detection_stamp)
        if detection_dt > 0.15 or image_dt > 0.15 or pose_dt > 0.15:
            self.get_logger().warn(
                f'无法保存同步验证图: detection-TF={detection_dt:.3f}s '
                f'image-detection={image_dt:.3f}s '
                f'PnP-TF-detection={pose_dt:.3f}s')
            return None

        corners = np.asarray(
            [[corner.x, corner.y] for corner in detection.corners],
            dtype=np.float64)
        centre = np.asarray(
            [detection.centre.x, detection.centre.y], dtype=np.float64)
        canvas = image.copy()
        polygon = np.rint(corners).astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(canvas, [polygon], True, (0, 255, 0), 3, cv2.LINE_AA)
        corner_colours = [(0, 0, 255), (0, 165, 255),
                          (255, 0, 0), (255, 0, 255)]
        for index, (corner, colour) in enumerate(zip(corners, corner_colours)):
            point = tuple(np.rint(corner).astype(int))
            cv2.circle(canvas, point, 7, colour, -1, cv2.LINE_AA)
            cv2.putText(
                canvas, str(index), (point[0] + 8, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2, cv2.LINE_AA)
        centre_point = tuple(np.rint(centre).astype(int))
        cv2.drawMarker(
            canvas, centre_point, (0, 0, 255), cv2.MARKER_CROSS, 30, 3,
            cv2.LINE_AA)
        cv2.circle(canvas, centre_point, 9, (0, 255, 255), 2, cv2.LINE_AA)
        label = (f'{detection.family}:{detection.id}  centre='
                 f'({centre[0]:.1f},{centre[1]:.1f})')
        label_y = max(28, int(np.min(corners[:, 1])) - 14)
        cv2.putText(
            canvas, label, (max(8, int(np.min(corners[:, 0]))), label_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2,
            cv2.LINE_AA)
        try:
            pnp3d_canvas, pnp3d = self._draw_pnp_3d(
                canvas, corners, camera_info, camera_tag_same_frame)
            pnp3d['pose_to_detection_dt_sec'] = pose_dt
        except (ValueError, cv2.error) as exc:
            self.get_logger().warn(f'3D PnP投影失败: {exc}')
            pnp3d_canvas = canvas
            pnp3d = {'error': str(exc), 'pose_to_detection_dt_sec': pose_dt}

        os.makedirs(self.debug_output_dir, exist_ok=True)
        self.debug_save_index += 1
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        stem = f'locate_{timestamp}_{self.debug_save_index:03d}'
        raw_path = os.path.join(self.debug_output_dir, f'{stem}_raw.jpg')
        annotated_path = os.path.join(
            self.debug_output_dir, f'{stem}_annotated.jpg')
        pnp3d_path = os.path.join(
            self.debug_output_dir, f'{stem}_pnp3d.jpg')
        json_path = os.path.join(self.debug_output_dir, f'{stem}.json')
        if (not cv2.imwrite(raw_path, image) or
                not cv2.imwrite(annotated_path, canvas) or
                not cv2.imwrite(pnp3d_path, pnp3d_canvas)):
            self.get_logger().warn('Tag验证图写入失败')
            return None

        localization = self.last_localization or {}
        record = {
            'created_at': datetime.now().isoformat(timespec='milliseconds'),
            'image': {
                'raw_path': raw_path,
                'annotated_path': annotated_path,
                'pnp3d_path': pnp3d_path,
                'stamp': image_stamp,
                'frame_id': frame_id,
                'width': int(image.shape[1]),
                'height': int(image.shape[0]),
            },
            'detection': {
                'stamp': detection_stamp,
                'family': detection.family,
                'id': int(detection.id),
                'hamming': int(detection.hamming),
                'decision_margin': float(detection.decision_margin),
                'centre_px': centre.tolist(),
                'corners_px': corners.tolist(),
                'detection_to_tf_dt_sec': detection_dt,
                'image_to_detection_dt_sec': image_dt,
            },
            'pnp3d': pnp3d,
            'camera_info': camera_info,
            'pose': {
                'frame_contract': (
                    'T_base_tag = T_base_flange(GetPose in User0) * '
                    'T_flange_camera(handeye) * T_camera_tag(AprilTag)'),
                'base_flange_4x4': localization.get(
                    'base_flange_at_detection', np.eye(4)).tolist(),
                'flange_camera_4x4': self.flange_from_camera.tolist(),
                'camera_tag_4x4': localization.get(
                    'camera_tag', np.eye(4)).tolist(),
                'base_tag_4x4': localization.get(
                    'base_tag', np.eye(4)).tolist(),
                'predicted_tag_center_base_mm': localization.get(
                    'base_tag', np.eye(4))[:3, 3].tolist(),
                'tcp_offset_flange_to_tip_mm': self.tcp_offset_flange.tolist(),
                'handeye_path': self.handeye_path,
                'tcp_calibration_path': self.tcp_calibration_path,
            },
        }
        with open(json_path, 'w', encoding='utf-8') as stream:
            json.dump(record, stream, ensure_ascii=False, indent=2)
        return pnp3d_path

    def locate(self):
        if not self.robot.use_base_frame():
            raise RuntimeError('定位前无法锁定 User(0) 基座原点')
        flange_pose = self.robot.get_tool(warn=False)
        if flange_pose is None:
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
        base_flange = tool_pose_matrix(flange_pose)
        base_tag = base_flange @ self.flange_from_camera @ camera_tag
        self._check_localization_chain(base_flange, camera_tag, base_tag)
        # The pivot calibration defines tip position, not tip orientation.
        # Keep the manually selected current tool orientation; only take the
        # desired contact position from Tag + configured positional offset.
        tag_tip = base_tag @ self.tag_to_tcp
        base_tip = base_flange.copy()
        base_tip[:3, 3] = tag_tip[:3, 3]
        # TCP file defines flange origin -> physical tip. Therefore the
        # commanded flange origin is desired_tip - R_base_flange @ offset.
        base_flange_target = base_tip.copy()
        base_flange_target[:3, 3] -= (
            base_flange_target[:3, :3] @ self.tcp_offset_flange)
        xyz = base_flange_target[:3, 3]
        radius = float(np.linalg.norm(xyz[:2]))
        if xyz[2] < self.min_base_z or radius > self.max_reach:
            raise RuntimeError(
                f'目标越过安全边界: xyz={xyz.round(1).tolist()}, radius={radius:.1f}')
        self.localization_index += 1
        self.last_localization = {
            'index': self.localization_index,
            'stamp': newest_stamp,
            'detection_age_sec': age,
            'flange_xyzrpy': list(flange_pose),
            'base_flange_at_detection': base_flange.copy(),
            'camera_tag': camera_tag.copy(),
            'base_tag': base_tag.copy(),
            'predicted_contact_tip': tag_tip[:3, 3].copy(),
            'base_flange_target': base_flange_target.copy(),
            'frames': len(recent),
            'spread_mm': spread,
        }
        self.last_localization['debug_image_path'] = self.save_detection_debug(
            newest_stamp)
        self.localization_cache.append(self.last_localization)
        self.localization_cache = self.localization_cache[-50:]
        print(
            f'[l#{self.localization_index}] '
            f'tag={base_tag[:3, 3].round(2).tolist()}mm  '
            f'tip={tag_tip[:3, 3].round(2).tolist()}mm  '
            f'flange_target={xyz.round(2).tolist()}mm  '
            f'spread={spread:.2f}mm age={age:.3f}s')
        return base_flange_target

    @staticmethod
    def _tag_view_angles(camera_tag):
        """Return optical off-axis and tag-normal incidence angles in degrees."""
        center = camera_tag[:3, 3]
        distance = float(np.linalg.norm(center))
        if distance < 1e-9:
            return 0.0, 0.0, distance
        ray = center / distance
        optical_angle = float(np.degrees(np.arccos(np.clip(
            np.dot(ray, [0.0, 0.0, 1.0]), -1.0, 1.0))))
        normal = camera_tag[:3, 2]
        incidence = float(np.degrees(np.arccos(np.clip(
            abs(np.dot(normal, ray)), -1.0, 1.0))))
        return optical_angle, incidence, distance

    def record_tag_diagnostic_prediction(self):
        """Record one viewpoint's flange pose and AprilTag prediction."""
        if (self.tag_diagnostic_samples and
                self.tag_diagnostic_samples[-1].get('truth') is None):
            raise RuntimeError('上一组还没有真值；请先按 v 接触Tag中心，或按 x 清空')
        self.locate()
        localization = self.last_localization
        optical, incidence, distance = self._tag_view_angles(
            localization['camera_tag'])
        sample = {
            'index': len(self.tag_diagnostic_samples) + 1,
            'prediction': {
                'stamp': float(localization['stamp']),
                'detection_age_sec': float(localization['detection_age_sec']),
                'flange_xyzrpy': list(localization['flange_xyzrpy']),
                'T_base_flange_4x4': localization[
                    'base_flange_at_detection'].tolist(),
                'T_camera_tag_4x4': localization['camera_tag'].tolist(),
                'predicted_tag_center_base_mm': localization[
                    'base_tag'][:3, 3].tolist(),
                'tag_center_camera_mm': localization[
                    'camera_tag'][:3, 3].tolist(),
                'camera_distance_mm': distance,
                'optical_off_axis_deg': optical,
                'tag_incidence_deg': incidence,
                'detection_frames': int(localization['frames']),
                'detection_spread_mm': float(localization['spread_mm']),
                'debug_image_path': localization.get('debug_image_path'),
            },
            'truth': None,
        }
        self.tag_diagnostic_samples.append(sample)
        print('\n  ✅ 多视角诊断预测 P%02d 已记录' % sample['index'])
        print('  检测法兰 xyz=%s mm' % np.round(
            sample['prediction']['flange_xyzrpy'][:3], 2).tolist())
        print('  预测中心=%s mm  距离=%.1fmm  入射角=%.2f°  光轴偏角=%.2f°' % (
            np.round(sample['prediction']['predicted_tag_center_base_mm'], 2).tolist(),
            distance, incidence, optical))
        print('  现在按 v 拖动针尖接触同一个Tag中心，记录本组真值。')
        return sample

    def _check_localization_chain(self, base_flange, camera_tag, base_tag):
        """Fail fast when flange/camera transforms are malformed or mixed."""
        named = {
            'T_base_flange(GetPose in User0)': base_flange,
            'T_flange_camera(handeye)': self.flange_from_camera,
            'T_camera_tag(AprilTag)': camera_tag,
            'T_base_tag(result)': base_tag,
        }
        for name, transform in named.items():
            if transform.shape != (4, 4) or not np.isfinite(transform).all():
                raise RuntimeError(f'坐标链自检失败: {name} 不是有效 4x4 矩阵')
            if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
                raise RuntimeError(f'坐标链自检失败: {name} 齐次行错误')
            determinant = np.linalg.det(transform[:3, :3])
            orthogonal_error = np.linalg.norm(
                transform[:3, :3].T @ transform[:3, :3] - np.eye(3))
            if abs(determinant - 1.0) > 1e-3 or orthogonal_error > 1e-3:
                raise RuntimeError(
                    f'坐标链自检失败: {name} 旋转矩阵异常 '
                    f'(det={determinant:.6f}, ortho={orthogonal_error:.2e})')
        recomputed = base_flange @ self.flange_from_camera @ camera_tag
        closure_mm = float(np.linalg.norm(recomputed[:3, 3] - base_tag[:3, 3]))
        closure_deg = self._rotation_distance_deg(recomputed, base_tag)
        # arccos-based rotation distance has a few micro-degrees of numerical
        # noise even when both matrices are mathematically identical.
        if closure_mm > 1e-3 or closure_deg > 1e-3:
            raise RuntimeError(
                f'坐标链闭合失败: {closure_mm:.6f}mm/{closure_deg:.6f}°')

    def record_manual_tag_center(self):
        """Record the manually contacted tag center and report vision error."""
        if self.last_localization is None:
            raise RuntimeError('请先按 l 定位并缓存视觉预测')
        if 'manual_truth' in self.last_localization:
            raise RuntimeError('当前缓存已经记录过真值；请重新按 l 建立新缓存')
        if not self.robot.use_base_frame():
            raise RuntimeError('记录真值前无法锁定 User(0) 基座原点')
        tool = self.robot.get_tool(warn=False)
        if tool is None:
            raise RuntimeError('记录真值时 GetPose 无有效位姿')
        base_flange = tool_pose_matrix(tool)
        predicted_center = self.last_localization['base_tag'][:3, 3]
        candidates = self._tcp_position_variants(base_flange)
        actual_tip = candidates['tip_plus']
        localization_error = predicted_center - actual_tip
        self.manual_tag_center = {
            'base_flange': base_flange.copy(),
            'tip_position': actual_tip.copy(),
            'flange_xyzrpy': list(tool),
        }

        pending = (self.tag_diagnostic_samples and
                   self.tag_diagnostic_samples[-1].get('truth') is None)
        if pending:
            sample = self.tag_diagnostic_samples[-1]
            detection_flange = np.asarray(
                sample['prediction']['T_base_flange_4x4'], dtype=float)
            sample['truth'] = {
                'recorded_at': datetime.now().isoformat(timespec='milliseconds'),
                'flange_xyzrpy': list(tool),
                'T_base_flange_4x4': base_flange.tolist(),
                'tip_plus_base_mm': actual_tip.tolist(),
                'tip_minus_base_mm': candidates['tip_minus'].tolist(),
                'rotated_offset_base_mm': candidates['offset_base'].tolist(),
                'prediction_minus_truth_mm': localization_error.tolist(),
                'error_norm_mm': float(np.linalg.norm(localization_error)),
                'detection_to_contact_flange_rotation_deg': (
                    self._rotation_distance_deg(detection_flange, base_flange)),
            }

        self.last_localization['manual_truth'] = {
            'flange_xyzrpy': list(tool),
            'base_flange_4x4': base_flange.copy(),
            'tip_plus_base_mm': actual_tip.copy(),
            'prediction_minus_truth_mm': localization_error.copy(),
            'error_norm_mm': float(np.linalg.norm(localization_error)),
        }
        print(
            f'[v#{self.last_localization["index"]}] '
            f'pred={predicted_center.round(2).tolist()}mm  '
            f'truth={actual_tip.round(2).tolist()}mm  '
            f'{self._error_text(localization_error)}')
        if pending:
            complete = sum(
                item.get('truth') is not None
                for item in self.tag_diagnostic_samples)
            if complete >= 3:
                self.report_tag_diagnostics()
            else:
                self._save_tag_diagnostic_json()
        return localization_error

    @staticmethod
    def _point_consistency(points):
        points = np.asarray(points, dtype=float)
        center = np.mean(points, axis=0)
        deltas = points - center
        norms = np.linalg.norm(deltas, axis=1)
        return center, deltas, norms, float(np.sqrt(np.mean(norms ** 2)))

    def _save_tag_diagnostic_json(self, summary=None):
        os.makedirs(self.debug_output_dir, exist_ok=True)
        self.tag_diagnostic_save_index += 1
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        path = os.path.join(
            self.debug_output_dir,
            f'tag_multiview_{stamp}_{self.tag_diagnostic_save_index:03d}.json')
        payload = {
            'created_at': datetime.now().isoformat(timespec='milliseconds'),
            'frame_contract': (
                'T_base_tag = T_base_flange * T_flange_camera * T_camera_tag'),
            'truth_contract': (
                'p_base_tip = p_base_flange + R_base_flange * p_flange_tip'),
            'handeye_path': self.handeye_path,
            'tcp_calibration_path': self.tcp_calibration_path,
            'tcp_offset_flange_mm': self.tcp_offset_flange.tolist(),
            'complete_pairs': sum(
                item.get('truth') is not None
                for item in self.tag_diagnostic_samples),
            'samples': self.tag_diagnostic_samples,
            'summary': summary,
        }
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
        print(f'  多视角诊断JSON: {path}')
        return path

    def report_tag_diagnostics(self):
        """Separate fixed bias, view-dependent vision error, and truth scatter."""
        samples = [item for item in self.tag_diagnostic_samples
                   if item.get('truth') is not None]
        if len(samples) < 3:
            raise RuntimeError(f'完整预测/真值只有{len(samples)}组，至少需要3组')
        predictions = np.asarray([
            item['prediction']['predicted_tag_center_base_mm']
            for item in samples], dtype=float)
        truths = np.asarray([
            item['truth']['tip_plus_base_mm'] for item in samples], dtype=float)
        errors = predictions - truths
        pred_center, _, pred_norms, pred_rms = self._point_consistency(predictions)
        truth_center, _, truth_norms, truth_rms = self._point_consistency(truths)
        mean_error = np.mean(errors, axis=0)
        bias_norm = float(np.linalg.norm(mean_error))
        residuals = errors - mean_error
        residual_norms = np.linalg.norm(residuals, axis=1)
        residual_rms = float(np.sqrt(np.mean(residual_norms ** 2)))
        error_norms = np.linalg.norm(errors, axis=1)
        error_rms = float(np.sqrt(np.mean(error_norms ** 2)))

        print('\n' + '=' * 78)
        print(f'  AprilTag 多视角误差来源报告（{len(samples)}组）')
        print('  error = 预测Tag中心 - 手动tip_plus真值')
        print('-' * 78)
        for index, (sample, error, error_norm) in enumerate(
                zip(samples, errors, error_norms), 1):
            prediction = sample['prediction']
            truth = sample['truth']
            print(
                f'  G{index:02d}: 入射={prediction["tag_incidence_deg"]:.2f}° '
                f'光轴偏={prediction["optical_off_axis_deg"]:.2f}° '
                f'距离={prediction["camera_distance_mm"]:.1f}mm  '
                f'检测spread={prediction["detection_spread_mm"]:.2f}mm')
            print(f'       预测={np.round(predictions[index-1], 2).tolist()}  '
                  f'真值={np.round(truths[index-1], 2).tolist()}')
            print(f'       Δ=[{error[0]:+.2f}, {error[1]:+.2f}, '
                  f'{error[2]:+.2f}]mm  |Δ|={error_norm:.2f}mm  '
                  f'接触姿态差={truth["detection_to_contact_flange_rotation_deg"]:.2f}°')
        print('-' * 78)
        print(f'  预测中心共识: {np.round(pred_center, 2).tolist()} mm  '
              f'跨视角RMS={pred_rms:.2f}mm  最大={max(pred_norms):.2f}mm')
        print(f'  tip_plus真值共识: {np.round(truth_center, 2).tolist()} mm  '
              f'跨姿态RMS={truth_rms:.2f}mm  最大={max(truth_norms):.2f}mm')
        print(f'  总配对误差: RMS={error_rms:.2f}mm  最大={max(error_norms):.2f}mm')
        print(f'  固定偏置(mean error): [{mean_error[0]:+.2f}, '
              f'{mean_error[1]:+.2f}, {mean_error[2]:+.2f}]mm  '
              f'|bias|={bias_norm:.2f}mm')
        print(f'  去固定偏置后的视角相关残差: RMS={residual_rms:.2f}mm  '
              f'最大={max(residual_norms):.2f}mm')
        print('\n  初步判读:')
        diagnoses = []
        if truth_rms > 3.0:
            diagnoses.append(
                'tip_plus真值自身不一致：优先检查TCP、针尖接触重复性或Tag是否移动')
        if residual_rms > 3.0:
            diagnoses.append(
                '误差随观察角度变化：优先检查AprilTag姿态估计、tag_size、畸变和时间同步')
        if bias_norm > 3.0 and residual_rms <= 3.0:
            diagnoses.append(
                '以固定偏置为主：优先检查Tag几何中心定义、tag_to_tcp偏移或TCP系统偏差')
        if not diagnoses:
            diagnoses.append('预测与真值均较稳定，当前没有明显大误差来源')
        for diagnosis in diagnoses:
            print(f'    - {diagnosis}')
        print('=' * 78)

        summary = {
            'prediction_consensus_base_mm': pred_center.tolist(),
            'prediction_viewpoint_rms_mm': pred_rms,
            'prediction_viewpoint_max_mm': float(max(pred_norms)),
            'truth_tip_consensus_base_mm': truth_center.tolist(),
            'truth_pose_rms_mm': truth_rms,
            'truth_pose_max_mm': float(max(truth_norms)),
            'paired_error_rms_mm': error_rms,
            'paired_error_max_mm': float(max(error_norms)),
            'mean_error_xyz_mm': mean_error.tolist(),
            'fixed_bias_norm_mm': bias_norm,
            'view_dependent_residual_rms_mm': residual_rms,
            'view_dependent_residual_max_mm': float(max(residual_norms)),
            'diagnoses': diagnoses,
        }
        self._save_tag_diagnostic_json(summary)
        return summary

    def clear_tag_diagnostics(self):
        self.tag_diagnostic_samples.clear()
        print('  ↩ 已清空 AprilTag 多视角诊断数据')

    def validate_cached_motion(self):
        """Move to the cached prediction and separate motion from vision error."""
        if self.last_localization is None:
            raise RuntimeError('请先按 l 定位并缓存视觉预测')
        if not self.execute_enabled:
            raise RuntimeError('当前禁止运动；将 execute_enabled 改为 true 后才能按 m')

        localization = self.last_localization
        target = localization['base_flange_target'].copy()
        predicted_contact = localization['predicted_contact_tip'].copy()
        pregrasp = self.retract_pose(target, self.pregrasp_height)
        print('\n▶ 补偿验证：使用缓存预测，不重新识别Tag')
        if not self.robot.move_pose(pregrasp):
            raise RuntimeError('补偿验证预接近失败')
        if not self.robot.move_pose(target):
            self.robot.move_pose(pregrasp)
            raise RuntimeError('补偿验证到预测点失败')

        tool = self.robot.get_tool()
        if tool is None:
            self.robot.move_pose(pregrasp)
            raise RuntimeError('到达预测点后 GetPose 无有效位姿')
        actual_tool = tool_pose_matrix(tool)
        actual_tip = self._physical_tip_position(actual_tool)
        tool_tracking_error = actual_tool[:3, 3] - target[:3, 3]
        motion_error = actual_tip - predicted_contact

        print('\n' + '=' * 62)
        print('  MovJ执行误差（实际法兰原点 - 指令法兰原点）')
        print(f'  {self._error_text(tool_tracking_error)}')
        print('\n  代码补偿结果（计算的实际针尖 - 视觉预测接触点）')
        print(f'  视觉预测接触点: {predicted_contact.round(2).tolist()} mm')
        print(f'  自动到位实际针尖: {actual_tip.round(2).tolist()} mm')
        print(f'  {self._error_text(motion_error)}')

        total_error = None
        if self.manual_tag_center is not None:
            manual_center = self.manual_tag_center['tip_position']
            tag_offset = self.tag_to_tcp[:3, 3]
            manual_contact = (
                manual_center + localization['base_tag'][:3, :3] @ tag_offset)
            total_error = actual_tip - manual_contact
            vision_error = predicted_contact - manual_contact
            print('\n  分项诊断')
            print(f'  视觉定位部分: {self._error_text(vision_error)}')
            print(f'  运动补偿部分: {self._error_text(motion_error)}')
            print(f'  最终戳点总误差: {self._error_text(total_error)}')
        else:
            print('  尚未按 v 记录手动真值，无法计算最终戳点总误差。')
        print('=' * 62)

        if not self.robot.move_pose(pregrasp):
            raise RuntimeError('误差已输出，但验证结束回撤失败')
        return motion_error, total_error

    def retract_pose(self, tool_target, distance_mm):
        """Retract the calibrated tip backward along the tool-to-tip axis."""
        axis_tool = (self.tcp_offset_flange /
                     np.linalg.norm(self.tcp_offset_flange))
        axis_base = tool_target[:3, :3] @ axis_tool
        retracted = tool_target.copy()
        retracted[:3, 3] -= axis_base * float(distance_mm)
        return retracted

    def pick(self):
        if self.last_localization is None:
            raise RuntimeError('没有缓存目标；请先按 l 定位')
        target = self.last_localization['base_flange_target'].copy()
        if not self.execute_enabled:
            raise RuntimeError('当前是定位模式；将 execute_enabled 改为 true 才允许运动')
        print(f'[p#{self.last_localization["index"]}] 使用缓存目标 '
              f'{target[:3, 3].round(2).tolist()}mm')
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
        self.get_logger().info('戳点流程完成：机械臂停留在目标位姿')


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
    menu = 'l=定位缓存  d=进入拖拽  t=退出拖拽  v=记录真值  p=抓取缓存目标  q=退出'
    print(f'\nAprilTag Pick: {menu}')
    try:
        while rclpy.ok():
            try:
                command = reader.read('apriltag> ').strip().lower()
                if command == 'l':
                    node.locate()
                elif command == 'd':
                    if node.robot.dragging:
                        print('[d] 已处于拖拽模式')
                    elif node.robot.start_drag():
                        print('[d] 已进入拖拽')
                    else:
                        raise RuntimeError('进入拖拽失败')
                elif command == 't':
                    if node.robot.stop_drag():
                        print('[t] 已退出拖拽并重新使能')
                    else:
                        raise RuntimeError('退出拖拽/重新使能失败')
                elif command == 'v':
                    if node.last_localization is None:
                        raise RuntimeError('请先按 l 定位，再按 v 记录Tag中心真值')
                    if not node.robot.dragging:
                        raise RuntimeError('请先按 d 进入拖拽，把针尖移动到Tag中心')
                    node.record_manual_tag_center()
                elif command == 'p':
                    if node.robot.dragging:
                        raise RuntimeError('当前仍在拖拽模式；请先按 t 退出拖拽')
                    node.pick()
                elif command == 'q':
                    rclpy.shutdown()
                    return
                else:
                    print(menu)
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
