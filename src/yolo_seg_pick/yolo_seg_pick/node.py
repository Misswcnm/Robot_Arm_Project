"""Manual, guarded YOLO11-seg RGB-D picking demo for D455 + CR5."""

import select
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from step_motor.msg import Motor
from visualization_msgs.msg import Marker

from apriltag_pick.robot import CR5Robot
from apriltag_pick.transforms import load_handeye, load_tcp_offset, tool_pose_matrix


class YoloSegPick(Node):
    def __init__(self):
        super().__init__('yolo_seg_pick')
        p = self.declare_parameter
        workspace_root = Path(__file__).resolve().parents[3]
        model_path = Path(str(p(
            'model_path',
            '/home/ylx/Robot_Arm_Project/models/yolo11n-seg.pt').value)).expanduser()
        self.model_path = model_path if model_path.is_absolute() else workspace_root / model_path
        self.detector_mode = str(p('detector_mode', 'opencv_template').value).lower()
        template_path = Path(str(p(
            'template_path',
            '/home/ylx/Robot_Arm_Project/src/yolo_seg_pick/templates/target.png').value)).expanduser()
        self.template_path = (template_path if template_path.is_absolute()
                              else workspace_root / template_path)
        self.template_ratio = float(p('template_ratio_test', .75).value)
        self.template_min_matches = int(p('template_min_matches', 12).value)
        self.template_min_inliers = int(p('template_min_inliers', 8).value)
        self.target_class = str(p('target_class', '').value)
        self.confidence = float(p('confidence', 0.55).value)
        self.min_depth_m = float(p('min_depth_m', .15).value)
        self.max_depth_m = float(p('max_depth_m', 1.2).value)
        self.min_points = int(p('min_mask_points', 150).value)
        self.trim = float(p('depth_trim_percent', 10.).value)
        self.execute_enabled = bool(p('execute_enabled', False).value)
        self.pregrasp_height = float(p('pregrasp_height_mm', 100.).value)
        self.lift_height = float(p('lift_height_mm', 120.).value)
        self.min_z = float(p('min_base_z_mm', 20.).value)
        self.max_radius = float(p('max_base_radius_mm', 850.).value)
        self.step_gripper_enabled = bool(p('step_gripper.enabled', True).value)
        self.step_gripper_topic = str(p('step_gripper.topic', '/motor_control').value)
        self.step_gripper_id = int(p('step_gripper.id', 1).value)
        self.step_gripper_speed = int(p('step_gripper.speed', 200).value)
        self.step_gripper_mode = int(p('step_gripper.mode', 2).value)
        self.step_gripper_angle = int(p('step_gripper.angle', 30000).value)
        self.step_gripper_subdivide = int(p('step_gripper.sub_divide', 32).value)
        self.step_gripper_open_dir = int(p('step_gripper.open_dir', 0).value)
        self.step_gripper_close_dir = int(p('step_gripper.close_dir', 1).value)
        self.step_gripper_settle = float(p('step_gripper.settle_sec', 1.5).value)
        self.flange_from_camera = load_handeye(p(
            'handeye_path',
            '/home/ylx/Robot_Arm_Project/scripts/active_handeye_calibration.json').value)
        self.tcp_offset = load_tcp_offset(p(
            'tcp_calibration_path',
            '/home/ylx/Robot_Arm_Project/scripts/tcp_calibration/'
            'tcp_calibration_20260714_095024.json').value)
        self.robot = CR5Robot(self, speed=int(p('robot_speed', 10).value))
        self.robot_ready = False

        self.lock = threading.Lock()
        self.model = None
        self.class_names = None
        self.template_image = self.template_keypoints = self.template_descriptors = None
        self.image = self.depth = self.info = None
        self.last_target = None
        self.annotated_pub = self.create_publisher(Image, 'annotated_image', 1)
        self.cloud_pub = self.create_publisher(PointCloud2, 'target_cloud', 1)
        self.center_pub = self.create_publisher(Marker, 'target_center', 1)
        self.direction_pub = self.create_publisher(Marker, 'pca_direction', 1)
        self.step_gripper_pub = self.create_publisher(
            Motor, self.step_gripper_topic, 10)
        self.create_subscription(
            Image, p('image_topic', '/camera/camera/color/image_raw').value,
            self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            Image, p('aligned_depth_topic',
                     '/camera/camera/aligned_depth_to_color/image_raw').value,
            self.on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, p('camera_info_topic',
                          '/camera/camera/color/camera_info').value,
            self.on_info, qos_profile_sensor_data)
        self.get_logger().info(
            'RGB-D demo ready: s=定位 t=拖拽 2=退出拖拽 '
            'c=类别 p=抓取 q=退出; execute_enabled=%s' % self.execute_enabled)

    def ensure_model(self):
        if self.model is not None:
            return
        try:
            from ultralytics import YOLO
            self.get_logger().info(f'正在加载 YOLO 模型: {self.model_path}')
            self.model = YOLO(self.model_path)
            self.class_names = self.model.names
        except ImportError as exc:
            raise RuntimeError('无法导入 ultralytics；请检查 ultralytics/NumPy 环境') from exc
        except Exception as exc:
            raise RuntimeError(f'无法加载 YOLO 分割权重 {self.model_path}: {exc}') from exc

    def ensure_template(self):
        if self.template_image is not None:
            return
        image = cv2.imread(str(self.template_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(
                f'模板图不存在: {self.template_path}；请提供仅包含目标的清晰 PNG/JPG')
        orb = cv2.ORB_create(nfeatures=1600)
        keypoints, descriptors = orb.detectAndCompute(image, None)
        if descriptors is None or len(keypoints) < self.template_min_matches:
            count = 0 if descriptors is None else len(keypoints)
            raise RuntimeError(f'模板 ORB 特征不足: {count}')
        self.template_image = image
        self.template_keypoints = keypoints
        self.template_descriptors = descriptors
        self.get_logger().info(
            f'OpenCV ORB 模板已加载: {self.template_path}, features={len(keypoints)}')

    def ensure_robot_ready(self):
        """Avoid controller service waits until a drag or physical pick is requested."""
        if not self.robot_ready:
            self.robot.initialize()
            self.robot_ready = True

    def print_classes(self):
        """Print the class names accepted by the loaded YOLO segmentation model."""
        if self.detector_mode == 'opencv_template':
            print('\n当前为 OpenCV ORB 模板模式：识别目标由 template_path 图片定义。')
            print(f'  template_path={self.template_path}')
            print('  将 detector_mode 改为 yolo11_seg 后，c 会列出 YOLO 类别。')
            return
        self.ensure_model()
        if isinstance(self.class_names, dict):
            items = self.class_names.items()
        else:
            items = enumerate(self.class_names)
        labels = [f'{int(index)}:{name}' for index, name in items]
        print('\n可识别类别（target_class 使用名称，例如 bottle）：')
        for start in range(0, len(labels), 8):
            print('  ' + '  '.join(labels[start:start + 8]))

    def detect_template(self, image):
        """ORB + RANSAC homography gives a depth mask for one known object."""
        self.ensure_template()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        orb = cv2.ORB_create(nfeatures=2000)
        scene_keypoints, scene_descriptors = orb.detectAndCompute(gray, None)
        if scene_descriptors is None:
            raise RuntimeError('当前 RGB 图像没有足够的 ORB 特征')
        matches = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(
            self.template_descriptors, scene_descriptors, k=2)
        good = [first for first, second in matches
                if first.distance < self.template_ratio * second.distance]
        if len(good) < self.template_min_matches:
            raise RuntimeError(
                f'模板匹配点不足: {len(good)} < {self.template_min_matches}')
        source = np.float32([self.template_keypoints[item.queryIdx].pt for item in good])
        destination = np.float32([scene_keypoints[item.trainIdx].pt for item in good])
        homography, inlier_mask = cv2.findHomography(source, destination, cv2.RANSAC, 3.0)
        inliers = int(np.count_nonzero(inlier_mask)) if inlier_mask is not None else 0
        if homography is None or inliers < self.template_min_inliers:
            raise RuntimeError(
                f'模板单应性不可靠: inliers={inliers} < {self.template_min_inliers}')
        height, width = self.template_image.shape
        source_corners = np.float32([[[0, 0], [width - 1, 0],
                                     [width - 1, height - 1], [0, height - 1]]])
        polygon = cv2.perspectiveTransform(source_corners, homography)[0]
        polygon_int = np.rint(polygon).astype(np.int32)
        if not cv2.isContourConvex(polygon_int):
            raise RuntimeError('模板投影不是凸四边形，拒绝本次识别')
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, polygon_int, 1)
        if int(np.count_nonzero(mask)) < self.min_points:
            raise RuntimeError('模板投影面积过小')
        return mask.astype(bool), 'opencv_template', min(1.0, inliers / len(good))

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def on_image(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        image = data[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
        if msg.encoding == 'rgb8':
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.image = (self.stamp(msg), image, msg.header)

    def on_depth(self, msg):
        if msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width].astype(np.float32) * .001
        elif msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width].copy()
        else:
            self.get_logger().warn(f'不支持的深度编码: {msg.encoding}')
            return
        with self.lock:
            self.depth = (self.stamp(msg), depth)

    def on_info(self, msg):
        with self.lock:
            self.info = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]), msg.width, msg.height)

    def snapshot(self):
        with self.lock:
            return self.image, self.depth, self.info

    def publish_visuals(self, header, image, mask, name, score, points_camera,
                        center_camera, direction_camera):
        """Publish RViz-friendly overlay, XYZ cloud and centre/PCA markers."""
        overlay = image.copy()
        overlay[mask] = (0.35 * overlay[mask] + 0.65 * np.array([0, 220, 0])).astype(np.uint8)
        contour, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contour, -1, (0, 255, 255), 2)
        pixel = np.mean(np.argwhere(mask), axis=0).astype(int)  # v, u
        cv2.drawMarker(overlay, (int(pixel[1]), int(pixel[0])), (0, 0, 255),
                       cv2.MARKER_CROSS, 28, 2)
        cv2.putText(overlay, f'{name} {score:.2f}', (12, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, .9, (0, 255, 255), 2)
        output = Image()
        output.header = header
        output.height, output.width = overlay.shape[:2]
        output.encoding, output.is_bigendian = 'bgr8', False
        output.step = output.width * 3
        output.data = overlay.tobytes()
        self.annotated_pub.publish(output)

        # RViz PointCloud2 coordinates are metres in the camera optical frame.
        cloud_points = (points_camera * .001).astype(np.float32)
        if len(cloud_points) > 12000:
            cloud_points = cloud_points[::int(np.ceil(len(cloud_points) / 12000))]
        cloud = PointCloud2()
        cloud.header = header
        cloud.height, cloud.width = 1, len(cloud_points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian, cloud.point_step = False, 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = cloud_points.tobytes()
        self.cloud_pub.publish(cloud)

        center_m = center_camera * .001
        center = Marker()
        center.header = header
        center.ns, center.id, center.type, center.action = 'yolo_seg', 0, Marker.SPHERE, Marker.ADD
        center.pose.position.x, center.pose.position.y, center.pose.position.z = center_m
        center.pose.orientation.w = 1.0
        center.scale.x = center.scale.y = center.scale.z = .035
        center.color.r, center.color.g, center.color.b, center.color.a = 1., .1, .1, 1.
        self.center_pub.publish(center)

        arrow = Marker()
        arrow.header = header
        arrow.ns, arrow.id, arrow.type, arrow.action = 'yolo_seg', 1, Marker.ARROW, Marker.ADD
        arrow.scale.x, arrow.scale.y, arrow.scale.z = .012, .025, .035
        start, end = Point(), Point()
        start.x, start.y, start.z = center_m
        end_vector = center_m + .15 * direction_camera
        end.x, end.y, end.z = end_vector
        arrow.points = [start, end]
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = .1, .4, 1., 1.
        self.direction_pub.publish(arrow)

    def select_mask(self, result, shape):
        if result.masks is None or result.boxes is None:
            raise RuntimeError('YOLO 未检测到分割目标')
        names = result.names
        candidates = []
        for index, box in enumerate(result.boxes):
            score = float(box.conf[0])
            class_id = int(box.cls[0])
            name = str(names[class_id])
            if score >= self.confidence and (not self.target_class or name == self.target_class):
                candidates.append((score, index, name))
        if not candidates:
            requested = self.target_class or '任意类别'
            raise RuntimeError(f'没有置信度≥{self.confidence:.2f}的 {requested} mask')
        score, index, name = max(candidates)
        mask = result.masks.data[index].cpu().numpy().astype(np.uint8)
        mask = cv2.resize(mask, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
        return mask, name, score

    def locate(self):
        image_data, depth_data, info = self.snapshot()
        if image_data is None or depth_data is None or info is None:
            raise RuntimeError('等待 RGB、aligned depth 和 CameraInfo')
        image_stamp, image, image_header = image_data
        depth_stamp, depth = depth_data
        if abs(image_stamp - depth_stamp) > .15:
            raise RuntimeError(f'RGB/depth 未同步: Δt={abs(image_stamp-depth_stamp):.3f}s')
        if depth.shape != image.shape[:2] or (info[4], info[5]) != (image.shape[1], image.shape[0]):
            raise RuntimeError('RGB、aligned depth、CameraInfo 分辨率必须一致')
        if self.detector_mode == 'opencv_template':
            mask, name, score = self.detect_template(image)
        elif self.detector_mode == 'yolo11_seg':
            self.ensure_model()
            result = self.model.predict(image, conf=self.confidence, verbose=False)[0]
            mask, name, score = self.select_mask(result, image.shape)
        else:
            raise RuntimeError(
                f'未知 detector_mode={self.detector_mode}；可用 opencv_template 或 yolo11_seg')
        valid = mask & np.isfinite(depth) & (depth >= self.min_depth_m) & (depth <= self.max_depth_m)
        values = depth[valid]
        if values.size < self.min_points:
            raise RuntimeError(f'mask 有效深度点不足: {values.size} < {self.min_points}')
        lo, hi = np.percentile(values, [self.trim, 100. - self.trim])
        valid &= (depth >= lo) & (depth <= hi)
        v, u = np.nonzero(valid)
        z = depth[v, u]
        fx, fy, cx, cy, _, _ = info
        points_camera = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z)) * 1000.
        center_camera = np.median(points_camera, axis=0)
        covariance = np.cov(points_camera - center_camera, rowvar=False)
        _, vectors = np.linalg.eigh(covariance)
        direction_camera = vectors[:, -1]
        self.publish_visuals(image_header, image, mask, name, score, points_camera,
                             center_camera, direction_camera)

        if not self.robot.use_base_frame():
            raise RuntimeError('无法锁定 User(0) 基座原点')
        pose = self.robot.get_tool()
        if pose is None:
            raise RuntimeError('GetPose 无有效位姿')
        base_flange = tool_pose_matrix(pose)
        base_camera = base_flange @ self.flange_from_camera
        center_base = base_camera[:3, :3] @ center_camera + base_camera[:3, 3]
        direction_base = base_camera[:3, :3] @ direction_camera
        target = base_flange.copy()  # Preserve verified gripper attitude.
        target[:3, 3] = center_base - target[:3, :3] @ self.tcp_offset
        radius = float(np.linalg.norm(target[:2, 3]))
        if target[2, 3] < self.min_z or radius > self.max_radius:
            raise RuntimeError(f'抓取目标越界: {target[:3,3].round(1).tolist()} mm, radius={radius:.1f}')
        self.last_target = {'target': target, 'center_base': center_base, 'class': name, 'score': score}
        print(f'\n{self.detector_mode} RGB-D 目标')
        print(f'  class={name}, confidence={score:.3f}, depth points={len(points_camera)}')
        print(f'  center_camera={center_camera.round(2).tolist()} mm')
        print(f'  PCA direction camera={direction_camera.round(4).tolist()}')
        print(f'  center_base={center_base.round(2).tolist()} mm')
        print(f'  PCA direction base={direction_base.round(4).tolist()}')
        print(f'  flange target={target[:3,3].round(2).tolist()} mm')
        current_tip = (base_flange[:3, 3] +
                       base_flange[:3, :3] @ self.tcp_offset)
        vision_delta = center_base - current_tip
        print('  定位时 GetPose flange=%s mm' % base_flange[:3, 3].round(2).tolist())
        print('  定位时 TCP针尖(flange + R·offset)=%s mm' % current_tip.round(2).tolist())
        print('  视觉中心 - 当前针尖: Δ=%s mm, |Δ|=%.2f mm' % (
            vision_delta.round(2).tolist(), np.linalg.norm(vision_delta)))

    def pick(self):
        if not self.execute_enabled:
            raise RuntimeError('定位模式：先验证，再将 execute_enabled 改为 true')
        if self.last_target is None:
            raise RuntimeError('请先按 s 获得有效目标')
        self.ensure_robot_ready()
        target = self.last_target['target']
        pregrasp = target.copy(); pregrasp[2, 3] += self.pregrasp_height
        lift = target.copy(); lift[2, 3] += self.lift_height
        self.command_step_gripper(closed=False)
        for label, pose in [('预抓取', pregrasp), ('抓取', target)]:
            if not self.robot.move_pose(pose):
                raise RuntimeError(f'{label}运动失败')
        self.command_step_gripper(closed=True)
        if not self.robot.move_pose(lift):
            raise RuntimeError('抬升失败')

    def command_step_gripper(self, closed):
        """Publish one documented relative-angle command to the step-motor gripper."""
        if not self.step_gripper_enabled:
            self.get_logger().warn('step_gripper.enabled=false：跳过夹爪%s' % ('闭合' if closed else '张开'))
            return
        message = Motor()
        message.id = self.step_gripper_id
        message.speed = self.step_gripper_speed
        message.dir = self.step_gripper_close_dir if closed else self.step_gripper_open_dir
        message.mode = self.step_gripper_mode
        message.angle = self.step_gripper_angle
        message.state = 0
        message.sub_divide = self.step_gripper_subdivide
        self.step_gripper_pub.publish(message)
        action = '闭合' if closed else '张开'
        self.get_logger().info(
            '步进夹爪%s: topic=%s id=%d speed=%d dir=%d mode=%d angle=%d sub_divide=%d' % (
                action, self.step_gripper_topic, message.id, message.speed,
                message.dir, message.mode, message.angle, message.sub_divide))
        time.sleep(self.step_gripper_settle)


def main(args=None):
    # `ros2 run` does not load a package YAML automatically.  Add ours unless
    # the caller already passed an explicit params file.
    ros_args = list(sys.argv[1:] if args is None else args)
    if '--params-file' not in ros_args:
        config = Path(get_package_share_directory('yolo_seg_pick')) / 'config' / 'yolo_seg_pick.yaml'
        ros_args += ['--ros-args', '--params-file', str(config)]
    rclpy.init(args=ros_args)
    node = YoloSegPick()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        print('yolo_seg_pick> s=定位 t=拖拽 2=退出拖拽 c=类别 p=抓取 q=退出')
        node.print_classes()
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], .1)
            if not ready:
                continue
            command = sys.stdin.readline().strip().lower()
            if command == 'q':
                break
            try:
                if command == 's':
                    node.locate()
                elif command == 't':
                    node.ensure_robot_ready()
                    if not node.robot.start_drag():
                        raise RuntimeError('进入拖拽模式失败')
                    print('拖拽中：移动相机寻找目标；可按 s 定位，按 2 安全退出拖拽。')
                elif command == '2':
                    if not node.robot.stop_drag():
                        raise RuntimeError('退出拖拽或重新使能失败')
                    print('已退出拖拽，机械臂已重新使能。')
                elif command == 'c':
                    node.print_classes()
                elif command == 'p':
                    node.pick()
                else:
                    print('s=定位 t=拖拽 2=退出拖拽 c=类别 p=抓取 q=退出')
            except Exception as exc:
                node.get_logger().error(str(exc))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
