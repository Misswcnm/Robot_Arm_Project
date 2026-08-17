"""ESTUN entrypoint reusing the existing executor, protocol and storage."""

import os
import time

import rclpy
from apriltag_pick import pick_node as apriltag_pick_module
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from vision_arm_executor.executor import VisionExecutor
from vision_arm_executor.server import JsonLineServer
from vision_arm_executor.store import atomic_json, now

from .robot import EstunRobot


class EstunVisionExecutor(VisionExecutor):
    """Change only vendor metadata; task behavior remains the shared code."""

    def _status(self, request_id, action):
        metrics = self._owner_metrics()
        try:
            state = self._icp().robot_state()
            metrics.update({'robot_mode': state.mode, 'pose': state.pose})
            if state.mode is None or state.pose is None:
                return self.response(
                    request_id, action, 'failed',
                    'ESTUN Codroid state/get_pose unavailable',
                    'robot_unavailable', metrics=metrics)
        except Exception as error:
            metrics['robot_error'] = str(error)
            return self.response(
                request_id, action, 'failed', str(error),
                'robot_unavailable', metrics=metrics)
        return self.response(
            request_id, action, 'succeeded', 'status read', metrics=metrics)

    def _record(self, name, payload, record_id=None):
        import uuid

        record_id = record_id or name + '-' + uuid.uuid4().hex
        payload = dict(payload)
        payload.update(
            schema_version=2, record_id=record_id, created_at=now(),
            robot_model='ESTUN')
        versioned = os.path.join(
            self.root, 'records', name, record_id + '.json')
        active = os.path.join(self.root, name + '.json')
        atomic_json(versioned, payload)
        atomic_json(active, payload)
        return payload, [versioned, active]

    @staticmethod
    def _record_payload(record_id, values):
        payload = dict(values)
        payload.update(
            schema_version=2,
            record_id=record_id,
            created_at=now(),
            robot_model='ESTUN')
        return payload


class EstunExecutorNode(Node):
    def __init__(self):
        super().__init__('vision_arm_executor')
        defaults = {
            'rpc_host': '127.0.0.1',
            'rpc_port': 17881,
            'rpc_allowed_clients': ['127.0.0.1/32'],
            'rpc_auth_token': '',
            'data_dir': '~/Robot_Arm_Project/data/vision_arm_estun',
            'handeye_path': (
                '~/Robot_Arm_Project/data/vision_arm_estun/'
                'calibration/handeye/active.json'),
            'tcp_calibration_path': (
                '~/Robot_Arm_Project/data/vision_arm_estun/'
                'calibration/tcp/active.json'),
            'robot_speed': 15,
            'estun_reset_joints': [
                86.0, 23.0, -112.0, -176.0, -85.0, 0.0],
            'reset_after_each_point': True,
            'max_message_bytes': 65536,
            'socket_timeout_sec': 5.0,
            'task_ttl_sec': 3600.0,
            'apriltag_cache_ttl_sec': 60.0,
            'apriltag_max_robot_drift_mm': 10.0,
            'apriltag_max_robot_drift_deg': 3.0,
            'motion_limits_enabled': False,
            'icp_frames': 5,
            'icp_max_iters': 30,
            'icp_final_trans_thresh_mm': 4.0,
            'icp_final_rot_thresh_deg': 1.5,
            'icp_final_stable_frames': 1,
        }
        self.cfg = {
            key: self.declare_parameter(key, value).value
            for key, value in defaults.items()}
        rpc_host = os.environ.get('VISION_RPC_HOST', '').strip()
        vehicle_ip = os.environ.get('VEHICLE_IP', '').strip()
        rpc_token = os.environ.get('VISION_RPC_AUTH_TOKEN', '')
        if rpc_host:
            self.cfg['rpc_host'] = rpc_host
        if vehicle_ip:
            clients = ['127.0.0.1/32']
            if self.cfg['rpc_host'] not in (
                    '127.0.0.1', 'localhost', '0.0.0.0', '::1'):
                clients.append(self.cfg['rpc_host'] + '/32')
            if vehicle_ip not in ('127.0.0.1', '::1'):
                clients.append(vehicle_ip + '/32')
            self.cfg['rpc_allowed_clients'] = list(dict.fromkeys(clients))
        if rpc_token:
            self.cfg['rpc_auth_token'] = rpc_token

        self.latest_pc = None
        self.pc_seq = 0
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self._pc, 10)

        # AprilTagPickNode contains the proven localization/pick workflow but
        # constructs its robot internally. Replace that construction boundary
        # only; the imported package source remains untouched.
        original_robot = apriltag_pick_module.CR5Robot
        apriltag_pick_module.CR5Robot = EstunRobot
        try:
            self.tag_node = apriltag_pick_module.AprilTagPickNode()
        finally:
            apriltag_pick_module.CR5Robot = original_robot
        self.tag_node.execute_enabled = False

        self.task_executor = EstunVisionExecutor(
            self,
            self.cfg,
            robot_factory=EstunRobot,
            tag_factory=lambda unused: self.tag_node)
        self.server = JsonLineServer(
            self.cfg['rpc_host'],
            self.cfg['rpc_port'],
            self.task_executor.submit,
            self.cfg['socket_timeout_sec'],
            self.cfg['max_message_bytes'],
            allowed_clients=self.cfg['rpc_allowed_clients'],
            auth_token=self.cfg['rpc_auth_token'])
        self.server.start()
        self.get_logger().info(
            'ESTUN vision RPC listening on %s:%s; allowed_clients=%s; '
            'NX protocol unchanged; execution disabled' % (
                self.cfg['rpc_host'], self.cfg['rpc_port'],
                self.cfg['rpc_allowed_clients']))

    def _pc(self, message):
        try:
            from icp_servoing.pointcloud import cloud_to_xyz
            points = cloud_to_xyz(message)
        except Exception as error:
            self.get_logger().error(
                'PointCloud2 conversion failed; frame skipped: %s' % error)
            return
        self.latest_pc = points
        self.pc_seq += 1

    def wait_fresh(self, timeout=2.0):
        target = self.pc_seq + 1
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.pc_seq >= target:
                return True
            time.sleep(0.02)
        return False

    def destroy_node(self):
        self.server.close()
        self.tag_node.destroy_node()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EstunExecutorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(node.tag_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
