import os
import time

import rclpy
from apriltag_pick.pick_node import AprilTagPickNode
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from .executor import VisionExecutor
from .server import JsonLineServer


class ExecutorNode(Node):
    def __init__(self):
        super().__init__('vision_arm_executor')
        defaults = {
            'rpc_host': '127.0.0.1',
            'rpc_port': 17881,
            'rpc_allowed_clients': ['127.0.0.1/32'],
            'rpc_auth_token': '',
            'data_dir': '~/Robot_Arm_Project/data/vision_arm',
            'handeye_path': (
                '~/Robot_Arm_Project/scripts/active_handeye_calibration.json'),
            'tcp_calibration_path': '',
            'robot_speed': 15,
            'max_message_bytes': 65536,
            'socket_timeout_sec': 5.0,
            'task_ttl_sec': 3600.0,
            'apriltag_cache_ttl_sec': 60.0,
            'apriltag_max_robot_drift_mm': 10.0,
            'apriltag_max_robot_drift_deg': 3.0,
            'max_move_translation_mm': 500.0,
            'max_move_rotation_deg': 45.0,
            'workspace_min_xyz_mm': [-850.0, -850.0, 20.0],
            'workspace_max_xyz_mm': [850.0, 850.0, 1200.0],
            'icp_frames': 5,
            'icp_max_iters': 30,
        }
        self.cfg = {
            key: self.declare_parameter(key, value).value
            for key, value in defaults.items()}
        # Network settings are operational secrets/configuration. Environment
        # overrides avoid ROS2 CLI/YAML parsing failures for empty or special
        # token values and keep the token out of the process command line.
        rpc_host = os.environ.get('VISION_RPC_HOST', '').strip()
        vehicle_ip = os.environ.get('VEHICLE_IP', '').strip()
        rpc_token = os.environ.get('VISION_RPC_AUTH_TOKEN', '')
        if rpc_host:
            self.cfg['rpc_host'] = rpc_host
        if vehicle_ip:
            allowed_clients = ['127.0.0.1/32']
            if self.cfg['rpc_host'] not in (
                    '127.0.0.1', 'localhost', '0.0.0.0', '::1'):
                allowed_clients.append(self.cfg['rpc_host'] + '/32')
            if vehicle_ip not in ('127.0.0.1', '::1'):
                allowed_clients.append(vehicle_ip + '/32')
            self.cfg['rpc_allowed_clients'] = list(dict.fromkeys(
                allowed_clients))
        if rpc_token:
            self.cfg['rpc_auth_token'] = rpc_token
        self.latest_pc = None
        self.pc_seq = 0
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self._pc, 10)
        self.tag_node = AprilTagPickNode()
        # Only execution_enable on this executor may authorize a cached pick.
        self.tag_node.execute_enabled = False
        self.task_executor = VisionExecutor(
            self, self.cfg, tag_factory=lambda unused: self.tag_node)
        self.server = JsonLineServer(
            self.cfg['rpc_host'], self.cfg['rpc_port'],
            self.task_executor.submit,
            self.cfg['socket_timeout_sec'], self.cfg['max_message_bytes'],
            allowed_clients=self.cfg['rpc_allowed_clients'],
            auth_token=self.cfg['rpc_auth_token'])
        self.server.start()
        self.get_logger().info(
            'vision RPC listening on %s:%s; allowed_clients=%s; '
            'execution disabled' % (
                self.cfg['rpc_host'], self.cfg['rpc_port'],
                self.cfg['rpc_allowed_clients']))

    def _pc(self, message):
        try:
            from icp_servoing.pointcloud import cloud_to_xyz
            points = cloud_to_xyz(message)
        except Exception as error:
            # A malformed/transitional camera frame must not terminate the
            # executor and tear down the complete one-key stack.
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
    node = ExecutorNode()
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
