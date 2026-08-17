"""Configure, test and polarity-calibrate an ESTUN digital-output gripper."""

import argparse
from datetime import datetime, timezone
import json
import os
import time

import rclpy
from estun_codroid_bridge.srv import SetDo
from rclpy.node import Node

from vision_arm_executor.store import atomic_json


DEFAULT_CONFIG = os.path.expanduser(
    '~/Robot_Arm_Project/data/vision_arm_estun/'
    'calibration/gripper/active.json')


def parse_bool(value):
    value = str(value).strip().lower()
    if value in ('1', 'true', 'yes', 'on', 'high'):
        return True
    if value in ('0', 'false', 'no', 'off', 'low'):
        return False
    raise argparse.ArgumentTypeError('expected true/false')


class EstunGripperTool(Node):
    def __init__(self, namespace):
        super().__init__('estun_gripper_tool')
        namespace = namespace.rstrip('/')
        self.set_client = self.create_client(SetDo, namespace + '/set_do')

    def _call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=min(1.0, timeout)):
            raise RuntimeError('ESTUN DO service unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('ESTUN DO service timed out')
        return future.result()

    def set_value(self, port, value):
        request = SetDo.Request()
        request.port = int(port)
        request.value = bool(value)
        response = self._call(self.set_client, request)
        if not response.success:
            raise RuntimeError(response.message or 'SetDo failed')

def save_config(path, port, active_high, namespace):
    payload = {
        'schema_version': 1,
        'robot_model': 'ESTUN',
        'created_at': datetime.now(timezone.utc).isoformat(),
        'service_namespace': namespace,
        'do_port': int(port),
        'active_high': bool(active_high),
        'closed_value': bool(active_high),
        'open_value': not bool(active_high),
    }
    atomic_json(os.path.expanduser(path), payload)
    return payload


def load_config(path):
    try:
        with open(os.path.expanduser(path), encoding='utf-8') as stream:
            payload = json.load(stream)
        return int(payload['do_port']), bool(payload['active_high'])
    except FileNotFoundError:
        return None, None
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) \
            as error:
        raise RuntimeError('invalid gripper config: %s' % error)


def yes(prompt):
    while True:
        answer = input(prompt + ' [y/n] ').strip().lower()
        if answer in ('y', 'yes'):
            return True
        if answer in ('n', 'no'):
            return False


def parser():
    result = argparse.ArgumentParser(
        description='ESTUN DO gripper test and polarity calibration')
    result.add_argument(
        'action', choices=['open', 'close', 'cycle', 'calibrate'])
    result.add_argument('--port', type=int)
    result.add_argument('--active-high', type=parse_bool)
    result.add_argument('--service-namespace', default='/estun_codroid')
    result.add_argument('--config', default=DEFAULT_CONFIG)
    result.add_argument('--save', action='store_true',
                        help='save --port and --active-high after the action')
    return result


def main(args=None):
    options, ros_args = parser().parse_known_args(args)
    saved_port, saved_active_high = load_config(options.config)
    port = options.port if options.port is not None else (saved_port or 18)
    active_high = (
        options.active_high if options.active_high is not None
        else saved_active_high if saved_active_high is not None else True)
    if not 1 <= port <= 256:
        raise SystemExit('Codroid DO port must be in 1..256')
    rclpy.init(args=ros_args)
    node = EstunGripperTool(options.service_namespace)
    try:
        if options.action == 'open':
            node.set_value(port, not active_high)
            print('夹爪打开命令已发送。')
        elif options.action == 'close':
            node.set_value(port, active_high)
            print('夹爪闭合命令已发送。')
        elif options.action == 'cycle':
            node.set_value(port, not active_high)
            time.sleep(1.0)
            node.set_value(port, active_high)
            time.sleep(1.0)
            node.set_value(port, not active_high)
            print('夹爪开-合-开测试完成。')
        elif options.action == 'calibrate':
            print('将依次输出低、高电平；观察夹爪，不要把手放在夹爪内。')
            node.set_value(port, False)
            low_closed = yes('DO低电平时夹爪是否闭合？')
            node.set_value(port, True)
            high_closed = yes('DO高电平时夹爪是否闭合？')
            if low_closed == high_closed:
                raise RuntimeError(
                    '两种电平的机械状态相同，无法确定极性；检查接线/气源')
            active_high = high_closed
            node.set_value(port, not active_high)
            options.save = True
            print('标定结果: DO%d，闭合电平=%s' % (
                port, 'HIGH' if active_high else 'LOW'))
        if options.save:
            save_config(
                options.config, port, active_high,
                options.service_namespace)
            print('夹爪配置已保存: %s' % os.path.expanduser(options.config))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
