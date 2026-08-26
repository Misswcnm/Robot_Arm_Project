"""Isolate ESTUN Codroid MovJ(CPos) failures from AprilTag localization.

The default mode is read-only.  Motion modes deliberately provide no
Cartesian-to-joint fallback: a rejected CPos must remain visible so the first
failing endpoint can be identified.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
from apriltag_pick.transforms import (
    load_handeye,
    load_tcp_offset,
    matrix_to_tool_pose,
    tool_pose_matrix,
)
from apriltag_pick.rotation_compat import (
    rotation_from_matrix,
    rotation_magnitude,
)
from estun_codroid_bridge.srv import GetRobotState

from .robot import EstunRobot


DEFAULT_CACHE = (
    '~/Robot_Arm_Project/data/vision_arm_estun/apriltag_cache.json')


def _matrix(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (4, 4) or not np.all(np.isfinite(result)):
        raise ValueError('%s must be a finite 4x4 matrix' % name)
    return result


def _rotation_error_deg(first, second):
    delta = rotation_from_matrix(second[:3, :3]) * rotation_from_matrix(
        first[:3, :3]).inv()
    return float(np.degrees(rotation_magnitude(delta)))


def load_probe_data(cache_path, pregrasp_mm=80.0):
    """Load a cache record and independently recompute all motion endpoints."""
    cache_path = Path(cache_path).expanduser().resolve()
    with cache_path.open(encoding='utf-8') as stream:
        cache = json.load(stream)
    if not isinstance(cache, dict):
        raise ValueError('AprilTag cache root must be an object')

    localization = cache.get('localization')
    if not isinstance(localization, dict):
        raise ValueError('cache is missing localization')
    detection = _matrix(
        localization['base_flange_at_detection'],
        'localization.base_flange_at_detection')
    target = _matrix(
        localization.get('base_flange_target', cache.get('target')),
        'localization.base_flange_target')
    cached_target = _matrix(cache['target'], 'target')
    base_tag = _matrix(localization['base_tag'], 'localization.base_tag')
    camera_tag = _matrix(
        localization['camera_tag'], 'localization.camera_tag')

    tcp_path = cache['tcp_calibration_path']
    handeye_path = cache['handeye_path']
    tcp = load_tcp_offset(tcp_path)
    tcp_norm = float(np.linalg.norm(tcp))
    if tcp_norm <= 1e-9:
        raise ValueError('TCP offset norm must be non-zero')
    handeye = load_handeye(handeye_path)

    pregrasp = target.copy()
    pregrasp[:3, 3] -= target[:3, :3] @ (tcp / tcp_norm) * float(
        pregrasp_mm)
    predicted_tip = np.asarray(
        localization['predicted_contact_tip'], dtype=float).reshape(-1)
    if predicted_tip.size != 3 or not np.all(np.isfinite(predicted_tip)):
        raise ValueError('predicted_contact_tip must contain 3 finite values')

    chain_tag = detection @ handeye @ camera_tag
    compensated_tip = target[:3, 3] + target[:3, :3] @ tcp
    target_pose = np.asarray(matrix_to_tool_pose(target), dtype=float)
    roundtrip = tool_pose_matrix(target_pose)
    return {
        'cache_path': cache_path,
        'cache': cache,
        'detection': detection,
        'target': target,
        'pregrasp': pregrasp,
        'base_tag': base_tag,
        'chain_tag': chain_tag,
        'tcp_offset': tcp,
        'predicted_tip': predicted_tip,
        'compensated_tip': compensated_tip,
        'target_pose': target_pose,
        'pregrasp_pose': np.asarray(matrix_to_tool_pose(pregrasp), dtype=float),
        'detection_pose': np.asarray(
            matrix_to_tool_pose(detection), dtype=float),
        'cache_target_difference_mm': float(np.linalg.norm(
            cached_target[:3, 3] - target[:3, 3])),
        'cache_target_rotation_difference_deg': _rotation_error_deg(
            cached_target, target),
        'tag_chain_difference_mm': float(np.linalg.norm(
            chain_tag[:3, 3] - base_tag[:3, 3])),
        'tag_chain_rotation_difference_deg': _rotation_error_deg(
            chain_tag, base_tag),
        'tcp_closure_difference_mm': float(np.linalg.norm(
            compensated_tip - predicted_tip)),
        'target_roundtrip_matrix_max_error': float(np.max(np.abs(
            roundtrip - target))),
        'target_rotation_difference_deg': _rotation_error_deg(
            detection, target),
    }


def codroid_movj_preview(pose, speed, acceleration=None):
    """Mirror the C++ bridge's CodroidApi movJ(PointType::Cart) payload."""
    pose = np.asarray(pose, dtype=float).reshape(-1)
    if pose.size != 6 or not np.all(np.isfinite(pose)):
        raise ValueError('pose must contain 6 finite values')
    acceleration = speed if acceleration is None else acceleration
    cpos = {
        'x': float(pose[0]),
        'y': float(pose[1]),
        'z': float(pose[2]),
        'a': float(pose[3]),
        'b': float(pose[4]),
        'c': float(pose[5]),
        'e': 0.0,
        'poscfg': {'mode': -1, **{
            'cf%d' % index: 0 for index in range(1, 8)}},
    }
    return {
        'type': 'movj',
        'target': {'type': 'cpos', 'cpos': cpos},
        'speed': {
            'sper': float(speed), 'stcp': 0.0, 'sori': 0.0,
            'sexjl': 0.0, 'sexjr': 0.0,
        },
        'acc': {
            'aper': float(acceleration), 'atcp': 0.0, 'aori': 0.0,
            'aexjl': 0.0, 'aexjr': 0.0,
        },
    }


def _format_pose(pose):
    return '[%s]' % ', '.join('%.3f' % value for value in pose)


def print_probe_data(data, speed):
    detection = data['detection_pose']
    pregrasp = data['pregrasp_pose']
    target = data['target_pose']
    print('AprilTag cache: %s' % data['cache_path'])
    print('record_id: %s' % data['cache'].get('record_id', '<missing>'))
    print('cached_at: %s' % data['cache'].get('cached_at', '<missing>'))
    print('tag_id: %s, frames=%s, spread=%.3f mm' % (
        data['cache'].get('tag_id'), data['cache'].get('frames'),
        float(data['cache'].get('spread_mm', float('nan')))))
    print('\nIndependent geometry checks')
    print('  observation flange xyzrpy: %s' % _format_pose(detection))
    print('  pregrasp   flange xyzrpy: %s' % _format_pose(pregrasp))
    print('  contact    flange xyzrpy: %s' % _format_pose(target))
    print('  observation -> pregrasp: %.3f mm' % np.linalg.norm(
        pregrasp[:3] - detection[:3]))
    print('  observation -> contact:  %.3f mm' % np.linalg.norm(
        target[:3] - detection[:3]))
    print('  pregrasp -> contact:      %.3f mm' % np.linalg.norm(
        target[:3] - pregrasp[:3]))
    for name, pose in (
            ('observation', detection), ('pregrasp', pregrasp),
            ('contact', target)):
        print('  %-11s |xy|=%7.3f mm  |xyz|=%7.3f mm' % (
            name, np.linalg.norm(pose[:2]), np.linalg.norm(pose[:3])))
    print('  observation -> target rotation: %.9f deg' %
          data['target_rotation_difference_deg'])
    print('  cache target duplicate: %.9f mm / %.9f deg' % (
        data['cache_target_difference_mm'],
        data['cache_target_rotation_difference_deg']))
    print('  handeye chain closure:   %.9f mm / %.9f deg' % (
        data['tag_chain_difference_mm'],
        data['tag_chain_rotation_difference_deg']))
    print('  TCP compensation closure: %.9f mm' %
          data['tcp_closure_difference_mm'])
    print('  matrix -> XYZ-RPY -> matrix max error: %.3e' %
          data['target_roundtrip_matrix_max_error'])
    print('  tcp_offset_flange_mm: %s' % _format_pose(
        data['tcp_offset']))
    print('  predicted contact tip: %s' % _format_pose(
        data['predicted_tip']))
    print('  compensated target tip: %s' % _format_pose(
        data['compensated_tip']))

    print('\nExact pregrasp MovJ(CPos) payload preview')
    print(json.dumps(codroid_movj_preview(pregrasp, speed),
                     ensure_ascii=False, indent=2))


def load_observation_joints(manifest_path, label):
    path = Path(manifest_path).expanduser().resolve()
    with path.open(encoding='utf-8') as stream:
        manifest = json.load(stream)
    if isinstance(manifest, dict):
        points = manifest.get('points', [manifest])
    elif isinstance(manifest, list):
        points = manifest
    else:
        raise ValueError('station manifest must be an object or list')
    matches = [item for item in points if item.get('label') == label]
    if len(matches) != 1:
        raise ValueError('manifest must contain exactly one label=%s' % label)
    joints = np.asarray(
        matches[0]['apriltag']['observe_joints'], dtype=float).reshape(-1)
    if joints.size != 6 or not np.all(np.isfinite(joints)):
        raise ValueError('observe_joints must contain 6 finite values')
    return joints


def parse_fractions(text):
    values = [float(value.strip()) for value in text.split(',')
              if value.strip()]
    if not values or any(not np.isfinite(value) or value <= 0.0 or value > 1.0
                         for value in values):
        raise ValueError('fractions must be comma-separated values in (0, 1]')
    if any(second <= first for first, second in zip(values, values[1:])):
        raise ValueError('fractions must be strictly increasing')
    return values


def parse_vector3(text, name='vector'):
    values = np.asarray([
        float(value.strip()) for value in text.split(',')], dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError('%s must contain three comma-separated values' % name)
    return values


def _confirm(options, data, fractions):
    print('\nWARNING: the following test moves real hardware.')
    print('  mode=%s speed=%d no_fallback=true' % (
        options.mode, options.speed))
    if options.mode == 'delta':
        print('  direct current CPos delta=%s mm' % _format_pose(
            options.delta))
    elif options.skip_observation:
        print('  observation APos is explicitly skipped')
    else:
        print('  observation joints=%s' % _format_pose(
            load_observation_joints(options.manifest, options.label)))
    if options.mode == 'sweep':
        print('  fractions=%s (1.0 is pregrasp)' % fractions)
    if options.mode == 'contact':
        print('  path=observation -> pregrasp -> contact')
    print('  A kernel rejection stops immediately; no reset and no gripper action.')
    if options.yes:
        return
    answer = input('Type MOVE to continue: ').strip().upper()
    if answer != 'MOVE':
        raise RuntimeError('motion cancelled')


def _move_and_report(robot, name, pose, timeout):
    def state_snapshot():
        response = robot._call(
            robot._get_state, GetRobotState.Request(), timeout=3.0)
        if response is None or not response.success:
            return '<unavailable>'
        return 'state=%d status_flag=0x%08x' % (
            int(response.state), int(response.status_flag))

    before = robot.get_tool(warn=False)
    state_before = state_snapshot()
    status = robot.movj_pose_status(
        pose, label='probe %s MovJ(CPos)' % name, timeout=timeout)
    after = robot.get_tool(warn=False)
    state_after = state_snapshot()
    print('\n[%s] status=%s' % (name, status))
    print('  requested: %s' % _format_pose(pose))
    print('  before:    %s' % (
        _format_pose(before) if before is not None else '<unavailable>'))
    print('  after:     %s' % (
        _format_pose(after) if after is not None else '<unavailable>'))
    print('  state:     %s -> %s' % (state_before, state_after))
    if after is not None:
        position, rotation = EstunRobot._pose_residual(after, pose)
        print('  residual:  %.3f mm / %.3f deg' % (position, rotation))
    return status == 'arrived'


def execute_probe(options, data, fractions):
    if options.mode != 'delta' and not options.manifest:
        raise ValueError('--manifest is required for every motion mode')
    _confirm(options, data, fractions)
    rclpy.init(args=[])
    node = rclpy.create_node('estun_apriltag_motion_probe')
    node.declare_parameter(
        'estun_service_namespace', options.service_namespace)
    node.declare_parameter('estun_motion_timeout_sec', options.timeout_sec)
    robot = EstunRobot(node, speed=options.speed)
    try:
        if not robot._get_state.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(
                '%s/get_robot_state is unavailable: the C++ Codroid bridge '
                'is not running. Start it in another terminal with: ros2 '
                'launch estun_codroid_bridge codroid_bridge.launch.py '
                'robot_ip:=192.168.2.5 robot_port:=9000' %
                options.service_namespace.rstrip('/'))
        state_response = robot._call(
            robot._get_state, GetRobotState.Request(), timeout=3.0)
        if state_response is None or not state_response.success:
            raise RuntimeError('failed to query Codroid robot state')
        if int(state_response.state) != robot.ESTUN_STATE_AUTO:
            raise RuntimeError(
                'robot must already be in Codroid Auto state=4; actual '
                'state=%d status_flag=0x%08x. Probe will not enable or '
                'recover it' % (int(state_response.state),
                                int(state_response.status_flag)))
        if options.mode == 'delta':
            current_pose = robot.get_tool(warn=False)
            if current_pose is None:
                raise RuntimeError('current CPos is unavailable')
            target_pose = np.asarray(current_pose, dtype=float)
            target_pose[:3] += options.delta
            print('\n[delta] direct MovJ(CPos), no APos command')
            print('  current: %s' % _format_pose(current_pose))
            print('  delta:   %s mm' % _format_pose(options.delta))
            return 0 if _move_and_report(
                robot, 'current-delta', target_pose,
                options.timeout_sec) else 2

        observation_joints = load_observation_joints(
            options.manifest, options.label)
        if options.skip_observation:
            print('\n[observation] skipped by --skip-observation')
            actual_observation = robot.get_tool(warn=False)
            if actual_observation is None:
                raise RuntimeError('current CPos is unavailable')
            position, rotation = EstunRobot._pose_residual(
                actual_observation, data['detection_pose'])
            print('  current vs cached observation: %.3f mm / %.3f deg' % (
                position, rotation))
            if position > 10.0 or rotation > 3.0:
                raise RuntimeError(
                    '--skip-observation requires current pose within 10 mm '
                    'and 3 deg of the cached observation pose')
        else:
            print('\n[observation] MovJ(APos), joints=%s' % _format_pose(
                observation_joints))
            if not robot.movj(observation_joints):
                raise RuntimeError('observation MovJ(APos) failed')
            actual_observation = robot.get_tool(warn=False)
        if actual_observation is not None:
            position, rotation = EstunRobot._pose_residual(
                actual_observation, data['detection_pose'])
            print('  cached observation residual: %.3f mm / %.3f deg' % (
                position, rotation))
        if options.mode == 'observation':
            return 0

        if options.mode == 'pregrasp':
            return 0 if _move_and_report(
                robot, 'pregrasp-direct', data['pregrasp_pose'],
                options.timeout_sec) else 2

        if options.mode == 'sweep':
            start = data['detection_pose']
            finish = data['pregrasp_pose']
            for fraction in fractions:
                pose = start.copy()
                pose[:3] = start[:3] + fraction * (finish[:3] - start[:3])
                if not _move_and_report(
                        robot, 'pregrasp-%05.1f%%' % (100.0 * fraction),
                        pose, options.timeout_sec):
                    print('\nFIRST FAILING FRACTION: %.3f' % fraction)
                    return 2
            return 0

        if not _move_and_report(
                robot, 'pregrasp', data['pregrasp_pose'],
                options.timeout_sec):
            return 2
        return 0 if _move_and_report(
            robot, 'contact', data['target_pose'],
            options.timeout_sec) else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parser():
    result = argparse.ArgumentParser(
        description=(
            'Inspect or isolate AprilTag ESTUN MovJ(CPos) kernel failures; '
            'no CPos-to-APos fallback is used.'))
    result.add_argument('--cache', default=DEFAULT_CACHE)
    result.add_argument('--pregrasp-mm', type=float, default=80.0)
    result.add_argument(
        '--mode', choices=('inspect', 'observation', 'delta', 'pregrasp',
                           'sweep', 'contact'), default='inspect')
    result.add_argument(
        '--execute', action='store_true',
        help='required in addition to a non-inspect mode')
    result.add_argument('--manifest')
    result.add_argument('--label', default='P1')
    result.add_argument(
        '--skip-observation', action='store_true',
        help='send the Cartesian test directly when already at observation')
    result.add_argument(
        '--delta-mm', default='5,0,0',
        help='base-frame XYZ millimetres for mode=delta')
    result.add_argument('--fractions', default='0.05,0.10,0.25,0.50,0.75,1.0')
    result.add_argument('--speed', type=int, default=5)
    result.add_argument('--timeout-sec', type=float, default=30.0)
    result.add_argument('--service-namespace', default='/estun_codroid')
    result.add_argument(
        '--yes', action='store_true',
        help='skip the interactive MOVE confirmation')
    return result


def main(args=None):
    options = parser().parse_args(args)
    if not np.isfinite(options.pregrasp_mm) or options.pregrasp_mm <= 0.0:
        raise SystemExit('--pregrasp-mm must be positive')
    if options.speed < 1 or options.speed > 100:
        raise SystemExit('--speed must be in 1..100')
    if not np.isfinite(options.timeout_sec) or options.timeout_sec <= 0.0:
        raise SystemExit('--timeout-sec must be positive')
    try:
        data = load_probe_data(options.cache, options.pregrasp_mm)
        fractions = parse_fractions(options.fractions)
        options.delta = parse_vector3(options.delta_mm, '--delta-mm')
        print_probe_data(data, options.speed)
        if options.mode == 'inspect':
            if options.execute:
                print('\n--execute ignored because mode=inspect')
            return 0
        if not options.execute:
            raise ValueError(
                'motion mode requires --execute; inspect mode never moves')
        return execute_probe(options, data, fractions)
    except (OSError, KeyError, TypeError, ValueError, RuntimeError,
            json.JSONDecodeError) as error:
        print('\nFAILED: %s' % error)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
