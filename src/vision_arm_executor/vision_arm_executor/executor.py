import math
import os
import threading
import time
import traceback

import numpy as np
from scipy.spatial.transform import Rotation

from .store import atomic_json, digest, load_json, now


RESOURCE_ACTIONS = {
    'vision_icp_record_a',
    'vision_icp_record_b',
    'vision_icp_align',
    'vision_icp_align_and_move_b',
    'apriltag_locate',
    'apriltag_validate',
    'apriltag_pick',
}
TERMINAL = {'succeeded', 'failed', 'timeout', 'cancelled'}


class TaskCancelled(RuntimeError):
    pass


class TaskTimeout(RuntimeError):
    pass


class VisionExecutor:
    """ROS2 visual-task state machine. This module never imports rospy."""

    def __init__(self, node, cfg, robot_factory=None, servo_factory=None,
                 tag_factory=None):
        self.node = node
        self.cfg = cfg
        self.lock = threading.RLock()
        self.tasks = {}
        self.cancelled = set()
        self.deadlines = {}
        self.owner = None
        # Deliberately process-local: every restart returns to disabled.
        self.execution_enabled = False
        self.root = os.path.expanduser(cfg['data_dir'])
        self.robot_factory = robot_factory
        self.servo_factory = servo_factory
        self.tag_factory = tag_factory
        self.robot = None
        self.robot_initialized = False
        self.servo = None
        self.tag = None
        os.makedirs(self.root, exist_ok=True)
        os.makedirs(os.path.join(self.root, 'records'), exist_ok=True)

    def response(self, request_id, action, status='accepted', message='',
                 code='', **values):
        result = {
            'request_id': request_id,
            'action': action,
            'backend': 'vision',
            'status': status,
            'progress': 0.0,
            'message': message,
            'error_code': code,
            'metrics': {},
            'artifacts': [],
            'started_at': now(),
            'finished_at': '',
        }
        result.update(values)
        return result

    def submit(self, request):
        request_id = request['request_id']
        action = request['action']
        with self.lock:
            if request_id in self.tasks:
                return self.tasks[request_id].copy()
            if action in {
                    'health', 'arm_status', 'task_status', 'task_cancel',
                    'execution_enable', 'execution_disable'}:
                result = self._immediate(request)
                self.tasks[request_id] = result
                return result.copy()
            if action in RESOURCE_ACTIONS and self.owner:
                result = self.response(
                    request_id, action, 'failed',
                    'another visual task owns the mechanical arm', 'busy',
                    metrics={'owner': self.owner})
                self.tasks[request_id] = result
                return result.copy()
            record = self.response(request_id, action, 'accepted', 'queued')
            self.tasks[request_id] = record
            if action in RESOURCE_ACTIONS:
                self.owner = request_id
            timeout = max(0.1, float(request.get('timeout_sec', 120.0)))
            self.deadlines[request_id] = time.monotonic() + timeout
            worker = threading.Thread(
                target=self._run, args=(request,), daemon=True)
            worker.start()
            return record.copy()

    def _immediate(self, request):
        action = request['action']
        request_id = request['request_id']
        if action == 'health':
            return self.response(
                request_id, action, 'succeeded', 'vision RPC healthy',
                metrics=self._owner_metrics())
        if action == 'task_status':
            target = request['params'].get('request_id')
            with self.lock:
                task = self.tasks.get(target)
                if task:
                    return task.copy()
            return self.response(
                request_id, action, 'failed', 'unknown request_id',
                'not_found')
        if action == 'task_cancel':
            target = request['params'].get('request_id')
            with self.lock:
                task = self.tasks.get(target)
                if not task:
                    return self.response(
                        request_id, action, 'failed', 'unknown request_id',
                        'not_found')
                if task.get('status') in TERMINAL:
                    return self.response(
                        request_id, action, 'succeeded',
                        'target task is already terminal',
                        metrics={'target_request_id': target,
                                 'target_status': task.get('status')})
                self.cancelled.add(target)
            return self.response(
                request_id, action, 'succeeded',
                'cancel requested at next safety checkpoint',
                metrics={'target_request_id': target})
        if action == 'execution_enable':
            self.execution_enabled = True
            self._sync_tag_execution(False)
            return self.response(
                request_id, action, 'succeeded',
                'real motion explicitly enabled')
        if action == 'execution_disable':
            self.execution_enabled = False
            self._sync_tag_execution(False)
            with self.lock:
                if self.owner:
                    self.cancelled.add(self.owner)
            return self.response(
                request_id, action, 'succeeded',
                'real motion disabled; active task will stop at a safety checkpoint')
        return self._status(request_id, action)

    def _owner_metrics(self):
        return {
            'execution_enabled': self.execution_enabled,
            'owner': self.owner,
            'lock_held': bool(self.owner),
        }

    def _status(self, request_id, action):
        metrics = self._owner_metrics()
        try:
            robot = self._robot(initialize=False)
            metrics.update({
                'robot_mode': self._robot_mode(robot),
                'pose': robot.get_tool(warn=False),
            })
        except Exception as error:
            metrics['robot_error'] = str(error)
        return self.response(
            request_id, action, 'succeeded', 'status read', metrics=metrics)

    def _run(self, request):
        request_id = request['request_id']
        action = request['action']
        with self.lock:
            self.tasks[request_id].update(
                status='running', progress=0.01, message='running',
                started_at=now())
            started_at = self.tasks[request_id]['started_at']
        try:
            self._checkpoint(request_id)
            result = self._dispatch(request)
            self._checkpoint(request_id)
            result['finished_at'] = now()
            result['started_at'] = started_at
        except TaskCancelled as error:
            result = self.response(
                request_id, action, 'cancelled', str(error), 'cancelled')
            result.update(started_at=started_at, finished_at=now())
        except TaskTimeout as error:
            result = self.response(
                request_id, action, 'timeout', str(error), 'timeout')
            result.update(started_at=started_at, finished_at=now())
        except Exception as error:
            result = self.response(
                request_id, action, 'failed', str(error), 'execution_error',
                metrics={'traceback': traceback.format_exc()[-2000:]})
            result.update(started_at=started_at, finished_at=now())
        finally:
            with self.lock:
                self.tasks[request_id] = result
                self.cancelled.discard(request_id)
                self.deadlines.pop(request_id, None)
                if self.owner == request_id:
                    self.owner = None

    def _checkpoint(self, request_id):
        with self.lock:
            if request_id in self.cancelled:
                raise TaskCancelled('cancelled at a safe checkpoint')
            deadline = self.deadlines.get(request_id)
        if deadline is not None and time.monotonic() >= deadline:
            raise TaskTimeout('task deadline reached at a safe checkpoint')

    def _should_stop(self, request_id):
        with self.lock:
            cancelled = request_id in self.cancelled
            deadline = self.deadlines.get(request_id)
        return cancelled or (
            deadline is not None and time.monotonic() >= deadline)

    def _progress(self, request_id, completed, total, result=None):
        progress = min(0.95, max(0.05, float(completed) / max(1, total)))
        metrics = self._jsonify(result or {})
        with self.lock:
            task = self.tasks.get(request_id)
            if task and task.get('status') == 'running':
                task.update(progress=progress, metrics=metrics)

    @staticmethod
    def _robot_mode(robot):
        getter = getattr(robot, 'get_mode', None)
        if getter is None:
            getter = getattr(robot, 'get_robot_mode')
        return getter()

    def _robot(self, initialize=True):
        if self.robot is None:
            if self.robot_factory:
                self.robot = self.robot_factory(
                    self.node, int(self.cfg['robot_speed']))
            else:
                from icp_servoing.robot import CR5Robot
                self.robot = CR5Robot(
                    self.node, speed=int(self.cfg['robot_speed']))
        if initialize and not self.robot_initialized:
            initialized = self.robot.init()
            if initialized is False:
                raise RuntimeError('CR5 initialization failed')
            self.robot_initialized = True
        return self.robot

    def _servo(self):
        if self.servo is None:
            from icp_servoing.handeye import load_X
            from icp_servoing.servoing import VisualServo
            factory = self.servo_factory or VisualServo
            self.servo = factory(
                self._robot(),
                load_X(os.path.expanduser(self.cfg['handeye_path'])))
            self.servo._pc_node = self.node
        return self.servo

    def _tag(self):
        if self.tag is None:
            if self.tag_factory:
                self.tag = self.tag_factory(self.node)
            else:
                from apriltag_pick.pick_node import AprilTagPickNode
                self.tag = AprilTagPickNode()
        return self.tag

    def _sync_tag_execution(self, enabled):
        if self.tag is not None:
            self.tag.execute_enabled = bool(enabled)

    def _motion_safety(self, request, robot=None, target=None):
        if request.get('dry_run'):
            return
        request_id = request['request_id']
        self._checkpoint(request_id)
        if not self.execution_enabled:
            raise RuntimeError('execution_disabled')
        robot = robot or self._robot()
        mode = self._robot_mode(robot)
        dragging = bool(getattr(robot, 'dragging', False))
        if dragging or mode == getattr(robot, 'MODE_BACKDRIVE', 6):
            raise RuntimeError('robot_in_drag_mode')
        if mode != getattr(robot, 'MODE_ENABLED', 5):
            raise RuntimeError('robot_not_enabled')
        if target is not None:
            current = robot.get_tool(warn=False)
            if current is None:
                raise RuntimeError('GetPose failed')
            target = np.asarray(target, dtype=float)
            xyz = target[:3, 3]
            low = np.asarray(self.cfg['workspace_min_xyz_mm'], dtype=float)
            high = np.asarray(self.cfg['workspace_max_xyz_mm'], dtype=float)
            if np.any(xyz < low) or np.any(xyz > high):
                raise RuntimeError('target_outside_workspace')
            current_rotation = Rotation.from_euler(
                'xyz', current[3:6], degrees=True).as_matrix()
            relative_rotation = current_rotation.T @ target[:3, :3]
            translation_delta = float(
                np.linalg.norm(xyz - np.asarray(current[:3], dtype=float)))
            rotation_delta = float(np.degrees(np.linalg.norm(
                Rotation.from_matrix(relative_rotation).as_rotvec())))
            if translation_delta > float(self.cfg['max_move_translation_mm']):
                raise RuntimeError('target_translation_exceeds_limit')
            if rotation_delta > float(self.cfg['max_move_rotation_deg']):
                raise RuntimeError('target_rotation_exceeds_limit')

    def _record(self, name, payload, record_id=None):
        record_id = record_id or (
            name + '-' + str(int(time.time() * 1000)))
        payload = dict(payload)
        payload.update(
            schema_version=2, record_id=record_id, created_at=now(),
            robot_model='CR5', user_id=0, tool_id=0)
        versioned = os.path.join(
            self.root, 'records', name, record_id + '.json')
        active = os.path.join(self.root, name + '.json')
        atomic_json(versioned, payload)
        atomic_json(active, payload)
        return payload, [versioned, active]

    def _calibration_digest(self, key):
        path = os.path.expanduser(self.cfg[key])
        return path, digest(path)

    def _dispatch(self, request):
        action = request['action']
        request_id = request['request_id']
        dry_run = bool(request.get('dry_run', False))

        if action == 'vision_icp_record_a':
            if dry_run:
                return self.response(
                    request_id, action, 'succeeded',
                    'ICP A dry-run passed; no template was overwritten',
                    metrics={'dry_run': True})
            self._motion_safety(request)
            servo = self._servo()
            frames = int(request['params'].get(
                'frames', self.cfg['icp_frames']))
            if not servo.record_template(frames):
                raise RuntimeError('ICP template recording failed')
            record_id = 'icp_a-' + str(int(time.time() * 1000))
            template_path = os.path.join(
                self.root, 'records', 'icp_a', record_id + '.npz')
            servo.save_template(template_path)
            handeye_path, handeye_sha = self._calibration_digest(
                'handeye_path')
            payload, artifacts = self._record(
                'icp_a', {
                    'request_id': request_id,
                    'T_base_flange_A': servo._T_base_tool_ref.tolist(),
                    'template_frames': frames,
                    'template_path': template_path,
                    'template_sha256': digest(template_path),
                    'handeye_path': handeye_path,
                    'handeye_sha256': handeye_sha,
                }, record_id=record_id)
            artifacts.insert(0, template_path)
            return self.response(
                request_id, action, 'succeeded', 'ICP reference A recorded',
                metrics={'record_id': payload['record_id'], 'frames': frames},
                artifacts=artifacts)

        if action == 'vision_icp_record_b':
            a_record = load_json(os.path.join(self.root, 'icp_a.json'))
            if not a_record:
                raise RuntimeError('ICP reference A missing')
            if not request['params'].get('manual_positioned', False):
                raise RuntimeError(
                    'manual_positioned=true required after the standard '
                    'StopDrag/Enable/User0/Tool0 sequence')
            if dry_run:
                return self.response(
                    request_id, action, 'succeeded',
                    'ICP B dry-run passed; no record was overwritten',
                    metrics={'a_record_id': a_record['record_id']})
            self._motion_safety(request)
            pose = self._robot().get_tool(warn=False)
            if pose is None:
                raise RuntimeError('GetPose failed')
            transform = np.eye(4)
            transform[:3, 3] = pose[:3]
            transform[:3, :3] = Rotation.from_euler(
                'xyz', pose[3:], degrees=True).as_matrix()
            delta = np.linalg.inv(
                np.asarray(a_record['T_base_flange_A'], dtype=float)) @ transform
            payload, artifacts = self._record('icp_b', {
                'request_id': request_id,
                'T_base_flange_B': transform.tolist(),
                'T_A_to_B': delta.tolist(),
                'a_record_id': a_record['record_id'],
                'template_sha256': a_record['template_sha256'],
                'handeye_sha256': a_record['handeye_sha256'],
            })
            return self.response(
                request_id, action, 'succeeded',
                'ICP relative B recorded',
                metrics={
                    'record_id': payload['record_id'],
                    'a_record_id': a_record['record_id'],
                }, artifacts=artifacts)

        if action in {'vision_icp_align', 'vision_icp_align_and_move_b'}:
            a_record = self._load_and_validate_icp_a()
            servo = self._servo()
            if dry_run:
                return self.response(
                    request_id, action, 'succeeded',
                    'ICP artifacts and calibration passed dry-run',
                    metrics={'converged': False, 'dry_run': True,
                             'a_record_id': a_record['record_id']})
            self._motion_safety(request)
            servo.load_template(a_record['template_path'])
            maximum = int(request['params'].get(
                'max_iters', self.cfg['icp_max_iters']))
            servo.motion_guard = lambda target: self._motion_safety(
                request, target=target)
            try:
                converged = servo.align(
                    maximum,
                    should_stop=lambda: self._should_stop(request_id),
                    progress_callback=lambda current, total, result: self._progress(
                        request_id, current, total, result))
            finally:
                servo.motion_guard = None
            self._checkpoint(request_id)
            metrics = self._jsonify(servo.last_align_result)
            metrics['a_record_id'] = a_record['record_id']
            if not converged:
                raise RuntimeError(
                    'ICP did not converge; B motion prohibited: ' +
                    str(metrics.get('reason', 'unknown')))
            if action == 'vision_icp_align_and_move_b':
                b_record = self._load_and_validate_icp_b(a_record)
                current = servo._get_tool_matrix()
                if current is None:
                    raise RuntimeError('GetPose failed after ICP convergence')
                target = current @ np.asarray(
                    b_record['T_A_to_B'], dtype=float)
                self._motion_safety(request, target=target)
                self._checkpoint(request_id)
                if not self._robot().movj_pose(target, 'ICP A-to-B'):
                    raise RuntimeError('relative B move failed')
                metrics.update(
                    b_record_id=b_record['record_id'],
                    T_target=target.tolist())
            return self.response(
                request_id, action, 'succeeded', 'ICP aligned',
                metrics=metrics)

        if action == 'apriltag_locate':
            tag = self._tag()
            target = tag.locate()
            localization = tag.last_localization
            handeye_path = os.path.expanduser(tag.handeye_path)
            tcp_path = os.path.expanduser(tag.tcp_calibration_path)
            payload, artifacts = self._record('apriltag_cache', {
                'request_id': request_id,
                'cached_at': now(),
                'tag_id': tag._target_tag_id(),
                'tag_frame': tag.tag_frame,
                'target': target.tolist(),
                'localization': self._jsonify(localization),
                'frames': int(localization['frames']),
                'spread_mm': float(localization['spread_mm']),
                'ttl_sec': float(self.cfg['apriltag_cache_ttl_sec']),
                'handeye_path': handeye_path,
                'handeye_sha256': digest(handeye_path),
                'tcp_calibration_path': tcp_path,
                'tcp_calibration_sha256': digest(tcp_path),
            })
            debug_path = localization.get('debug_image_path')
            if debug_path:
                artifacts.insert(0, debug_path)
            return self.response(
                request_id, action, 'succeeded',
                'AprilTag target located and cached',
                metrics={
                    'cache_record_id': payload['record_id'],
                    'tag_id': payload['tag_id'],
                    'frames': payload['frames'],
                    'spread_mm': payload['spread_mm'],
                }, artifacts=artifacts)

        if action == 'apriltag_validate':
            tag = self._tag()
            mode = tag.robot.get_mode()
            if not tag.robot.dragging and mode != tag.robot.MODE_BACKDRIVE:
                raise RuntimeError(
                    'apriltag_validate requires manual drag positioning at '
                    'the Tag center before recording truth')
            value = tag.record_manual_tag_center()
            return self.response(
                request_id, action, 'succeeded',
                'AprilTag manual validation truth recorded',
                metrics={'prediction_minus_truth_mm': self._jsonify(value),
                         'error_norm_mm': float(np.linalg.norm(value))})

        if action == 'apriltag_pick':
            tag = self._tag()
            cache = self._load_and_validate_tag_cache(tag)
            target = np.asarray(cache['target'], dtype=float)
            if dry_run:
                return self.response(
                    request_id, action, 'succeeded',
                    'AprilTag cached target passed dry-run',
                    metrics={'cache_record_id': cache['record_id'],
                             'tag_id': cache['tag_id'], 'dry_run': True})
            self._check_tag_robot_drift(tag, cache)
            self._sync_tag_execution(True)
            tag.last_localization = self._restore_localization(
                cache['localization'])

            def tag_safety(stage, stage_target):
                self._checkpoint(request_id)
                self._motion_safety(
                    request, robot=tag.robot, target=stage_target)

            try:
                self._motion_safety(request, robot=tag.robot, target=target)
                tag.pick(safety_check=tag_safety)
            finally:
                # The executor is the sole enable authority.
                self._sync_tag_execution(False)
            return self.response(
                request_id, action, 'succeeded',
                'AprilTag cached pick complete',
                metrics={'cache_record_id': cache['record_id'],
                         'tag_id': cache['tag_id']})

        raise RuntimeError('unsupported action')

    def _load_and_validate_icp_a(self):
        record = load_json(os.path.join(self.root, 'icp_a.json'))
        if not record:
            raise RuntimeError('ICP reference A missing')
        template = record.get('template_path')
        if not template or not os.path.isfile(template):
            raise RuntimeError('ICP template artifact missing')
        if digest(template) != record.get('template_sha256'):
            raise RuntimeError('ICP template checksum mismatch')
        _, current_handeye = self._calibration_digest('handeye_path')
        if current_handeye != record.get('handeye_sha256'):
            raise RuntimeError('hand-eye calibration changed since ICP A')
        return record

    def _load_and_validate_icp_b(self, a_record):
        record = load_json(os.path.join(self.root, 'icp_b.json'))
        if not record:
            raise RuntimeError('ICP B missing')
        if record.get('a_record_id') != a_record.get('record_id'):
            raise RuntimeError('ICP B is not bound to the active ICP A')
        if record.get('template_sha256') != a_record.get('template_sha256'):
            raise RuntimeError('ICP A/B template version mismatch')
        if record.get('handeye_sha256') != a_record.get('handeye_sha256'):
            raise RuntimeError('ICP A/B calibration version mismatch')
        return record

    def _load_and_validate_tag_cache(self, tag):
        path = os.path.join(self.root, 'apriltag_cache.json')
        cache = load_json(path)
        if not cache:
            raise RuntimeError('AprilTag cache missing')
        age = time.time() - os.path.getmtime(path)
        if age > float(self.cfg['apriltag_cache_ttl_sec']):
            raise RuntimeError('AprilTag cache expired')
        if cache.get('tag_id') != tag._target_tag_id():
            raise RuntimeError('AprilTag ID changed since locate')
        if cache.get('handeye_sha256') != digest(tag.handeye_path):
            raise RuntimeError('hand-eye calibration changed since locate')
        if cache.get('tcp_calibration_sha256') != digest(
                tag.tcp_calibration_path):
            raise RuntimeError('TCP calibration changed since locate')
        return cache

    def _check_tag_robot_drift(self, tag, cache):
        current = tag.robot.get_tool(warn=False)
        detection = cache.get('localization', {}).get('flange_xyzrpy')
        if current is None or not detection:
            raise RuntimeError('cannot validate robot pose against Tag cache')
        translation = float(np.linalg.norm(
            np.asarray(current[:3]) - np.asarray(detection[:3])))
        current_rotation = Rotation.from_euler(
            'xyz', current[3:6], degrees=True)
        detection_rotation = Rotation.from_euler(
            'xyz', detection[3:6], degrees=True)
        rotation = float(np.degrees(
            (current_rotation * detection_rotation.inv()).magnitude()))
        if translation > float(self.cfg['apriltag_max_robot_drift_mm']):
            raise RuntimeError('robot moved since AprilTag locate')
        if rotation > float(self.cfg['apriltag_max_robot_drift_deg']):
            raise RuntimeError('robot rotated since AprilTag locate')

    @staticmethod
    def _restore_localization(value):
        result = dict(value)
        array_keys = {
            'base_flange_at_detection', 'camera_tag', 'base_tag',
            'predicted_contact_tip', 'base_flange_target'}
        for key in array_keys:
            if key in result:
                result[key] = np.asarray(result[key], dtype=float)
        return result

    def _jsonify(self, value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, (np.floating, np.integer)):
            return value.item()
        if isinstance(value, dict):
            return {key: self._jsonify(item)
                    for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._jsonify(item) for item in value]
        if isinstance(value, float) and not math.isfinite(value):
            return str(value)
        return value
