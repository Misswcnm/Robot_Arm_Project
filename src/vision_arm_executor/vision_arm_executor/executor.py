import math
import os
import threading
import time
import traceback
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

from .backends import AprilTagBackend, IcpBackend, RobotState
from .station_store import (
    APRILTAG_POINT, ICP_POINT, StationStore, normalize_point_type,
    point_type_of)
from .store import atomic_json, digest, load_json, now
from .teaching import TeachingSession


RESOURCE_ACTIONS = {
    'robot_reset',
    'robot_enable',
    'robot_disable',
    'robot_start_drag',
    'robot_stop_drag',
    'robot_clear_error',
    'vision_point_teach',
    'vision_station_execute',
    'vision_icp_align',
    'vision_icp_align_and_move_b',
    'apriltag_locate',
    'apriltag_locate_and_pick',
    'apriltag_locate_and_touch',
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
                 tag_factory=None, icp_backend_factory=None,
                 apriltag_backend_factory=None):
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
        self.station_store = StationStore(self.root)
        self.teaching = TeachingSession()
        self.robot_factory = robot_factory
        self.servo_factory = servo_factory
        self.tag_factory = tag_factory
        self.icp_backend_factory = icp_backend_factory
        self.apriltag_backend_factory = apriltag_backend_factory
        self._icp_instance = None
        self._apriltag_instance = None
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
                    'execution_enable', 'execution_disable',
                    'vision_station_points'}:
                result = self._immediate(request)
                self.tasks[request_id] = result
                return result.copy()
            if (action in RESOURCE_ACTIONS and
                    self.teaching.active and
                    action != 'vision_point_teach'):
                result = self.response(
                    request_id, action, 'failed',
                    'teaching drag is active; finish teaching first',
                    'teaching_active',
                    metrics=self._owner_metrics())
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
            if self.teaching.active:
                return self.response(
                    request_id, action, 'failed',
                    'cannot enable production execution during teaching',
                    'teaching_active', metrics=self._owner_metrics())
            self.execution_enabled = True
            self._disable_apriltag_motion()
            return self.response(
                request_id, action, 'succeeded',
                'real motion explicitly enabled')
        if action == 'execution_disable':
            self.execution_enabled = False
            self._disable_apriltag_motion()
            with self.lock:
                if self.owner:
                    self.cancelled.add(self.owner)
            return self.response(
                request_id, action, 'succeeded',
                'real motion disabled; active task will stop at a safety checkpoint')
        if action == 'vision_station_points':
            mapid, poseid = self.station_store.identity(
                request.get('params', {}))
            summaries, path = self.station_store.summaries(mapid, poseid)
            return self.response(
                request_id, action, 'succeeded',
                'station points read',
                metrics={'mapid': mapid, 'poseid': poseid,
                         'points': summaries},
                artifacts=[path] if os.path.isfile(path) else [])
        return self._status(request_id, action)

    def _owner_metrics(self):
        with self.lock:
            metrics = {
                'execution_enabled': self.execution_enabled,
                'owner': self.owner,
                'lock_held': bool(self.owner),
            }
            metrics.update(self.teaching.metrics())
            return metrics

    def _reset_icp_teaching(self):
        with self.lock:
            self.teaching.reset()

    @property
    def icp_teaching(self):
        """Read-only compatibility view for existing diagnostics/tests."""
        with self.lock:
            return self.teaching.legacy_snapshot()

    def _status(self, request_id, action):
        metrics = self._owner_metrics()
        try:
            state = self._icp().robot_state()
            metrics.update({'robot_mode': state.mode, 'pose': state.pose})
            if state.mode is None or state.pose is None:
                return self.response(
                    request_id, action, 'failed',
                    'CR5 RobotMode/GetPose unavailable', 'robot_unavailable',
                    metrics=metrics)
        except Exception as error:
            metrics['robot_error'] = str(error)
            return self.response(
                request_id, action, 'failed', str(error),
                'robot_unavailable', metrics=metrics)
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
                self.cancelled.discard(request_id)
                self.deadlines.pop(request_id, None)
                if self.owner == request_id:
                    self.owner = None
                # Terminal responses are observed only after this critical
                # section, so report the released ownership state instead of
                # the transient worker ownership captured inside _dispatch().
                metrics = result.get('metrics')
                if isinstance(metrics, dict) and (
                        'owner' in metrics or 'lock_held' in metrics):
                    metrics.update(
                        owner=self.owner,
                        lock_held=bool(self.owner))
                self.tasks[request_id] = result

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

    def _icp(self):
        if self._icp_instance is None:
            factory = self.icp_backend_factory or IcpBackend
            self._icp_instance = factory(
                self.node, self.cfg, robot_factory=self.robot_factory,
                servo_factory=self.servo_factory)
        return self._icp_instance

    def _apriltag(self):
        if self._apriltag_instance is None:
            factory = self.apriltag_backend_factory or AprilTagBackend
            self._apriltag_instance = factory(
                self.node, tag_factory=self.tag_factory)
        return self._apriltag_instance

    def _disable_apriltag_motion(self):
        # The backend only grants motion during its bounded pick() call.
        if self._apriltag_instance is not None:
            self._apriltag_instance.disable_motion()

    def _motion_safety(self, request, backend=None, target=None):
        if request.get('dry_run'):
            return
        request_id = request['request_id']
        self._checkpoint(request_id)
        if not self.execution_enabled:
            raise RuntimeError('execution_disabled')
        selected_backend = backend or self._icp()
        if backend is None:
            selected_backend.initialize_robot()
        state = selected_backend.robot_state()
        if state.dragging or state.mode == state.backdrive_mode:
            raise RuntimeError('robot_in_drag_mode')
        if state.mode != state.enabled_mode:
            raise RuntimeError('robot_not_enabled')
        if target is not None:
            current = state.pose
            if current is None:
                raise RuntimeError('GetPose failed')
            # Vendor deployments may intentionally delegate all Cartesian
            # amplitude limits to the robot controller (for example during a
            # supervised demonstration).  Connection/mode/pose checks above
            # remain mandatory.
            if not bool(self.cfg.get('motion_limits_enabled', True)):
                return
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
            translation_limit = float(
                self.cfg['max_move_translation_mm'])
            rotation_limit = float(self.cfg['max_move_rotation_deg'])
            if translation_delta > translation_limit:
                raise RuntimeError(
                    'target_translation_exceeds_limit: %.1fmm > %.1fmm' %
                    (translation_delta, translation_limit))
            if rotation_delta > rotation_limit:
                raise RuntimeError(
                    'target_rotation_exceeds_limit: %.1fdeg > %.1fdeg' %
                    (rotation_delta, rotation_limit))

    def _record(self, name, payload, record_id=None):
        record_id = record_id or (
            name + '-' + uuid.uuid4().hex)
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

    def _station_identity(self, params):
        return self.station_store.identity(params)

    def _station_manifest_path(self, mapid, poseid):
        return self.station_store.manifest_path(mapid, poseid)

    def _load_station_points(self, mapid, poseid):
        return self.station_store.load(mapid, poseid)

    def _next_station_label(self, mapid, poseid):
        return self.station_store.next_label(mapid, poseid)

    def _teaching_identity(self, params):
        return self.station_store.teaching_identity(params)

    def _save_station_icp_point(self, mapid, poseid, label,
                                a_record, b_record):
        return self.station_store.save_icp(
            mapid, poseid, label, a_record, b_record)

    def _icp_record_dir(self, context=None):
        context = context or {}
        required = ('mapid', 'poseid', 'label')
        missing = [
            key for key in required
            if context.get(key) in (None, '')]
        if missing:
            raise RuntimeError(
                'ICP storage requires mapid, poseid and label')
        return self.station_store.record_dir(
            context['mapid'], context['poseid'], context['label'],
            ICP_POINT)

    @staticmethod
    def _record_payload(record_id, values):
        payload = dict(values)
        payload.update(
            schema_version=2,
            record_id=record_id,
            created_at=now(),
            robot_model='CR5',
            user_id=0,
            tool_id=0)
        return payload

    def _select_station_icp_points(self, params):
        return self.station_store.select(
            params or {}, expected_type=ICP_POINT)

    def _record_icp_a(self, request_id, frames, teaching=False,
                      context=None):
        record_id = 'icp_a-' + uuid.uuid4().hex
        record_dir = self._icp_record_dir(context)
        template_path = os.path.join(record_dir, 'icp_a.npz')
        metadata_path = os.path.join(record_dir, 'icp_a.json')
        backend = self._icp()
        if teaching:
            reference = backend.record_reference_teaching(
                frames, template_path)
        else:
            reference = backend.record_reference(frames, template_path)
        handeye_path, handeye_sha = self._calibration_digest('handeye_path')
        values = {
                'request_id': request_id,
                'T_base_flange_A': reference.flange_transform.tolist(),
                'template_frames': reference.frames,
                'template_path': reference.template_path,
                'template_sha256': digest(reference.template_path),
                'handeye_path': handeye_path,
                'handeye_sha256': handeye_sha,
                'teaching_drag': bool(teaching),
                'record_dir': record_dir,
            }
        if reference.joints is not None:
            values['joints_A'] = list(reference.joints)
        if context:
            values.update(context)
        payload = self._record_payload(record_id, values)
        atomic_json(metadata_path, payload)
        return payload, [reference.template_path, metadata_path]

    def _record_icp_b(self, request_id, a_record, transform,
                      teaching=False, context=None, joints=None):
        record_dir = self._icp_record_dir(context)
        metadata_path = os.path.join(record_dir, 'icp_b.json')
        record_id = 'icp_b-' + uuid.uuid4().hex
        delta = np.linalg.inv(
            np.asarray(a_record['T_base_flange_A'], dtype=float)) @ transform
        values = {
            'request_id': request_id,
            'T_base_flange_B': transform.tolist(),
            'T_A_to_B': delta.tolist(),
            'a_record_id': a_record['record_id'],
            'template_sha256': a_record['template_sha256'],
            'handeye_sha256': a_record['handeye_sha256'],
            'teaching_drag': bool(teaching),
            'record_dir': record_dir,
        }
        if joints is not None:
            values['joints_B'] = [float(value) for value in joints]
        if context:
            values.update(context)
        payload = self._record_payload(record_id, values)
        atomic_json(metadata_path, payload)
        return payload, [metadata_path]

    def _locate_apriltag(self, request_id, persist=True, tag_id=None,
                         tag_offset_xyz_mm=None):
        backend = self._apriltag()
        if tag_id is None and tag_offset_xyz_mm is None:
            location = backend.locate()
        else:
            location = backend.locate(
                tag_id=tag_id, tag_offset_xyz_mm=tag_offset_xyz_mm)
        identity = location.identity
        localization = location.localization
        metrics = {
            'tag_id': identity.tag_id,
            'frames': int(localization['frames']),
            'spread_mm': float(localization['spread_mm']),
            'tag_offset_xyz_mm': self._jsonify(
                localization.get('tag_offset_xyz_mm', [0, 0, 0])),
        }
        artifacts = []
        debug_path = localization.get('debug_image_path')
        if debug_path:
            artifacts.append(debug_path)
        if not persist:
            return None, metrics, artifacts
        payload, saved = self._record('apriltag_cache', {
            'request_id': request_id,
            'cached_at': now(),
            'tag_id': identity.tag_id,
            'tag_frame': identity.tag_frame,
            'target': location.target.tolist(),
            'localization': self._jsonify(localization),
            'frames': metrics['frames'],
            'spread_mm': metrics['spread_mm'],
            'ttl_sec': float(self.cfg['apriltag_cache_ttl_sec']),
            'handeye_path': identity.handeye_path,
            'handeye_sha256': digest(identity.handeye_path),
            'tcp_calibration_path': identity.tcp_calibration_path,
            'tcp_calibration_sha256': digest(identity.tcp_calibration_path),
        })
        artifacts.extend(saved)
        metrics['cache_record_id'] = payload['record_id']
        return payload, metrics, artifacts

    def _pick_apriltag_cache(self, request, backend, cache):
        request_id = request['request_id']
        target = np.asarray(cache['target'], dtype=float)
        backend.prepare_pick(cache['localization'])
        self._check_tag_robot_drift(backend, cache)

        def tag_safety(stage, stage_target):
            self._checkpoint(request_id)
            self._motion_safety(
                request, backend=backend, target=stage_target)

        self._motion_safety(request, backend=backend, target=target)
        backend.pick(safety_check=tag_safety)

    @staticmethod
    def _teach_command(value):
        aliases = {
            0: 'start',
            1: 'record_a',
            2: 'record_b',
            3: 'finish',
            4: 'status',
            '0': 'start',
            '1': 'record_a',
            '2': 'record_b',
            '3': 'finish',
            '4': 'status',
            'start': 'start',
            'a': 'record_a',
            'record_a': 'record_a',
            'b': 'record_b',
            'record_b': 'record_b',
            'finish': 'finish',
            'stop': 'finish',
            'status': 'status',
        }
        command = aliases.get(value)
        if command is None:
            command = aliases.get(str(value).strip().lower())
        if command is None:
            raise RuntimeError(
                'unsupported teaching command; use '
                '0/start, 1/record_a, 2/record_b, 3/finish or 4/status')
        return command

    @staticmethod
    def _apriltag_teaching_config(params):
        params = params or {}
        if params.get('tag_id') in (None, ''):
            raise RuntimeError(
                'tag_id is required for AprilTag teaching')
        try:
            tag_id = int(params.get('tag_id'))
        except (TypeError, ValueError):
            raise RuntimeError('tag_id must be a non-negative integer')
        if tag_id < 0:
            raise RuntimeError('tag_id must be a non-negative integer')
        offset = np.asarray(
            params.get('tag_offset_xyz_mm', [0, 0, 0]),
            dtype=float).reshape(-1)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise RuntimeError(
                'tag_offset_xyz_mm must contain three finite numbers')
        return tag_id, offset.tolist()

    def _dispatch_point_teach(self, request):
        request_id = request['request_id']
        action = request['action']
        dry_run = bool(request.get('dry_run', False))
        params = request.get('params', {})
        command = self._teach_command(params.get('command'))
        point_type = normalize_point_type(
            params.get('point_type'), required=False)
        if point_type is None and self.teaching.active:
            point_type = self.teaching.point_type
        if point_type is None:
            point_type = normalize_point_type(None, required=True)
        if dry_run:
            return self.response(
                request_id, action, 'succeeded',
                'teaching dry-run passed; robot and records unchanged',
                metrics={'command': command, 'point_type': point_type,
                         'dry_run': True,
                         **self._owner_metrics()})

        backend = self._icp()
        if command == 'start':
            mapid, poseid, label = self._teaching_identity(
                params)
            tag_id = None
            tag_offset = None
            if point_type == APRILTAG_POINT:
                tag_id, tag_offset = self._apriltag_teaching_config(params)
            if self.teaching.active:
                current = self.teaching.identity()
                if current != (mapid, poseid, label, point_type):
                    raise RuntimeError(
                        'another teaching point is active: '
                        'mapid=%s poseid=%s label=%s point_type=%s' % current)
                state = backend.robot_state()
                return self.response(
                    request_id, action, 'succeeded',
                    'teaching drag already active',
                    metrics={'command': command, 'robot_mode': state.mode,
                             **self._owner_metrics()})
            state = backend.start_drag()
            self.execution_enabled = False
            self._disable_apriltag_motion()
            with self.lock:
                self.teaching.begin(
                    point_type, mapid, poseid, label,
                    tag_id=tag_id, tag_offset_xyz_mm=tag_offset)
            if point_type == ICP_POINT:
                instruction = 'drag robot to A then send command 1'
            else:
                instruction = (
                    'drag robot to the AprilTag observation pose then send '
                    'command 1')
            return self.response(
                request_id, action, 'succeeded',
                '%s teaching started for %s/%s/%s; %s' %
                (self.teaching.kind, mapid, poseid, label, instruction),
                metrics={'command': command, 'robot_mode': state.mode,
                         **self._owner_metrics()})

        if command == 'status':
            state = backend.robot_state()
            return self.response(
                request_id, action, 'succeeded',
                'teaching status read',
                metrics={'command': command, 'robot_mode': state.mode,
                         'pose': state.pose, **self._owner_metrics()})

        if command == 'finish':
            if not self.teaching.active:
                return self.response(
                    request_id, action, 'succeeded',
                    'teaching is already idle',
                    metrics={'command': command,
                             **self._owner_metrics()})
            state = backend.stop_drag_and_enable()
            self.execution_enabled = False
            self._disable_apriltag_motion()
            self._reset_icp_teaching()
            return self.response(
                request_id, action, 'succeeded',
                'teaching aborted safely; robot enabled and no point '
                'was committed by this command',
                metrics={'command': command, 'robot_mode': state.mode,
                         **self._owner_metrics()})

        if not self.teaching.active:
            raise RuntimeError(
                'teaching is not active; send command 0/start first')

        if point_type != self.teaching.point_type:
            raise RuntimeError(
                'teaching point_type does not match active session')

        if command == 'record_a':
            if self.teaching.point_type == APRILTAG_POINT:
                if self.teaching.phase != 'awaiting_pose':
                    raise RuntimeError('AprilTag observation pose is already saved')
                pose = backend.flange_pose_teaching()
                joints_getter = getattr(
                    backend, 'flange_joints_teaching', None)
                joints = (
                    joints_getter() if joints_getter is not None else None)
                context = {
                    'mapid': self.teaching.mapid,
                    'poseid': self.teaching.poseid,
                    'label': self.teaching.label,
                    'tag_id': self.teaching.tag_id,
                    'tag_offset_xyz_mm': list(
                        self.teaching.tag_offset_xyz_mm),
                }
                with self.lock:
                    self.teaching.phase = 'finishing'
                state = backend.stop_drag_and_enable()
                self.execution_enabled = False
                self._disable_apriltag_motion()
                try:
                    unused_point, manifest_path = (
                        self.station_store.save_apriltag(
                            context['mapid'], context['poseid'],
                            context['label'], pose, context['tag_id'],
                            context['tag_offset_xyz_mm'], joints))
                    next_label = self._next_station_label(
                        context['mapid'], context['poseid'])
                except Exception:
                    self._reset_icp_teaching()
                    raise
                self._reset_icp_teaching()
                return self.response(
                    request_id, action, 'succeeded',
                    'AprilTag point %s saved; teaching finished and robot '
                    'enabled' % context['label'],
                    metrics={
                        'command': command,
                        'point_type': APRILTAG_POINT,
                        'saved_label': context['label'],
                        'next_label': next_label,
                        'mapid': context['mapid'],
                        'poseid': context['poseid'],
                        'tag_id': context['tag_id'],
                        'tag_offset_xyz_mm':
                            context['tag_offset_xyz_mm'],
                        'observe_flange_pose': pose,
                        'observe_joints': joints,
                        'robot_mode': state.mode,
                        **self._owner_metrics(),
                    },
                    artifacts=[manifest_path])

            if self.teaching.phase != 'awaiting_a':
                raise RuntimeError(
                    'ICP A is already recorded; drag robot to B and send '
                    'command 2')
            frames = int(params.get(
                'frames', self.cfg['icp_frames']))
            context = {
                'mapid': self.teaching.mapid,
                'poseid': self.teaching.poseid,
                'label': self.teaching.label,
            }
            payload, artifacts = self._record_icp_a(
                request_id, frames, teaching=True, context=context)
            with self.lock:
                self.teaching.phase = 'awaiting_b'
                self.teaching.a_record_id = payload['record_id']
                self.teaching.b_record_id = None
            return self.response(
                request_id, action, 'succeeded',
                'ICP A point cloud and flange pose recorded; '
                'drag robot to B then send command 2',
                metrics={'command': command, 'record_id': payload['record_id'],
                         'frames': frames, **self._owner_metrics()},
                artifacts=artifacts)

        if self.teaching.point_type != ICP_POINT:
            raise RuntimeError(
                'command 2 is only used by ICP teaching; AprilTag saves with '
                'command 1')
        if self.teaching.phase != 'awaiting_b':
            raise RuntimeError(
                'ICP B cannot be recorded before A; send command 1 first')
        teaching_record_dir = self.station_store.record_dir(
            self.teaching.mapid, self.teaching.poseid, self.teaching.label,
            ICP_POINT)
        a_record = self._validate_icp_a_record(load_json(
            os.path.join(teaching_record_dir, 'icp_a.json'),
            strict=True))
        expected_a = self.teaching.a_record_id
        if not expected_a or a_record.get('record_id') != expected_a:
            raise RuntimeError(
                'active teaching session has no matching ICP A; '
                'send command 1 first')
        transform = backend.flange_transform_teaching()
        joints_getter = getattr(backend, 'flange_joints_teaching', None)
        joints = joints_getter() if joints_getter is not None else None
        context = {
            'mapid': self.teaching.mapid,
            'poseid': self.teaching.poseid,
            'label': self.teaching.label,
        }
        with self.lock:
            self.teaching.phase = 'finishing'

        # Complete the physical command sequence before acknowledging command 2:
        # StopDrag -> EnableRobot -> confirmed enabled mode.  Only then commit
        # the B record and station manifest.  A failed stop leaves the session
        # active in "finishing", so command 3 can safely retry the recovery.
        state = backend.stop_drag_and_enable()
        self.execution_enabled = False
        self._disable_apriltag_motion()
        try:
            payload, artifacts = self._record_icp_b(
                request_id, a_record, transform, teaching=True,
                context=context, joints=joints)
            saved_label = context['label']
            unused_point, manifest_path = self._save_station_icp_point(
                context['mapid'], context['poseid'], context['label'],
                a_record, payload)
            artifacts.append(manifest_path)
            next_label = self._next_station_label(
                context['mapid'], context['poseid'])
        except Exception:
            # The robot is already out of drag mode.  Do not advertise a live
            # session when persistence failed; command 1 can be retried after a
            # fresh type1/start request.
            self._reset_icp_teaching()
            raise

        self._reset_icp_teaching()
        return self.response(
            request_id, action, 'succeeded',
            'ICP point %s saved; teaching finished and robot enabled' %
            saved_label,
            metrics={'command': command, 'record_id': payload['record_id'],
                     'saved_label': saved_label,
                     'next_label': next_label,
                     'a_record_id': a_record['record_id'],
                     'mapid': context['mapid'],
                     'poseid': context['poseid'],
                     'robot_mode': state.mode,
                     **self._owner_metrics()},
            artifacts=artifacts)

    @staticmethod
    def _apriltag_point_config(point):
        apriltag = point.get('apriltag')
        if not isinstance(apriltag, dict):
            raise RuntimeError(
                'AprilTag point %s has no apriltag configuration' %
                str(point.get('label', '<unknown>')))
        pose = np.asarray(
            apriltag.get('observe_flange_pose'), dtype=float).reshape(-1)
        if pose.shape != (6,) or not np.all(np.isfinite(pose)):
            raise RuntimeError(
                'AprilTag point %s has invalid observe_flange_pose' %
                str(point.get('label', '<unknown>')))
        try:
            tag_id = int(apriltag.get('tag_id', 0))
        except (TypeError, ValueError):
            raise RuntimeError(
                'AprilTag point %s has invalid tag_id' %
                str(point.get('label', '<unknown>')))
        if tag_id < 0:
            raise RuntimeError(
                'AprilTag point %s has invalid tag_id' %
                str(point.get('label', '<unknown>')))
        offset = np.asarray(
            apriltag.get('tag_offset_xyz_mm', [0, 0, 0]),
            dtype=float).reshape(-1)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise RuntimeError(
                'AprilTag point %s has invalid tag_offset_xyz_mm' %
                str(point.get('label', '<unknown>')))
        raw_joints = apriltag.get('observe_joints')
        joints = None
        if raw_joints is not None:
            values = np.asarray(raw_joints, dtype=float).reshape(-1)
            if values.shape != (6,) or not np.all(np.isfinite(values)):
                raise RuntimeError(
                    'AprilTag point %s has invalid observe_joints' %
                    str(point.get('label', '<unknown>')))
            joints = values.astype(float).tolist()
        return pose.tolist(), joints, tag_id, offset.tolist()

    def _dispatch_station_execute(self, request):
        request_id = request['request_id']
        params = dict(request.get('params', {}))
        # Production is station-scoped.  point_type and label are teaching
        # metadata and must never split a vehicle arrival into partial runs.
        params.pop('point_type', None)
        params.pop('pointType', None)
        params.pop('label', None)
        mapid, poseid, points, manifest_path = self.station_store.select(params)
        dry_run = bool(request.get('dry_run'))
        if not dry_run and not self.execution_enabled:
            raise RuntimeError('execution_disabled')

        point_metrics = []
        artifacts = [manifest_path]
        logger = (
            self.node.get_logger()
            if self.node is not None and hasattr(self.node, 'get_logger')
            else None)
        for point_index, point in enumerate(points):
            self._checkpoint(request_id)
            label = str(point.get('label'))
            point_type = point_type_of(point)
            if logger is not None:
                logger.info(
                    'station %s/%s: start point %d/%d %s type=%d' %
                    (mapid, poseid, point_index + 1, len(points),
                     label, point_type))

            if point_type == ICP_POINT:
                nested = dict(request)
                nested['action'] = 'vision_icp_align_and_move_b'
                nested_params = dict(params)
                nested_params.update(
                    mapid=mapid,
                    poseid=poseid,
                    label=label,
                    point_type=ICP_POINT)
                nested['params'] = nested_params
                result = self._dispatch(nested)
                current = {
                    'label': label,
                    'point_type': ICP_POINT,
                }
                nested_metrics = dict(result.get('metrics', {}))
                nested_points = nested_metrics.pop('points', [])
                for key in ('mapid', 'poseid', 'labels', 'point_count'):
                    nested_metrics.pop(key, None)
                current.update(nested_metrics)
                if len(nested_points) == 1:
                    current.update(nested_points[0])
                if self._reset_after_completed_point(
                        request_id, mapid, poseid, label, point_type,
                        dry_run):
                    current['reset_completed'] = True
                point_metrics.append(current)
                artifacts.extend(result.get('artifacts', []))
                self._progress(
                    request_id, point_index + 1, len(points),
                    {'label': label, 'point_type': point_type})
                if logger is not None:
                    logger.info(
                        'station %s/%s: completed point %d/%d %s type=%d' %
                        (mapid, poseid, point_index + 1, len(points),
                         label, point_type))
                continue

            observe_pose, observe_joints, tag_id, offset = (
                self._apriltag_point_config(point))
            backend = self._apriltag()
            if dry_run:
                identity = backend.identity(tag_id)
                point_metrics.append({
                    'label': label,
                    'point_type': APRILTAG_POINT,
                    'dry_run': True,
                    'tag_id': identity.tag_id,
                    'tag_frame': identity.tag_frame,
                    'observe_flange_pose': observe_pose,
                    'observe_joints': observe_joints,
                    'tag_offset_xyz_mm': offset,
                })
                self._progress(
                    request_id, point_index + 1, len(points),
                    {'label': label, 'point_type': point_type})
                continue

            def observation_safety(unused_stage, target):
                self._checkpoint(request_id)
                self._motion_safety(
                    request, backend=backend, target=target)

            if observe_joints is None:
                observation_target = backend.move_to_observation(
                    observe_pose, observation_safety)
            else:
                observation_target = backend.move_to_observation(
                    observe_pose, observation_safety,
                    joints=observe_joints)
            payload, metrics, current_artifacts = self._locate_apriltag(
                request_id, persist=True, tag_id=tag_id,
                tag_offset_xyz_mm=offset)
            cache = self._load_and_validate_tag_cache(backend)
            self._pick_apriltag_cache(request, backend, cache)
            current = dict(metrics)
            current.update(
                label=label,
                point_type=APRILTAG_POINT,
                observe_flange_pose=observe_pose,
                observation_target=observation_target.tolist(),
                cache_record_id=payload['record_id'])
            if self._reset_after_completed_point(
                    request_id, mapid, poseid, label, point_type, dry_run):
                current['reset_completed'] = True
            point_metrics.append(current)
            artifacts.extend(current_artifacts)
            self._progress(
                request_id, point_index + 1, len(points),
                {'label': label, 'point_type': point_type})
            if logger is not None:
                logger.info(
                    'station %s/%s: completed point %d/%d %s type=%d' %
                    (mapid, poseid, point_index + 1, len(points),
                     label, point_type))
        return self.response(
            request_id, 'vision_station_execute', 'succeeded',
            ('station points passed dry-run; robot was not moved'
             if dry_run else 'station points completed'),
            metrics={
                'dry_run': dry_run,
                'mapid': mapid,
                'poseid': poseid,
                'labels': [
                    str(point.get('label')) for point in points],
                'point_count': len(point_metrics),
                'points': point_metrics,
            },
            artifacts=list(dict.fromkeys(artifacts)))

    def _reset_after_completed_point(self, request_id, mapid, poseid,
                                     label, point_type, dry_run):
        """Run command-1 reset semantics after one successful station point."""
        if dry_run or not bool(self.cfg.get(
                'reset_after_each_point', False)):
            return False
        self._checkpoint(request_id)
        result = self._icp().reset_robot()
        if result is False:
            raise RuntimeError(
                'point %s type=%d completed but reset position failed' %
                (label, point_type))
        logger = (
            self.node.get_logger()
            if self.node is not None and hasattr(self.node, 'get_logger')
            else None)
        if logger is not None:
            logger.info(
                'station %s/%s: point %s type=%d reset completed' %
                (mapid, poseid, label, point_type))
        return True

    def _dispatch(self, request):
        action = request['action']
        request_id = request['request_id']
        dry_run = bool(request.get('dry_run', False))

        if action == 'vision_point_teach':
            return self._dispatch_point_teach(request)

        if action == 'vision_station_execute':
            return self._dispatch_station_execute(request)

        if action in {
                'robot_reset', 'robot_enable', 'robot_disable',
                'robot_start_drag', 'robot_stop_drag',
                'robot_clear_error'}:
            if dry_run:
                return self.response(
                    request_id, action, 'succeeded',
                    '%s dry-run passed; robot state was not changed' % action,
                    metrics={'dry_run': True})
            backend = self._icp()
            if action in {'robot_start_drag', 'robot_stop_drag'}:
                if action == 'robot_start_drag':
                    state = backend.start_drag()
                else:
                    state = backend.stop_drag_and_enable()
                self.execution_enabled = False
                self._disable_apriltag_motion()
                return self.response(
                    request_id, action, 'succeeded',
                    '%s completed' % action,
                    metrics={'robot_mode': state.mode,
                             'dragging': state.dragging,
                             'pose': state.pose})
            method = {
                'robot_reset': 'reset_robot',
                'robot_enable': 'enable_robot',
                'robot_disable': 'disable_robot',
                'robot_clear_error': 'clear_error',
            }[action]
            result = getattr(backend, method)()
            if result is False:
                raise RuntimeError('%s failed' % action)
            if action in {'robot_reset', 'robot_disable'}:
                self.execution_enabled = False
                self._disable_apriltag_motion()
            metrics = {
                'robot_mode': (
                    result.mode if isinstance(result, RobotState)
                    else backend.robot_state().mode)}
            if action == 'robot_enable':
                metrics['already_enabled'] = bool(
                    getattr(backend, 'last_enable_was_noop', False))
            return self.response(
                request_id, action, 'succeeded', '%s completed' % action,
                metrics=metrics)

        if action in {'vision_icp_align', 'vision_icp_align_and_move_b'}:
            mapid, poseid, points, manifest_path = (
                self._select_station_icp_points(request['params']))
            artifacts = [manifest_path]
            profiles = []
            for point in points:
                a_record = self._validate_icp_a_record(
                    point.get('icp_a'))
                b_record = (
                    self._validate_icp_b_record(
                        point.get('icp_b'), a_record)
                    if action == 'vision_icp_align_and_move_b' else None)
                profiles.append(
                    (str(point.get('label')), a_record, b_record))
            station_metrics = {
                'mapid': mapid,
                'poseid': poseid,
                'labels': [profile[0] for profile in profiles],
            }
            backend = self._icp()
            if dry_run:
                return self.response(
                    request_id, action, 'succeeded',
                    'ICP station artifacts and calibration passed dry-run',
                    metrics={
                        'converged': False,
                        'dry_run': True,
                        'point_count': len(profiles),
                        'a_record_ids': [
                            profile[1]['record_id'] for profile in profiles],
                        **station_metrics,
                    }, artifacts=artifacts)
            maximum = int(request['params'].get(
                'max_iters', self.cfg['icp_max_iters']))
            point_metrics = []
            for point_index, (label, a_record, b_record) in enumerate(
                    profiles):
                recovery_args = (
                    a_record['T_base_flange_A'],
                    'ICP %s recovery-to-A' % (label or '<global>'),
                    lambda target: self._motion_safety(
                        request, backend=backend, target=target))
                if a_record.get('joints_A') is None:
                    recovery_target = backend.move_to_reference(*recovery_args)
                else:
                    recovery_target = backend.move_to_reference(
                        *recovery_args, joints=a_record['joints_A'])
                self._checkpoint(request_id)
                alignment = backend.align(
                    a_record['template_path'], maximum,
                    should_stop=lambda: self._should_stop(request_id),
                    progress_callback=(
                        lambda current, total, result, base=point_index:
                        self._progress(
                            request_id, base * maximum + current,
                            len(profiles) * maximum, result)),
                    motion_guard=lambda target: self._motion_safety(
                        request, backend=backend, target=target))
                self._checkpoint(request_id)
                current_metrics = self._jsonify(alignment.metrics)
                current_metrics.update(
                    label=label,
                    a_record_id=a_record['record_id'],
                    T_recovery_A=recovery_target.tolist())
                if not alignment.converged:
                    raise RuntimeError(
                        'ICP point %s did not converge; B motion prohibited: %s'
                        % (label or '<global>',
                           current_metrics.get('reason', 'unknown')))
                if action == 'vision_icp_align_and_move_b':
                    relative_args = (
                        b_record['T_A_to_B'],
                        'ICP %s A-to-B' % (label or '<global>'),
                        lambda value: self._motion_safety(
                            request, backend=backend, target=value))
                    if b_record.get('joints_B') is None:
                        target = backend.move_relative(*relative_args)
                    else:
                        target = backend.move_relative(
                            *relative_args,
                            reference_transform=b_record[
                                'T_base_flange_B'],
                            reference_joints=b_record['joints_B'])
                    current_metrics.update(
                        b_record_id=b_record['record_id'],
                        T_target=target.tolist())
                point_metrics.append(current_metrics)
            metrics = dict(station_metrics)
            metrics.update(
                converged=True, point_count=len(point_metrics),
                points=point_metrics)
            return self.response(
                request_id, action, 'succeeded',
                'ICP station points completed',
                metrics=metrics, artifacts=artifacts)

        if action in {
                'apriltag_locate', 'apriltag_locate_and_pick',
                'apriltag_locate_and_touch'}:
            execute_after_locate = action in {
                'apriltag_locate_and_pick',
                'apriltag_locate_and_touch'}
            if (execute_after_locate and
                    not dry_run and not self.execution_enabled):
                raise RuntimeError('execution_disabled')
            payload, metrics, artifacts = self._locate_apriltag(
                request_id, persist=not dry_run)
            if execute_after_locate:
                if dry_run:
                    metrics['dry_run'] = True
                    return self.response(
                        request_id, action, 'succeeded',
                        'AprilTag locate-and-touch dry-run passed; '
                        'robot was not moved',
                        metrics=metrics, artifacts=artifacts)
                backend = self._apriltag()
                cache = self._load_and_validate_tag_cache(backend)
                self._pick_apriltag_cache(request, backend, cache)
                return self.response(
                    request_id, action, 'succeeded',
                    'AprilTag located and touch point completed',
                    metrics=metrics, artifacts=artifacts)
            return self.response(
                request_id, action, 'succeeded',
                'AprilTag target located and cached',
                metrics=metrics, artifacts=artifacts)

        if action == 'apriltag_validate':
            value = self._apriltag().validate_manual()
            return self.response(
                request_id, action, 'succeeded',
                'AprilTag manual validation truth recorded',
                metrics={'prediction_minus_truth_mm': self._jsonify(value),
                         'error_norm_mm': float(np.linalg.norm(value))})

        if action == 'apriltag_pick':
            backend = self._apriltag()
            cache = self._load_and_validate_tag_cache(backend)
            if dry_run:
                return self.response(
                    request_id, action, 'succeeded',
                    'AprilTag cached target passed dry-run',
                    metrics={'cache_record_id': cache['record_id'],
                             'tag_id': cache['tag_id'], 'dry_run': True})
            if not self.execution_enabled:
                raise RuntimeError('execution_disabled')
            # Only a real pick after explicit execution_enable may change the
            # controller state. initialize() performs the standard safe
            # ClearError/Disable/Enable/Speed/Collision/User0/Tool0 sequence.
            self._pick_apriltag_cache(request, backend, cache)
            return self.response(
                request_id, action, 'succeeded',
                'AprilTag cached pick complete',
                metrics={'cache_record_id': cache['record_id'],
                         'tag_id': cache['tag_id']})

        raise RuntimeError('unsupported action')

    def _validate_icp_a_record(self, record):
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
        self._validate_optional_joints(record, 'joints_A', 'ICP A')
        return record

    def _validate_icp_b_record(self, record, a_record):
        if not record:
            raise RuntimeError('ICP B missing')
        if record.get('a_record_id') != a_record.get('record_id'):
            raise RuntimeError('ICP B is not bound to the active ICP A')
        if record.get('template_sha256') != a_record.get('template_sha256'):
            raise RuntimeError('ICP A/B template version mismatch')
        if record.get('handeye_sha256') != a_record.get('handeye_sha256'):
            raise RuntimeError('ICP A/B calibration version mismatch')
        self._validate_optional_joints(record, 'joints_B', 'ICP B')
        return record

    @staticmethod
    def _validate_optional_joints(record, key, label):
        raw = record.get(key)
        if raw is None:
            return None
        values = np.asarray(raw, dtype=float).reshape(-1)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise RuntimeError(
                '%s %s must contain six finite joint angles' % (label, key))
        record[key] = values.astype(float).tolist()
        return record[key]

    def _load_and_validate_tag_cache(self, backend):
        path = os.path.join(self.root, 'apriltag_cache.json')
        cache = load_json(path)
        if not cache:
            raise RuntimeError('AprilTag cache missing')
        age = time.time() - os.path.getmtime(path)
        if age > float(self.cfg['apriltag_cache_ttl_sec']):
            raise RuntimeError('AprilTag cache expired')
        identity = backend.identity(cache.get('tag_id'))
        if cache.get('tag_id') != identity.tag_id:
            raise RuntimeError('AprilTag ID changed since locate')
        if cache.get('handeye_sha256') != digest(identity.handeye_path):
            raise RuntimeError('hand-eye calibration changed since locate')
        if cache.get('tcp_calibration_sha256') != digest(
                identity.tcp_calibration_path):
            raise RuntimeError('TCP calibration changed since locate')
        return cache

    def _check_tag_robot_drift(self, backend, cache):
        translation, rotation = backend.localization_drift(
            cache.get('localization', {}))
        if translation > float(self.cfg['apriltag_max_robot_drift_mm']):
            raise RuntimeError('robot moved since AprilTag locate')
        if rotation > float(self.cfg['apriltag_max_robot_drift_deg']):
            raise RuntimeError('robot rotated since AprilTag locate')

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
