import math
import os
import shutil
import threading
import time
import traceback
import uuid

import numpy as np
from apriltag_pick.pick_node import TagDetectionTimeout
from apriltag_pick.rotation_compat import (
    rotation_as_matrix,
    rotation_from_matrix,
)
from scipy.spatial.transform import Rotation

from .backends import AprilTagBackend, IcpBackend, RobotState
from .station_store import (
    APRILTAG_POINT, ICP_POINT, NORMAL_POINT, StationStore,
    normalize_point_type, normalize_task_command, point_type_name,
    point_type_of, task_command_of)
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
                    'vision_station_points', 'vision_station_delete'}:
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
        if action == 'vision_station_delete':
            params = request.get('params', {})
            mapid, poseid = self.station_store.identity(params)
            raw_task_command = params.get('task_command')
            task_command = (
                None if raw_task_command in (None, '')
                else normalize_task_command(raw_task_command))
            deleting_active = bool(
                self.teaching.active and
                (self.teaching.mapid, self.teaching.poseid) ==
                (mapid, poseid) and
                (task_command is None or
                 self.teaching.task_command == task_command))
            active_point_type = (
                self.teaching.point_type if deleting_active else None)
            active_task_command = (
                self.teaching.task_command if deleting_active else None)
            if request.get('dry_run'):
                points, path = self.station_store.load(mapid, poseid)
                stored = bool(points) if task_command is None else (
                    task_command in [
                        task_command_of(point) for point in points])
                if not stored and not deleting_active:
                    return self.response(
                        request_id, action, 'failed',
                        ('station not found: %s/%s' % (mapid, poseid)
                         if task_command is None else
                         'task group not found: %s/%s/%s' %
                         (mapid, poseid, task_command)), 'not_found')
                return self.response(
                    request_id, action, 'succeeded',
                    ('station deletion dry-run passed'
                     if task_command is None else
                     'task-group deletion dry-run passed'),
                    metrics={'mapid': mapid, 'poseid': poseid,
                             'command': task_command, 'dry_run': True,
                             'teaching_active': deleting_active},
                    artifacts=[path] if os.path.isfile(path) else [])

            robot_state = None
            if deleting_active:
                robot_state = self._icp().stop_drag_and_enable()
                self.execution_enabled = False
                self._disable_apriltag_motion()
                self._reset_icp_teaching()
            try:
                if task_command is None:
                    removed, path, removed_dirs = (
                        self.station_store.delete_station(mapid, poseid))
                    removed_commands = [
                        task_command_of(point) for point in removed]
                    removed_point_type = None
                else:
                    removed, path, removed_dirs = self.station_store.delete(
                        mapid, poseid, task_command)
                    removed_commands = [task_command]
                    removed_point_type = point_type_of(removed)
            except RuntimeError as error:
                if not deleting_active or 'not found' not in str(error):
                    raise
                path = self.station_store.manifest_path(mapid, poseid)
                directory = self.station_store.record_dir(
                    mapid, poseid, active_task_command, active_point_type)
                removed_dirs = []
                if os.path.isdir(directory):
                    shutil.rmtree(directory)
                    removed_dirs.append(directory)
                removed_commands = (
                    [active_task_command]
                    if active_task_command is not None else [])
                removed_point_type = active_point_type
            if deleting_active and active_task_command is not None:
                active_directory = self.station_store.record_dir(
                    mapid, poseid, active_task_command, active_point_type)
                if os.path.isdir(active_directory):
                    shutil.rmtree(active_directory)
                    removed_dirs.append(active_directory)
            metrics = {
                'mapid': mapid,
                'poseid': poseid,
                'command': task_command,
                'deleted_commands': removed_commands,
                'point_type': removed_point_type,
                'teaching_cancelled': deleting_active,
            }
            if robot_state is not None:
                metrics['robot_mode'] = robot_state.mode
            metrics.update(self._owner_metrics())
            return self.response(
                request_id, action, 'succeeded',
                ('station %s/%s deleted' % (mapid, poseid)
                 if task_command is None else
                 'task group command=%s deleted' % task_command),
                metrics=metrics,
                artifacts=([path] if os.path.isfile(path) else []) +
                removed_dirs)
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

    def _finish_active_teaching_for_switch(self, backend):
        """End the old group so a new type1 is an absolute switch command.

        Ref/Work commands are persisted as they succeed, so a group with at
        least one Work is already fully saved.  A group without a Work is not
        executable; remove that incomplete residue instead of leaving a broken
        manifest entry that can fail later during production execution.
        """
        context = self.teaching.identity()
        work_count = int(self.teaching.work_count)
        phase = self.teaching.phase
        backend.stop_drag_and_enable()
        self.execution_enabled = False
        self._disable_apriltag_motion()

        incomplete_discarded = work_count < 1
        if incomplete_discarded:
            try:
                self.station_store.delete(context[0], context[1], context[2])
            except RuntimeError as error:
                if 'not found' not in str(error):
                    raise
            shutil.rmtree(
                self.station_store.record_dir(*context), ignore_errors=True)
        self._reset_icp_teaching()
        return {
            'previous_teaching_finalized': True,
            'previous_mapid': context[0],
            'previous_poseid': context[1],
            'previous_task_command': context[2],
            'previous_point_type': context[3],
            'previous_phase': phase,
            'previous_work_count': work_count,
            'previous_saved': not incomplete_discarded,
            'previous_incomplete_discarded': incomplete_discarded,
        }

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
            current_rotation = rotation_as_matrix(Rotation.from_euler(
                'xyz', current[3:6], degrees=True))
            relative_rotation = current_rotation.T @ target[:3, :3]
            translation_delta = float(
                np.linalg.norm(xyz - np.asarray(current[:3], dtype=float)))
            rotation_delta = float(np.degrees(np.linalg.norm(
                rotation_from_matrix(relative_rotation).as_rotvec())))
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

    def _teaching_identity(self, params):
        return self.station_store.teaching_identity(params)

    def _save_station_icp_point(self, mapid, poseid, task_command,
                                a_record, b_record):
        return self.station_store.save_icp(
            mapid, poseid, task_command, a_record, b_record)

    def _icp_record_dir(self, context=None):
        context = context or {}
        required = ('mapid', 'poseid', 'task_command')
        missing = [
            key for key in required
            if context.get(key) in (None, '')]
        if missing:
            raise RuntimeError(
                'ICP storage requires mapid, poseid and task_command')
        return self.station_store.record_dir(
            context['mapid'], context['poseid'], context['task_command'],
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
                      teaching=False, context=None, joints=None,
                      work_index=1, command=2):
        record_dir = self._icp_record_dir(context)
        metadata_path = os.path.join(
            record_dir, 'work_%d.json' % int(work_index))
        record_id = 'icp_work-' + uuid.uuid4().hex
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
            'work_id': 'W%d' % int(work_index),
            'index': int(work_index),
            'command': int(command),
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
        detection_timeout = float(self.cfg.get(
            'apriltag_detection_timeout_sec', 5.0))
        if tag_id is None and tag_offset_xyz_mm is None:
            location = backend.locate(
                wait_for_new_detection=True,
                detection_timeout_sec=detection_timeout)
        else:
            location = backend.locate(
                tag_id=tag_id, tag_offset_xyz_mm=tag_offset_xyz_mm,
                wait_for_new_detection=True,
                detection_timeout_sec=detection_timeout)
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
    def _transform_pose(transform):
        value = np.asarray(transform, dtype=float)
        if value.shape != (4, 4) or not np.all(np.isfinite(value)):
            raise RuntimeError('flange transform must be a finite 4x4 matrix')
        return np.concatenate((
            value[:3, 3],
            rotation_from_matrix(value[:3, :3]).as_euler(
                'xyz', degrees=True))).astype(float).tolist()

    @staticmethod
    def _teach_command(value):
        text = str(value).strip().lower()
        if text == 'start':
            return 'start'
        if text == 'status':
            return 'status'
        if text in {'finish', 'stop'}:
            return 'finish'
        if text in {'cancel', 'abort'}:
            return 'cancel'
        try:
            command = int(value)
        except (TypeError, ValueError):
            raise RuntimeError(
                'teaching command must be start, -1, 0 or a positive integer')
        if command == -1:
            return 'cancel'
        if command == 0:
            return 'finish'
        if command < 1:
            raise RuntimeError(
                'teaching command must be -1, 0 or a positive integer')
        return command

    @staticmethod
    def _apriltag_teaching_config(params):
        params = params or {}
        raw_tag_id = params.get('tag_id')
        if raw_tag_id in (None, ''):
            tag_id = 0
        else:
            try:
                tag_id = int(raw_tag_id)
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
        explicit_type = params.get('point_type', params.get('pointType'))
        point_type = normalize_point_type(explicit_type)
        if self.teaching.active and explicit_type in (None, ''):
            point_type = self.teaching.point_type
        if dry_run:
            return self.response(
                request_id, action, 'succeeded',
                'teaching dry-run passed; robot and records unchanged',
                metrics={'command': command, 'point_type': point_type,
                         'dry_run': True,
                         **self._owner_metrics()})

        backend = self._icp()
        if command == 'start':
            mapid, poseid, task_command = self._teaching_identity(
                params)
            tag_id = None
            tag_offset = None
            if point_type == APRILTAG_POINT:
                tag_id, tag_offset = self._apriltag_teaching_config(params)
            switch_metrics = {}
            if self.teaching.active:
                current = self.teaching.identity()
                if current != (mapid, poseid, task_command, point_type):
                    switch_metrics = self._finish_active_teaching_for_switch(
                        backend)
                    # An incomplete group may have occupied the command number
                    # selected above. Reallocate after removing it.
                    mapid, poseid, task_command = self._teaching_identity(
                        params)
                else:
                    state = backend.robot_state()
                    return self.response(
                        request_id, action, 'succeeded',
                        'teaching drag already active',
                        metrics={'command': command, 'robot_mode': state.mode,
                                 **self._owner_metrics()})
            existing, unused_path = self.station_store.load(mapid, poseid)
            if task_command in [
                    task_command_of(point) for point in existing]:
                raise RuntimeError(
                    'task command %s already exists for %s/%s; delete it '
                    'with type3 before teaching again' %
                    (task_command, mapid, poseid))
            state = backend.start_drag()
            self.execution_enabled = False
            self._disable_apriltag_motion()
            with self.lock:
                self.teaching.begin(
                    point_type, mapid, poseid, task_command,
                    tag_id=tag_id, tag_offset_xyz_mm=tag_offset)
            instruction = {
                NORMAL_POINT: 'send command 1..N to record W1..Wn',
                ICP_POINT: (
                    'send command 1 to record Ref/template, then command '
                    '2..N to record W1..Wn'),
                APRILTAG_POINT: (
                    'send command 1 to record Tag Ref, then command 2..N '
                    'to record W1..Wn'),
            }[point_type]
            return self.response(
                request_id, action, 'succeeded',
                '%s teaching started for %s/%s command=%s; %s' %
                (self.teaching.kind, mapid, poseid, task_command,
                 instruction),
                metrics={'command': command, 'robot_mode': state.mode,
                         **switch_metrics,
                         **self._owner_metrics()})

        if command == 'status':
            state = backend.robot_state()
            return self.response(
                request_id, action, 'succeeded',
                'teaching status read',
                metrics={'command': command, 'robot_mode': state.mode,
                         'pose': state.pose, **self._owner_metrics()})

        if command == 'cancel':
            if not self.teaching.active:
                return self.response(
                    request_id, action, 'succeeded',
                    'teaching is already idle; nothing was cancelled',
                    metrics={'command': command,
                             **self._owner_metrics()})
            context = self.teaching.identity()
            state = backend.stop_drag_and_enable()
            self.execution_enabled = False
            self._disable_apriltag_motion()
            try:
                self.station_store.delete(
                    context[0], context[1], context[2])
            except RuntimeError as error:
                if 'not found' not in str(error):
                    raise
            shutil.rmtree(self.station_store.record_dir(*context),
                          ignore_errors=True)
            self._reset_icp_teaching()
            return self.response(
                request_id, action, 'succeeded',
                'teaching cancelled; current command group discarded and robot '
                'enabled',
                metrics={'command': command, 'robot_mode': state.mode,
                         **self._owner_metrics()})

        if command == 'finish':
            if not self.teaching.active:
                return self.response(
                    request_id, action, 'succeeded',
                    'teaching is already idle',
                    metrics={'command': command,
                             **self._owner_metrics()})
            if self.teaching.work_count < 1:
                raise RuntimeError(
                    'cannot finish teaching before at least one Work point')
            context = self.teaching.identity()
            work_count = self.teaching.work_count
            state = backend.stop_drag_and_enable()
            self.execution_enabled = False
            self._disable_apriltag_motion()
            self._reset_icp_teaching()
            # This is an internal robot operation, not an NX command=1
            # message, so it cannot be reclassified as another Ref capture.
            # Only an active-session finish reaches this branch; an idle or
            # repeated type2 query therefore does not reset again.
            if backend.reset_robot() is False:
                raise RuntimeError(
                    'teaching data was saved but robot reset failed')
            return self.response(
                request_id, action, 'succeeded',
                'teaching group %s saved with %d Work points; robot reset' %
                (context[2], work_count),
                metrics={'command': command, 'robot_mode': state.mode,
                         'saved_task_command': context[2],
                         'work_count': work_count,
                         'reset_completed': True,
                         **self._owner_metrics()})

        if not self.teaching.active:
            raise RuntimeError(
                'teaching is not active; send command 0/start first')

        requested_mapid = params.get('mapid')
        requested_poseid = params.get('poseid')
        if ((requested_mapid not in (None, '') and
             str(requested_mapid) != str(self.teaching.mapid)) or
                (requested_poseid not in (None, '') and
                 str(requested_poseid) != str(self.teaching.poseid))):
            raise RuntimeError(
                'teaching command station does not match active teaching: '
                '%s/%s' % (self.teaching.mapid, self.teaching.poseid))

        if point_type != self.teaching.point_type:
            raise RuntimeError(
                'teaching point_type does not match active session')

        command_value = int(command)
        backend.wait_teaching_stable(
            min_duration_sec=float(
                self.cfg.get('teaching_stable_sec', 2.0)),
            timeout_sec=float(
                self.cfg.get('teaching_stable_timeout_sec', 8.0)))
        if (self.teaching.point_type in (ICP_POINT, APRILTAG_POINT) and
                command_value == 1):
            if self.teaching.phase != 'awaiting_reference':
                raise RuntimeError(
                    'visual Ref is already recorded; send command 2..N '
                    'to record Work points')
            context = {
                'mapid': self.teaching.mapid,
                'poseid': self.teaching.poseid,
                'task_command': self.teaching.task_command,
            }
            if self.teaching.point_type == APRILTAG_POINT:
                location = self._apriltag().locate(
                    tag_id=self.teaching.tag_id,
                    tag_offset_xyz_mm=self.teaching.tag_offset_xyz_mm,
                    wait_for_new_detection=True,
                    detection_timeout_sec=float(self.cfg.get(
                        'apriltag_detection_timeout_sec', 5.0)))
                localization = location.localization
                transform = np.asarray(
                    localization.get('base_flange_at_detection'),
                    dtype=float)
                if (transform.shape != (4, 4) or
                        not np.all(np.isfinite(transform))):
                    raise RuntimeError(
                        'AprilTag Ref localization has no valid '
                        'T_base_flange_at_detection')
                pose = self._transform_pose(transform)
                joints = backend.flange_joints_teaching()
                base_tag = np.asarray(
                    localization.get('base_tag'), dtype=float)
                if base_tag.shape != (4, 4) or not np.all(np.isfinite(base_tag)):
                    raise RuntimeError(
                        'AprilTag Ref localization has no valid T_base_tag')
                identity = location.identity
                reference = self._record_payload(
                    'apriltag_ref-' + uuid.uuid4().hex, {
                        'mapid': context['mapid'],
                        'poseid': context['poseid'],
                        'task_command': context['task_command'],
                        'tag_id': identity.tag_id,
                        'tag_frame': identity.tag_frame,
                        'observe_flange_pose': pose,
                        'observe_joints': joints,
                        'T_base_flange_ref': transform.tolist(),
                        'T_base_tag_ref': base_tag.tolist(),
                        'handeye_path': identity.handeye_path,
                        'handeye_sha256': digest(identity.handeye_path),
                        'tcp_calibration_path': identity.tcp_calibration_path,
                        'tcp_calibration_sha256': digest(
                            identity.tcp_calibration_path),
                    })
                record_dir = self.station_store.record_dir(
                    context['mapid'], context['poseid'],
                    context['task_command'],
                    APRILTAG_POINT)
                reference_path = os.path.join(
                    record_dir, 'tag_reference.json')
                atomic_json(reference_path, reference)
                unused_group, manifest_path = self.station_store.save_group(
                    context['mapid'], context['poseid'],
                    context['task_command'],
                    APRILTAG_POINT, reference, [])
                with self.lock:
                    self.teaching.phase = 'awaiting_work'
                    self.teaching.reference_record_id = reference['record_id']
                return self.response(
                    request_id, action, 'succeeded',
                    'AprilTag Ref saved; drag robot to W1 and send command 2',
                    metrics={'command': command_value,
                             'record_id': reference['record_id'],
                             'tag_id': identity.tag_id,
                             **self._owner_metrics()},
                    artifacts=[reference_path, manifest_path])

            frames = int(params.get(
                'frames', self.cfg['icp_frames']))
            payload, artifacts = self._record_icp_a(
                request_id, frames, teaching=True, context=context)
            unused_group, manifest_path = self.station_store.save_group(
                context['mapid'], context['poseid'],
                context['task_command'],
                ICP_POINT, payload, [])
            artifacts.append(manifest_path)
            with self.lock:
                self.teaching.phase = 'awaiting_work'
                self.teaching.a_record_id = payload['record_id']
                self.teaching.reference_record_id = payload['record_id']
                self.teaching.b_record_id = None
            return self.response(
                request_id, action, 'succeeded',
                'ICP Ref/template recorded; drag robot to W1 then send '
                'command 2',
                metrics={'command': command, 'record_id': payload['record_id'],
                         'frames': frames, **self._owner_metrics()},
                artifacts=artifacts)

        if (self.teaching.point_type in (ICP_POINT, APRILTAG_POINT) and
                self.teaching.phase != 'awaiting_work'):
            raise RuntimeError(
                'visual Work cannot be recorded before Ref; send command 1')

        work_index = (
            command_value if self.teaching.point_type is NORMAL_POINT
            else command_value - 1)
        if work_index < 1:
            raise RuntimeError('visual command 1 is reserved for Ref')
        transform = backend.flange_transform_teaching()
        pose = self._transform_pose(transform)
        joints = backend.flange_joints_teaching()
        context = {
            'mapid': self.teaching.mapid,
            'poseid': self.teaching.poseid,
            'task_command': self.teaching.task_command,
        }
        points, unused_path = self.station_store.load(
            context['mapid'], context['poseid'])
        group = next((point for point in points
                      if task_command_of(point) ==
                      context['task_command']), None)
        reference = (group or {}).get('reference')
        artifacts = []
        if self.teaching.point_type == ICP_POINT:
            a_record = self._validate_icp_a_record(reference)
            payload, artifacts = self._record_icp_b(
                request_id, a_record, transform, teaching=True,
                context=context, joints=joints, work_index=work_index,
                command=command_value)
        elif self.teaching.point_type == APRILTAG_POINT:
            base_tag = np.asarray(
                (reference or {}).get('T_base_tag_ref'), dtype=float)
            if base_tag.shape != (4, 4) or not np.all(np.isfinite(base_tag)):
                raise RuntimeError('active AprilTag Ref is invalid')
            relative = np.linalg.inv(base_tag) @ transform
            payload = self._record_payload(
                'apriltag_work-' + uuid.uuid4().hex, {
                    **context,
                    'work_id': 'W%d' % work_index,
                    'index': work_index,
                    'command': command_value,
                    'flange_pose': pose,
                    'joints': joints,
                    'T_base_flange_work': transform.tolist(),
                    'T_tag_to_flange_work': relative.tolist(),
                    'tag_reference_record_id': reference.get('record_id'),
                })
            record_dir = self.station_store.record_dir(
                context['mapid'], context['poseid'],
                context['task_command'],
                APRILTAG_POINT)
            work_path = os.path.join(
                record_dir, 'work_%d.json' % work_index)
            atomic_json(work_path, payload)
            artifacts.append(work_path)
        else:
            payload = self._record_payload(
                'normal_work-' + uuid.uuid4().hex, {
                    **context,
                    'work_id': 'W%d' % work_index,
                    'index': work_index,
                    'command': command_value,
                    'flange_pose': pose,
                    'joints': joints,
                    'T_base_flange_work': transform.tolist(),
                })
            record_dir = self.station_store.record_dir(
                context['mapid'], context['poseid'],
                context['task_command'],
                NORMAL_POINT)
            work_path = os.path.join(
                record_dir, 'work_%d.json' % work_index)
            atomic_json(work_path, payload)
            artifacts.append(work_path)

        saved_group, manifest_path = self.station_store.upsert_work(
            context['mapid'], context['poseid'], context['task_command'],
            self.teaching.point_type, payload, reference=reference)
        artifacts.append(manifest_path)
        work_count = len(saved_group['work_points'])
        with self.lock:
            self.teaching.phase = 'awaiting_work'
            self.teaching.work_count = work_count
            self.teaching.b_record_id = payload['record_id']
        metrics = {
            'command': command_value,
            'record_id': payload['record_id'],
            'saved_task_command': context['task_command'],
            'work_id': payload['work_id'],
            'work_count': work_count,
            'mapid': context['mapid'],
            'poseid': context['poseid'],
            **self._owner_metrics(),
        }
        if self.teaching.point_type == APRILTAG_POINT:
            relative = np.asarray(payload['T_tag_to_flange_work'], dtype=float)
            metrics['tag_relative_xyz_mm'] = relative[:3, 3].tolist()
            metrics['tag_relative_rpy_deg'] = rotation_from_matrix(
                relative[:3, :3]).as_euler('xyz', degrees=True).tolist()
        return self.response(
            request_id, action, 'succeeded',
            '%s %s saved; drag robot to the next Work point or send '
            'command 0 to finish' %
            (point_type_name(self.teaching.point_type), payload['work_id']),
            metrics=metrics,
            artifacts=artifacts)

    @staticmethod
    def _apriltag_point_config(point):
        reference = point.get('reference')
        if not isinstance(reference, dict):
            raise RuntimeError(
                'AprilTag command=%s has no Tag Ref' %
                task_command_of(point))
        pose = np.asarray(
            reference.get('observe_flange_pose'), dtype=float).reshape(-1)
        if pose.shape != (6,) or not np.all(np.isfinite(pose)):
            raise RuntimeError(
                'AprilTag point %s has invalid observe_flange_pose' %
                task_command_of(point))
        try:
            tag_id = int(reference.get('tag_id', 0))
        except (TypeError, ValueError):
            raise RuntimeError(
                'AprilTag point %s has invalid tag_id' %
                task_command_of(point))
        if tag_id < 0:
            raise RuntimeError(
                'AprilTag point %s has invalid tag_id' %
                task_command_of(point))
        raw_joints = reference.get('observe_joints')
        joints = None
        if raw_joints is not None:
            values = np.asarray(raw_joints, dtype=float).reshape(-1)
            if values.shape != (6,) or not np.all(np.isfinite(values)):
                raise RuntimeError(
                    'AprilTag point %s has invalid observe_joints' %
                    task_command_of(point))
            joints = values.astype(float).tolist()
        works = list(point.get('work_points', []))
        if not works:
            raise RuntimeError(
                'AprilTag command=%s has no Work points' %
                task_command_of(point))
        for work in works:
            relative = np.asarray(
                work.get('T_tag_to_flange_work'), dtype=float)
            if relative.shape != (4, 4) or not np.all(np.isfinite(relative)):
                raise RuntimeError(
                    'AprilTag %s has invalid %s relative transform' %
                    (task_command_of(point), work.get('work_id')))
        return pose.tolist(), joints, tag_id, works

    @staticmethod
    def _normal_point_config(point):
        works = list(point.get('work_points', []))
        if not works:
            raise RuntimeError(
                'normal command=%s has no Work points' %
                task_command_of(point))
        for work in works:
            joints = np.asarray(work.get('joints'), dtype=float).reshape(-1)
            if joints.shape != (6,) or not np.all(np.isfinite(joints)):
                raise RuntimeError(
                    'normal %s has invalid %s joints' %
                    (task_command_of(point), work.get('work_id')))
        return works

    def _dispatch_station_execute(self, request):
        request_id = request['request_id']
        params = dict(request.get('params', {}))
        # Production is station-scoped. point_type/task_command are teaching
        # metadata and must never split a vehicle arrival into partial runs.
        params.pop('point_type', None)
        params.pop('pointType', None)
        params.pop('task_command', None)
        mapid, poseid, points, manifest_path = self.station_store.select(params)
        dry_run = bool(request.get('dry_run'))
        if not dry_run and not self.execution_enabled:
            raise RuntimeError('execution_disabled')

        point_metrics = []
        artifacts = [manifest_path]
        backend = self._icp()
        logger = (
            self.node.get_logger()
            if self.node is not None and hasattr(self.node, 'get_logger')
            else None)
        for point_index, point in enumerate(points):
            self._checkpoint(request_id)
            task_command = task_command_of(point)
            point_type = point_type_of(point)
            if logger is not None:
                logger.info(
                    'station %s/%s: start group %d/%d %s type=%s' %
                    (mapid, poseid, point_index + 1, len(points),
                     task_command, point_type_name(point_type)))

            if point_type == ICP_POINT:
                nested = dict(request)
                nested['action'] = 'vision_icp_align_and_move_b'
                nested_params = dict(params)
                nested_params.update(
                    mapid=mapid,
                    poseid=poseid,
                    task_command=task_command,
                    point_type=ICP_POINT)
                nested['params'] = nested_params
                try:
                    result = self._dispatch(nested)
                except Exception as icp_error:
                    # A failed alignment can leave the arm at its last visual
                    # correction.  Production station semantics require a
                    # best-effort reset before this failed request returns.
                    try:
                        self._reset_after_completed_point(
                            request_id, mapid, poseid, task_command,
                            point_type, dry_run, check_request=False)
                    except Exception as reset_error:
                        raise RuntimeError(
                            '%s; ICP失败后的机械臂复位也失败: %s' %
                            (icp_error, reset_error)) from icp_error
                    raise
                current = {
                    'command': task_command,
                    'point_type': ICP_POINT,
                }
                nested_metrics = dict(result.get('metrics', {}))
                nested_points = nested_metrics.pop('points', [])
                for key in ('mapid', 'poseid', 'commands', 'point_count'):
                    nested_metrics.pop(key, None)
                current.update(nested_metrics)
                if len(nested_points) == 1:
                    current.update(nested_points[0])
                if self._reset_after_completed_point(
                        request_id, mapid, poseid, task_command, point_type,
                        dry_run):
                    current['reset_completed'] = True
                point_metrics.append(current)
                artifacts.extend(result.get('artifacts', []))
                self._progress(
                    request_id, point_index + 1, len(points),
                    {'command': task_command, 'point_type': point_type})
                if logger is not None:
                    logger.info(
                        'station %s/%s: completed group %d/%d %s type=%s' %
                        (mapid, poseid, point_index + 1, len(points),
                         task_command, point_type_name(point_type)))
                continue

            if point_type is NORMAL_POINT:
                works = self._normal_point_config(point)
                current = {
                    'command': task_command,
                    'point_type': NORMAL_POINT,
                    'point_kind': 'normal',
                    'work_count': len(works),
                    'works': [],
                }
                for work in works:
                    self._checkpoint(request_id)
                    target_transform = np.asarray(
                        work.get('T_base_flange_work'), dtype=float)
                    if dry_run:
                        moved = list(work.get('joints'))
                    else:
                        moved = backend.move_joints(
                            work.get('joints'),
                            'normal %s/%s' % (
                                task_command, work.get('work_id')),
                            motion_guard=lambda target: self._motion_safety(
                                request, backend=backend, target=target),
                            target_transform=target_transform)
                    current['works'].append({
                        'work_id': work.get('work_id'),
                        'joints': moved,
                    })
                if self._reset_after_completed_point(
                        request_id, mapid, poseid, task_command, point_type,
                        dry_run):
                    current['reset_completed'] = True
                point_metrics.append(current)
                self._progress(
                    request_id, point_index + 1, len(points),
                    {'command': task_command, 'point_type': point_type})
                continue

            observe_pose, observe_joints, tag_id, works = (
                self._apriltag_point_config(point))
            tag_backend = self._apriltag()
            if dry_run:
                identity = tag_backend.identity(tag_id)
                point_metrics.append({
                    'command': task_command,
                    'point_type': APRILTAG_POINT,
                    'dry_run': True,
                    'tag_id': identity.tag_id,
                    'tag_frame': identity.tag_frame,
                    'observe_flange_pose': observe_pose,
                    'observe_joints': observe_joints,
                    'work_count': len(works),
                    'work_ids': [work.get('work_id') for work in works],
                })
                self._progress(
                    request_id, point_index + 1, len(points),
                    {'command': task_command, 'point_type': point_type})
                continue

            def observation_safety(unused_stage, target):
                self._checkpoint(request_id)
                self._motion_safety(
                    request, backend=tag_backend, target=target)

            if observe_joints is None:
                observation_target = tag_backend.move_to_observation(
                    observe_pose, observation_safety)
            else:
                observation_target = tag_backend.move_to_observation(
                    observe_pose, observation_safety,
                    joints=observe_joints)
            settle_sec = max(0.0, float(self.cfg.get(
                'apriltag_post_observation_settle_sec', 2.0)))
            detection_timeout = max(0.1, float(self.cfg.get(
                'apriltag_search_detection_timeout_sec', 2.0)))
            lateral = abs(float(self.cfg.get(
                'apriltag_search_lateral_mm', 20.0)))
            search_offsets = [0.0]
            if lateral > 0.0:
                search_offsets.extend([lateral, -lateral])

            def wait_stable():
                settle_deadline = time.monotonic() + settle_sec
                while True:
                    self._checkpoint(request_id)
                    remaining = settle_deadline - time.monotonic()
                    if remaining <= 0.0:
                        return
                    time.sleep(min(0.1, remaining))

            location = None
            search_target = observation_target
            selected_offset = 0.0
            for search_index, offset_mm in enumerate(search_offsets):
                if search_index:
                    search_target = tag_backend.lateral_observation_target(
                        observation_target, offset_mm)
                    tag_backend.move_flange_target(
                        search_target, observation_safety,
                        label='AprilTag search %+.1fmm' % offset_mm)
                wait_stable()
                try:
                    location = tag_backend.locate(
                        tag_id=tag_id,
                        wait_for_new_detection=True,
                        detection_timeout_sec=detection_timeout)
                    selected_offset = offset_mm
                    break
                except TagDetectionTimeout:
                    self._checkpoint(request_id)
            if location is None:
                if not np.allclose(search_target, observation_target):
                    tag_backend.move_flange_target(
                        observation_target, observation_safety,
                        label='AprilTag search return')
                    wait_stable()
                raise RuntimeError(
                    '未检测到Tag ID %s: 中心/左/右各等待%.1fs' %
                    (tag_id, detection_timeout))
            new_base_tag = np.asarray(
                location.localization.get('base_tag'), dtype=float)
            if (new_base_tag.shape != (4, 4) or
                    not np.all(np.isfinite(new_base_tag))):
                raise RuntimeError('AprilTag production has no valid T_base_tag')
            current = {
                'command': task_command,
                'point_type': APRILTAG_POINT,
                'observe_flange_pose': observe_pose,
                'observation_target': observation_target.tolist(),
                'search_lateral_offset_mm': selected_offset,
                'tag_id': tag_id,
                'frames': int(location.localization['frames']),
                'spread_mm': float(location.localization['spread_mm']),
                'work_count': len(works),
                'works': [],
            }
            for work in works:
                target = new_base_tag @ np.asarray(
                    work['T_tag_to_flange_work'], dtype=float)

                def work_safety(unused_stage, stage_target):
                    self._checkpoint(request_id)
                    self._motion_safety(
                        request, backend=tag_backend, target=stage_target)

                moved = tag_backend.move_flange_target(
                    target, work_safety,
                    label='AprilTag %s/%s' %
                    (task_command, work.get('work_id')))
                current['works'].append({
                    'work_id': work.get('work_id'),
                    'T_target': moved.tolist(),
                })
            if self._reset_after_completed_point(
                    request_id, mapid, poseid, task_command, point_type,
                    dry_run):
                current['reset_completed'] = True
            point_metrics.append(current)
            self._progress(
                request_id, point_index + 1, len(points),
                {'command': task_command, 'point_type': point_type})
            if logger is not None:
                logger.info(
                    'station %s/%s: completed group %d/%d %s type=%s' %
                    (mapid, poseid, point_index + 1, len(points),
                     task_command, point_type_name(point_type)))
        return self.response(
            request_id, 'vision_station_execute', 'succeeded',
            ('station points passed dry-run; robot was not moved'
             if dry_run else 'station points completed'),
            metrics={
                'dry_run': dry_run,
                'mapid': mapid,
                'poseid': poseid,
                'commands': [task_command_of(point) for point in points],
                'point_count': len(point_metrics),
                'points': point_metrics,
            },
            artifacts=list(dict.fromkeys(artifacts)))

    def _reset_after_completed_point(self, request_id, mapid, poseid,
                                     task_command, point_type, dry_run,
                                     check_request=True):
        """Run command-1 reset after a successful or failed point attempt."""
        if dry_run or not bool(self.cfg.get(
                'reset_after_each_point', False)):
            return False
        if check_request:
            self._checkpoint(request_id)
        result = self._icp().reset_robot()
        if result is False:
            raise RuntimeError(
                'group %s type=%s reset position failed' %
                (task_command, point_type_name(point_type)))
        logger = (
            self.node.get_logger()
            if self.node is not None and hasattr(self.node, 'get_logger')
            else None)
        if logger is not None:
            logger.info(
                'station %s/%s: group %s type=%s reset completed' %
                (mapid, poseid, task_command,
                 point_type_name(point_type)))
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
                    point.get('reference', point.get('icp_a')))
                work_records = []
                if action == 'vision_icp_align_and_move_b':
                    for work in point.get('work_points', []):
                        work_records.append(
                            self._validate_icp_b_record(work, a_record))
                    if not work_records:
                        raise RuntimeError(
                            'ICP command=%s has no Work points' %
                            task_command_of(point))
                profiles.append(
                    (task_command_of(point), a_record, work_records))
            station_metrics = {
                'mapid': mapid,
                'poseid': poseid,
                'commands': [profile[0] for profile in profiles],
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
            for point_index, (task_command, a_record, work_records) in enumerate(
                    profiles):
                recovery_args = (
                    a_record['T_base_flange_A'],
                    'ICP %s recovery-to-A' % task_command,
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
                    command=task_command,
                    a_record_id=a_record['record_id'],
                    T_recovery_A=recovery_target.tolist())
                if not alignment.converged:
                    raise RuntimeError(
                        'ICP point %s did not converge; B motion prohibited: %s'
                        % (task_command,
                           current_metrics.get('reason', 'unknown')))
                current_metrics['works'] = []
                if action == 'vision_icp_align_and_move_b':
                    corrected_ref = backend.flange_transform()
                    for work in work_records:
                        self._checkpoint(request_id)
                        relative_args = (
                            work['T_A_to_B'],
                            'ICP %s/%s' % (
                                task_command, work.get('work_id')),
                            lambda value: self._motion_safety(
                                request, backend=backend, target=value))
                        target = backend.move_relative(
                            *relative_args,
                            reference_transform=work.get(
                                'T_base_flange_B'),
                            reference_joints=work.get('joints_B'),
                            base_transform=corrected_ref)
                        current_metrics['works'].append({
                            'work_id': work.get('work_id'),
                            'record_id': work['record_id'],
                            'T_target': target.tolist(),
                        })
                    current_metrics['work_count'] = len(work_records)
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
