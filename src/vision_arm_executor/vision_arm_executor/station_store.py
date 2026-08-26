"""Validated, atomic storage for command-identified station task groups."""

import os
import re
import shutil
import threading

import numpy as np

from .store import atomic_json, load_json, now


# The wire protocol intentionally omits point_type for an ordinary joint group.
NORMAL_POINT = None
APRILTAG_POINT = 0
ICP_POINT = 1
POINT_TYPES = {APRILTAG_POINT, ICP_POINT}


def normalize_point_type(value, required=False):
    """Normalize the wire value; a missing field is the normal point type."""
    if value in (None, ''):
        return NORMAL_POINT
    if isinstance(value, bool):
        raise RuntimeError('point_type must be omitted, 0 or 1')
    try:
        point_type = int(value)
    except (TypeError, ValueError):
        raise RuntimeError('point_type must be omitted, 0 or 1')
    if point_type not in POINT_TYPES:
        raise RuntimeError('point_type must be omitted, 0 or 1')
    return point_type


def point_type_of(point):
    if not isinstance(point, dict):
        raise RuntimeError('station task group must be an object')
    return normalize_point_type(point.get('point_type'))


def point_type_name(point_type):
    point_type = normalize_point_type(point_type)
    return {
        NORMAL_POINT: 'normal',
        APRILTAG_POINT: 'apriltag',
        ICP_POINT: 'icp',
    }[point_type]


def normalize_task_command(value):
    """Return the backend task identifier in one canonical string form."""
    if isinstance(value, bool) or value in (None, ''):
        raise RuntimeError('task command must be a positive integer')
    text = str(value).strip()
    if not re.match(r'^\d+$', text):
        raise RuntimeError('task command must be a positive integer')
    command = int(text)
    if command < 1:
        raise RuntimeError('task command must be a positive integer')
    return str(command)


def task_command_of(point):
    if not isinstance(point, dict):
        raise RuntimeError('station task group must be an object')
    return normalize_task_command(point.get('command'))


_NO_TYPE_FILTER = object()


class StationStore:
    """Own one station manifest whose command groups contain Ref + W1..Wn."""

    def __init__(self, root):
        self.root = os.path.expanduser(root)
        self.lock = threading.RLock()

    @staticmethod
    def component(value, field):
        value = str(value or '').strip()
        if (not value or len(value) > 128 or value in {'.', '..'} or
                '/' in value or '\\' in value or '\x00' in value or
                not re.match(r'^[\w .-]+$', value, flags=re.UNICODE)):
            raise RuntimeError(
                '%s must be a non-empty safe path component' % field)
        return value

    def identity(self, params):
        params = params or {}
        return (
            self.component(params.get('mapid'), 'mapid'),
            self.component(params.get('poseid'), 'poseid'),
        )

    def manifest_path(self, mapid, poseid):
        mapid = self.component(mapid, 'mapid')
        poseid = self.component(poseid, 'poseid')
        return os.path.join(
            self.root, 'tasks', mapid, poseid,
            '%s_%s.json' % (mapid, poseid))

    def record_dir(self, mapid, poseid, task_command, point_type):
        mapid = self.component(mapid, 'mapid')
        poseid = self.component(poseid, 'poseid')
        task_command = normalize_task_command(task_command)
        point_type = normalize_point_type(point_type)
        suffix = 'normal' if point_type is NORMAL_POINT else str(point_type)
        return os.path.join(
            self.root, 'records',
            '%s_%s_%s_%s' % (mapid, poseid, task_command, suffix))

    def load(self, mapid, poseid):
        path = self.manifest_path(mapid, poseid)
        with self.lock:
            points = load_json(path, default=[], strict=True)
        if not isinstance(points, list):
            raise RuntimeError('station manifest must be a task-group array')
        normalized = []
        used_commands = set()
        for index, value in enumerate(points):
            if not isinstance(value, dict):
                raise RuntimeError(
                    'station manifest contains a non-object task group')
            point = dict(value)
            raw_command = point.get('command', point.get('task_command'))
            if raw_command in (None, ''):
                match = re.match(r'^P(\d+)$', str(point.get('label', '')))
                raw_command = match.group(1) if match else index + 1
            task_command = normalize_task_command(raw_command)
            if task_command in used_commands:
                raise RuntimeError(
                    'station manifest contains duplicate command %s' %
                    task_command)
            used_commands.add(task_command)
            point['command'] = task_command
            point.pop('label', None)
            point.pop('task_command', None)
            point_type = point_type_of(point)
            if point_type is NORMAL_POINT:
                point.pop('point_type', None)
            else:
                point['point_type'] = point_type
            works = point.get('work_points')
            if works is None and point_type == ICP_POINT and point.get('icp_b'):
                work = dict(point['icp_b'])
                work.update(work_id='W1', index=1, command=2)
                works = [work]
                point.setdefault('reference', point.get('icp_a'))
            elif (works is None and point_type == APRILTAG_POINT and
                  isinstance(point.get('apriltag'), dict)):
                # A legacy AprilTag entry has an observation pose and XYZ
                # offset, but not the full Ref-to-Work transform required by
                # the new multi-work execution path. Keep it queryable and
                # deletable without inventing production geometry.
                point.setdefault('reference', point.get('apriltag'))
                works = []
            elif works is None:
                works = []
            if not isinstance(works, list):
                raise RuntimeError('work_points must be an array')
            point['work_points'] = [dict(work) for work in works]
            normalized.append(point)
        return normalized, path

    def teaching_identity(self, params):
        mapid, poseid = self.identity(params)
        raw_command = (params or {}).get('task_command')
        task_command = (
            self.next_task_command(mapid, poseid)
            if raw_command in (None, '')
            else normalize_task_command(raw_command))
        return mapid, poseid, task_command

    def next_task_command(self, mapid, poseid):
        """Allocate a local numeric group id when type1 starts teaching."""
        points, unused_path = self.load(mapid, poseid)
        used = {int(task_command_of(point)) for point in points}
        command = 1
        while command in used:
            command += 1
        return str(command)

    def save(self, point):
        point = dict(point)
        mapid = self.component(point.get('mapid'), 'mapid')
        poseid = self.component(point.get('poseid'), 'poseid')
        task_command = normalize_task_command(point.get('command'))
        point_type = normalize_point_type(point.get('point_type'))
        point.update(
            schema_version=4, mapid=mapid, poseid=poseid,
            command=task_command, updated_at=now())
        point.pop('label', None)
        point.pop('task_command', None)
        if point_type is NORMAL_POINT:
            point.pop('point_type', None)
        else:
            point['point_type'] = point_type
        works = point.get('work_points', [])
        if not isinstance(works, list):
            raise RuntimeError('work_points must be an array')
        point['work_points'] = [dict(work) for work in works]
        path = self.manifest_path(mapid, poseid)
        with self.lock:
            existing, unused_path = self.load(mapid, poseid)
            for index, current in enumerate(existing):
                if task_command_of(current) == task_command:
                    if point_type_of(current) != point_type:
                        raise RuntimeError(
                            'task command point_type cannot be changed')
                    existing[index] = point
                    break
            else:
                existing.append(point)
            atomic_json(path, existing)
        return point, path

    def save_group(self, mapid, poseid, task_command, point_type,
                   reference=None, work_points=None):
        value = {
            'mapid': mapid,
            'poseid': poseid,
            'command': normalize_task_command(task_command),
            'reference': reference,
            'work_points': list(work_points or []),
        }
        if point_type is not NORMAL_POINT:
            value['point_type'] = normalize_point_type(point_type)
        return self.save(value)

    def upsert_work(self, mapid, poseid, task_command, point_type, work,
                    reference=None):
        task_command = normalize_task_command(task_command)
        work = dict(work)
        work_id = self.component(work.get('work_id'), 'work_id')
        with self.lock:
            points, unused_path = self.load(mapid, poseid)
            group = next((value for value in points
                          if task_command_of(value) == task_command), None)
            if group is None:
                group = {
                    'mapid': mapid, 'poseid': poseid,
                    'command': task_command,
                    'reference': reference, 'work_points': [],
                }
                if point_type is not NORMAL_POINT:
                    group['point_type'] = point_type
            elif point_type_of(group) != normalize_point_type(point_type):
                raise RuntimeError('task-group point_type cannot be changed')
            if reference is not None:
                group['reference'] = reference
            works = list(group.get('work_points', []))
            for index, current in enumerate(works):
                if str(current.get('work_id')) == work_id:
                    works[index] = work
                    break
            else:
                works.append(work)
            works.sort(key=lambda value: int(value.get('index', 0)))
            group['work_points'] = works
            return self.save(group)

    def save_icp(self, mapid, poseid, task_command, a_record, b_record):
        work = dict(b_record)
        work.update(work_id='W1', index=1, command=2)
        return self.save_group(
            mapid, poseid, task_command, ICP_POINT, dict(a_record), [work])

    def save_apriltag(self, mapid, poseid, task_command, observe_flange_pose,
                      tag_id, tag_offset_xyz_mm, observe_joints=None):
        reference = {
            'tag_id': int(tag_id),
            'observe_flange_pose': [float(v) for v in observe_flange_pose],
            'tag_offset_xyz_mm': [float(v) for v in tag_offset_xyz_mm],
        }
        if observe_joints is not None:
            joints = np.asarray(observe_joints, dtype=float).reshape(-1)
            if joints.shape != (6,) or not np.all(np.isfinite(joints)):
                raise RuntimeError(
                    'observe_joints must contain six finite joint angles')
            reference['observe_joints'] = joints.astype(float).tolist()
        return self.save_group(
            mapid, poseid, task_command, APRILTAG_POINT, reference, [])

    def delete(self, mapid, poseid, task_command):
        mapid = self.component(mapid, 'mapid')
        poseid = self.component(poseid, 'poseid')
        task_command = normalize_task_command(task_command)
        path = self.manifest_path(mapid, poseid)
        with self.lock:
            points, unused_path = self.load(mapid, poseid)
            removed = [point for point in points
                       if task_command_of(point) == task_command]
            if not removed:
                raise RuntimeError(
                    'task group not found: %s/%s/%s' %
                    (mapid, poseid, task_command))
            atomic_json(path, [point for point in points
                               if task_command_of(point) != task_command])
            directory = self.record_dir(
                mapid, poseid, task_command, point_type_of(removed[0]))
            removed_dirs = self._remove_record_directories(
                removed[0], [directory])
        return removed[0], path, removed_dirs

    def _remove_record_directories(self, point, directories):
        records_root = os.path.realpath(os.path.join(self.root, 'records'))
        candidates = list(directories)
        records = [
            point.get('reference'), point.get('icp_a'), point.get('icp_b')]
        records.extend(point.get('work_points', []))
        for record in records:
            if isinstance(record, dict) and record.get('record_dir'):
                candidates.append(record['record_dir'])
        removed = []
        for candidate in dict.fromkeys(candidates):
            path = os.path.realpath(os.path.expanduser(str(candidate)))
            try:
                inside_records = os.path.commonpath(
                    [records_root, path]) == records_root
            except ValueError:
                inside_records = False
            if inside_records and path != records_root and os.path.isdir(path):
                shutil.rmtree(path)
                removed.append(path)
        return removed

    def delete_station(self, mapid, poseid):
        """Delete every command group belonging to one map/pose UUID."""
        mapid = self.component(mapid, 'mapid')
        poseid = self.component(poseid, 'poseid')
        path = self.manifest_path(mapid, poseid)
        with self.lock:
            points, unused_path = self.load(mapid, poseid)
            if not points:
                raise RuntimeError(
                    'station not found: %s/%s' % (mapid, poseid))
            atomic_json(path, [])
            removed_dirs = []
            for point in points:
                directory = self.record_dir(
                    mapid, poseid, task_command_of(point),
                    point_type_of(point))
                removed_dirs.extend(self._remove_record_directories(
                    point, [directory]))
        return points, path, removed_dirs

    @staticmethod
    def _requested_commands(params):
        requested = (params or {}).get('task_command')
        if requested in (None, ''):
            return []
        return [normalize_task_command(requested)]

    def select(self, params, expected_type=_NO_TYPE_FILTER):
        params = params or {}
        mapid, poseid = self.identity(params)
        points, path = self.load(mapid, poseid)
        requested_type = _NO_TYPE_FILTER
        raw_type = params.get('point_type', params.get('pointType'))
        if raw_type not in (None, ''):
            requested_type = normalize_point_type(raw_type)
        if expected_type is not _NO_TYPE_FILTER:
            expected_type = normalize_point_type(expected_type)
            if (requested_type is not _NO_TYPE_FILTER and
                    requested_type != expected_type):
                raise RuntimeError('requested point_type does not match action')
            requested_type = expected_type

        commands = self._requested_commands(params)
        if commands:
            by_command = {task_command_of(point): point for point in points}
            missing = [command for command in commands
                       if command not in by_command]
            if missing:
                raise RuntimeError(
                    'task commands not found for mapid=%s poseid=%s: %s' %
                    (mapid, poseid, ','.join(missing)))
            selected = [by_command[command] for command in commands]
        else:
            selected = list(points)

        if requested_type is not _NO_TYPE_FILTER:
            mismatched = [task_command_of(point) for point in selected
                          if point_type_of(point) != requested_type]
            if commands and mismatched:
                raise RuntimeError(
                    'point_type mismatch for task commands: %s' %
                    ','.join(mismatched))
            selected = [point for point in selected
                        if point_type_of(point) == requested_type]

        if not selected:
            suffix = ''
            if requested_type is not _NO_TYPE_FILTER:
                suffix = ' point_type=%s' % point_type_name(requested_type)
            raise RuntimeError(
                'no station task groups for mapid=%s poseid=%s%s' %
                (mapid, poseid, suffix))
        return mapid, poseid, selected, path

    def summaries(self, mapid, poseid):
        points, path = self.load(mapid, poseid)
        summaries = []
        for point in points:
            point_type = point_type_of(point)
            works = list(point.get('work_points', []))
            summary = {
                'command': task_command_of(point),
                'point_type': point_type,
                'point_kind': point_type_name(point_type),
                'mapid': point.get('mapid', mapid),
                'poseid': point.get('poseid', poseid),
                'work_count': len(works),
                'work_points': [
                    {'work_id': work.get('work_id'),
                     'index': work.get('index'),
                     'command': work.get('command')}
                    for work in works],
                'ready': bool(works),
                'updated_at': point.get('updated_at'),
            }
            reference = point.get('reference') or {}
            if point_type == ICP_POINT:
                summary['reference_record_id'] = reference.get('record_id')
            elif point_type == APRILTAG_POINT:
                summary['tag_id'] = reference.get('tag_id')
            summaries.append(summary)
        return summaries, path
