"""Validated, atomic storage for mixed ICP and AprilTag station points."""

import os
import re
import threading

import numpy as np

from .store import atomic_json, load_json, now


APRILTAG_POINT = 0
ICP_POINT = 1
POINT_TYPES = {APRILTAG_POINT, ICP_POINT}


def normalize_point_type(value, required=True):
    if value in (None, ''):
        if required:
            raise RuntimeError('point_type is required (0=AprilTag, 1=ICP)')
        return None
    if isinstance(value, bool):
        raise RuntimeError('point_type must be 0 or 1')
    try:
        point_type = int(value)
    except (TypeError, ValueError):
        raise RuntimeError('point_type must be 0 or 1')
    if point_type not in POINT_TYPES:
        raise RuntimeError('point_type must be 0 or 1')
    return point_type


def point_type_of(point):
    """Read the required point type from the current station schema."""
    if not isinstance(point, dict):
        raise RuntimeError('station point must be an object')
    return normalize_point_type(point.get('point_type'))


class StationStore:
    """Own the mapid/poseid manifest contract and all atomic replacements."""

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

    def record_dir(self, mapid, poseid, label, point_type):
        """Return the one directory that owns all artifacts for a point."""
        mapid = self.component(mapid, 'mapid')
        poseid = self.component(poseid, 'poseid')
        label = self.component(label, 'label')
        point_type = normalize_point_type(point_type)
        return os.path.join(
            self.root, 'records',
            '%s_%s_%s_%d' % (mapid, poseid, label, point_type))

    def load(self, mapid, poseid):
        path = self.manifest_path(mapid, poseid)
        with self.lock:
            points = load_json(path, default=[], strict=True)
        if not isinstance(points, list):
            raise RuntimeError('station manifest must be a point array')
        normalized = []
        for value in points:
            if not isinstance(value, dict):
                raise RuntimeError('station manifest contains a non-object point')
            point = dict(value)
            point['point_type'] = point_type_of(point)
            normalized.append(point)
        return normalized, path

    def next_label(self, mapid, poseid):
        points, unused_path = self.load(mapid, poseid)
        used = {str(point.get('label', '')) for point in points}
        index = 1
        while 'P%d' % index in used:
            index += 1
        return 'P%d' % index

    def teaching_identity(self, params):
        mapid, poseid = self.identity(params)
        label = str((params or {}).get('label', '') or '').strip()
        if not label:
            label = self.next_label(mapid, poseid)
        if ',' in label:
            raise RuntimeError('teaching label cannot contain a comma')
        return mapid, poseid, self.component(label, 'label')

    def save(self, point):
        point = dict(point)
        mapid = self.component(point.get('mapid'), 'mapid')
        poseid = self.component(point.get('poseid'), 'poseid')
        label = self.component(point.get('label'), 'label')
        point_type = normalize_point_type(point.get('point_type'))
        point.update(
            schema_version=2,
            mapid=mapid,
            poseid=poseid,
            label=label,
            point_type=point_type,
            updated_at=now())
        path = self.manifest_path(mapid, poseid)
        with self.lock:
            existing, unused_path = self.load(mapid, poseid)
            for current in existing:
                current.update(
                    schema_version=2,
                    mapid=self.component(
                        current.get('mapid', mapid), 'mapid'),
                    poseid=self.component(
                        current.get('poseid', poseid), 'poseid'),
                    label=self.component(current.get('label'), 'label'),
                    point_type=point_type_of(current))
            replaced = False
            for index, current in enumerate(existing):
                if str(current.get('label')) == label:
                    existing[index] = point
                    replaced = True
                    break
            if not replaced:
                existing.append(point)
            atomic_json(path, existing)
        return point, path

    def save_icp(self, mapid, poseid, label, a_record, b_record):
        return self.save({
            'mapid': mapid,
            'poseid': poseid,
            'label': label,
            'point_type': ICP_POINT,
            'icp_a': dict(a_record),
            'icp_b': dict(b_record),
        })

    def save_apriltag(self, mapid, poseid, label, observe_flange_pose,
                      tag_id, tag_offset_xyz_mm, observe_joints=None):
        apriltag = {
            'tag_id': int(tag_id),
            'observe_flange_pose': [
                float(value) for value in observe_flange_pose],
            'tag_offset_xyz_mm': [
                float(value) for value in tag_offset_xyz_mm],
        }
        if observe_joints is not None:
            joints = np.asarray(observe_joints, dtype=float).reshape(-1)
            if joints.shape != (6,) or not np.all(np.isfinite(joints)):
                raise RuntimeError(
                    'observe_joints must contain six finite joint angles')
            apriltag['observe_joints'] = joints.astype(float).tolist()
        return self.save({
            'mapid': mapid,
            'poseid': poseid,
            'label': label,
            'point_type': APRILTAG_POINT,
            'apriltag': apriltag,
        })

    @staticmethod
    def _requested_labels(params):
        requested = str((params or {}).get('label', '') or '').strip()
        return [value.strip() for value in requested.split(',')
                if value.strip()]

    def select(self, params, expected_type=None):
        params = params or {}
        mapid, poseid = self.identity(params)
        points, path = self.load(mapid, poseid)
        requested_type = normalize_point_type(
            params.get('point_type', params.get('pointType')),
            required=False)
        if expected_type is not None:
            expected_type = normalize_point_type(expected_type)
            if (requested_type is not None and
                    requested_type != expected_type):
                raise RuntimeError('requested point_type does not match action')
            requested_type = expected_type

        labels = self._requested_labels(params)
        if labels:
            by_label = {str(point.get('label')): point for point in points}
            missing = [label for label in labels if label not in by_label]
            if missing:
                raise RuntimeError(
                    'labels not found for mapid=%s poseid=%s: %s' %
                    (mapid, poseid, ','.join(missing)))
            selected = [by_label[label] for label in labels]
        else:
            selected = list(points)

        if requested_type is not None:
            mismatched = [
                str(point.get('label')) for point in selected
                if point_type_of(point) != requested_type]
            if labels and mismatched:
                raise RuntimeError(
                    'point_type mismatch for labels: %s' %
                    ','.join(mismatched))
            selected = [
                point for point in selected
                if point_type_of(point) == requested_type]

        if not selected:
            suffix = (
                ' point_type=%d' % requested_type
                if requested_type is not None else '')
            raise RuntimeError(
                'no station points for mapid=%s poseid=%s%s' %
                (mapid, poseid, suffix))

        return mapid, poseid, selected, path

    def summaries(self, mapid, poseid):
        points, path = self.load(mapid, poseid)
        summaries = []
        for point in points:
            point_type = point_type_of(point)
            summary = {
                'label': point.get('label'),
                'point_type': point_type,
                'mapid': point.get('mapid', mapid),
                'poseid': point.get('poseid', poseid),
                'updated_at': point.get('updated_at'),
            }
            if point_type == ICP_POINT:
                summary.update(
                    a_record_id=(point.get('icp_a') or {}).get('record_id'),
                    b_record_id=(point.get('icp_b') or {}).get('record_id'))
            else:
                apriltag = point.get('apriltag') or {}
                summary.update(
                    tag_id=apriltag.get('tag_id'),
                    tag_offset_xyz_mm=apriltag.get('tag_offset_xyz_mm'))
            summaries.append(summary)
        return summaries, path
