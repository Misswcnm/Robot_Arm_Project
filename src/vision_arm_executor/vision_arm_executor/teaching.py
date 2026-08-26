"""Process-local teaching session state, independent from robot and storage."""

from dataclasses import dataclass, field

from .station_store import (
    APRILTAG_POINT, ICP_POINT, NORMAL_POINT, normalize_point_type,
    point_type_name)


@dataclass
class TeachingSession:
    active: bool = False
    phase: str = 'idle'
    point_type: object = None
    mapid: object = None
    poseid: object = None
    task_command: object = None
    a_record_id: object = None
    b_record_id: object = None
    reference_record_id: object = None
    work_count: int = 0
    tag_id: object = None
    tag_offset_xyz_mm: list = field(default_factory=list)

    def begin(self, point_type, mapid, poseid, task_command, tag_id=None,
              tag_offset_xyz_mm=None):
        point_type = normalize_point_type(point_type)
        self.active = True
        self.point_type = point_type
        self.phase = (
            'awaiting_reference'
            if point_type in (ICP_POINT, APRILTAG_POINT)
            else 'awaiting_work')
        self.mapid = mapid
        self.poseid = poseid
        self.task_command = task_command
        self.a_record_id = None
        self.b_record_id = None
        self.reference_record_id = None
        self.work_count = 0
        self.tag_id = int(tag_id) if tag_id is not None else None
        self.tag_offset_xyz_mm = list(tag_offset_xyz_mm or [])

    def reset(self):
        self.active = False
        self.phase = 'idle'
        self.point_type = None
        self.mapid = None
        self.poseid = None
        self.task_command = None
        self.a_record_id = None
        self.b_record_id = None
        self.reference_record_id = None
        self.work_count = 0
        self.tag_id = None
        self.tag_offset_xyz_mm = []

    @property
    def kind(self):
        return point_type_name(self.point_type)

    def identity(self):
        return self.mapid, self.poseid, self.task_command, self.point_type

    def metrics(self):
        # Keep the old icp_* keys for the current application/gateway while
        # exposing neutral names for the mixed teaching state machine.
        return {
            'teaching_active': self.active,
            'teaching_phase': self.phase,
            'teaching_point_type': self.point_type,
            'teaching_kind': self.kind,
            'teaching_mapid': self.mapid,
            'teaching_poseid': self.poseid,
            'teaching_task_command': self.task_command,
            'teaching_work_count': self.work_count,
            'teaching_reference_record_id': self.reference_record_id,
            'icp_teaching_active': self.active,
            'icp_teaching_phase': self.phase,
            'icp_a_record_id': self.a_record_id,
            'icp_b_record_id': self.b_record_id,
            'icp_mapid': self.mapid,
            'icp_poseid': self.poseid,
            'icp_task_command': self.task_command,
            'apriltag_tag_id': self.tag_id,
            'apriltag_tag_offset_xyz_mm': list(self.tag_offset_xyz_mm),
        }

    def legacy_snapshot(self):
        return {
            'active': self.active,
            'phase': self.phase,
            'point_type': self.point_type,
            'a_record_id': self.a_record_id,
            'b_record_id': self.b_record_id,
            'reference_record_id': self.reference_record_id,
            'work_count': self.work_count,
            'mapid': self.mapid,
            'poseid': self.poseid,
            'task_command': self.task_command,
            'tag_id': self.tag_id,
            'tag_offset_xyz_mm': list(self.tag_offset_xyz_mm),
        }
