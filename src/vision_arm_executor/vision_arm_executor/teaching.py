"""Process-local teaching session state, independent from robot and storage."""

from dataclasses import dataclass, field

from .station_store import APRILTAG_POINT, ICP_POINT, normalize_point_type


@dataclass
class TeachingSession:
    active: bool = False
    phase: str = 'idle'
    point_type: object = None
    mapid: object = None
    poseid: object = None
    label: object = None
    a_record_id: object = None
    b_record_id: object = None
    tag_id: object = None
    tag_offset_xyz_mm: list = field(default_factory=list)

    def begin(self, point_type, mapid, poseid, label, tag_id=None,
              tag_offset_xyz_mm=None):
        point_type = normalize_point_type(point_type)
        self.active = True
        self.point_type = point_type
        self.phase = (
            'awaiting_a' if point_type == ICP_POINT else 'awaiting_pose')
        self.mapid = mapid
        self.poseid = poseid
        self.label = label
        self.a_record_id = None
        self.b_record_id = None
        self.tag_id = int(tag_id) if tag_id is not None else None
        self.tag_offset_xyz_mm = list(tag_offset_xyz_mm or [])

    def reset(self):
        self.active = False
        self.phase = 'idle'
        self.point_type = None
        self.mapid = None
        self.poseid = None
        self.label = None
        self.a_record_id = None
        self.b_record_id = None
        self.tag_id = None
        self.tag_offset_xyz_mm = []

    @property
    def kind(self):
        if self.point_type == ICP_POINT:
            return 'icp'
        if self.point_type == APRILTAG_POINT:
            return 'apriltag'
        return None

    def identity(self):
        return self.mapid, self.poseid, self.label, self.point_type

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
            'teaching_label': self.label,
            'icp_teaching_active': self.active,
            'icp_teaching_phase': self.phase,
            'icp_a_record_id': self.a_record_id,
            'icp_b_record_id': self.b_record_id,
            'icp_mapid': self.mapid,
            'icp_poseid': self.poseid,
            'icp_label': self.label,
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
            'mapid': self.mapid,
            'poseid': self.poseid,
            'label': self.label,
            'tag_id': self.tag_id,
            'tag_offset_xyz_mm': list(self.tag_offset_xyz_mm),
        }
