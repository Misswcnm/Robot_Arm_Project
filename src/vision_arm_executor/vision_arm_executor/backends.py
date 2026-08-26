"""Stable backend adapters around the original ICP and AprilTag demos.

Only this module knows the demos' internal attributes.  The RPC/task layer
works with the explicit contracts below and is therefore independent from
interactive demo implementation details.
"""

from dataclasses import dataclass
import os
import threading
import time

import numpy as np
from apriltag_pick.rotation_compat import (
    rotation_as_matrix,
    rotation_magnitude,
)
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class RobotState:
    mode: object
    pose: object
    dragging: bool
    enabled_mode: int
    backdrive_mode: int


@dataclass(frozen=True)
class IcpReference:
    flange_transform: np.ndarray
    template_path: str
    frames: int
    joints: object = None


@dataclass(frozen=True)
class IcpAlignment:
    converged: bool
    metrics: dict


@dataclass(frozen=True)
class AprilTagIdentity:
    tag_id: int
    tag_frame: str
    handeye_path: str
    tcp_calibration_path: str


@dataclass(frozen=True)
class AprilTagLocation:
    target: np.ndarray
    localization: dict
    identity: AprilTagIdentity


def _robot_mode(robot):
    getter = getattr(robot, 'get_mode', None)
    if getter is None:
        getter = getattr(robot, 'get_robot_mode')
    return getter()


def _robot_state(robot):
    return RobotState(
        mode=_robot_mode(robot),
        pose=robot.get_tool(warn=False),
        dragging=bool(getattr(robot, 'dragging', False)),
        enabled_mode=int(getattr(robot, 'MODE_ENABLED', 5)),
        backdrive_mode=int(getattr(robot, 'MODE_BACKDRIVE', 6)),
    )


def _pose_transform(pose):
    if pose is None:
        raise RuntimeError('GetPose failed')
    transform = np.eye(4)
    transform[:3, 3] = pose[:3]
    transform[:3, :3] = rotation_as_matrix(Rotation.from_euler(
        'xyz', pose[3:6], degrees=True))
    return transform


class IcpBackend:
    """Public ICP/CR5 operations consumed by the vision task executor."""

    def __init__(self, node, cfg, robot_factory=None, servo_factory=None,
                 handeye_loader=None):
        self.node = node
        self.cfg = cfg
        self.robot_factory = robot_factory
        self.servo_factory = servo_factory
        self.handeye_loader = handeye_loader
        self._robot_instance = None
        self._servo_instance = None
        self.initialized = False
        self.last_enable_was_noop = False

    def _robot(self):
        if self._robot_instance is None:
            if self.robot_factory:
                self._robot_instance = self.robot_factory(
                    self.node, int(self.cfg['robot_speed']))
            else:
                from icp_servoing.robot import CR5Robot
                self._robot_instance = CR5Robot(
                    self.node, speed=int(self.cfg['robot_speed']))
        return self._robot_instance

    def _servo(self):
        if self._servo_instance is None:
            if self.handeye_loader is None:
                from icp_servoing.handeye import load_X
                loader = load_X
            else:
                loader = self.handeye_loader
            if self.servo_factory is None:
                from icp_servoing.servoing import VisualServo
                factory = VisualServo
            else:
                factory = self.servo_factory
            self._servo_instance = factory(
                self._robot(),
                loader(os.path.expanduser(self.cfg['handeye_path'])))
            for key, attribute, cast in (
                    ('icp_final_trans_thresh_mm',
                     'final_icp_trans_thresh_mm', float),
                    ('icp_final_rot_thresh_deg',
                     'final_icp_rot_thresh_deg', float),
                    ('icp_final_stable_frames',
                     'final_stable_frames', int),
                    ('icp_fresh_frames_after_motion',
                     'fresh_frames_after_motion', int),
                    ('icp_pointcloud_timeout_sec',
                     'pointcloud_timeout_sec', float),
                    ('icp_motion_confirmation_frames',
                     'motion_confirmation_frames', int),
                    ('icp_motion_confirmation_translation_mm',
                     'motion_confirmation_translation_mm', float),
                    ('icp_motion_confirmation_rotation_deg',
                     'motion_confirmation_rotation_deg', float),
                    ('icp_p2plane_max_refinement_translation_mm',
                     'p2plane_max_refinement_translation_mm', float),
                    ('icp_p2plane_max_refinement_rotation_deg',
                     'p2plane_max_refinement_rotation_deg', float)):
                if key in self.cfg:
                    setattr(
                        self._servo_instance, attribute,
                        cast(self.cfg[key]))
            # Adapter boundary for the original demo's point-cloud source.
            self._servo_instance._pc_node = self.node
        return self._servo_instance

    def robot_state(self):
        return _robot_state(self._robot())

    def initialize_robot(self):
        # ``initialized`` only means that this process completed an earlier
        # initialization.  The pendant, emergency-stop recovery, another
        # client, or a controller reconnect can still put the CR5 back in mode
        # 4.  Always reconcile the cache with live RobotMode before relying on
        # it.
        state = self.robot_state()
        if self.initialized and state.mode == state.enabled_mode:
            return state

        self.initialized = False
        result = self._robot().init()
        if result is False:
            raise RuntimeError('CR5 initialization failed')

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            state = self.robot_state()
            if state.mode == state.enabled_mode:
                self.initialized = True
                return state
            time.sleep(0.1)
        raise RuntimeError(
            'CR5 initialization completed but RobotMode did not become '
            'enabled (mode=%s, expected=%s)' %
            (state.mode, state.enabled_mode))

    def reset_robot(self):
        result = self._robot().reset_robot()
        if result is not False:
            self.initialized = False
        return result

    def enable_robot(self):
        # EnableRobot on an already ENABLED CR5 is redundant and can take
        # several seconds because the low-level helper also reapplies all
        # safety/User/Tool settings.  Read live state once and acknowledge the
        # idempotent request without sending another EnableRobot command.
        state = self.robot_state()
        if state.mode == state.enabled_mode and not state.dragging:
            self.initialized = True
            self.last_enable_was_noop = True
            return state
        self.last_enable_was_noop = False
        result = self._robot().enable_robot()
        if result is False:
            return False
        self.initialized = True
        return self.robot_state()

    def disable_robot(self):
        result = self._robot().disable_robot()
        if result is not False:
            self.initialized = False
        return result

    def clear_error(self):
        return self._robot().clear_error()

    def start_drag(self):
        """Enter CR5 backdrive mode for one bounded teaching workflow."""
        robot = self._robot()
        state = self.robot_state()
        if state.dragging or state.mode == state.backdrive_mode:
            setattr(robot, 'dragging', True)
            self.initialized = True
            return state
        # This validates a cached initialized=True against live RobotMode and
        # automatically re-runs the CR5 enable/safety/User0/Tool0 sequence when
        # the controller has fallen back to DISABLED (mode 4).
        state = self.initialize_robot()
        if state.mode != state.enabled_mode:
            raise RuntimeError(
                'cannot start teaching drag: robot mode=%s, expected=%s' %
                (state.mode, state.enabled_mode))
        if not robot.start_drag():
            raise RuntimeError('StartDrag failed')
        setattr(robot, 'dragging', True)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            state = self.robot_state()
            if state.mode == state.backdrive_mode:
                return state
            time.sleep(0.1)
        # Do not advertise a usable teaching session when the controller never
        # confirmed BACKDRIVE. Best-effort StopDrag leaves the arm in a safer,
        # recoverable state for a later explicit retry.
        setattr(robot, 'dragging', False)
        try:
            robot.stop_drag()
        except Exception:
            pass
        raise RuntimeError(
            'StartDrag returned success but RobotMode did not enter backdrive')

    def stop_drag_and_enable(self):
        """Leave backdrive and restore Enable/User0/Tool0/safety settings."""
        robot = self._robot()
        state = self.robot_state()
        if state.dragging or state.mode == state.backdrive_mode:
            if not robot.stop_drag():
                raise RuntimeError('StopDrag failed')
        setattr(robot, 'dragging', False)
        enable = getattr(robot, 'enable', None)
        if enable is None:
            enable = robot.enable_robot
        if not enable():
            self.initialized = False
            raise RuntimeError('EnableRobot after StopDrag failed')
        self.initialized = True
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            state = self.robot_state()
            if state.mode == state.enabled_mode:
                return state
            time.sleep(0.1)
        self.initialized = False
        raise RuntimeError(
            'EnableRobot returned success but RobotMode did not become enabled')

    def _require_teaching_drag(self):
        state = self.robot_state()
        if not state.dragging and state.mode != state.backdrive_mode:
            raise RuntimeError(
                'teaching record requires active drag/backdrive mode')
        return state

    def _record_reference(self, frames, template_path, require_joints=False):
        # A production teaching point is not complete without its exact joint
        # branch.  Read APos before the slower point-cloud capture so a failed
        # joint query cannot leave a record that later falls back to cpos-only
        # motion.
        joints = self._teaching_joints(required=require_joints)
        servo = self._servo()
        if not servo.record_template(int(frames)):
            raise RuntimeError('ICP template recording failed')
        servo.save_template(template_path)
        transform = np.asarray(servo._T_base_tool_ref, dtype=float).copy()
        return IcpReference(transform, template_path, int(frames), joints)

    def _teaching_joints(self, required=True):
        getter = getattr(self._robot(), 'get_joints', None)
        values = getter() if getter is not None else None
        joints = np.asarray(
            [] if values is None else values, dtype=float).reshape(-1)
        if joints.shape == (6,) and np.all(np.isfinite(joints)):
            return joints.astype(float).tolist()
        if required:
            raise RuntimeError('GetJoints failed during teaching')
        return None

    def record_reference(self, frames, template_path):
        self.initialize_robot()
        return self._record_reference(frames, template_path)

    def record_reference_teaching(self, frames, template_path):
        """Record point-cloud A and its flange pose without leaving drag mode."""
        self._require_teaching_drag()
        return self._record_reference(
            frames, template_path, require_joints=True)

    def flange_transform(self):
        self.initialize_robot()
        return _pose_transform(self._robot().get_tool(warn=False))

    def flange_transform_teaching(self):
        """Read the manually positioned B flange pose while drag remains active."""
        return _pose_transform(self.flange_pose_teaching())

    def flange_pose_teaching(self):
        """Read one GetPose sample without leaving the active drag session."""
        self._require_teaching_drag()
        pose = self._robot().get_tool(warn=False)
        if pose is None:
            raise RuntimeError('GetPose failed during teaching')
        return [float(value) for value in pose[:6]]

    def flange_joints_teaching(self):
        """Read the exact taught joint configuration while drag is active."""
        self._require_teaching_drag()
        return self._teaching_joints(required=True)

    def wait_teaching_stable(self, min_duration_sec=1.0, timeout_sec=8.0):
        """Require the manually dragged arm to remain still continuously."""
        self._require_teaching_drag()
        minimum = max(0.0, float(min_duration_sec))
        deadline = time.monotonic() + max(minimum, float(timeout_sec))
        previous = None
        stable_since = None
        while time.monotonic() < deadline:
            pose = self._robot().get_tool(warn=False)
            current_time = time.monotonic()
            if pose is None:
                previous = None
                stable_since = None
                time.sleep(0.1)
                continue
            current = np.asarray(pose[:6], dtype=float)
            if previous is not None:
                position_delta = float(np.linalg.norm(
                    current[:3] - previous[:3]))
                rotation_delta = float(np.linalg.norm(
                    (current[3:6] - previous[3:6] + 180.0) % 360.0 -
                    180.0))
                if position_delta < 0.3 and rotation_delta < 0.1:
                    if stable_since is None:
                        stable_since = current_time
                    if current_time - stable_since >= minimum:
                        return True
                else:
                    stable_since = None
            previous = current
            time.sleep(0.1)
        raise RuntimeError(
            '机械臂未连续停稳 %.1fs，未记录示教点' % minimum)

    def align(self, template_path, max_iters, should_stop,
              progress_callback, motion_guard):
        self.initialize_robot()
        servo = self._servo()
        begin_session = getattr(
            self.node, 'begin_pointcloud_session', None)
        end_session = getattr(self.node, 'end_pointcloud_session', None)
        session_started = False
        try:
            if begin_session is not None:
                begin_session()
                session_started = True
            servo.load_template(template_path)
            servo.motion_guard = motion_guard
            try:
                converged = servo.align(
                    int(max_iters), should_stop=should_stop,
                    progress_callback=progress_callback)
            finally:
                servo.motion_guard = None
        finally:
            if session_started and end_session is not None:
                end_session()
        return IcpAlignment(
            bool(converged), dict(servo.last_align_result))

    def move_to_reference(self, transform, label, motion_guard, joints=None):
        """Move to the taught absolute A pose before visual correction."""
        self.initialize_robot()
        target = np.asarray(transform, dtype=float)
        if target.shape != (4, 4) or not np.all(np.isfinite(target)):
            raise RuntimeError('ICP A flange transform is invalid')
        motion_guard(target)
        if joints is not None:
            if not self._robot().movj(joints):
                raise RuntimeError('move to ICP A joint reference failed')
        else:
            solved_move = getattr(
                self._robot(), 'movj_solved_pose', None)
            if solved_move is not None:
                moved = solved_move(target, label)
            else:
                moved = self._robot().movj_pose(target, label)
            if not moved:
                raise RuntimeError('move to ICP A reference failed')
        return target.copy()

    def move_relative(self, delta, label, motion_guard,
                      reference_transform=None, reference_joints=None,
                      base_transform=None):
        base = (
            self.flange_transform() if base_transform is None
            else np.asarray(base_transform, dtype=float))
        if base.shape != (4, 4) or not np.all(np.isfinite(base)):
            raise RuntimeError('corrected Ref flange transform is invalid')
        target = base @ np.asarray(delta, dtype=float)
        # Preserve the demonstrated shoulder/elbow/wrist branch with APos,
        # then apply only the small ICP correction in Cartesian space.  This
        # mirrors TCP_asdun_arm, whose production task points are all APos.
        if reference_joints is not None:
            nominal = np.asarray(reference_transform, dtype=float)
            if nominal.shape != (4, 4) or not np.all(np.isfinite(nominal)):
                raise RuntimeError('ICP B flange transform is invalid')
            motion_guard(nominal)
            if not self._robot().movj(reference_joints):
                raise RuntimeError('move to ICP B joint reference failed')
        motion_guard(target)
        solved_move = getattr(self._robot(), 'movj_solved_pose', None)
        if solved_move is not None:
            moved = solved_move(target, label)
        else:
            moved = self._robot().movj_pose(target, label)
        if not moved:
            raise RuntimeError('relative B move failed')
        return target

    def move_joints(self, joints, label, motion_guard=None,
                    target_transform=None):
        self.initialize_robot()
        values = np.asarray(joints, dtype=float).reshape(-1)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise RuntimeError('%s has invalid six-axis joints' % label)
        if target_transform is not None and motion_guard is not None:
            motion_guard(np.asarray(target_transform, dtype=float))
        if not self._robot().movj(values.astype(float).tolist()):
            raise RuntimeError('%s joint move failed' % label)
        return values.astype(float).tolist()


class AprilTagBackend:
    """Public AprilTag localization/validation/pick operations."""

    _ARRAY_KEYS = {
        'base_flange_at_detection', 'camera_tag', 'base_tag',
        'predicted_contact_tip', 'base_flange_target'}

    def __init__(self, node, tag_factory=None):
        if tag_factory:
            self._tag = tag_factory(node)
        else:
            from apriltag_pick.pick_node import AprilTagPickNode
            self._tag = AprilTagPickNode()
        self._configuration_lock = threading.RLock()
        self._robot_ready = False

    def identity(self, tag_id=None):
        tag_frame = str(self._tag.tag_frame)
        if tag_id is not None:
            try:
                tag_id = int(tag_id)
            except (TypeError, ValueError):
                raise RuntimeError('tag_id must be a non-negative integer')
            if tag_id < 0:
                raise RuntimeError('tag_id must be a non-negative integer')
            tag_frame = '%s:%d' % (
                tag_frame.rsplit(':', 1)[0], tag_id)
        else:
            tag_id = self._tag._target_tag_id()
        return AprilTagIdentity(
            tag_id=int(tag_id),
            tag_frame=tag_frame,
            handeye_path=os.path.expanduser(self._tag.handeye_path),
            tcp_calibration_path=os.path.expanduser(
                self._tag.tcp_calibration_path),
        )

    def robot_state(self):
        return _robot_state(self._tag.robot)

    def disable_motion(self):
        self._tag.execute_enabled = False

    @staticmethod
    def _tag_offset(value):
        values = np.asarray(value if value is not None else [0, 0, 0],
                            dtype=float).reshape(-1)
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise RuntimeError(
                'tag_offset_xyz_mm must contain three finite numbers')
        return values

    def locate(self, tag_id=None, tag_offset_xyz_mm=None,
               wait_for_new_detection=True, detection_timeout_sec=5.0):
        identity = self.identity(tag_id)
        locate_kwargs = {}
        if wait_for_new_detection:
            locate_kwargs.update(
                wait_for_new_detection=True,
                detection_timeout_sec=detection_timeout_sec)
        if tag_offset_xyz_mm is None:
            target = np.asarray(
                self._tag.locate(tag_id=identity.tag_id, **locate_kwargs),
                dtype=float).copy()
            localization = dict(self._tag.last_localization or {})
            if not localization:
                raise RuntimeError('AprilTag locate returned no localization')
            return AprilTagLocation(target, localization, identity)
        offset = self._tag_offset(tag_offset_xyz_mm)
        # The original AprilTag node already implements the calibrated
        # base/tag/tip/flange chain. Override only the per-point translation
        # for this bounded call and restore its launch-time default afterward.
        with self._configuration_lock:
            original = np.asarray(self._tag.tag_to_tcp, dtype=float).copy()
            configured = original.copy()
            configured[:3, 3] = offset
            self._tag.tag_to_tcp = configured
            try:
                target = np.asarray(
                    self._tag.locate(
                        tag_id=identity.tag_id, **locate_kwargs),
                    dtype=float).copy()
                localization = dict(self._tag.last_localization or {})
            finally:
                self._tag.tag_to_tcp = original
        if not localization:
            raise RuntimeError('AprilTag locate returned no localization')
        localization['tag_offset_xyz_mm'] = offset.copy()
        return AprilTagLocation(target, localization, identity)

    def _ensure_robot_ready(self):
        state = self.robot_state()
        if (not self._robot_ready or state.mode != state.enabled_mode or
                state.dragging):
            self._tag.robot.initialize()
            self._robot_ready = True
        return self.robot_state()

    def move_to_observation(self, flange_pose, safety_check, joints=None):
        values = np.asarray(flange_pose, dtype=float).reshape(-1)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise RuntimeError(
                'observe_flange_pose must contain six finite numbers')
        self._ensure_robot_ready()
        target = _pose_transform(values)
        safety_check('before_observation', target)
        if joints is not None:
            mover = getattr(self._tag.robot, 'movj', None)
            if mover is None or not mover(joints):
                raise RuntimeError('AprilTag observation joint move failed')
        elif not self._tag.robot.move_pose(target):
            raise RuntimeError('AprilTag observation pose move failed')
        return target

    def move_flange_target(self, target, safety_check, label='AprilTag Work'):
        target = np.asarray(target, dtype=float)
        if target.shape != (4, 4) or not np.all(np.isfinite(target)):
            raise RuntimeError('%s target transform is invalid' % label)
        self._ensure_robot_ready()
        safety_check('before_work', target)
        solved_move = getattr(self._tag.robot, 'movj_solved_pose', None)
        if solved_move is not None:
            moved = solved_move(target, label)
        else:
            moved = self._tag.robot.move_pose(target)
        if not moved:
            raise RuntimeError('%s move failed' % label)
        return target.copy()

    def lateral_observation_target(self, observation_target, offset_mm):
        """Shift an observation pose along the camera image horizontal axis."""
        target = np.asarray(observation_target, dtype=float).copy()
        flange_camera = np.asarray(
            self._tag.flange_from_camera, dtype=float)
        if (target.shape != (4, 4) or flange_camera.shape != (4, 4) or
                not np.all(np.isfinite(target)) or
                not np.all(np.isfinite(flange_camera))):
            raise RuntimeError('AprilTag search transform is invalid')
        camera_x_in_base = (
            target[:3, :3] @ flange_camera[:3, :3])[:, 0]
        target[:3, 3] += camera_x_in_base * float(offset_mm)
        return target

    def validate_manual(self):
        state = self.robot_state()
        if not state.dragging and state.mode != state.backdrive_mode:
            raise RuntimeError(
                'apriltag_validate requires manual drag positioning at '
                'the Tag center before recording truth')
        return self._tag.record_manual_tag_center()

    @classmethod
    def restore_localization(cls, value):
        result = dict(value)
        for key in cls._ARRAY_KEYS:
            if key in result:
                result[key] = np.asarray(result[key], dtype=float)
        return result

    def prepare_pick(self, localization):
        self._ensure_robot_ready()
        self._tag.last_localization = self.restore_localization(localization)

    def localization_drift(self, localization):
        current = self._tag.robot.get_tool(warn=False)
        detection = localization.get('flange_xyzrpy')
        if current is None or not detection:
            raise RuntimeError('cannot validate robot pose against Tag cache')
        translation = float(np.linalg.norm(
            np.asarray(current[:3]) - np.asarray(detection[:3])))
        current_rotation = Rotation.from_euler(
            'xyz', current[3:6], degrees=True)
        detection_rotation = Rotation.from_euler(
            'xyz', detection[3:6], degrees=True)
        rotation = float(np.degrees(rotation_magnitude(
            current_rotation * detection_rotation.inv())))
        return translation, rotation

    def pick(self, safety_check):
        self._tag.execute_enabled = True
        try:
            self._tag.pick(safety_check=safety_check)
        finally:
            self._tag.execute_enabled = False
