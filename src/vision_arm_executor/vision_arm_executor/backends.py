"""Stable backend adapters around the original ICP and AprilTag demos.

Only this module knows the demos' internal attributes.  The RPC/task layer
works with the explicit contracts below and is therefore independent from
interactive demo implementation details.
"""

from dataclasses import dataclass
import os

import numpy as np
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
    transform[:3, :3] = Rotation.from_euler(
        'xyz', pose[3:6], degrees=True).as_matrix()
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
            # Adapter boundary for the original demo's point-cloud source.
            self._servo_instance._pc_node = self.node
        return self._servo_instance

    def robot_state(self):
        return _robot_state(self._robot())

    def initialize_robot(self):
        if not self.initialized:
            result = self._robot().init()
            if result is False:
                raise RuntimeError('CR5 initialization failed')
            self.initialized = True

    def reset_robot(self):
        result = self._robot().reset_robot()
        if result is not False:
            self.initialized = False
        return result

    def enable_robot(self):
        result = self._robot().enable_robot()
        if result is not False:
            self.initialized = True
        return result

    def disable_robot(self):
        result = self._robot().disable_robot()
        if result is not False:
            self.initialized = False
        return result

    def clear_error(self):
        return self._robot().clear_error()

    def record_reference(self, frames, template_path):
        self.initialize_robot()
        servo = self._servo()
        if not servo.record_template(int(frames)):
            raise RuntimeError('ICP template recording failed')
        servo.save_template(template_path)
        transform = np.asarray(servo._T_base_tool_ref, dtype=float).copy()
        return IcpReference(transform, template_path, int(frames))

    def flange_transform(self):
        self.initialize_robot()
        return _pose_transform(self._robot().get_tool(warn=False))

    def align(self, template_path, max_iters, should_stop,
              progress_callback, motion_guard):
        self.initialize_robot()
        servo = self._servo()
        servo.load_template(template_path)
        servo.motion_guard = motion_guard
        try:
            converged = servo.align(
                int(max_iters), should_stop=should_stop,
                progress_callback=progress_callback)
        finally:
            servo.motion_guard = None
        return IcpAlignment(
            bool(converged), dict(servo.last_align_result))

    def move_relative(self, delta, label, motion_guard):
        current = self.flange_transform()
        target = current @ np.asarray(delta, dtype=float)
        motion_guard(target)
        if not self._robot().movj_pose(target, label):
            raise RuntimeError('relative B move failed')
        return target


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

    def identity(self):
        return AprilTagIdentity(
            tag_id=int(self._tag._target_tag_id()),
            tag_frame=str(self._tag.tag_frame),
            handeye_path=os.path.expanduser(self._tag.handeye_path),
            tcp_calibration_path=os.path.expanduser(
                self._tag.tcp_calibration_path),
        )

    def robot_state(self):
        return _robot_state(self._tag.robot)

    def disable_motion(self):
        self._tag.execute_enabled = False

    def locate(self):
        target = np.asarray(self._tag.locate(), dtype=float).copy()
        localization = dict(self._tag.last_localization or {})
        if not localization:
            raise RuntimeError('AprilTag locate returned no localization')
        return AprilTagLocation(target, localization, self.identity())

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
        self._tag.robot.initialize()
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
        rotation = float(np.degrees(
            (current_rotation * detection_rotation.inv()).magnitude()))
        return translation, rotation

    def pick(self, safety_check):
        self._tag.execute_enabled = True
        try:
            self._tag.pick(safety_check=safety_check)
        finally:
            self._tag.execute_enabled = False
