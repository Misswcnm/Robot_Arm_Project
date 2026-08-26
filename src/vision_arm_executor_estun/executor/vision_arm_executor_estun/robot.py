"""CR-compatible robot facade backed by the ESTUN Codroid ROS 2 bridge."""

import json
import os
import threading
import time

import numpy as np
import rclpy
from apriltag_pick.rotation_compat import (
    rotation_as_matrix,
    rotation_between_vectors,
    rotation_from_matrix,
    rotation_magnitude,
    rotation_stack,
)
from estun_codroid_bridge.srv import (
    GetJoints,
    GetPose,
    GetRobotState,
    Move,
    RobotCommand,
    SetDo,
    SolvePose,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from scipy.spatial.transform import Rotation, Slerp


class EstunRobot:
    """Methods consumed by the shared ICP and AprilTag backends.

    The public NX JSON protocol ends above this class. All ESTUN-specific
    command/state and Euler-order conversion stays here or in the C++ bridge.
    """

    MODE_DISABLED = 4
    MODE_ENABLED = 5
    MODE_BACKDRIVE = 6
    MODE_RUNNING = 7
    MODE_ERROR = 9
    MODE_COLLISION = 11

    ESTUN_STATE_INIT = 0
    ESTUN_STATE_STANDBY = 1
    ESTUN_STATE_READY = 2
    ESTUN_STATE_RESCUE = 3
    ESTUN_STATE_AUTO = 4
    ESTUN_STATE_ERROR = 6

    def __init__(self, node, speed=15):
        self._node = node
        self._logger = node.get_logger()
        self._speed = max(1, min(100, int(speed)))
        self.dragging = False
        self.last_enable_was_noop = False
        self.last_icp_motion_mode = None
        self.last_icp_waypoint_count = 0
        self._warned_manual_drag = False
        self._service_group = ReentrantCallbackGroup()

        self._service_namespace = str(self._param(
            'estun_service_namespace', '/estun_codroid')).rstrip('/')
        self._manual_drag_passthrough = bool(self._param(
            'estun_manual_drag_passthrough', True))
        self._arrival_position_mm = float(self._param(
            'estun_arrival_position_mm', 1.5))
        self._arrival_rotation_deg = float(self._param(
            'estun_arrival_rotation_deg', 1.0))
        self._arrival_joint_deg = float(self._param(
            'estun_arrival_joint_deg', 0.2))
        self._motion_timeout_sec = float(self._param(
            'estun_motion_timeout_sec', 30.0))
        self._apriltag_cartesian_step_mm = float(self._param(
            'estun_apriltag_cartesian_step_mm', 30.0))
        self._apriltag_rotation_step_deg = float(self._param(
            'estun_apriltag_rotation_step_deg', 5.0))
        self._apriltag_approach_mm = float(self._param(
            'estun_apriltag_approach_mm', 50.0))
        self._apriltag_search_max_tilt_deg = float(self._param(
            'estun_apriltag_search_max_tilt_deg', 25.0))
        self._apriltag_search_tilt_step_deg = float(self._param(
            'estun_apriltag_search_tilt_step_deg', 5.0))
        self._icp_waypoint_step_mm = float(self._param(
            'estun_icp_waypoint_step_mm', 30.0))
        self._icp_waypoint_max_steps = int(self._param(
            'estun_icp_waypoint_max_steps', 4))
        if self._apriltag_cartesian_step_mm <= 0.0:
            raise ValueError('estun_apriltag_cartesian_step_mm must be positive')
        if self._apriltag_rotation_step_deg <= 0.0:
            raise ValueError('estun_apriltag_rotation_step_deg must be positive')
        if self._apriltag_approach_mm <= 0.0:
            raise ValueError('estun_apriltag_approach_mm must be positive')
        if (self._apriltag_search_max_tilt_deg <= 0.0 or
                self._apriltag_search_tilt_step_deg <= 0.0):
            raise ValueError('ESTUN AprilTag orientation search must be positive')
        if self._icp_waypoint_step_mm <= 0.0:
            raise ValueError('estun_icp_waypoint_step_mm must be positive')
        if self._icp_waypoint_max_steps < 2:
            raise ValueError('estun_icp_waypoint_max_steps must be at least 2')
        self._reset_joints = np.asarray(self._param(
            'estun_reset_joints',
            [86.0, 23.0, -112.0, -176.0, -85.0, 0.0]),
            dtype=float).reshape(-1)
        if (self._reset_joints.size != 6 or
                not np.all(np.isfinite(self._reset_joints))):
            raise ValueError(
                'estun_reset_joints must contain six finite joint angles')
        self._reset_joints = self._reset_joints.astype(float).tolist()
        self._gripper_do_port = int(self._param(
            'estun_gripper_do_port', 18))
        self._gripper_config_path = os.path.expanduser(str(self._param(
            'estun_gripper_config_path',
            '~/Robot_Arm_Project/data/vision_arm_estun/'
            'calibration/gripper/active.json')))
        self._gripper_active_high = None
        self._load_gripper_config()

        def service(service_type, endpoint):
            return node.create_client(
                service_type,
                self._service_namespace + '/' + endpoint,
                callback_group=self._service_group)

        self._get_pose = service(GetPose, 'get_pose')
        self._get_joints = service(GetJoints, 'get_joints')
        self._get_state = service(GetRobotState, 'get_robot_state')
        self._robot_command = service(RobotCommand, 'command')
        self._move = service(Move, 'move')
        self._set_do = service(SetDo, 'set_do')
        self._solve_pose = service(SolvePose, 'solve_pose')

    def _param(self, name, default):
        if self._node.has_parameter(name):
            return self._node.get_parameter(name).value
        return self._node.declare_parameter(name, default).value

    def _load_gripper_config(self):
        try:
            with open(self._gripper_config_path, encoding='utf-8') as stream:
                config = json.load(stream)
            port = int(config['do_port'])
            if not 1 <= port <= 256:
                raise ValueError('do_port must be in 1..256')
            self._gripper_do_port = port
            self._gripper_active_high = bool(config['active_high'])
            self._logger.info(
                'ESTUN gripper config loaded: DO%d active_high=%s' %
                (port, self._gripper_active_high))
        except FileNotFoundError:
            return
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) \
                as error:
            self._logger.error(
                'invalid ESTUN gripper config %s: %s' %
                (self._gripper_config_path, error))

    def _call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=min(1.0, timeout)):
            self._logger.error(
                'ESTUN Codroid service unavailable: %s' % client.srv_name)
            return None
        future = client.call_async(request)
        if getattr(self._node, 'executor', None) is not None:
            completed = threading.Event()
            future.add_done_callback(lambda unused: completed.set())
            completed.wait(timeout)
        else:
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            self._logger.error(
                'ESTUN Codroid service timed out: %s' % client.srv_name)
            return None
        try:
            return future.result()
        except Exception as error:
            self._logger.error('ESTUN Codroid service call failed: %s' % error)
            return None

    @staticmethod
    def estun_pose_to_cr(values):
        """Legacy ERI [xyz,yaw,pitch,roll] mm/rad to vision XYZ-RPY."""
        values = np.asarray(values, dtype=float).reshape(-1)
        if values.size < 6 or not np.all(np.isfinite(values[:6])):
            return None
        return [
            float(values[0]), float(values[1]), float(values[2]),
            float(np.degrees(values[5])),
            float(np.degrees(values[4])),
            float(np.degrees(values[3])),
        ]

    @staticmethod
    def cr_pose_to_estun(values):
        """Legacy helper kept for calibration compatibility tests."""
        values = np.asarray(values, dtype=float).reshape(-1)
        if values.size < 6 or not np.all(np.isfinite(values[:6])):
            raise ValueError('pose must contain six finite values')
        return [
            float(values[0]), float(values[1]), float(values[2]),
            float(np.radians(values[5])),
            float(np.radians(values[4])),
            float(np.radians(values[3])),
        ]

    @staticmethod
    def matrix_to_pose(transform):
        transform = np.asarray(transform, dtype=float)
        rpy = rotation_from_matrix(transform[:3, :3]).as_euler(
            'xyz', degrees=True)
        return [
            float(transform[0, 3]), float(transform[1, 3]),
            float(transform[2, 3]), float(rpy[0]), float(rpy[1]),
            float(rpy[2]),
        ]

    @staticmethod
    def _pose_residual(actual, target):
        position = float(np.linalg.norm(
            np.asarray(actual[:3], dtype=float) -
            np.asarray(target[:3], dtype=float)))
        actual_rotation = Rotation.from_euler(
            'xyz', actual[3:6], degrees=True)
        target_rotation = Rotation.from_euler(
            'xyz', target[3:6], degrees=True)
        rotation = float(np.degrees(rotation_magnitude(
            target_rotation * actual_rotation.inv())))
        return position, rotation

    def get_tool(self, warn=True):
        response = self._call(self._get_pose, GetPose.Request(), timeout=3.0)
        if response is None or not response.success:
            if warn:
                detail = response.message if response is not None else ''
                self._logger.warn('ESTUN get_pose unavailable: %s' % detail)
            return None
        pose = np.asarray(response.pose, dtype=float).reshape(-1)
        if pose.size != 6 or not np.all(np.isfinite(pose)):
            if warn:
                self._logger.warn('ESTUN get_pose returned invalid data')
            return None
        return pose.astype(float).tolist()

    def get_joints(self):
        response = self._call(
            self._get_joints, GetJoints.Request(), timeout=3.0)
        if response is None or not response.success:
            return None
        values = np.asarray(response.joints, dtype=float).reshape(-1)
        if values.size != 6 or not np.all(np.isfinite(values)):
            return None
        return values.astype(float).tolist()

    def _state(self):
        response = self._call(
            self._get_state, GetRobotState.Request(), timeout=3.0)
        if response is None or not response.success:
            return None
        return int(response.state)

    def get_mode(self):
        state = self._state()
        if state is None:
            return None
        if state == self.ESTUN_STATE_ERROR:
            return self.MODE_ERROR
        if state == self.ESTUN_STATE_AUTO:
            return self.MODE_ENABLED
        if state == self.ESTUN_STATE_READY:
            return self.MODE_BACKDRIVE
        return self.MODE_DISABLED

    def get_robot_mode(self):
        return self.get_mode()

    def _wait_mode(self, target_mode, timeout=5.0):
        """Poll until the controller confirms ``target_mode``.

        Codroid's Standby -> Ready -> Auto transition is asynchronous, so a
        single ``get_mode()`` immediately after SwitchOn/ToAuto/ToReady can
        read a stale state and report a false failure. This bounds the wait
        and only returns True once the requested mode is actually observed.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.get_mode() == target_mode:
                return True
            time.sleep(0.1)
        return False

    def _robot_error(self):
        return self._state() == self.ESTUN_STATE_ERROR

    def get_error_id(self):
        state = self._state()
        return '<query failed>' if state is None else 'state=%d' % state

    def _command(self, command, timeout=8.0):
        request = RobotCommand.Request()
        request.command = int(command)
        response = self._call(self._robot_command, request, timeout=timeout)
        if response is None or not response.success:
            detail = response.message if response is not None else 'no response'
            self._logger.error('ESTUN command %s failed: %s' % (command, detail))
            return False
        return True

    def use_base_tool0(self):
        # Compatibility hook for the shared CR5 backend.  Codroid does not
        # expose selectable User/Tool frames; getCurCPos is used directly.
        return True

    def use_base_frame(self):
        return True

    def apply_motion_safety(self):
        # Speed is attached to every Codroid motion request. Controller-side
        # collision and workspace limits remain authoritative.
        return True

    def init(self):
        if self.get_tool(warn=False) is None:
            raise RuntimeError('ESTUN Codroid get_pose unavailable')
        if self._robot_error() and not self.clear_error():
            raise RuntimeError(
                'ESTUN controller reports an error: ' + self.get_error_id())
        if not self.enable_robot():
            raise RuntimeError('ESTUN SwitchOn/ToAuto failed')
        return True

    def initialize(self):
        return self.init()

    def enable_robot(self):
        already_enabled = self.get_mode() == self.MODE_ENABLED
        self.last_enable_was_noop = already_enabled
        if already_enabled:
            self.dragging = False
            return True
        if not self._command(RobotCommand.Request.SWITCH_ON):
            return False
        if not self._command(RobotCommand.Request.TO_AUTO):
            return False
        self.dragging = False
        return self._wait_mode(self.MODE_ENABLED)

    def enable(self):
        return self.enable_robot()

    def disable_robot(self):
        if self.get_mode() == self.MODE_DISABLED:
            return True
        result = self._command(RobotCommand.Request.SWITCH_OFF)
        if result:
            self.dragging = False
        return result

    def reset_robot(self):
        """Restore the legacy ESTUN global reset joint position."""
        if self._robot_error() and not self.clear_error():
            return False
        if not self.enable_robot():
            return False
        if not self.movj(self._reset_joints):
            self._logger.error(
                'ESTUN reset position failed: joints=%s' %
                self._reset_joints)
            return False
        self._logger.info(
            'ESTUN reset position reached: joints=%s' % self._reset_joints)
        return True

    def clear_error(self):
        acknowledged = self._command(
            RobotCommand.Request.ACKNOWLEDGE_ERROR)
        warning_cleared = self._command(RobotCommand.Request.CLEAR_WARNING)
        return acknowledged and warning_cleared

    def recover(self):
        return self.clear_error() and self.enable_robot()

    def start_drag(self):
        if not self._manual_drag_passthrough:
            self._logger.error('ESTUN manual teaching passthrough is disabled')
            return False
        if self.get_tool(warn=False) is None:
            return False
        if not self._command(RobotCommand.Request.TO_READY):
            return False
        self.dragging = True
        if not self._warned_manual_drag:
            self._logger.warn(
                'ESTUN teaching is in Ready/manual mode. Move with the '
                'pendant or configured hand-guiding function, then send the '
                'unchanged command=1/2; ToReady itself is not freedrive.')
            self._warned_manual_drag = True
        return self._wait_mode(self.MODE_BACKDRIVE)

    def stop_drag(self):
        # Keep the original frontend semantics: finish teaching restores the
        # powered automatic state.
        if not self._command(RobotCommand.Request.SWITCH_ON):
            return False
        if not self._command(RobotCommand.Request.TO_AUTO):
            return False
        self.dragging = False
        return self._wait_mode(self.MODE_ENABLED)

    def _stop_motion(self):
        return self._command(RobotCommand.Request.STOP_MOTION, timeout=5.0)

    def wait_pose_arrival(self, target, start_pose=None, timeout=None,
                          pos_tol=None, rot_tol=None, **unused):
        del start_pose
        timeout = float(timeout or self._motion_timeout_sec)
        pos_tol = float(pos_tol or self._arrival_position_mm)
        rot_tol = float(rot_tol or self._arrival_rotation_deg)
        stable = 0
        position = float('inf')
        rotation = float('inf')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            current = self.get_tool(warn=False)
            if current is None:
                time.sleep(0.1)
                continue
            position, rotation = self._pose_residual(current, target)
            if position <= pos_tol and rotation <= rot_tol:
                stable += 1
                if stable >= 3:
                    return True, position, rotation
            else:
                stable = 0
            if self._robot_error():
                return False, position, rotation
            time.sleep(0.1)
        self._stop_motion()
        return False, position, rotation

    def _send_move(self, move_type, target, timeout):
        values = np.asarray(target, dtype=float).reshape(-1)
        if values.size != 6 or not np.all(np.isfinite(values)):
            return False
        request = Move.Request()
        request.move_type = int(move_type)
        request.target = values.astype(float).tolist()
        request.speed = float(self._speed)
        request.acceleration = float(self._speed)
        for attempt in range(2):
            response = self._call(
                self._move, request, timeout=float(timeout) + 5.0)
            if response is not None and response.success:
                return True
            detail = response.message if response is not None else 'no response'
            transient = ('End of file' in detail or 'Broken pipe' in detail)
            if transient and attempt == 0:
                # Codroid sometimes closes a completed WebSocket without a
                # final reply. Repeating the same absolute target is
                # idempotent and the failed call has already reset the socket.
                self._logger.warn(
                    'ESTUN motion acknowledgement disconnected; retrying '
                    'the same absolute target once: %s' % detail)
                continue
            self._logger.error('ESTUN move rejected: %s' % detail)
            return False
        return False

    def solve_pose(self, target, reference_joints=None):
        pose = (
            self.matrix_to_pose(target)
            if isinstance(target, np.ndarray) else list(target))
        request = SolvePose.Request()
        request.target = np.asarray(pose, dtype=float).reshape(6).tolist()
        request.use_reference = reference_joints is not None
        request.reference_joints = (
            np.asarray(reference_joints, dtype=float).reshape(6).tolist()
            if reference_joints is not None else [0.0] * 6)
        for attempt in range(2):
            response = self._call(self._solve_pose, request, timeout=8.0)
            if response is not None and response.success:
                return np.asarray(response.joints, dtype=float)
            detail = response.message if response is not None else 'no response'
            if (attempt == 0 and
                    ('End of file' in detail or 'Broken pipe' in detail)):
                continue
            return None
        return None

    def movj_solved_pose_status(
            self, target, label='ESTUN Codroid solved MovJ',
            start_pose=None, timeout=None):
        """Resolve one Cartesian target with Codroid, then move as APos.

        ICP uses this path instead of its numerical Jacobian fallback.  The
        controller must return a complete joint solution before any motion is
        sent, and the current joints are supplied to keep the same kinematic
        branch whenever possible.
        """
        if self.dragging or self.get_mode() != self.MODE_ENABLED:
            self._logger.error(
                '%s rejected: robot is not in Auto mode' % label)
            return 'failed'
        pose = (
            self.matrix_to_pose(target)
            if isinstance(target, np.ndarray) else list(target))
        timeout = float(timeout or self._motion_timeout_sec)
        current_pose = (
            list(start_pose) if start_pose is not None
            else self.get_tool(warn=False))
        if current_pose is not None:
            position, rotation = self._pose_residual(current_pose, pose)
            if (position <= self._arrival_position_mm and
                    rotation <= self._arrival_rotation_deg):
                self._logger.info(
                    '%s already at target: %.2f mm / %.2f deg; '
                    'motion skipped' % (label, position, rotation))
                return 'arrived'

        reference = self.get_joints()
        if reference is None:
            self._logger.error(
                '%s failed: current APos unavailable before solve' % label)
            return 'failed'
        joints = self.solve_pose(pose, reference)
        if joints is None:
            self._logger.error(
                '%s failed: Codroid cpostoapos found no solution' % label)
            return 'failed'
        self._logger.info(
            '%s solved before motion: joints=%s' % (
                label, np.round(joints, 4).tolist()))
        if not self.movj(joints):
            self._logger.error('%s failed: solved APos move rejected' % label)
            return 'failed'

        actual = self.get_tool(warn=False)
        if actual is None:
            self._logger.error(
                '%s failed: CPos feedback unavailable after APos move' % label)
            return 'failed'
        position, rotation = self._pose_residual(actual, pose)
        if (position <= self._arrival_position_mm and
                rotation <= self._arrival_rotation_deg):
            self._logger.info(
                '%s arrived through solved APos: %.2f mm / %.2f deg' % (
                    label, position, rotation))
            return 'arrived'
        self._logger.error(
            '%s solved APos reached wrong CPos: %.2f mm / %.2f deg' % (
                label, position, rotation))
        return 'failed'

    def movj_solved_pose(self, target, label='ESTUN Codroid solved MovJ',
                         timeout=None):
        return self.movj_solved_pose_status(
            target, label=label, timeout=timeout) == 'arrived'

    @staticmethod
    def _pose_transform(pose):
        transform = np.eye(4)
        transform[:3, :3] = rotation_as_matrix(Rotation.from_euler(
            'xyz', pose[3:], degrees=True))
        transform[:3, 3] = pose[:3]
        return transform

    def _smooth_icp_candidate_paths(self, start_pose, target_pose):
        """Yield short curved Cartesian paths ending at the exact target."""
        start_pose = np.asarray(start_pose, dtype=float)
        target_pose = np.asarray(target_pose, dtype=float)
        start_xyz = start_pose[:3]
        target_xyz = target_pose[:3]
        displacement = target_xyz - start_xyz
        distance = float(np.linalg.norm(displacement))
        direction = (
            displacement / distance if distance > 1e-6
            else np.asarray([1.0, 0.0, 0.0]))
        helper = np.asarray([0.0, 0.0, 1.0])
        if abs(float(np.dot(direction, helper))) > 0.9:
            helper = np.asarray([0.0, 1.0, 0.0])
        basis_u = np.cross(direction, helper)
        basis_u /= np.linalg.norm(basis_u)
        basis_v = np.cross(direction, basis_u)

        start_rotation = Rotation.from_euler(
            'xyz', start_pose[3:], degrees=True)
        target_rotation = Rotation.from_euler(
            'xyz', target_pose[3:], degrees=True)
        for radius in (
                self._icp_waypoint_step_mm,
                2.0 * self._icp_waypoint_step_mm):
            steps = min(
                self._icp_waypoint_max_steps,
                max(2, int(np.ceil(
                    max(distance, 2.0 * radius) /
                    self._icp_waypoint_step_mm))))
            fractions = np.linspace(0.0, 1.0, steps + 1)[1:]
            rotations = Slerp(
                [0.0, 1.0],
                rotation_stack([start_rotation, target_rotation]))(
                    fractions).as_euler('xyz', degrees=True)
            for azimuth_deg in np.arange(0.0, 360.0, 45.0):
                azimuth = np.radians(azimuth_deg)
                offset = radius * (
                    np.cos(azimuth) * basis_u +
                    np.sin(azimuth) * basis_v)
                control = 0.5 * (start_xyz + target_xyz) + offset
                path = []
                for fraction, rpy in zip(fractions, rotations):
                    one_minus = 1.0 - float(fraction)
                    xyz = (
                        one_minus * one_minus * start_xyz +
                        2.0 * one_minus * float(fraction) * control +
                        float(fraction) * float(fraction) * target_xyz)
                    path.append(np.concatenate([xyz, rpy]).tolist())
                # The Bezier endpoint is mathematically exact; assign the
                # original values to avoid any floating-point drift.
                path[-1] = target_pose.tolist()
                yield path

    def _solve_icp_path(self, current_pose, target_pose, reference_joints):
        """Plan the complete APos path without moving the robot."""
        direct = self.solve_pose(target_pose, reference_joints)
        if direct is not None:
            return [(list(target_pose), np.asarray(direct, dtype=float))], \
                'controller_ik_direct'

        for poses in self._smooth_icp_candidate_paths(
                current_pose, target_pose):
            reference = np.asarray(reference_joints, dtype=float)
            solved_path = []
            for pose in poses:
                joints = self.solve_pose(pose, reference)
                if joints is None:
                    break
                reference = np.asarray(joints, dtype=float)
                solved_path.append((pose, reference.copy()))
            if len(solved_path) == len(poses):
                return solved_path, 'controller_ik_smooth_path'
        return None, None

    def movj_solved_path_status(
            self, target, label='ESTUN ICP solved MovJ', start_pose=None,
            timeout=None, motion_guard=None):
        """Move to one unchanged ICP target through a fully solved path.

        The final target is solved first. If its current IK seed fails, Codroid
        is asked to solve short curved paths around the blocked region. Every
        waypoint, including the unchanged final target, must have a solution
        before the first real motion is sent. No Jacobian probing is performed.
        """
        del timeout
        self.last_icp_motion_mode = None
        self.last_icp_waypoint_count = 0
        if self.dragging or self.get_mode() != self.MODE_ENABLED:
            self._logger.error(
                '%s rejected: robot is not in Auto mode' % label)
            return 'failed'
        target_pose = (
            self.matrix_to_pose(target)
            if isinstance(target, np.ndarray) else list(target))
        current_pose = (
            list(start_pose) if start_pose is not None
            else self.get_tool(warn=False))
        if current_pose is None:
            self._logger.error('%s failed: current CPos unavailable' % label)
            return 'failed'
        position, rotation = self._pose_residual(current_pose, target_pose)
        if (position <= self._arrival_position_mm and
                rotation <= self._arrival_rotation_deg):
            return 'arrived'
        reference = self.get_joints()
        if reference is None:
            self._logger.error(
                '%s failed: current APos unavailable before path solve' %
                label)
            return 'failed'

        solved_path, motion_mode = self._solve_icp_path(
            current_pose, target_pose, reference)
        if solved_path is None:
            self._logger.error(
                '%s failed: final target and all smooth waypoint paths '
                'have no complete Codroid IK solution' % label)
            return 'failed'
        for pose, _ in solved_path:
            if motion_guard is not None:
                motion_guard(self._pose_transform(pose))

        self.last_icp_motion_mode = motion_mode
        self.last_icp_waypoint_count = max(0, len(solved_path) - 1)
        self._logger.info(
            '%s accepted via %s: %d transition waypoint(s), final target '
            'unchanged' % (
                label, motion_mode, self.last_icp_waypoint_count))
        for index, (_, joints) in enumerate(solved_path, start=1):
            if not self.movj(joints):
                self._logger.error(
                    '%s failed at path move %d/%d' % (
                        label, index, len(solved_path)))
                return 'failed'

        actual = self.get_tool(warn=False)
        if actual is None:
            self._logger.error(
                '%s failed: CPos feedback unavailable after APos path' %
                label)
            return 'failed'
        position, rotation = self._pose_residual(actual, target_pose)
        if (position > self._arrival_position_mm or
                rotation > self._arrival_rotation_deg):
            self._logger.error(
                '%s reached wrong final CPos: %.2f mm / %.2f deg' % (
                    label, position, rotation))
            return 'failed'
        return 'arrived'

    @staticmethod
    def _rotation_distance_deg(left, right):
        return float(np.degrees(rotation_magnitude(rotation_from_matrix(
            np.asarray(left).T @ np.asarray(right)))))

    def _apriltag_candidate_rotations(
            self, nominal_rotation, tag_normal, tcp_axis_flange):
        """Generate near-normal tool attitudes, closest attitude first."""
        nominal_rotation = np.asarray(nominal_rotation, dtype=float)
        tag_normal = np.asarray(tag_normal, dtype=float)
        tag_normal /= np.linalg.norm(tag_normal)
        tcp_axis_flange = np.asarray(tcp_axis_flange, dtype=float)
        tcp_axis_flange /= np.linalg.norm(tcp_axis_flange)
        nominal_axis = nominal_rotation @ tcp_axis_flange
        normal = tag_normal if np.dot(nominal_axis, tag_normal) >= 0.0 \
            else -tag_normal

        helper = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(helper, normal))) > 0.9:
            helper = np.array([0.0, 1.0, 0.0])
        tangent_x = np.cross(normal, helper)
        tangent_x /= np.linalg.norm(tangent_x)
        tangent_y = np.cross(normal, tangent_x)

        directions = [normal]
        tilts = np.arange(
            self._apriltag_search_tilt_step_deg,
            self._apriltag_search_max_tilt_deg + 1e-6,
            self._apriltag_search_tilt_step_deg)
        for tilt_deg in tilts:
            for azimuth_deg in np.arange(0.0, 360.0, 45.0):
                azimuth = np.radians(azimuth_deg)
                tangent = (
                    np.cos(azimuth) * tangent_x +
                    np.sin(azimuth) * tangent_y)
                direction = Rotation.from_rotvec(
                    tangent * np.radians(tilt_deg)).apply(normal)
                directions.append(direction / np.linalg.norm(direction))

        candidates = [nominal_rotation]
        for direction in directions:
            align = rotation_between_vectors(tcp_axis_flange, direction)
            aligned = rotation_as_matrix(align)
            twists = []
            for twist_deg in np.arange(-180.0, 180.0, 5.0):
                twist = rotation_as_matrix(Rotation.from_rotvec(
                    direction * np.radians(twist_deg)))
                candidate = twist @ aligned
                twists.append((
                    self._rotation_distance_deg(
                        nominal_rotation, candidate), candidate))
            candidates.append(min(twists, key=lambda item: item[0])[1])

        unique = []
        for candidate in sorted(
                candidates,
                key=lambda item: self._rotation_distance_deg(
                    nominal_rotation, item)):
            if any(self._rotation_distance_deg(candidate, old) < 0.1
                   for old in unique):
                continue
            unique.append(candidate)
        return unique

    def move_apriltag_target(
            self, nominal_target, contact_tip, base_tag, tcp_offset_flange,
            safety_check=None, timeout=30.0):
        """Solve approach/contact first, then execute the two joint targets."""
        nominal_target = np.asarray(nominal_target, dtype=float)
        contact_tip = np.asarray(contact_tip, dtype=float).reshape(3)
        base_tag = np.asarray(base_tag, dtype=float)
        tcp_offset = np.asarray(tcp_offset_flange, dtype=float).reshape(3)
        tcp_norm = float(np.linalg.norm(tcp_offset))
        if tcp_norm <= 1e-6:
            self._logger.error('ESTUN TCP offset is zero; cannot define approach axis')
            return False
        reference = self.get_joints()
        if reference is None:
            self._logger.error('ESTUN current joints unavailable before IK search')
            return False
        rotations = self._apriltag_candidate_rotations(
            nominal_target[:3, :3], base_tag[:3, 2], tcp_offset / tcp_norm)
        for index, rotation in enumerate(rotations, start=1):
            axis_base = rotation @ (tcp_offset / tcp_norm)
            contact = np.eye(4)
            contact[:3, :3] = rotation
            contact[:3, 3] = contact_tip - rotation @ tcp_offset
            approach = contact.copy()
            approach[:3, 3] -= axis_base * self._apriltag_approach_mm
            approach_joints = self.solve_pose(approach, reference)
            if approach_joints is None:
                continue
            contact_joints = self.solve_pose(contact, approach_joints)
            if contact_joints is None:
                continue
            normal = np.asarray(base_tag[:3, 2], dtype=float)
            normal /= np.linalg.norm(normal)
            normal_angle = np.degrees(np.arccos(np.clip(
                abs(float(np.dot(axis_base, normal))), -1.0, 1.0)))
            self._logger.info(
                'ESTUN AprilTag IK candidate %d/%d accepted: '
                'normal deviation %.1f deg, approach %.1f mm' %
                (index, len(rotations), normal_angle,
                 self._apriltag_approach_mm))
            if safety_check is not None:
                safety_check('before_precontact', approach)
                safety_check('before_contact', contact)
            if not self.movj(approach_joints):
                return False
            if not self.movj(contact_joints):
                return False
            return True
        self._logger.error(
            'ESTUN AprilTag has no solvable approach/contact pair within '
            '%.1f deg of the Tag normal' %
            self._apriltag_search_max_tilt_deg)
        return False

    def _cartesian_pose_status(self, target, move_type, label,
                               start_pose=None, timeout=None):
        if self.dragging or self.get_mode() != self.MODE_ENABLED:
            self._logger.error('%s rejected: robot is not in Auto mode' % label)
            return 'failed'
        pose = (
            self.matrix_to_pose(target)
            if isinstance(target, np.ndarray) else list(target))
        timeout = float(timeout or self._motion_timeout_sec)

        # Codroid may reject a zero-distance Cartesian move with
        # "运动内核错误".  Treat an already reached target as an idempotent
        # success instead of sending a redundant motion command.
        current = self.get_tool(warn=False)
        if current is not None:
            position, rotation = self._pose_residual(current, pose)
            if (position <= self._arrival_position_mm and
                    rotation <= self._arrival_rotation_deg):
                self._logger.info(
                    '%s already at target: %.2f mm / %.2f deg; '
                    'Codroid move skipped' % (label, position, rotation))
                return 'arrived'

        if not self._send_move(move_type, pose, timeout):
            return 'failed'
        arrived, position, rotation = self.wait_pose_arrival(
            pose, start_pose=start_pose, timeout=timeout)
        if arrived:
            self._logger.info(
                '%s arrived: %.2f mm / %.2f deg' %
                (label, position, rotation))
            return 'arrived'
        self._logger.error(
            '%s timed out and StopMotion was sent: %.2f mm / %.2f deg' %
            (label, position, rotation))
        return 'failed'

    def movj_pose_status(self, target, label='ESTUN Codroid MovJ',
                         start_pose=None, timeout=None):
        """Execute a joint-space point-to-point move to a Cartesian CPos."""
        return self._cartesian_pose_status(
            target, Move.Request.MOVE_CARTESIAN_J, label,
            start_pose=start_pose, timeout=timeout)

    def movl_pose_status(self, target, label='ESTUN Codroid MovL',
                         start_pose=None, timeout=None):
        """Execute a TCP-linear move to a Cartesian CPos."""
        return self._cartesian_pose_status(
            target, Move.Request.MOVE_CARTESIAN_L, label,
            start_pose=start_pose, timeout=timeout)

    def movj_pose(self, target, label='ESTUN Codroid MovJ', timeout=None):
        return self.movj_pose_status(
            target, label=label, timeout=timeout) == 'arrived'

    def movl_pose(self, target, label='ESTUN Codroid MovL', timeout=None):
        return self.movl_pose_status(
            target, label=label, timeout=timeout) == 'arrived'

    def move_pose(self, transform, timeout=30.0):
        """Reach the exact AprilTag target through nearby native CPos moves."""
        start = self.get_tool(warn=False)
        if start is None:
            self._logger.error('ESTUN AprilTag start CPos is unavailable')
            return False
        target = self.matrix_to_pose(transform)
        translation_mm = float(np.linalg.norm(
            np.asarray(target[:3]) - np.asarray(start[:3])))
        start_rotation = Rotation.from_euler(
            'xyz', start[3:], degrees=True)
        target_rotation = Rotation.from_euler(
            'xyz', target[3:], degrees=True)
        rotation_deg = float(np.degrees(rotation_magnitude(
            start_rotation.inv() * target_rotation)))
        steps = max(
            1,
            int(np.ceil(
                translation_mm / self._apriltag_cartesian_step_mm)),
            int(np.ceil(rotation_deg / self._apriltag_rotation_step_deg)),
        )
        fractions = np.linspace(0.0, 1.0, steps + 1)[1:]
        rotations = Slerp(
            [0.0, 1.0],
            rotation_stack([start_rotation, target_rotation]))(
                fractions).as_euler('xyz', degrees=True)
        start_xyz = np.asarray(start[:3], dtype=float)
        target_xyz = np.asarray(target[:3], dtype=float)
        self._logger.info(
            'ESTUN AprilTag smooth CPos path: %.1f mm / %.1f deg, '
            '%d steps; final target unchanged' %
            (translation_mm, rotation_deg, steps))
        for index, (fraction, rpy) in enumerate(
                zip(fractions, rotations), start=1):
            xyz = start_xyz + float(fraction) * (target_xyz - start_xyz)
            pose = np.concatenate([xyz, rpy]).tolist()
            status = self._cartesian_pose_status(
                pose, Move.Request.MOVE_CARTESIAN_J,
                'ESTUN AprilTag CPos step %d/%d' % (index, steps),
                timeout=timeout)
            if status != 'arrived':
                return False
        return True

    def movj(self, joints):
        target = np.asarray(joints, dtype=float).reshape(-1)
        if target.size != 6 or not np.all(np.isfinite(target)):
            return False
        current = self.get_joints()
        if current is not None:
            residual = float(np.max(np.abs(
                np.asarray(current, dtype=float) - target)))
            if residual <= self._arrival_joint_deg:
                self._logger.info(
                    'ESTUN Codroid MovJ(APos) already at target: '
                    'max joint residual %.3f deg; move skipped' % residual)
                return True
        if not self._send_move(
                Move.Request.MOVE_JOINT, target, self._motion_timeout_sec):
            return False
        deadline = time.monotonic() + self._motion_timeout_sec
        stable = 0
        while time.monotonic() < deadline:
            current = self.get_joints()
            if current is not None and np.max(np.abs(
                    np.asarray(current) - target)) <= self._arrival_joint_deg:
                stable += 1
                if stable >= 3:
                    return True
            else:
                stable = 0
            time.sleep(0.1)
        self._stop_motion()
        return False

    def wait_tool_stable(self, timeout=5.0, **unused):
        previous = None
        stable = 0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            current = self.get_tool(warn=False)
            if current is not None and previous is not None:
                position, rotation = self._pose_residual(current, previous)
                stable = stable + 1 if (
                    position < 0.3 and rotation < 0.1) else 0
                if stable >= 3:
                    return True
            previous = current
            time.sleep(0.1)
        return False

    def set_gripper(self, closed, index=1, active_high=True):
        del index
        request = SetDo.Request()
        request.port = self._gripper_do_port
        polarity = (
            self._gripper_active_high
            if self._gripper_active_high is not None else bool(active_high))
        request.value = bool(closed) == polarity
        response = self._call(self._set_do, request, timeout=3.0)
        return bool(response is not None and response.success)
