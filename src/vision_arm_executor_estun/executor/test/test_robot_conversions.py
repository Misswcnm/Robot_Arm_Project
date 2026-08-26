import threading

import numpy as np
from apriltag_pick.pick_node import AprilTagPickNode
from apriltag_pick.transforms import load_tcp_offset

from vision_arm_executor_estun.calibration_math import (
    PoseSample,
    pose_to_matrix,
    solve_handeye,
    solve_pivot,
)
from vision_arm_executor_estun.calibration_stack import (
    camera_command,
    driver_command,
)
from vision_arm_executor_estun.gripper_tool import load_config, save_config
from vision_arm_executor_estun.robot import EstunRobot


def test_estun_pose_round_trip():
    original = [100.0, -200.0, 300.0, 10.0, -20.0, 30.0]
    estun = EstunRobot.cr_pose_to_estun(original)
    restored = EstunRobot.estun_pose_to_cr(estun)
    np.testing.assert_allclose(restored, original, atol=1e-9)


def test_matrix_pose_uses_shared_xyz_rpy_contract():
    transform = np.eye(4)
    transform[:3, 3] = [1.0, 2.0, 3.0]
    pose = EstunRobot.matrix_to_pose(transform)
    np.testing.assert_allclose(pose, [1, 2, 3, 0, 0, 0], atol=1e-9)


def test_pose_residual_handles_wrapped_rotation():
    position, rotation = EstunRobot._pose_residual(
        [0, 0, 0, 0, 0, 179],
        [3, 4, 0, 0, 0, -179])
    assert abs(position - 5.0) < 1e-9
    assert abs(rotation - 2.0) < 1e-9


def test_teaching_uses_ready_then_restores_switch_on_auto():
    robot = object.__new__(EstunRobot)
    robot._manual_drag_passthrough = True
    robot._warned_manual_drag = True
    robot.dragging = False
    robot._logger = type('Logger', (), {
        'info': lambda self, message: None,
        'error': lambda self, message: None,
        'warn': lambda self, message: None,
    })()
    calls = []
    robot.get_tool = lambda warn=False: [0.0] * 6
    robot._command = lambda command, timeout=8.0: (
        calls.append(command) or True)
    robot.get_mode = lambda: robot.MODE_BACKDRIVE
    assert robot.start_drag()
    robot.get_mode = lambda: robot.MODE_ENABLED
    assert robot.stop_drag()
    from estun_codroid_bridge.srv import RobotCommand
    assert calls == [
        RobotCommand.Request.TO_READY,
        RobotCommand.Request.SWITCH_ON,
        RobotCommand.Request.TO_AUTO,
    ]


def test_movj_already_at_target_does_not_send_motion():
    robot = object.__new__(EstunRobot)
    robot.dragging = False
    robot._arrival_position_mm = 1.5
    robot._arrival_rotation_deg = 1.0
    robot._motion_timeout_sec = 30.0
    robot._logger = type('Logger', (), {
        'info': lambda self, message: None,
        'error': lambda self, message: None,
    })()
    target = [100.0, -200.0, 300.0, 10.0, -20.0, 30.0]
    robot.get_mode = lambda: robot.MODE_ENABLED
    robot.get_tool = lambda warn=False: [
        100.2, -200.1, 300.1, 10.1, -20.1, 30.1]
    robot._send_move = lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError('redundant Codroid move must not be sent'))

    assert robot.movj_pose_status(target) == 'arrived'


def test_joint_movj_already_at_target_does_not_send_motion():
    robot = object.__new__(EstunRobot)
    robot._arrival_joint_deg = 0.2
    robot._logger = type('Logger', (), {
        'info': lambda self, message: None,
    })()
    target = np.asarray([86.0, 23.0, -90.0, -67.0, -173.0, 0.5])
    robot.get_joints = lambda: (target + np.asarray(
        [0.01, -0.02, 0.03, 0.01, 0.0, -0.01])).tolist()
    robot._send_move = lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError('redundant joint move must not be sent'))

    assert robot.movj(target)


def test_icp_cartesian_target_is_solved_before_joint_motion():
    robot = object.__new__(EstunRobot)
    robot.dragging = False
    robot._motion_timeout_sec = 30.0
    robot._arrival_position_mm = 1.5
    robot._arrival_rotation_deg = 1.0
    robot._logger = type('Logger', (), {
        'info': lambda self, message: None,
        'error': lambda self, message: None,
    })()
    target = np.eye(4)
    target[:3, 3] = [100.0, 20.0, 30.0]
    feedback = iter([
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [100.0, 20.0, 30.0, 0.0, 0.0, 0.0],
    ])
    reference = np.asarray([1, 2, 3, 4, 5, 6], dtype=float)
    solution = np.asarray([11, 12, 13, 14, 15, 16], dtype=float)
    events = []
    robot.get_mode = lambda: robot.MODE_ENABLED
    robot.get_tool = lambda warn=False: next(feedback)
    robot.get_joints = lambda: reference.copy()
    robot.solve_pose = lambda pose, joints: (
        events.append(('solve', np.asarray(joints).copy())) or
        solution.copy())
    robot.movj = lambda joints: (
        events.append(('move', np.asarray(joints).copy())) or True)

    assert robot.movj_solved_pose_status(target) == 'arrived'
    assert [event[0] for event in events] == ['solve', 'move']
    np.testing.assert_allclose(events[0][1], reference)
    np.testing.assert_allclose(events[1][1], solution)


def test_icp_unsolved_cartesian_target_does_not_move():
    robot = object.__new__(EstunRobot)
    robot.dragging = False
    robot._motion_timeout_sec = 30.0
    robot._arrival_position_mm = 1.5
    robot._arrival_rotation_deg = 1.0
    robot._logger = type('Logger', (), {
        'info': lambda self, message: None,
        'error': lambda self, message: None,
    })()
    robot.get_mode = lambda: robot.MODE_ENABLED
    robot.get_tool = lambda warn=False: [0.0] * 6
    robot.get_joints = lambda: [1.0] * 6
    robot.solve_pose = lambda pose, joints: None
    robot.movj = lambda joints: (_ for _ in ()).throw(
        AssertionError('unsolved target must never move'))

    target = np.eye(4)
    target[0, 3] = 100.0
    assert robot.movj_solved_pose_status(target) == 'failed'


def test_icp_smooth_path_reaches_unchanged_target_after_direct_ik_fails():
    robot = object.__new__(EstunRobot)
    robot.dragging = False
    robot._arrival_position_mm = 1.5
    robot._arrival_rotation_deg = 1.0
    robot._logger = type('Logger', (), {
        'info': lambda self, message: None,
        'error': lambda self, message: None,
    })()
    robot.get_mode = lambda: robot.MODE_ENABLED
    initial_joints = np.ones(6)
    waypoint_joints = np.ones(6) * 2.0
    target_joints = np.ones(6) * 3.0
    robot.get_joints = lambda: initial_joints.copy()
    waypoint = [30.0, 20.0, 0.0, 0.0, 0.0, 0.0]
    final_target = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot._smooth_icp_candidate_paths = lambda start, target: iter([
        [waypoint, final_target],
    ])
    solves = []

    def solve(candidate, reference):
        pose = list(candidate)
        reference = np.asarray(reference)
        solves.append((pose, reference.copy()))
        if pose == final_target and np.allclose(reference, initial_joints):
            return None
        if pose == waypoint and np.allclose(reference, initial_joints):
            return waypoint_joints.copy()
        if pose == final_target and np.allclose(reference, waypoint_joints):
            return target_joints.copy()
        return None

    robot.solve_pose = solve
    moved = []
    robot.movj = lambda joints: moved.append(np.asarray(joints).copy()) or True
    robot.get_tool = lambda warn=False: final_target.copy()
    guarded = []
    target = np.eye(4)
    target[0, 3] = 100.0

    status = robot.movj_solved_path_status(
        target, start_pose=[0.0] * 6,
        motion_guard=lambda value: guarded.append(value.copy()))

    assert status == 'arrived'
    assert len(solves) == 3
    assert len(moved) == 2
    np.testing.assert_allclose(moved[0], waypoint_joints)
    np.testing.assert_allclose(moved[1], target_joints)
    assert len(guarded) == 2
    np.testing.assert_allclose(guarded[-1][:3, 3], [100.0, 0.0, 0.0])
    assert robot.last_icp_waypoint_count == 1
    assert robot.last_icp_motion_mode == 'controller_ik_smooth_path'


def test_icp_incomplete_smooth_paths_do_not_move():
    robot = object.__new__(EstunRobot)
    robot.dragging = False
    robot._arrival_position_mm = 1.5
    robot._arrival_rotation_deg = 1.0
    robot._logger = type('Logger', (), {
        'info': lambda self, message: None,
        'error': lambda self, message: None,
    })()
    robot.get_mode = lambda: robot.MODE_ENABLED
    robot.get_joints = lambda: [1.0] * 6
    robot._smooth_icp_candidate_paths = lambda start, target: iter([[
        [20.0, 10.0, 0.0, 0.0, 0.0, 0.0],
        [100.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]])
    robot.solve_pose = lambda candidate, reference: (
        np.ones(6) * 2.0 if float(candidate[0]) == 20.0 else None)
    robot.movj = lambda joints: (_ for _ in ()).throw(
        AssertionError('an incomplete IK path must never move'))
    robot.get_tool = lambda warn=False: [0.0] * 6
    target = np.eye(4)
    target[0, 3] = 100.0

    assert robot.movj_solved_path_status(
        target, start_pose=[0.0] * 6) == 'failed'


def test_icp_candidate_path_is_curved_and_ends_at_exact_target():
    robot = object.__new__(EstunRobot)
    robot._icp_waypoint_step_mm = 30.0
    robot._icp_waypoint_max_steps = 4
    start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    path = next(robot._smooth_icp_candidate_paths(start, target))

    assert 2 <= len(path) <= 4
    assert any(abs(point[1]) > 1e-6 or abs(point[2]) > 1e-6
               for point in path[:-1])
    np.testing.assert_allclose(path[-1], target, atol=0.0)


def test_apriltag_move_pose_splits_path_and_keeps_exact_final_target():
    from estun_codroid_bridge.srv import Move

    robot = object.__new__(EstunRobot)
    robot._apriltag_cartesian_step_mm = 30.0
    robot._apriltag_rotation_step_deg = 5.0
    robot._logger = type('Logger', (), {
        'info': lambda self, message: None,
        'error': lambda self, message: None,
    })()
    robot.get_tool = lambda warn=False: [0.0] * 6
    calls = []
    robot._cartesian_pose_status = lambda target, move_type, label, timeout: (
        calls.append((np.asarray(target), move_type, label, timeout)) or
        'arrived')
    target = np.eye(4)
    target[:3, 3] = [100.0, 0.0, 0.0]

    assert robot.move_pose(target, timeout=12.0)
    assert len(calls) == 4
    assert all(call[1] == Move.Request.MOVE_CARTESIAN_J for call in calls)
    np.testing.assert_allclose(calls[-1][0], [100, 0, 0, 0, 0, 0])
    assert calls[-1][2] == 'ESTUN AprilTag CPos step 4/4'
    assert calls[-1][3] == 12.0


def test_apriltag_pick_dispatches_only_final_cached_target():
    node = object.__new__(AprilTagPickNode)
    target = np.eye(4)
    target[:3, 3] = [282.0, -199.0, 908.0]
    node.last_localization = {
        'index': 1,
        'base_flange_target': target,
    }
    node.execute_enabled = True
    node.gripper_enabled = False
    moved = []
    node.robot = type('Robot', (), {
        'move_pose': lambda self, pose: moved.append(pose.copy()) or True,
    })()
    node.get_logger = lambda: type('Logger', (), {
        'info': lambda self, message: None,
    })()
    stages = []

    node.pick(lambda stage, pose: stages.append((stage, pose.copy())))

    assert len(moved) == 1
    np.testing.assert_allclose(moved[0], target)
    assert [stage for stage, _ in stages] == ['before_contact']


def test_apriltag_localization_keeps_large_spread_as_diagnostic_only():
    node = object.__new__(AprilTagPickNode)
    node.robot = type('Robot', (), {
        'use_base_frame': lambda self: True,
        'get_tool': lambda self, warn=False: [0.0] * 6,
    })()
    node.tag_frame = 'tag36h11:0'
    first = np.eye(4)
    second = np.eye(4)
    second[0, 3] = 157.8
    node.tag_histories = {
        node.tag_frame: [(9.8, first), (9.9, second)],
    }
    node.lock = threading.Lock()
    node.get_clock = lambda: type('Clock', (), {
        'now': lambda self: type('Time', (), {
            'nanoseconds': int(10.0e9),
        })(),
    })()
    node.max_detection_age = 1.0
    node.sample_window = 2.0
    node.samples = 10
    node.flange_from_camera = np.eye(4)
    node.tag_to_tcp = np.eye(4)
    node.tcp_offset_flange = np.zeros(3)
    node.motion_limits_enabled = False
    node.localization_index = 0
    node.last_localization = None
    node.localization_cache = []
    node.save_detection_debug = lambda *args, **kwargs: None

    target = node.locate(wait_for_new_detection=False)

    np.testing.assert_allclose(target[:3, 3], [78.9, 0.0, 0.0])
    assert node.last_localization['spread_mm'] == 78.9


def test_estun_apriltag_solves_approach_and_contact_before_motion():
    robot = object.__new__(EstunRobot)
    robot._apriltag_approach_mm = 50.0
    robot._apriltag_search_max_tilt_deg = 25.0
    robot._logger = type('Logger', (), {
        'info': lambda self, message: None,
        'error': lambda self, message: None,
    })()
    nominal = np.eye(4)
    robot._apriltag_candidate_rotations = lambda *args: [np.eye(3)]
    robot.get_joints = lambda: np.zeros(6)
    events = []
    solutions = [np.ones(6), np.ones(6) * 2.0]

    def solve(target, reference):
        events.append(('solve', np.asarray(reference).copy()))
        return solutions[len(events) - 1]

    robot.solve_pose = solve
    robot.movj = lambda joints: events.append(
        ('move', np.asarray(joints).copy())) or True
    stages = []

    assert robot.move_apriltag_target(
        nominal,
        contact_tip=[100.0, 0.0, 0.0],
        base_tag=np.eye(4),
        tcp_offset_flange=[0.0, 0.0, 200.0],
        safety_check=lambda stage, pose: stages.append(stage))

    assert [event[0] for event in events] == [
        'solve', 'solve', 'move', 'move']
    np.testing.assert_allclose(events[1][1], np.ones(6))
    assert stages == ['before_precontact', 'before_contact']


def test_movl_pose_status_sends_cartesian_l_request():
    from estun_codroid_bridge.srv import Move

    robot = object.__new__(EstunRobot)
    robot.dragging = False
    robot._arrival_position_mm = 1.5
    robot._arrival_rotation_deg = 1.0
    robot._motion_timeout_sec = 30.0
    robot._logger = type('Logger', (), {
        'info': lambda self, message: None,
        'error': lambda self, message: None,
    })()
    robot.get_mode = lambda: robot.MODE_ENABLED
    robot.get_tool = lambda warn=False: [0.0] * 6
    calls = []
    robot._send_move = lambda move_type, target, timeout: (
        calls.append((move_type, list(target), timeout)) or True)
    robot.wait_pose_arrival = lambda *args, **kwargs: (True, 0.0, 0.0)

    assert robot.movl_pose_status([10.0, 0.0, 0.0, 0.0, 0.0, 0.0]) == 'arrived'
    assert calls == [(
        Move.Request.MOVE_CARTESIAN_L,
        [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        30.0,
    )]


def test_reset_robot_moves_to_legacy_global_joint_position():
    robot = object.__new__(EstunRobot)
    robot._reset_joints = [86.0, 23.0, -112.0, -176.0, -85.0, 0.0]
    robot._logger = type('Logger', (), {
        'info': lambda self, message: None,
        'error': lambda self, message: None,
    })()
    robot._robot_error = lambda: False
    robot.enable_robot = lambda: True
    moved = []
    robot.movj = lambda joints: moved.append(list(joints)) or True

    assert robot.reset_robot()
    assert moved == [[86.0, 23.0, -112.0, -176.0, -85.0, 0.0]]


def test_pivot_solver_recovers_flange_tip_offset():
    tcp = np.array([12.0, -5.0, 85.0])
    fixed = np.array([400.0, 100.0, 250.0])
    samples = []
    rotations = [
        [0, 0, 0], [20, 0, 0], [0, 25, 0], [0, 0, 30],
        [-20, 10, 15], [15, -20, 25],
    ]
    from scipy.spatial.transform import Rotation
    for index, rpy in enumerate(rotations):
        rotation = Rotation.from_euler(
            'xyz', rpy, degrees=True).as_matrix()
        translation = fixed - rotation @ tcp
        samples.append(PoseSample(
            float(index), *translation.tolist(), *rpy))
    estimated_tcp, estimated_fixed, residuals = solve_pivot(samples)
    np.testing.assert_allclose(estimated_tcp, tcp, atol=1e-8)
    np.testing.assert_allclose(estimated_fixed, fixed, atol=1e-8)
    assert float(np.max(residuals)) < 1e-8


def test_handeye_solver_recovers_known_transform():
    from scipy.spatial.transform import Rotation
    handeye = np.eye(4)
    handeye[:3, :3] = Rotation.from_euler(
        'xyz', [8, -12, 20], degrees=True).as_matrix()
    handeye[:3, 3] = [45, -30, 110]
    board_base = np.eye(4)
    board_base[:3, :3] = Rotation.from_euler(
        'xyz', [0, 0, 15], degrees=True).as_matrix()
    board_base[:3, 3] = [600, 100, 200]
    robot = []
    camera = []
    poses = [
        [300, 0, 450, 0, 0, 0],
        [320, 30, 430, 10, 0, 0],
        [280, -40, 470, 0, 15, 0],
        [340, -20, 440, 0, 0, 20],
        [260, 50, 460, -15, 10, 5],
        [310, -60, 420, 20, -10, 15],
    ]
    for pose in poses:
        robot_transform = pose_to_matrix(pose)
        robot.append(robot_transform)
        camera.append(
            np.linalg.inv(handeye) @ np.linalg.inv(robot_transform) @
            board_base)
    estimated, pairs = solve_handeye(robot, camera)
    assert pairs >= 8
    np.testing.assert_allclose(estimated, handeye, atol=1e-8)


def test_gripper_configuration_round_trip(tmp_path):
    path = tmp_path / 'active.json'
    save_config(str(path), 23, False, '/estun')
    assert load_config(str(path)) == (23, False)


def test_tcp_active_file_can_point_to_immutable_result(tmp_path):
    result = tmp_path / 'tcp_result.json'
    result.write_text(
        '{"result":{"tcp_offset_flange_mm":[1,2,3]}}',
        encoding='utf-8')
    active = tmp_path / 'active.json'
    active.write_text('"tcp_result.json"', encoding='utf-8')
    np.testing.assert_allclose(load_tcp_offset(active), [1, 2, 3])


def test_one_key_calibration_commands_are_explicit():
    assert driver_command('ER20-1780-A6', '192.168.1.10') == [
        'ros2', 'launch', 'estun_codroid_bridge',
        'codroid_bridge.launch.py',
        'robot_ip:=192.168.1.10',
        'robot_port:=9000',
    ]
    assert camera_command() == [
        'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
        'pointcloud.enable:=true',
    ]
