"""Pure calibration math shared by the ESTUN interactive tools."""

from dataclasses import asdict, dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class PoseSample:
    stamp_sec: float
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float

    @property
    def values(self):
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]

    def to_dict(self):
        return asdict(self)


def pose_to_matrix(pose):
    """Convert shared [xyz mm, XYZ Euler deg] into T_base_flange."""
    values = np.asarray(pose, dtype=float).reshape(-1)
    if values.size != 6 or not np.all(np.isfinite(values)):
        raise ValueError('pose must contain six finite values')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler(
        'xyz', values[3:6], degrees=True).as_matrix()
    transform[:3, 3] = values[:3]
    return transform


def matrix_to_pose(transform):
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError('transform must be a finite 4x4 matrix')
    rpy = Rotation.from_matrix(transform[:3, :3]).as_euler(
        'xyz', degrees=True)
    return [*transform[:3, 3].tolist(), *rpy.tolist()]


def inverse(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def pose_delta(first, second):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    translation = float(np.linalg.norm(second[:3] - first[:3]))
    first_rotation = Rotation.from_euler('xyz', first[3:6], degrees=True)
    second_rotation = Rotation.from_euler('xyz', second[3:6], degrees=True)
    angle = float(np.degrees(
        (second_rotation * first_rotation.inv()).magnitude()))
    return translation, angle


def solve_handeye(robot_transforms, camera_transforms):
    """Solve AX=XB and return T_flange_camera plus the usable pair count."""
    if len(robot_transforms) != len(camera_transforms):
        raise ValueError('robot and camera sample counts differ')
    if len(robot_transforms) < 5:
        raise ValueError('hand-eye calibration needs at least 5 samples')

    motions_a = []
    motions_b = []
    for first in range(len(robot_transforms)):
        for second in range(first + 1, len(robot_transforms)):
            robot_motion = (
                inverse(robot_transforms[second]) @ robot_transforms[first])
            camera_motion = (
                camera_transforms[second] @ inverse(camera_transforms[first]))
            robot_angle = Rotation.from_matrix(
                robot_motion[:3, :3]).magnitude()
            camera_angle = Rotation.from_matrix(
                camera_motion[:3, :3]).magnitude()
            robot_distance = np.linalg.norm(robot_motion[:3, 3])
            camera_distance = np.linalg.norm(camera_motion[:3, 3])
            if ((robot_angle < np.radians(0.5) and robot_distance < 3.0) or
                    (camera_angle < np.radians(0.5) and
                     camera_distance < 3.0)):
                continue
            motions_a.append(robot_motion)
            motions_b.append(camera_motion)
    if len(motions_a) < 8:
        raise ValueError(
            'insufficient independent hand-eye motions: %d/8' %
            len(motions_a))

    correlation = np.zeros((3, 3))
    for motion_a, motion_b in zip(motions_a, motions_b):
        vector_a = Rotation.from_matrix(
            motion_a[:3, :3]).as_rotvec()
        vector_b = Rotation.from_matrix(
            motion_b[:3, :3]).as_rotvec()
        if np.linalg.norm(vector_a) < 1e-12:
            continue
        if np.linalg.norm(vector_b) < 1e-12:
            continue
        correlation += np.outer(vector_a, vector_b)
    left, unused, right_transpose = np.linalg.svd(correlation)
    rotation = left @ np.diag([
        1.0, 1.0, np.linalg.det(left @ right_transpose),
    ]) @ right_transpose

    rows = []
    targets = []
    for motion_a, motion_b in zip(motions_a, motions_b):
        rows.append(motion_a[:3, :3] - np.eye(3))
        targets.append(
            rotation @ motion_b[:3, 3] - motion_a[:3, 3])
    translation = np.linalg.lstsq(
        np.vstack(rows), np.concatenate(targets), rcond=None)[0]
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result, len(motions_a)


def handeye_consistency(robot_transforms, camera_transforms, handeye):
    board_transforms = [
        robot @ handeye @ camera
        for robot, camera in zip(robot_transforms, camera_transforms)
    ]
    translations = np.asarray([
        transform[:3, 3] for transform in board_transforms])
    translation_mean = translations.mean(axis=0)
    rotation_mean = Rotation.from_matrix(np.asarray([
        transform[:3, :3] for transform in board_transforms])).mean()
    translation_errors = np.linalg.norm(
        translations - translation_mean, axis=1)
    rotation_errors = [
        np.degrees((Rotation.from_matrix(transform[:3, :3]) *
                    rotation_mean.inv()).magnitude())
        for transform in board_transforms
    ]
    return {
        'board_world_rms_mm': float(np.sqrt(
            np.mean(np.square(translation_errors)))),
        'board_world_max_mm': float(np.max(translation_errors)),
        'board_world_rotation_rms_deg': float(np.sqrt(
            np.mean(np.square(rotation_errors)))),
        'board_world_rotation_max_deg': float(np.max(rotation_errors)),
    }


def solve_pivot(samples):
    """Solve R_i * p_flange_tip + t_i = p_fixed."""
    if len(samples) < 4:
        raise ValueError('pivot calibration needs at least 4 samples')
    rows = []
    targets = []
    for sample in samples:
        transform = pose_to_matrix(sample.values)
        rows.append(np.hstack([transform[:3, :3], -np.eye(3)]))
        targets.append(-transform[:3, 3])
    solution = np.linalg.lstsq(
        np.vstack(rows), np.concatenate(targets), rcond=None)[0]
    tcp_offset = solution[:3]
    fixed_point = solution[3:]
    residuals = []
    for sample in samples:
        transform = pose_to_matrix(sample.values)
        point = transform[:3, :3] @ tcp_offset + transform[:3, 3]
        residuals.append(float(np.linalg.norm(point - fixed_point)))
    return tcp_offset, fixed_point, np.asarray(residuals)
