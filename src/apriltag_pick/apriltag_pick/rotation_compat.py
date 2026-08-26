"""SciPy Rotation compatibility helpers for ROS 2 Foxy and newer systems.

Ubuntu 20.04/Foxy ships SciPy 1.3, where rotation matrices use the
``as_dcm``/``from_dcm`` names.  SciPy 1.4 renamed them to
``as_matrix``/``from_matrix``.  Keep that version boundary in one place so
the geometry code itself remains identical on both platforms.
"""

import numpy as np
from scipy.spatial.transform import Rotation


def rotation_as_matrix(rotation):
    """Return one or more rotation matrices on SciPy 1.3 and newer."""
    method = getattr(rotation, 'as_matrix', None)
    if method is not None:
        return method()
    return rotation.as_dcm()


def rotation_from_matrix(matrix):
    """Construct a Rotation from one or more matrices on all supported SciPy."""
    method = getattr(Rotation, 'from_matrix', None)
    if method is not None:
        return method(matrix)
    return Rotation.from_dcm(matrix)


def rotation_magnitude(rotation):
    """Return rotation angle(s) in radians without requiring magnitude()."""
    return np.linalg.norm(rotation.as_rotvec(), axis=-1)


def rotation_stack(rotations):
    """Build the Rotation stack required by Slerp without concatenate()."""
    quaternions = np.asarray(
        [rotation.as_quat() for rotation in rotations], dtype=float)
    return Rotation.from_quat(quaternions)


def rotation_mean(rotation):
    """Return the unweighted quaternion mean, including on SciPy 1.3."""
    method = getattr(rotation, 'mean', None)
    if method is not None:
        return method()
    quaternions = np.asarray(rotation.as_quat(), dtype=float)
    if quaternions.ndim == 1:
        return Rotation.from_quat(quaternions)
    accumulator = np.einsum('ni,nj->ij', quaternions, quaternions)
    unused, vectors = np.linalg.eigh(accumulator)
    return Rotation.from_quat(vectors[:, -1])


def rotation_between_vectors(source, target):
    """Return the shortest rotation that maps ``source`` onto ``target``.

    This replaces the newer ``Rotation.align_vectors`` API for the one-vector
    AprilTag case and explicitly handles parallel and anti-parallel vectors.
    """
    source = np.asarray(source, dtype=float).reshape(3)
    target = np.asarray(target, dtype=float).reshape(3)
    source_norm = float(np.linalg.norm(source))
    target_norm = float(np.linalg.norm(target))
    if source_norm <= 1e-12 or target_norm <= 1e-12:
        raise ValueError('rotation vectors must be non-zero')
    source /= source_norm
    target /= target_norm
    dot = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if dot >= 1.0 - 1e-12:
        return Rotation.from_quat([0.0, 0.0, 0.0, 1.0])
    if dot <= -1.0 + 1e-12:
        helper = np.asarray([1.0, 0.0, 0.0])
        if abs(float(np.dot(source, helper))) > 0.9:
            helper = np.asarray([0.0, 1.0, 0.0])
        axis = np.cross(source, helper)
        axis /= np.linalg.norm(axis)
        return Rotation.from_rotvec(np.pi * axis)
    quaternion = np.concatenate((np.cross(source, target), [1.0 + dot]))
    quaternion /= np.linalg.norm(quaternion)
    return Rotation.from_quat(quaternion)
