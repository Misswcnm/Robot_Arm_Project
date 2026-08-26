"""SciPy Rotation compatibility helpers for Ubuntu 20.04/Foxy."""

import numpy as np
from scipy.spatial.transform import Rotation


def rotation_as_matrix(rotation):
    method = getattr(rotation, 'as_matrix', None)
    if method is not None:
        return method()
    return rotation.as_dcm()


def rotation_from_matrix(matrix):
    method = getattr(Rotation, 'from_matrix', None)
    if method is not None:
        return method(matrix)
    return Rotation.from_dcm(matrix)


def rotation_magnitude(rotation):
    return np.linalg.norm(rotation.as_rotvec(), axis=-1)
