import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from apriltag_pick import rotation_compat


def test_current_rotation_matrix_round_trip():
    expected = Rotation.from_euler(
        'xyz', [12.0, -8.0, 31.0], degrees=True)
    matrix = rotation_compat.rotation_as_matrix(expected)
    actual = rotation_compat.rotation_from_matrix(matrix)
    assert np.allclose(actual.as_quat(), expected.as_quat()) or np.allclose(
        actual.as_quat(), -expected.as_quat())


def test_scipy_13_matrix_method_fallback(monkeypatch):
    expected = np.arange(9, dtype=float).reshape(3, 3)

    class OldRotationInstance:
        def as_dcm(self):
            return expected

    class OldRotationType:
        @staticmethod
        def from_dcm(matrix):
            return ('from_dcm', matrix)

    monkeypatch.setattr(rotation_compat, 'Rotation', OldRotationType)
    assert np.array_equal(
        rotation_compat.rotation_as_matrix(OldRotationInstance()), expected)
    name, matrix = rotation_compat.rotation_from_matrix(expected)
    assert name == 'from_dcm'
    assert np.array_equal(matrix, expected)


def test_rotation_magnitude_does_not_require_magnitude_method():
    rotation = Rotation.from_euler('z', 35.0, degrees=True)
    assert np.isclose(
        np.degrees(rotation_compat.rotation_magnitude(rotation)), 35.0)


def test_rotation_stack_is_valid_slerp_input():
    first = Rotation.from_euler('z', 0.0, degrees=True)
    second = Rotation.from_euler('z', 90.0, degrees=True)
    stack = rotation_compat.rotation_stack([first, second])
    middle = Slerp([0.0, 1.0], stack)([0.5]).as_euler(
        'xyz', degrees=True)[0]
    assert np.isclose(middle[2], 45.0)


def test_rotation_between_vectors_handles_opposite_direction():
    source = np.asarray([0.0, 0.0, 1.0])
    target = np.asarray([0.0, 0.0, -1.0])
    rotation = rotation_compat.rotation_between_vectors(source, target)
    assert np.allclose(rotation.apply(source), target, atol=1e-12)


def test_rotation_mean_fallback(monkeypatch):
    rotations = Rotation.from_euler(
        'z', [-10.0, 0.0, 10.0], degrees=True)

    class WithoutMean:
        def as_quat(self):
            return rotations.as_quat()

    average = rotation_compat.rotation_mean(WithoutMean())
    assert np.isclose(
        rotation_compat.rotation_magnitude(average), 0.0, atol=1e-12)
