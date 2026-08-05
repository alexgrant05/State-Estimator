import numpy as np

from digital_twin.frames import (
    exponential_quaternion,
    normalize_quaternion,
    quaternion_from_two_vectors,
    quaternion_multiply,
    rocketpy_initial_quaternion,
    rotation_matrix,
)


def test_rotation_is_orthonormal_and_preserves_norm():
    quaternion = normalize_quaternion([0.7, -0.2, 0.3, 0.6])
    rotation = rotation_matrix(quaternion)
    vector = np.array([3.0, -4.0, 5.0])
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(rotation), 1.0, atol=1e-12)
    assert np.isclose(np.linalg.norm(rotation @ vector), np.linalg.norm(vector), atol=1e-12)


def test_exponential_quaternion_composes_constant_rate():
    first = exponential_quaternion([0.1, -0.2, 0.3])
    half = exponential_quaternion([0.05, -0.1, 0.15])
    assert np.allclose(first, quaternion_multiply(half, half), atol=1e-12)


def test_vector_alignment():
    source = np.array([0.2, -0.4, 0.9])
    target = np.array([-0.3, 0.8, 0.2])
    quaternion = quaternion_from_two_vectors(source, target)
    result = rotation_matrix(quaternion) @ (source / np.linalg.norm(source))
    assert np.allclose(result, target / np.linalg.norm(target), atol=1e-12)


def test_rocketpy_rail_quaternion_has_expected_north_up_axis():
    quaternion = rocketpy_initial_quaternion(84.0, 0.0)
    rotation = rotation_matrix(quaternion)
    # RocketPy's body z axis points tail-to-nose along the rail.
    expected = np.array([0.0, np.cos(np.radians(84.0)), np.sin(np.radians(84.0))])
    assert np.allclose(rotation[:, 2], expected, atol=1e-12)

