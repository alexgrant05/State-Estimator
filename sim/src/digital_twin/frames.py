"""Frame and quaternion operations.

Navigation is east-north-up (ENU). Quaternions are scalar-first and rotate
body-frame vectors into the navigation frame.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


def normalize_quaternion(q: ArrayLike) -> NDArray[np.float64]:
    value = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(value)
    if norm == 0.0:
        raise ValueError("zero quaternion")
    return value / norm


def quaternion_multiply(a: ArrayLike, b: ArrayLike) -> NDArray[np.float64]:
    aw, ax, ay, az = np.asarray(a, dtype=np.float64)
    bw, bx, by, bz = np.asarray(b, dtype=np.float64)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def quaternion_conjugate(q: ArrayLike) -> NDArray[np.float64]:
    w, x, y, z = np.asarray(q, dtype=np.float64)
    return np.array([w, -x, -y, -z], dtype=np.float64)


def rotation_matrix(q: ArrayLike) -> NDArray[np.float64]:
    w, x, y, z = normalize_quaternion(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def exponential_quaternion(rotation_vector: ArrayLike) -> NDArray[np.float64]:
    phi = np.asarray(rotation_vector, dtype=np.float64)
    angle = np.linalg.norm(phi)
    if angle < 1e-12:
        return normalize_quaternion(np.array([1.0, *(0.5 * phi)], dtype=np.float64))
    axis = phi / angle
    half = 0.5 * angle
    return np.array([np.cos(half), *(np.sin(half) * axis)], dtype=np.float64)


def skew(v: ArrayLike) -> NDArray[np.float64]:
    x, y, z = np.asarray(v, dtype=np.float64)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)


def quaternion_from_two_vectors(source: ArrayLike, target: ArrayLike) -> NDArray[np.float64]:
    """Return the minimum rotation that maps ``source`` onto ``target``."""

    a = np.asarray(source, dtype=np.float64)
    b = np.asarray(target, dtype=np.float64)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    dot = float(np.clip(a @ b, -1.0, 1.0))
    if dot > 1.0 - 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    if dot < -1.0 + 1e-12:
        axis = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-9:
            axis = np.cross(a, np.array([0.0, 1.0, 0.0]))
        axis /= np.linalg.norm(axis)
        return np.array([0.0, *axis], dtype=np.float64)
    cross = np.cross(a, b)
    return normalize_quaternion(np.array([1.0 + dot, *cross], dtype=np.float64))


def rocketpy_initial_quaternion(inclination_deg: float, heading_deg: float) -> NDArray[np.float64]:
    """Reproduce RocketPy's zero-spin 3-1-3 rail quaternion."""

    phi = 0.0
    theta = np.radians(inclination_deg - 90.0)
    psi = np.radians(-heading_deg)
    cphi, sphi = np.cos(phi / 2), np.sin(phi / 2)
    ctheta, stheta = np.cos(theta / 2), np.sin(theta / 2)
    cpsi, spsi = np.cos(psi / 2), np.sin(psi / 2)
    return normalize_quaternion(
        np.array(
            [
                cphi * ctheta * cpsi - sphi * ctheta * spsi,
                cphi * cpsi * stheta + sphi * stheta * spsi,
                cphi * stheta * spsi - sphi * cpsi * stheta,
                cphi * ctheta * spsi + ctheta * cpsi * sphi,
            ]
        )
    )


def attitude_error_deg(estimate: ArrayLike, truth: ArrayLike) -> float:
    delta = quaternion_multiply(quaternion_conjugate(estimate), truth)
    return float(np.degrees(2.0 * np.arccos(np.clip(abs(normalize_quaternion(delta)[0]), -1.0, 1.0))))
