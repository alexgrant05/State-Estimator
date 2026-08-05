"""Typed contracts shared by truth, sensors, transport, and estimation."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
from typing import Mapping

import numpy as np
from numpy.typing import NDArray

Vector3 = NDArray[np.float64]
Quaternion = NDArray[np.float64]
Matrix15 = NDArray[np.float64]


class SensorId(IntEnum):
    ADIS16470 = 1


class StatusFlag(IntFlag):
    NONE = 0
    VALID = 1 << 0
    SATURATED = 1 << 1
    DIAGNOSTIC_ERROR = 1 << 2
    CHECKSUM_ERROR = 1 << 3
    SEQUENCE_DISCONTINUITY = 1 << 4
    PACKET_LOSS = 1 << 5


@dataclass(frozen=True, slots=True)
class TruthSample:
    """One truth state in launch-centered ENU coordinates.

    ``q_body_to_nav`` is scalar-first and maps a body-frame vector into ENU.
    RocketPy's MSL z coordinate is retained separately as ``altitude_msl_m``.
    """

    ticks: int
    position_enu_m: Vector3
    velocity_enu_mps: Vector3
    acceleration_enu_mps2: Vector3
    q_body_to_nav: Quaternion
    angular_rate_body_rps: Vector3
    altitude_msl_m: float


@dataclass(frozen=True, slots=True)
class MeasurementEvent:
    format_version: int
    sensor_id: SensorId
    sequence_number: int
    measurement_ticks: int
    arrival_ticks: int
    status_flags: StatusFlag
    payload: bytes

    def __post_init__(self) -> None:
        if self.format_version != 1:
            raise ValueError("only measurement event format_version=1 is supported")
        if self.measurement_ticks < 0 or self.arrival_ticks < self.measurement_ticks:
            raise ValueError("event timestamps are invalid")
        if not 0 <= self.sequence_number <= 0xFFFFFFFF:
            raise ValueError("sequence_number must fit uint32")


@dataclass(slots=True)
class StateEstimate:
    state_ticks: int
    publication_ticks: int
    position_enu_m: Vector3
    velocity_enu_mps: Vector3
    q_body_to_nav: Quaternion
    accel_bias_body_mps2: Vector3
    gyro_bias_body_rps: Vector3
    covariance: Matrix15
    valid: bool
    health: Mapping[str, int] = field(default_factory=dict)

