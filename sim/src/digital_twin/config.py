"""Configuration loading and validation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python 3.10
    import tomli as tomllib


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    clock_hz: int
    truth_rate_hz: int
    pad_duration_s: float
    gravity_mps2: float
    seed: int


@dataclass(frozen=True, slots=True)
class LaunchConfig:
    latitude_deg: float
    longitude_deg: float
    elevation_msl_m: float
    rail_length_m: float
    rail_inclination_deg: float
    rail_heading_deg: float


@dataclass(frozen=True, slots=True)
class AdisConfig:
    dec_rate: int
    spi_clock_hz: int
    temperature_c: float
    accel_noise_rms_mg: float
    gyro_noise_rms_dps: float
    accel_bias_mps2: NDArray[np.float64]
    gyro_bias_rps: NDArray[np.float64]
    accel_bias_rw_mps2_sqrt_s: float
    gyro_bias_rw_rps_sqrt_s: float
    accel_scale_error: NDArray[np.float64]
    gyro_scale_error: NDArray[np.float64]
    misalignment_rad: NDArray[np.float64]
    sensor_to_body: NDArray[np.float64]

    @property
    def output_rate_hz(self) -> float:
        return 2000.0 / (self.dec_rate + 1)


@dataclass(frozen=True, slots=True)
class EstimatorConfig:
    accel_bias_rw_mps2_sqrt_s: float
    gyro_bias_rw_rps_sqrt_s: float
    initial_position_sigma_m: float
    initial_velocity_sigma_mps: float
    initial_attitude_sigma_deg: float
    initial_accel_bias_sigma_mps2: float
    initial_gyro_bias_sigma_rps: float


@dataclass(frozen=True, slots=True)
class TwinConfig:
    simulation: SimulationConfig
    launch: LaunchConfig
    motor: dict[str, Any]
    rocket: dict[str, Any]
    adis16470: AdisConfig
    estimator: EstimatorConfig
    source_path: Path


def _vector(values: Any, length: int, name: str) -> NDArray[np.float64]:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (length,):
        raise ValueError(f"{name} must contain {length} values")
    return result


def load_config(path: str | Path) -> TwinConfig:
    source_path = Path(path).resolve()
    with source_path.open("rb") as stream:
        data = tomllib.load(stream)

    simulation = SimulationConfig(**data["simulation"])
    launch = LaunchConfig(**data["launch"])
    raw_adis = data["adis16470"].copy()
    raw_adis["accel_bias_mps2"] = _vector(raw_adis["accel_bias_mps2"], 3, "accel_bias_mps2")
    raw_adis["gyro_bias_rps"] = _vector(raw_adis["gyro_bias_rps"], 3, "gyro_bias_rps")
    raw_adis["accel_scale_error"] = _vector(raw_adis["accel_scale_error"], 3, "accel_scale_error")
    raw_adis["gyro_scale_error"] = _vector(raw_adis["gyro_scale_error"], 3, "gyro_scale_error")
    raw_adis["misalignment_rad"] = _vector(raw_adis["misalignment_rad"], 3, "misalignment_rad")
    raw_adis["sensor_to_body"] = np.asarray(raw_adis["sensor_to_body"], dtype=np.float64)
    adis = AdisConfig(**raw_adis)
    estimator = EstimatorConfig(**data["estimator"])

    if simulation.truth_rate_hz != 2000:
        raise ValueError("truth_rate_hz must be 2000 for the ADIS16470 internal clock model")
    if simulation.clock_hz % simulation.truth_rate_hz:
        raise ValueError("clock_hz must be an integer multiple of truth_rate_hz")
    if not 0 <= adis.dec_rate <= 1999:
        raise ValueError("ADIS16470 DEC_RATE must be between 0 and 1999")
    if adis.spi_clock_hz <= 0 or adis.spi_clock_hz > 1_000_000:
        raise ValueError("ADIS16470 burst SPI clock must be in (0, 1 MHz]")
    if adis.sensor_to_body.shape != (3, 3):
        raise ValueError("sensor_to_body must be a 3x3 matrix")
    if not np.allclose(adis.sensor_to_body.T @ adis.sensor_to_body, np.eye(3), atol=1e-10):
        raise ValueError("sensor_to_body must be orthonormal")
    if np.linalg.det(adis.sensor_to_body) < 0.0:
        raise ValueError("sensor_to_body must be a proper rotation")

    return TwinConfig(
        simulation=simulation,
        launch=launch,
        motor=data["motor"],
        rocket=data["rocket"],
        adis16470=adis,
        estimator=estimator,
        source_path=source_path,
    )

