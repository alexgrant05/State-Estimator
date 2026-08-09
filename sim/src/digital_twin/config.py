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
class AdxlConfig:
    enabled: bool
    output_rate_hz: int
    spi_clock_hz: int
    noise_density_mg_sqrt_hz: float
    bias_mps2: NDArray[np.float64]
    bias_rw_mps2_sqrt_s: float
    scale_error: NDArray[np.float64]
    misalignment_rad: NDArray[np.float64]
    sensor_to_body: NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class BmpConfig:
    enabled: bool
    output_rate_hz: int
    spi_clock_hz: int
    pressure_oversampling: int
    temperature_oversampling: int
    iir_coefficient: int
    pressure_noise_pa: float
    temperature_noise_c: float
    pressure_bias_pa: float
    pressure_bias_rw_pa_sqrt_s: float
    transonic_error_peak_m: float
    transonic_mach_center: float
    transonic_mach_sigma: float


@dataclass(frozen=True, slots=True)
class GnssConfig:
    enabled: bool
    output_rate_hz: int
    pps_rate_hz: int
    gps_week: int
    start_tow_s: float
    position_sigma_enu_m: NDArray[np.float64]
    velocity_sigma_enu_mps: NDArray[np.float64]
    latency_mean_s: float
    latency_jitter_s: float
    pps_jitter_ns: float
    clock_offset_ns: float
    clock_drift_ppm: float
    antenna_lever_arm_body_m: NDArray[np.float64]
    position_bias_rw_m_sqrt_s: float
    velocity_bias_rw_mps_sqrt_s: float
    outage_entry_probability: float
    outage_recovery_probability: float
    high_acceleration_outage_g: float


@dataclass(frozen=True, slots=True)
class IntegrationConfig:
    history_duration_s: float
    high_g_enter_fraction: float
    high_g_exit_fraction: float
    high_g_exit_hold_s: float
    high_g_max_age_samples: int
    overlap_nis_gate: float
    gnss_nis_gate: float
    baro_nis_gate: float
    baro_transonic_min_mach: float
    baro_transonic_max_mach: float


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
    adxl375: AdxlConfig
    bmp581: BmpConfig
    gnss: GnssConfig
    integration: IntegrationConfig
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
    raw_adxl = data.get("adxl375", {}).copy()
    raw_adxl.setdefault("enabled", False)
    raw_adxl.setdefault("output_rate_hz", 800)
    raw_adxl.setdefault("spi_clock_hz", 5_000_000)
    raw_adxl.setdefault("noise_density_mg_sqrt_hz", 5.0)
    raw_adxl.setdefault("bias_mps2", [0.0, 0.0, 0.0])
    raw_adxl.setdefault("bias_rw_mps2_sqrt_s", 0.0)
    raw_adxl.setdefault("scale_error", [0.0, 0.0, 0.0])
    raw_adxl.setdefault("misalignment_rad", [0.0, 0.0, 0.0])
    raw_adxl.setdefault("sensor_to_body", np.eye(3).tolist())
    raw_adxl["bias_mps2"] = _vector(raw_adxl["bias_mps2"], 3, "adxl375.bias_mps2")
    raw_adxl["scale_error"] = _vector(raw_adxl["scale_error"], 3, "adxl375.scale_error")
    raw_adxl["misalignment_rad"] = _vector(raw_adxl["misalignment_rad"], 3, "adxl375.misalignment_rad")
    raw_adxl["sensor_to_body"] = np.asarray(raw_adxl["sensor_to_body"], dtype=np.float64)
    adxl = AdxlConfig(**raw_adxl)
    bmp = BmpConfig(**{
        "enabled": False,
        "output_rate_hz": 50,
        "spi_clock_hz": 5_000_000,
        "pressure_oversampling": 16,
        "temperature_oversampling": 1,
        "iir_coefficient": 0,
        "pressure_noise_pa": 0.21,
        "temperature_noise_c": 0.05,
        "pressure_bias_pa": 0.0,
        "pressure_bias_rw_pa_sqrt_s": 0.0,
        "transonic_error_peak_m": -50.0,
        "transonic_mach_center": 1.0,
        "transonic_mach_sigma": 0.12,
        **data.get("bmp581", {}),
    })
    raw_gnss = {
        "enabled": False,
        "output_rate_hz": 10,
        "pps_rate_hz": 1,
        "gps_week": 0,
        "start_tow_s": 0.0,
        "position_sigma_enu_m": [2.0, 2.0, 3.0],
        "velocity_sigma_enu_mps": [0.1, 0.1, 0.15],
        "latency_mean_s": 0.12,
        "latency_jitter_s": 0.02,
        "pps_jitter_ns": 20.0,
        "clock_offset_ns": 0.0,
        "clock_drift_ppm": 0.0,
        "antenna_lever_arm_body_m": [0.0, 0.0, 0.0],
        "position_bias_rw_m_sqrt_s": 0.0,
        "velocity_bias_rw_mps_sqrt_s": 0.0,
        "outage_entry_probability": 0.0,
        "outage_recovery_probability": 1.0,
        "high_acceleration_outage_g": 0.0,
        **data.get("gnss", {}),
    }
    raw_gnss["position_sigma_enu_m"] = _vector(raw_gnss["position_sigma_enu_m"], 3, "gnss.position_sigma_enu_m")
    raw_gnss["velocity_sigma_enu_mps"] = _vector(raw_gnss["velocity_sigma_enu_mps"], 3, "gnss.velocity_sigma_enu_mps")
    raw_gnss["antenna_lever_arm_body_m"] = _vector(raw_gnss["antenna_lever_arm_body_m"], 3, "gnss.antenna_lever_arm_body_m")
    gnss = GnssConfig(**raw_gnss)
    integration = IntegrationConfig(**{
        "history_duration_s": 2.0,
        "high_g_enter_fraction": 0.85,
        "high_g_exit_fraction": 0.75,
        "high_g_exit_hold_s": 0.025,
        "high_g_max_age_samples": 2,
        "overlap_nis_gate": 16.27,
        "gnss_nis_gate": 22.46,
        "baro_nis_gate": 6.63,
        "baro_transonic_min_mach": 0.8,
        "baro_transonic_max_mach": 1.2,
        **data.get("integration", {}),
    })
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
    for name, rate in (("ADXL375", adxl.output_rate_hz), ("BMP581", bmp.output_rate_hz), ("GNSS", gnss.output_rate_hz), ("PPS", gnss.pps_rate_hz)):
        if rate <= 0 or simulation.clock_hz % rate:
            raise ValueError(f"{name} rate must be a positive integer divisor of clock_hz")
    if adxl.output_rate_hz > 800:
        raise ValueError("the reference ADXL375 codec supports rates through 800 Hz")
    if bmp.pressure_oversampling not in (1, 2, 4, 8, 16, 32, 64, 128):
        raise ValueError("BMP581 pressure oversampling is invalid")
    if bmp.temperature_oversampling not in (1, 2, 4, 8):
        raise ValueError("BMP581 temperature oversampling is invalid")
    if adxl.sensor_to_body.shape != (3, 3) or not np.allclose(adxl.sensor_to_body.T @ adxl.sensor_to_body, np.eye(3), atol=1e-10):
        raise ValueError("adxl375.sensor_to_body must be an orthonormal 3x3 matrix")
    if integration.history_duration_s <= gnss.latency_mean_s + 3.0 * gnss.latency_jitter_s:
        raise ValueError("history_duration_s must cover nominal GNSS latency and jitter")
    if not 0.0 <= gnss.outage_entry_probability <= 1.0 or not 0.0 <= gnss.outage_recovery_probability <= 1.0:
        raise ValueError("GNSS outage probabilities must be between zero and one")

    return TwinConfig(
        simulation=simulation,
        launch=launch,
        motor=data["motor"],
        rocket=data["rocket"],
        adis16470=adis,
        adxl375=adxl,
        bmp581=bmp,
        gnss=gnss,
        integration=integration,
        estimator=estimator,
        source_path=source_path,
    )
