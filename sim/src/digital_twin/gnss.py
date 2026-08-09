"""Receiver-neutral GNSS solution and PPS simulation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from .config import GnssConfig, LaunchConfig, SimulationConfig
from .frames import rotation_matrix
from .geodesy import ecef_from_enu_rotation, geodetic_to_ecef
from .transport import GnssPps, GnssSolution
from .truth import interpolate_truth
from .types import MeasurementEvent, SensorId, StatusFlag, TruthSample

GPS_WEEK_NS = 604_800_000_000_000


class GnssReceiverAdapter(Protocol):
    """Stable boundary implemented by a future receiver-specific wire codec."""

    def decode_solution(self, payload: bytes) -> GnssSolution: ...

    def decode_pps(self, payload: bytes) -> GnssPps: ...


class GenericGnssAdapter:
    def decode_solution(self, payload: bytes) -> GnssSolution:
        return GnssSolution.from_payload_bytes(payload)

    def decode_pps(self, payload: bytes) -> GnssPps:
        return GnssPps.from_payload_bytes(payload)


@dataclass(frozen=True, slots=True)
class GnssFaultSchedule:
    solution_loss: frozenset[int] = field(default_factory=frozenset)
    invalid_fix: frozenset[int] = field(default_factory=frozenset)
    pps_loss: frozenset[int] = field(default_factory=frozenset)
    additional_latency_s: dict[int, float] = field(default_factory=dict)


class GenericGnssModel:
    def __init__(self, config: GnssConfig, launch: LaunchConfig, simulation: SimulationConfig, seed: int):
        self.config = config
        self.launch = launch
        self.simulation = simulation
        self.solution_rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.GNSS_SOLUTION), 1]))
        self.latency_rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.GNSS_SOLUTION), 2]))
        self.pps_rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.GNSS_PPS)]))
        self.outage_rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.GNSS_SOLUTION), 3]))
        self.origin_ecef = geodetic_to_ecef(launch.latitude_deg, launch.longitude_deg, launch.elevation_msl_m)
        self.ecef_from_enu = ecef_from_enu_rotation(launch.latitude_deg, launch.longitude_deg)

    def _gps_epoch(self, ticks: int) -> tuple[int, int]:
        total_ns = int(round(self.config.start_tow_s * 1e9 + ticks / self.simulation.clock_hz * 1e9))
        week = self.config.gps_week + total_ns // GPS_WEEK_NS
        return week, total_ns % GPS_WEEK_NS

    def generate(self, truth: list[TruthSample], faults: GnssFaultSchedule | None = None) -> list[MeasurementEvent]:
        if not self.config.enabled or not truth:
            return []
        events: list[MeasurementEvent] = []
        faults = faults or GnssFaultSchedule()
        interval = self.simulation.clock_hz // self.config.output_rate_hz
        covariance_enu = np.diag(np.r_[self.config.position_sigma_enu_m**2, self.config.velocity_sigma_enu_mps**2])
        transform = np.zeros((6, 6))
        transform[:3, :3] = self.ecef_from_enu
        transform[3:, 3:] = self.ecef_from_enu
        covariance_ecef = transform @ covariance_enu @ transform.T
        position_bias = np.zeros(3)
        velocity_bias = np.zeros(3)
        outage = False
        for sequence, ticks in enumerate(range(truth[0].ticks, truth[-1].ticks + 1, interval)):
            sample = interpolate_truth(truth, ticks)
            dt = 1.0 / self.config.output_rate_hz
            position_bias += self.solution_rng.normal(0.0, self.config.position_bias_rw_m_sqrt_s * np.sqrt(dt), 3)
            velocity_bias += self.solution_rng.normal(0.0, self.config.velocity_bias_rw_mps_sqrt_s * np.sqrt(dt), 3)
            if outage:
                outage = self.outage_rng.random() >= self.config.outage_recovery_probability
            else:
                outage = self.outage_rng.random() < self.config.outage_entry_probability
                if self.config.high_acceleration_outage_g > 0.0:
                    outage |= np.linalg.norm(sample.acceleration_enu_mps2) >= self.config.high_acceleration_outage_g * self.simulation.gravity_mps2
            position_noise = self.solution_rng.normal(0.0, self.config.position_sigma_enu_m)
            velocity_noise = self.solution_rng.normal(0.0, self.config.velocity_sigma_enu_mps)
            lever_nav = rotation_matrix(sample.q_body_to_nav) @ self.config.antenna_lever_arm_body_m
            lever_velocity_nav = rotation_matrix(sample.q_body_to_nav) @ np.cross(sample.angular_rate_body_rps, self.config.antenna_lever_arm_body_m)
            position = self.origin_ecef + self.ecef_from_enu @ (sample.position_enu_m + lever_nav + position_bias + position_noise)
            velocity = self.ecef_from_enu @ (sample.velocity_enu_mps + lever_velocity_nav + velocity_bias + velocity_noise)
            week, tow = self._gps_epoch(ticks)
            valid = sequence not in faults.invalid_fix
            solution = GnssSolution(week, tow, tuple(position), tuple(velocity), tuple(covariance_ecef.ravel()), 3 if valid else 0, 12 if valid else 0)
            latency = max(0.0, self.latency_rng.normal(self.config.latency_mean_s, self.config.latency_jitter_s) + faults.additional_latency_s.get(sequence, 0.0))
            arrival = ticks + int(round(latency * self.simulation.clock_hz))
            flags = StatusFlag.VALID if valid else StatusFlag.FIX_INVALID
            if not outage and sequence not in faults.solution_loss:
                events.append(MeasurementEvent(1, SensorId.GNSS_SOLUTION, sequence, ticks, arrival, flags, solution.payload_bytes()))

        pps_interval = self.simulation.clock_hz // self.config.pps_rate_hz
        for sequence, nominal_ticks in enumerate(range(truth[0].ticks, truth[-1].ticks + 1, pps_interval)):
            time_s = nominal_ticks / self.simulation.clock_hz
            clock_error_ns = self.config.clock_offset_ns + self.config.clock_drift_ppm * 1e3 * time_s
            edge_ticks = nominal_ticks + int(round((clock_error_ns + self.pps_rng.normal(0.0, self.config.pps_jitter_ns)) * self.simulation.clock_hz / 1e9))
            edge_ticks = max(0, edge_ticks)
            week, tow = self._gps_epoch(nominal_ticks)
            pps = GnssPps(week, tow, self.config.pps_jitter_ns, True)
            if sequence not in faults.pps_loss:
                events.append(MeasurementEvent(1, SensorId.GNSS_PPS, sequence, edge_ticks, edge_ticks, StatusFlag.VALID, pps.payload_bytes()))
        return events
