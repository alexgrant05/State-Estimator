"""Event-driven 15-state ESKF inertial propagation reference."""

from __future__ import annotations

from collections import Counter

import numpy as np

from .adis16470 import DecodedAdisMeasurement, decode_event
from .config import TwinConfig
from .frames import (
    exponential_quaternion,
    normalize_quaternion,
    quaternion_from_two_vectors,
    quaternion_multiply,
    rotation_matrix,
    skew,
)
from .transport import AdisBurst
from .types import MeasurementEvent, StateEstimate, StatusFlag


class InertialEskf:
    """ADIS-only nominal-state propagation with 15x15 error covariance."""

    def __init__(self, config: TwinConfig):
        self.config = config
        self.health: Counter[str] = Counter()
        self.pad_measurements: list[tuple[int, DecodedAdisMeasurement]] = []
        self.initialized = False
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.covariance = np.zeros((15, 15))
        self.last_state_ticks: int | None = None
        self.expected_sequence: int | None = None
        self.expected_counter: int | None = None
        self.liftoff_ticks = int(round(config.simulation.pad_duration_s * config.simulation.clock_hz))

    def _valid_measurement(self, event: MeasurementEvent) -> DecodedAdisMeasurement | None:
        if self.expected_sequence is not None and event.sequence_number != self.expected_sequence:
            gap = (event.sequence_number - self.expected_sequence) & 0xFFFFFFFF
            self.health["sequence_discontinuities"] += 1
            self.health["packets_lost"] += gap
        self.expected_sequence = (event.sequence_number + 1) & 0xFFFFFFFF

        try:
            burst = AdisBurst.from_payload_bytes(event.payload)
        except ValueError:
            self.health["malformed_packets"] += 1
            return None
        if not burst.valid_checksum():
            self.health["checksum_failures"] += 1
            return None
        if burst.diag_stat:
            self.health["diagnostic_failures"] += 1
            return None
        if self.expected_counter is not None and burst.data_counter != self.expected_counter:
            self.health["counter_discontinuities"] += 1
        self.expected_counter = (burst.data_counter + 1) & 0xFFFF
        if event.status_flags & StatusFlag.SATURATED:
            self.health["saturated_measurements"] += 1
        try:
            return decode_event(event, self.config.adis16470, self.config.simulation)
        except ValueError:
            self.health["decode_failures"] += 1
            return None

    def _initialize(self) -> None:
        if not self.pad_measurements:
            raise RuntimeError("no valid pad measurements available for initialization")
        accel_mean = np.mean([measurement.accel_body_mps2 for _, measurement in self.pad_measurements], axis=0)
        gyro_mean = np.mean([measurement.gyro_body_rps for _, measurement in self.pad_measurements], axis=0)

        from .frames import rocketpy_initial_quaternion

        rail_q = rocketpy_initial_quaternion(
            self.config.launch.rail_inclination_deg,
            self.config.launch.rail_heading_deg,
        )
        expected_up_body = rotation_matrix(rail_q).T @ np.array([0.0, 0.0, 1.0])
        measured_up_body = accel_mean / np.linalg.norm(accel_mean)
        tilt_correction = quaternion_from_two_vectors(measured_up_body, expected_up_body)
        self.quaternion = normalize_quaternion(quaternion_multiply(rail_q, tilt_correction))
        expected_specific_force = rotation_matrix(self.quaternion).T @ np.array(
            [0.0, 0.0, self.config.simulation.gravity_mps2]
        )
        self.accel_bias = accel_mean - expected_specific_force
        self.gyro_bias = gyro_mean

        est = self.config.estimator
        self.covariance[0:3, 0:3] = np.eye(3) * est.initial_position_sigma_m**2
        self.covariance[3:6, 3:6] = np.eye(3) * est.initial_velocity_sigma_mps**2
        self.covariance[6:9, 6:9] = np.eye(3) * np.radians(est.initial_attitude_sigma_deg) ** 2
        self.covariance[9:12, 9:12] = np.eye(3) * est.initial_accel_bias_sigma_mps2**2
        self.covariance[12:15, 12:15] = np.eye(3) * est.initial_gyro_bias_sigma_rps**2
        self.last_state_ticks = self.pad_measurements[-1][0]
        self.initialized = True
        self.health["initializations"] += 1

    def process(self, event: MeasurementEvent) -> StateEstimate | None:
        measurement = self._valid_measurement(event)
        if measurement is None:
            return None
        if event.measurement_ticks < self.liftoff_ticks:
            self.pad_measurements.append((event.measurement_ticks, measurement))
            self.health["pad_samples"] += 1
            return None
        if not self.initialized:
            self._initialize()

        assert self.last_state_ticks is not None
        dt = (event.measurement_ticks - self.last_state_ticks) / self.config.simulation.clock_hz
        if dt <= 0.0:
            self.health["nonmonotonic_measurements"] += 1
            return None

        accel = measurement.accel_body_mps2 - self.accel_bias
        gyro = measurement.gyro_body_rps - self.gyro_bias
        rotation = rotation_matrix(self.quaternion)
        gravity_nav = np.array([0.0, 0.0, -self.config.simulation.gravity_mps2])
        acceleration_nav = rotation @ accel + gravity_nav
        self.position += self.velocity * dt + 0.5 * acceleration_nav * dt * dt
        self.velocity += acceleration_nav * dt
        self.quaternion = normalize_quaternion(
            quaternion_multiply(self.quaternion, exponential_quaternion(gyro * dt))
        )

        transition_rate = np.zeros((15, 15))
        transition_rate[0:3, 3:6] = np.eye(3)
        transition_rate[3:6, 6:9] = -rotation @ skew(accel)
        transition_rate[3:6, 9:12] = -rotation
        transition_rate[6:9, 6:9] = -skew(gyro)
        transition_rate[6:9, 12:15] = -np.eye(3)
        transition = np.eye(15) + transition_rate * dt

        adis = self.config.adis16470
        estimator = self.config.estimator
        accel_noise_density = adis.accel_noise_rms_mg * 1e-3 * self.config.simulation.gravity_mps2 / np.sqrt(600.0)
        gyro_noise_density = np.radians(adis.gyro_noise_rms_dps) / np.sqrt(550.0)
        process_noise = np.zeros((15, 15))
        process_noise[3:6, 3:6] = np.eye(3) * accel_noise_density**2 * dt
        process_noise[6:9, 6:9] = np.eye(3) * gyro_noise_density**2 * dt
        process_noise[9:12, 9:12] = np.eye(3) * estimator.accel_bias_rw_mps2_sqrt_s**2 * dt
        process_noise[12:15, 12:15] = np.eye(3) * estimator.gyro_bias_rw_rps_sqrt_s**2 * dt
        self.covariance = transition @ self.covariance @ transition.T + process_noise
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.last_state_ticks = event.measurement_ticks
        self.health["propagated_samples"] += 1

        return StateEstimate(
            state_ticks=event.measurement_ticks,
            publication_ticks=event.arrival_ticks,
            position_enu_m=self.position.copy(),
            velocity_enu_mps=self.velocity.copy(),
            q_body_to_nav=self.quaternion.copy(),
            accel_bias_body_mps2=self.accel_bias.copy(),
            gyro_bias_body_rps=self.gyro_bias.copy(),
            covariance=self.covariance.copy(),
            valid=True,
            health=dict(self.health),
        )

    def run(self, events: list[MeasurementEvent]) -> list[StateEstimate]:
        estimates: list[StateEstimate] = []
        for event in sorted(events, key=lambda item: (item.arrival_ticks, item.sequence_number)):
            estimate = self.process(event)
            if estimate is not None:
                estimates.append(estimate)
        return estimates
