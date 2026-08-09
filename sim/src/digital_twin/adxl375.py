"""ADXL375 high-g accelerometer and register acquisition model."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .config import AdxlConfig, SimulationConfig
from .frames import rotation_matrix, skew
from .transport import AdxlAcquisition
from .truth import interpolate_truth
from .types import MeasurementEvent, SensorId, StatusFlag, TruthSample

LSB_PER_G = 20.5
RANGE_G = 200.0


@dataclass(frozen=True, slots=True)
class DecodedAdxlMeasurement:
    accel_body_mps2: np.ndarray
    saturated: bool
    overrun: bool


@dataclass(frozen=True, slots=True)
class AdxlFaultSchedule:
    overrun: frozenset[int] = field(default_factory=frozenset)
    packet_loss: frozenset[int] = field(default_factory=frozenset)
    stuck_sample: frozenset[int] = field(default_factory=frozenset)


def encode_accel(value_mps2: float, gravity_mps2: float) -> tuple[int, bool]:
    value_g = value_mps2 / gravity_mps2
    clipped_g = float(np.clip(value_g, -RANGE_G, RANGE_G))
    return int(np.rint(clipped_g * LSB_PER_G)), clipped_g != value_g


def decode_accel(count: int, gravity_mps2: float) -> float:
    return count / LSB_PER_G * gravity_mps2


class Adxl375Model:
    def __init__(self, config: AdxlConfig, simulation: SimulationConfig, seed: int):
        self.config = config
        self.simulation = simulation
        self.rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.ADXL375)]))

    def generate(self, truth: list[TruthSample], faults: AdxlFaultSchedule | None = None) -> list[MeasurementEvent]:
        if not self.config.enabled or not truth:
            return []
        interval = self.simulation.clock_hz // self.config.output_rate_hz
        transfer_ticks = int(round(72 / self.config.spi_clock_hz * self.simulation.clock_hz))
        gravity_nav = np.array([0.0, 0.0, -self.simulation.gravity_mps2])
        body_to_sensor = self.config.sensor_to_body.T
        transform = (np.eye(3) + skew(self.config.misalignment_rad)) @ np.diag(1.0 + self.config.scale_error)
        noise_sigma = self.config.noise_density_mg_sqrt_hz * 1e-3 * self.simulation.gravity_mps2 * np.sqrt(self.config.output_rate_hz / 2.0)
        bias = self.config.bias_mps2.copy()
        events: list[MeasurementEvent] = []
        faults = faults or AdxlFaultSchedule()
        previous: AdxlAcquisition | None = None
        sequence = 0
        for ticks in range(truth[0].ticks, truth[-1].ticks + 1, interval):
            sample = interpolate_truth(truth, ticks)
            rotation = rotation_matrix(sample.q_body_to_nav)
            specific_force_body = rotation.T @ (sample.acceleration_enu_mps2 - gravity_nav)
            dt = 1.0 / self.config.output_rate_hz
            bias += self.rng.normal(0.0, self.config.bias_rw_mps2_sqrt_s * np.sqrt(dt), 3)
            measured = transform @ (body_to_sensor @ specific_force_body) + bias
            measured += self.rng.normal(0.0, noise_sigma, 3)
            encoded = [encode_accel(value, self.simulation.gravity_mps2) for value in measured]
            saturated = any(flag for _, flag in encoded)
            acquisition = AdxlAcquisition(tuple(value for value, _ in encoded))
            if sequence in faults.stuck_sample and previous is not None:
                acquisition = previous
            previous = acquisition
            flags = StatusFlag.VALID | (StatusFlag.SATURATED if saturated else StatusFlag.NONE)
            if sequence in faults.overrun:
                flags |= StatusFlag.OVERRUN
            if sequence not in faults.packet_loss:
                events.append(MeasurementEvent(1, SensorId.ADXL375, sequence, ticks, ticks + transfer_ticks, flags, acquisition.payload_bytes()))
            sequence = (sequence + 1) & 0xFFFFFFFF
        return events


def decode_event(event: MeasurementEvent, config: AdxlConfig, simulation: SimulationConfig) -> DecodedAdxlMeasurement:
    if event.sensor_id != SensorId.ADXL375:
        raise ValueError("event is not from ADXL375")
    acquisition = AdxlAcquisition.from_payload_bytes(event.payload)
    accel_sensor = np.array([decode_accel(value, simulation.gravity_mps2) for value in acquisition.counts])
    return DecodedAdxlMeasurement(
        accel_body_mps2=config.sensor_to_body @ accel_sensor,
        saturated=bool(event.status_flags & StatusFlag.SATURATED),
        overrun=bool(event.status_flags & StatusFlag.OVERRUN),
    )
