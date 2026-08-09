"""BMP581 pressure/temperature acquisition model."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

import numpy as np

from .config import BmpConfig, SimulationConfig
from .transport import BmpAcquisition
from .truth import interpolate_truth, standard_atmosphere
from .types import MeasurementEvent, SensorId, StatusFlag, TruthSample


@dataclass(frozen=True, slots=True)
class DecodedBmpMeasurement:
    pressure_pa: float
    temperature_c: float


@dataclass(frozen=True, slots=True)
class BmpFaultSchedule:
    invalid_status: frozenset[int] = field(default_factory=frozenset)
    packet_loss: frozenset[int] = field(default_factory=frozenset)
    pressure_spike_pa: Mapping[int, float] = field(default_factory=dict)


def encode_temperature_c(value: float) -> int:
    counts = int(np.clip(np.rint(value * 2**16), -(1 << 23), (1 << 23) - 1))
    return counts & 0xFFFFFF


def encode_pressure_pa(value: float) -> tuple[int, bool]:
    clipped = float(np.clip(value, 30_000.0, 125_000.0))
    return int(np.clip(np.rint(clipped * 2**6), 0, 0xFFFFFF)), clipped != value


class Bmp581Model:
    def __init__(self, config: BmpConfig, simulation: SimulationConfig, seed: int):
        self.config = config
        self.simulation = simulation
        self.rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.BMP581)]))

    def generate(self, truth: list[TruthSample], faults: BmpFaultSchedule | None = None) -> list[MeasurementEvent]:
        if not self.config.enabled or not truth:
            return []
        interval = self.simulation.clock_hz // self.config.output_rate_hz
        transfer_ticks = int(round(72 / self.config.spi_clock_hz * self.simulation.clock_hz))
        bias = self.config.pressure_bias_pa
        events: list[MeasurementEvent] = []
        faults = faults or BmpFaultSchedule()
        for sequence, ticks in enumerate(range(truth[0].ticks, truth[-1].ticks + 1, interval)):
            sample = interpolate_truth(truth, ticks)
            bias += self.rng.normal(0.0, self.config.pressure_bias_rw_pa_sqrt_s / np.sqrt(self.config.output_rate_hz))
            altitude_error = self.config.transonic_error_peak_m * np.exp(
                -0.5 * ((sample.mach - self.config.transonic_mach_center) / self.config.transonic_mach_sigma) ** 2
            )
            disturbed_pressure = standard_atmosphere(sample.altitude_msl_m + altitude_error)[0]
            pressure = disturbed_pressure + bias + self.rng.normal(0.0, self.config.pressure_noise_pa)
            pressure += faults.pressure_spike_pa.get(sequence, 0.0)
            temperature_c = sample.ambient_temperature_k - 273.15 + self.rng.normal(0.0, self.config.temperature_noise_c)
            pressure_raw, saturated = encode_pressure_pa(pressure)
            acquisition = BmpAcquisition(encode_temperature_c(temperature_c), pressure_raw)
            flags = StatusFlag.VALID | (StatusFlag.SATURATED if saturated else StatusFlag.NONE)
            if sequence in faults.invalid_status:
                flags &= ~StatusFlag.VALID
                flags |= StatusFlag.DIAGNOSTIC_ERROR
            if sequence not in faults.packet_loss:
                events.append(MeasurementEvent(1, SensorId.BMP581, sequence, ticks, ticks + transfer_ticks, flags, acquisition.payload_bytes()))
        return events


def decode_event(event: MeasurementEvent) -> DecodedBmpMeasurement:
    if event.sensor_id != SensorId.BMP581:
        raise ValueError("event is not from BMP581")
    acquisition = BmpAcquisition.from_payload_bytes(event.payload)
    temperature_raw = acquisition.temperature_raw
    if temperature_raw & 0x800000:
        temperature_raw -= 1 << 24
    return DecodedBmpMeasurement(acquisition.pressure_raw / 2**6, temperature_raw / 2**16)
