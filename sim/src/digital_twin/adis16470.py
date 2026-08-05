"""ADIS16470 sensor, data-ready, and burst-transport model."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .config import AdisConfig, SimulationConfig
from .frames import rotation_matrix, skew
from .transport import ADIS_TRANSACTION_BITS, AdisBurst
from .types import MeasurementEvent, SensorId, StatusFlag, TruthSample

ACCEL_LSB_PER_G = 800.0
GYRO_LSB_PER_DPS = 10.0
ACCEL_LIMIT_COUNTS = 32_000
GYRO_LIMIT_COUNTS = 20_000


@dataclass(frozen=True, slots=True)
class FaultSchedule:
    checksum_corruption: frozenset[int] = field(default_factory=frozenset)
    diagnostic_error: frozenset[int] = field(default_factory=frozenset)
    duplicate_counter: frozenset[int] = field(default_factory=frozenset)
    skipped_counter: frozenset[int] = field(default_factory=frozenset)
    packet_loss: frozenset[int] = field(default_factory=frozenset)


@dataclass(frozen=True, slots=True)
class DecodedAdisMeasurement:
    accel_body_mps2: NDArray[np.float64]
    gyro_body_rps: NDArray[np.float64]
    temperature_c: float
    data_counter: int
    saturated: bool


def _signed_word(value: int) -> int:
    return value & 0xFFFF


def _signed_count(word: int) -> int:
    return word - 0x10000 if word & 0x8000 else word


def encode_accel_mps2(value: float, gravity_mps2: float) -> tuple[int, bool]:
    counts_unclipped = int(np.rint(value / gravity_mps2 * ACCEL_LSB_PER_G))
    counts = int(np.clip(counts_unclipped, -ACCEL_LIMIT_COUNTS, ACCEL_LIMIT_COUNTS))
    return _signed_word(counts), counts != counts_unclipped


def decode_accel_mps2(word: int, gravity_mps2: float) -> float:
    return _signed_count(word) / ACCEL_LSB_PER_G * gravity_mps2


def encode_gyro_rps(value: float) -> tuple[int, bool]:
    counts_unclipped = int(np.rint(np.degrees(value) * GYRO_LSB_PER_DPS))
    counts = int(np.clip(counts_unclipped, -GYRO_LIMIT_COUNTS, GYRO_LIMIT_COUNTS))
    return _signed_word(counts), counts != counts_unclipped


def decode_gyro_rps(word: int) -> float:
    return float(np.radians(_signed_count(word) / GYRO_LSB_PER_DPS))


class Adis16470Model:
    def __init__(self, config: AdisConfig, simulation: SimulationConfig, seed: int):
        self.config = config
        self.simulation = simulation
        self.rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.ADIS16470)]))

    def generate(
        self,
        truth: Iterable[TruthSample],
        faults: FaultSchedule | None = None,
    ) -> list[MeasurementEvent]:
        samples = list(truth)
        divisor = self.config.dec_rate + 1
        if not samples:
            return []
        if len(samples) < divisor:
            raise ValueError("truth stream is shorter than one ADIS decimation interval")
        faults = faults or FaultSchedule()
        gravity_nav = np.array([0.0, 0.0, -self.simulation.gravity_mps2])
        sensor_to_body = self.config.sensor_to_body
        body_to_sensor = sensor_to_body.T
        misalignment = np.eye(3) + skew(self.config.misalignment_rad)
        accel_scale = np.diag(1.0 + self.config.accel_scale_error)
        gyro_scale = np.diag(1.0 + self.config.gyro_scale_error)

        accel_bias = self.config.accel_bias_mps2.copy()
        gyro_bias = self.config.gyro_bias_rps.copy()
        dt_internal = 1.0 / self.simulation.truth_rate_hz
        accel_noise_sigma = self.config.accel_noise_rms_mg * 1e-3 * self.simulation.gravity_mps2
        gyro_noise_sigma = np.radians(self.config.gyro_noise_rms_dps)
        transfer_ticks = int(round(ADIS_TRANSACTION_BITS / self.config.spi_clock_hz * self.simulation.clock_hz))

        internal_accel: list[NDArray[np.float64]] = []
        internal_gyro: list[NDArray[np.float64]] = []
        events: list[MeasurementEvent] = []
        counter = 0
        output_sequence = 0

        for sample in samples:
            rotation = rotation_matrix(sample.q_body_to_nav)
            specific_force_body = rotation.T @ (sample.acceleration_enu_mps2 - gravity_nav)
            accel_sensor = body_to_sensor @ specific_force_body
            gyro_sensor = body_to_sensor @ sample.angular_rate_body_rps

            accel_bias += self.rng.normal(0.0, self.config.accel_bias_rw_mps2_sqrt_s * np.sqrt(dt_internal), 3)
            gyro_bias += self.rng.normal(0.0, self.config.gyro_bias_rw_rps_sqrt_s * np.sqrt(dt_internal), 3)
            accel_measured = misalignment @ accel_scale @ accel_sensor + accel_bias
            gyro_measured = misalignment @ gyro_scale @ gyro_sensor + gyro_bias
            accel_measured += self.rng.normal(0.0, accel_noise_sigma, 3)
            gyro_measured += self.rng.normal(0.0, gyro_noise_sigma, 3)
            internal_accel.append(accel_measured)
            internal_gyro.append(gyro_measured)

            if len(internal_accel) != divisor:
                continue

            output_index = output_sequence
            accel_output = np.mean(internal_accel, axis=0)
            gyro_output = np.mean(internal_gyro, axis=0)
            internal_accel.clear()
            internal_gyro.clear()

            accel_encoded = [encode_accel_mps2(value, self.simulation.gravity_mps2) for value in accel_output]
            gyro_encoded = [encode_gyro_rps(value) for value in gyro_output]
            saturated = any(flag for _, flag in (*accel_encoded, *gyro_encoded))

            if output_index in faults.skipped_counter:
                counter = (counter + 1) & 0xFFFF
            burst_counter = (counter - 1) & 0xFFFF if output_index in faults.duplicate_counter else counter
            diag = 0x0001 if output_index in faults.diagnostic_error else 0x0000
            temperature_word = _signed_word(int(np.rint(self.config.temperature_c * 10.0)))
            burst = AdisBurst.create(
                diag,
                tuple(word for word, _ in gyro_encoded),
                tuple(word for word, _ in accel_encoded),
                temperature_word,
                burst_counter,
            )
            checksum_corrupted = output_index in faults.checksum_corruption
            if checksum_corrupted:
                burst = AdisBurst(
                    burst.diag_stat,
                    burst.gyro_words,
                    burst.accel_words,
                    burst.temperature_word,
                    burst.data_counter,
                    burst.checksum ^ 0x0001,
                )

            flags = StatusFlag.VALID
            if saturated:
                flags |= StatusFlag.SATURATED
            if diag:
                flags |= StatusFlag.DIAGNOSTIC_ERROR
                flags &= ~StatusFlag.VALID
            if checksum_corrupted:
                flags |= StatusFlag.CHECKSUM_ERROR
                flags &= ~StatusFlag.VALID
            event = MeasurementEvent(
                format_version=1,
                sensor_id=SensorId.ADIS16470,
                sequence_number=output_index & 0xFFFFFFFF,
                measurement_ticks=sample.ticks,
                arrival_ticks=sample.ticks + transfer_ticks,
                status_flags=flags,
                payload=burst.payload_bytes(),
            )
            if output_index not in faults.packet_loss:
                events.append(event)
            counter = (counter + 1) & 0xFFFF
            output_sequence += 1

        return events


def decode_event(
    event: MeasurementEvent,
    config: AdisConfig,
    simulation: SimulationConfig,
) -> DecodedAdisMeasurement:
    if event.sensor_id != SensorId.ADIS16470:
        raise ValueError("event is not from ADIS16470")
    burst = AdisBurst.from_payload_bytes(event.payload)
    if not burst.valid_checksum():
        raise ValueError("ADIS checksum failure")
    if burst.diag_stat:
        raise ValueError(f"ADIS diagnostic failure: 0x{burst.diag_stat:04x}")
    accel_sensor = np.array([decode_accel_mps2(word, simulation.gravity_mps2) for word in burst.accel_words])
    gyro_sensor = np.array([decode_gyro_rps(word) for word in burst.gyro_words])
    return DecodedAdisMeasurement(
        accel_body_mps2=config.sensor_to_body @ accel_sensor,
        gyro_body_rps=config.sensor_to_body @ gyro_sensor,
        temperature_c=_signed_count(burst.temperature_word) / 10.0,
        data_counter=burst.data_counter,
        saturated=bool(event.status_flags & StatusFlag.SATURATED),
    )
