"""BNO085 SH-2 reports transported in exact SHTP packets."""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

import numpy as np

from .config import BnoConfig, SimulationConfig
from .frames import quaternion_from_rotation_matrix, rotation_matrix, skew
from .truth import interpolate_truth
from .types import DecodedInertialReport, MeasurementEvent, SensorId, StatusFlag, TruthSample

ACCELEROMETER_REPORT = 0x01
UNCALIBRATED_GYROSCOPE_REPORT = 0x07
GAME_ROTATION_VECTOR_REPORT = 0x08
SENSOR_REPORTS_CHANNEL = 3
CONTROL_CHANNEL = 2
ACCEL_Q = 8
GYRO_Q = 9
QUATERNION_Q = 14
ACCEL_RANGE_MPS2_G = 8.0
GYRO_RANGE_DPS = 2000.0


@dataclass(frozen=True, slots=True)
class BnoShtpPacket:
    channel: int
    sequence: int
    cargo: bytes
    continuation: bool = False

    def payload_bytes(self) -> bytes:
        length = len(self.cargo) + 4
        if length > 0x7FFF:
            raise ValueError("SHTP packet is too long")
        encoded_length = length | (0x8000 if self.continuation else 0)
        return struct.pack("<HBB", encoded_length, self.channel & 0xFF, self.sequence & 0xFF) + self.cargo

    transaction_bytes = payload_bytes

    @classmethod
    def from_bytes(cls, packet: bytes) -> "BnoShtpPacket":
        if len(packet) < 4:
            raise ValueError("truncated SHTP header")
        encoded_length, channel, sequence = struct.unpack_from("<HBB", packet)
        length = encoded_length & 0x7FFF
        if length != len(packet):
            raise ValueError("SHTP packet length does not match header")
        return cls(channel, sequence, packet[4:], bool(encoded_length & 0x8000))


@dataclass(frozen=True, slots=True)
class BnoFaultSchedule:
    packet_loss: frozenset[int] = field(default_factory=frozenset)
    duplicate_channel_sequence: frozenset[int] = field(default_factory=frozenset)
    truncate: frozenset[int] = field(default_factory=frozenset)
    reset: frozenset[int] = field(default_factory=frozenset)


def _encode_q(value: np.ndarray, q_point: int) -> tuple[int, ...]:
    scaled = np.rint(np.asarray(value) * (1 << q_point))
    return tuple(int(item) for item in np.clip(scaled, -32768, 32767))


def _decode_q(values: tuple[int, ...], q_point: int) -> np.ndarray:
    return np.asarray(values, dtype=np.float64) / (1 << q_point)


def encode_sensor_report(report_id: int, report_sequence: int, status: int, values: np.ndarray, bias: np.ndarray | None = None) -> bytes:
    header = struct.pack("<BBBB", report_id, report_sequence & 0xFF, status & 0x03, 0)
    if report_id == ACCELEROMETER_REPORT:
        return header + struct.pack("<3h", *_encode_q(values, ACCEL_Q))
    if report_id == UNCALIBRATED_GYROSCOPE_REPORT:
        if bias is None:
            raise ValueError("uncalibrated gyro report requires a bias")
        return header + struct.pack("<6h", *(_encode_q(values, GYRO_Q) + _encode_q(bias, GYRO_Q)))
    if report_id == GAME_ROTATION_VECTOR_REPORT:
        quaternion = np.asarray(values, dtype=np.float64)
        scalar_last = quaternion[[1, 2, 3, 0]]
        return header + struct.pack("<4h", *_encode_q(scalar_last, QUATERNION_Q))
    raise ValueError(f"unsupported SH-2 report 0x{report_id:02x}")


def decode_packet(packet: BnoShtpPacket) -> DecodedInertialReport:
    if packet.channel != SENSOR_REPORTS_CHANNEL or len(packet.cargo) < 4:
        raise ValueError("packet is not a complete sensor report")
    report_id, sequence, status, _delay = struct.unpack_from("<BBBB", packet.cargo)
    if report_id == ACCELEROMETER_REPORT and len(packet.cargo) == 10:
        vector = _decode_q(struct.unpack_from("<3h", packet.cargo, 4), ACCEL_Q)
        return DecodedInertialReport(report_id, sequence, status, vector=vector)
    if report_id == UNCALIBRATED_GYROSCOPE_REPORT and len(packet.cargo) == 16:
        values = struct.unpack_from("<6h", packet.cargo, 4)
        return DecodedInertialReport(report_id, sequence, status, vector=_decode_q(values[:3], GYRO_Q), bias=_decode_q(values[3:], GYRO_Q))
    if report_id == GAME_ROTATION_VECTOR_REPORT and len(packet.cargo) == 12:
        x, y, z, w = _decode_q(struct.unpack_from("<4h", packet.cargo, 4), QUATERNION_Q)
        quaternion = np.array([w, x, y, z], dtype=np.float64)
        norm = np.linalg.norm(quaternion)
        if norm == 0.0:
            raise ValueError("zero game rotation quaternion")
        return DecodedInertialReport(report_id, sequence, status, quaternion_body_to_nav=quaternion / norm)
    raise ValueError("invalid SH-2 report length or identifier")


def decode_event(event: MeasurementEvent, config: BnoConfig) -> DecodedInertialReport:
    if event.sensor_id != SensorId.BNO085_SHTP:
        raise ValueError("event is not from BNO085")
    decoded = decode_packet(BnoShtpPacket.from_bytes(event.payload))
    if decoded.vector is not None and decoded.report_id in (ACCELEROMETER_REPORT, UNCALIBRATED_GYROSCOPE_REPORT):
        vector = config.sensor_to_body @ decoded.vector
        bias = None if decoded.bias is None else config.sensor_to_body @ decoded.bias
        return DecodedInertialReport(decoded.report_id, decoded.report_sequence, decoded.status, vector=vector, bias=bias)
    if decoded.quaternion_body_to_nav is not None:
        body_to_nav = rotation_matrix(decoded.quaternion_body_to_nav) @ config.sensor_to_body.T
        quaternion = quaternion_from_rotation_matrix(body_to_nav)
        return DecodedInertialReport(decoded.report_id, decoded.report_sequence, decoded.status, quaternion_body_to_nav=quaternion)
    return decoded


class Bno085Model:
    def __init__(self, config: BnoConfig, simulation: SimulationConfig, seed: int):
        self.config = config
        self.simulation = simulation
        self.rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.BNO085_SHTP)]))

    def startup_trace(self) -> list[BnoShtpPacket]:
        """Deterministic advertisement, product exchange, and Set Feature traffic."""

        advertisement = BnoShtpPacket(0, 0, b"SHTP\x00BNO085\x00")
        reset_complete = BnoShtpPacket(1, 0, b"\x01")
        product_request = BnoShtpPacket(CONTROL_CHANNEL, 0, b"\xf9\x00")
        product_response = BnoShtpPacket(CONTROL_CHANNEL, 0, struct.pack("<BBBBIIHB", 0xF8, 0, 3, 2, 0x00000000, 0, 0, 0))
        commands = []
        for sequence, (report_id, rate) in enumerate(((ACCELEROMETER_REPORT, self.config.accel_rate_hz), (UNCALIBRATED_GYROSCOPE_REPORT, self.config.gyro_rate_hz), (GAME_ROTATION_VECTOR_REPORT, self.config.game_rotation_rate_hz))):
            interval_us = int(round(1_000_000 / rate))
            commands.append(BnoShtpPacket(CONTROL_CHANNEL, sequence + 1, struct.pack("<BBBHIII", 0xFD, report_id, 0, 0, interval_us, 0, 0)))
        return [advertisement, reset_complete, product_request, product_response, *commands]

    @staticmethod
    def _low_pass(previous: np.ndarray | None, value: np.ndarray, rate_hz: int, cutoff_hz: float) -> np.ndarray:
        if previous is None or cutoff_hz <= 0.0:
            return value
        dt = 1.0 / rate_hz
        alpha = dt / (dt + 1.0 / (2.0 * np.pi * cutoff_hz))
        return previous + alpha * (value - previous)

    def generate(self, truth: list[TruthSample], faults: BnoFaultSchedule | None = None) -> list[MeasurementEvent]:
        if not self.config.enabled or not truth:
            return []
        faults = faults or BnoFaultSchedule()
        start = truth[0].ticks + int(round(self.config.startup_delay_s * self.simulation.clock_hz))
        end = truth[-1].ticks
        schedule: list[tuple[int, int]] = []
        for report_id, rate in ((ACCELEROMETER_REPORT, self.config.accel_rate_hz), (UNCALIBRATED_GYROSCOPE_REPORT, self.config.gyro_rate_hz), (GAME_ROTATION_VECTOR_REPORT, self.config.game_rotation_rate_hz)):
            schedule.extend((ticks, report_id) for ticks in range(start, end + 1, self.simulation.clock_hz // rate))
        schedule.sort(key=lambda item: (item[0], item[1]))
        gravity_nav = np.array([0.0, 0.0, -self.simulation.gravity_mps2])
        body_to_sensor = self.config.sensor_to_body.T
        transform = (np.eye(3) + skew(self.config.misalignment_rad))
        accel_transform = transform @ np.diag(1.0 + self.config.accel_scale_error)
        gyro_transform = transform @ np.diag(1.0 + self.config.gyro_scale_error)
        accel_bias = self.config.accel_bias_mps2.copy()
        gyro_bias = self.config.gyro_bias_rps.copy()
        report_sequences = {report: 0 for report in (ACCELEROMETER_REPORT, UNCALIBRATED_GYROSCOPE_REPORT, GAME_ROTATION_VECTOR_REPORT)}
        previous_accel: np.ndarray | None = None
        previous_gyro: np.ndarray | None = None
        planned: list[tuple[int, int, int, StatusFlag, bytes]] = []
        for logical_sequence, (ticks, report_id) in enumerate(schedule):
            if logical_sequence in faults.reset:
                report_sequences = {report: 0 for report in report_sequences}
            sample = interpolate_truth(truth, ticks)
            rotation = rotation_matrix(sample.q_body_to_nav)
            flags = StatusFlag.VALID
            if report_id == ACCELEROMETER_REPORT:
                accel_bias += self.rng.normal(0.0, self.config.accel_bias_rw_mps2_sqrt_s / np.sqrt(self.config.accel_rate_hz), 3)
                physical = rotation.T @ (sample.acceleration_enu_mps2 - gravity_nav)
                measured = accel_transform @ (body_to_sensor @ physical) + accel_bias
                noise = self.config.accel_noise_density_mg_sqrt_hz * 1e-3 * self.simulation.gravity_mps2 * np.sqrt(self.config.accel_rate_hz / 2.0)
                measured += self.rng.normal(0.0, noise, 3)
                physical_lsb = 16.0 * self.simulation.gravity_mps2 / 4096.0
                measured = np.rint(measured / physical_lsb) * physical_lsb
                limit = ACCEL_RANGE_MPS2_G * self.simulation.gravity_mps2
                if np.any(np.abs(measured) > limit):
                    flags |= StatusFlag.SATURATED
                measured = np.clip(measured, -limit, limit)
                measured = self._low_pass(previous_accel, measured, self.config.accel_rate_hz, self.config.acceleration_filter_hz)
                previous_accel = measured
                cargo = encode_sensor_report(report_id, report_sequences[report_id], 3, measured)
                delay = self.config.acceleration_processing_delay_s
            elif report_id == UNCALIBRATED_GYROSCOPE_REPORT:
                gyro_bias += self.rng.normal(0.0, self.config.gyro_bias_rw_rps_sqrt_s / np.sqrt(self.config.gyro_rate_hz), 3)
                measured = gyro_transform @ (body_to_sensor @ sample.angular_rate_body_rps) + gyro_bias
                noise = np.radians(self.config.gyro_noise_density_dps_sqrt_hz) * np.sqrt(self.config.gyro_rate_hz / 2.0)
                measured += self.rng.normal(0.0, noise, 3)
                physical_lsb = np.radians(2.0 * GYRO_RANGE_DPS / 65536.0)
                measured = np.rint(measured / physical_lsb) * physical_lsb
                limit = np.radians(GYRO_RANGE_DPS)
                if np.any(np.abs(measured) > limit):
                    flags |= StatusFlag.SATURATED
                measured = np.clip(measured, -limit, limit)
                measured = self._low_pass(previous_gyro, measured, self.config.gyro_rate_hz, self.config.gyro_filter_hz)
                previous_gyro = measured
                cargo = encode_sensor_report(report_id, report_sequences[report_id], 3, measured, gyro_bias)
                delay = self.config.gyro_processing_delay_s
            else:
                q_sensor_to_nav = quaternion_from_rotation_matrix(rotation @ self.config.sensor_to_body)
                cargo = encode_sensor_report(report_id, report_sequences[report_id], 3, q_sensor_to_nav)
                delay = self.config.game_rotation_processing_delay_s
            ready_ticks = ticks + int(round(delay * self.simulation.clock_hz))
            planned.append((ready_ticks, ticks, logical_sequence, flags, cargo))
            report_sequences[report_id] = (report_sequences[report_id] + 1) & 0xFF
        events: list[MeasurementEvent] = []
        channel_sequence = 0
        bus_free = 0
        for transport_sequence, (ready_ticks, ticks, logical_sequence, flags, cargo) in enumerate(sorted(planned, key=lambda item: (item[0], item[2]))):
            if logical_sequence in faults.reset:
                channel_sequence = 0
            packet_sequence = (channel_sequence - 1) & 0xFF if logical_sequence in faults.duplicate_channel_sequence else channel_sequence
            payload = BnoShtpPacket(SENSOR_REPORTS_CHANNEL, packet_sequence, cargo).payload_bytes()
            if logical_sequence in faults.truncate:
                payload = payload[:-1]
                flags |= StatusFlag.DIAGNOSTIC_ERROR
            transfer_ticks = int(np.ceil(len(payload) * 8 / self.config.spi_clock_hz * self.simulation.clock_hz))
            arrival = max(ready_ticks, bus_free) + transfer_ticks
            bus_free = arrival
            if logical_sequence not in faults.packet_loss:
                events.append(MeasurementEvent(1, SensorId.BNO085_SHTP, transport_sequence, ticks, arrival, flags, payload))
            channel_sequence = (channel_sequence + 1) & 0xFF
        return events
