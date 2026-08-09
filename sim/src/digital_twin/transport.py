"""Versioned logical events and deterministic per-sensor replay codecs."""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator

from .types import MeasurementEvent, SensorId, StatusFlag

ADIS_BURST_COMMAND = 0x6800
ADIS_RESPONSE_WORDS = 10
ADIS_TRANSACTION_WORDS = 11
ADIS_TRANSACTION_BITS = 176
ADXL_READ_COMMAND = 0xF2
BMP_READ_COMMAND = 0x9D
ADXL_INT_SOURCE_COMMAND = 0xB0
BMP_INT_STATUS_COMMAND = 0xA7


def adis_checksum(words: Iterable[int]) -> int:
    """Datasheet byte-sum checksum for the nine words before CHECKSUM."""

    values = tuple(words)
    if len(values) != 9:
        raise ValueError("ADIS checksum requires nine data words")
    return sum(((word >> 8) & 0xFF) + (word & 0xFF) for word in values) & 0xFFFF


@dataclass(frozen=True, slots=True)
class AdisBurst:
    diag_stat: int
    gyro_words: tuple[int, int, int]
    accel_words: tuple[int, int, int]
    temperature_word: int
    data_counter: int
    checksum: int

    @property
    def words(self) -> tuple[int, ...]:
        return (
            self.diag_stat,
            *self.gyro_words,
            *self.accel_words,
            self.temperature_word,
            self.data_counter,
            self.checksum,
        )

    @classmethod
    def create(
        cls,
        diag_stat: int,
        gyro_words: tuple[int, int, int],
        accel_words: tuple[int, int, int],
        temperature_word: int,
        data_counter: int,
    ) -> "AdisBurst":
        data = (diag_stat, *gyro_words, *accel_words, temperature_word, data_counter)
        return cls(diag_stat, gyro_words, accel_words, temperature_word, data_counter, adis_checksum(data))

    def valid_checksum(self) -> bool:
        return self.checksum == adis_checksum(self.words[:-1])

    def payload_bytes(self) -> bytes:
        return struct.pack(">10H", *(word & 0xFFFF for word in self.words))

    def transaction_bytes(self) -> bytes:
        return struct.pack(">11H", ADIS_BURST_COMMAND, *(word & 0xFFFF for word in self.words))

    @classmethod
    def from_payload_bytes(cls, payload: bytes) -> "AdisBurst":
        if len(payload) != ADIS_RESPONSE_WORDS * 2:
            raise ValueError("ADIS response payload must be 20 bytes")
        values = struct.unpack(">10H", payload)
        return cls(values[0], tuple(values[1:4]), tuple(values[4:7]), values[7], values[8], values[9])

    @classmethod
    def from_transaction_bytes(cls, transaction: bytes) -> "AdisBurst":
        if len(transaction) != ADIS_TRANSACTION_WORDS * 2:
            raise ValueError("ADIS transaction must be 22 bytes / 176 bits")
        command, *values = struct.unpack(">11H", transaction)
        if command != ADIS_BURST_COMMAND:
            raise ValueError("invalid ADIS burst command")
        return cls.from_payload_bytes(struct.pack(">10H", *values))


@dataclass(frozen=True, slots=True)
class AdxlAcquisition:
    counts: tuple[int, int, int]
    interrupt_source: int = 0x80

    def payload_bytes(self) -> bytes:
        return struct.pack("<3hB", *self.counts, self.interrupt_source & 0xFF)

    def transaction_bytes(self) -> bytes:
        data = struct.pack("<3h", *self.counts)
        return bytes((ADXL_READ_COMMAND,)) + data + bytes((ADXL_INT_SOURCE_COMMAND, self.interrupt_source & 0xFF))

    @classmethod
    def from_payload_bytes(cls, payload: bytes) -> "AdxlAcquisition":
        if len(payload) != 7:
            raise ValueError("ADXL375 acquisition payload must be 7 bytes")
        x, y, z, status = struct.unpack("<3hB", payload)
        return cls((x, y, z), status)


def _pack_u24(value: int) -> bytes:
    return int(value).to_bytes(3, "little", signed=False)


def _unpack_u24(value: bytes) -> int:
    return int.from_bytes(value, "little", signed=False)


@dataclass(frozen=True, slots=True)
class BmpAcquisition:
    temperature_raw: int
    pressure_raw: int
    interrupt_status: int = 0x01

    def payload_bytes(self) -> bytes:
        return _pack_u24(self.temperature_raw) + _pack_u24(self.pressure_raw) + bytes((self.interrupt_status & 0xFF,))

    def transaction_bytes(self) -> bytes:
        data = _pack_u24(self.temperature_raw) + _pack_u24(self.pressure_raw)
        return bytes((BMP_READ_COMMAND,)) + data + bytes((BMP_INT_STATUS_COMMAND, self.interrupt_status & 0xFF))

    @classmethod
    def from_payload_bytes(cls, payload: bytes) -> "BmpAcquisition":
        if len(payload) != 7:
            raise ValueError("BMP581 acquisition payload must be 7 bytes")
        return cls(_unpack_u24(payload[0:3]), _unpack_u24(payload[3:6]), payload[6])


GNSS_SOLUTION_FORMAT = "<HQ6d36dBBf"
GNSS_PPS_FORMAT = "<HQdB"


@dataclass(frozen=True, slots=True)
class GnssSolution:
    gps_week: int
    tow_ns: int
    position_ecef_m: tuple[float, float, float]
    velocity_ecef_mps: tuple[float, float, float]
    covariance: tuple[float, ...]
    fix_type: int
    satellites: int
    correction_age_s: float = 0.0

    def payload_bytes(self) -> bytes:
        if len(self.covariance) != 36:
            raise ValueError("GNSS covariance must contain 36 row-major values")
        return struct.pack(
            GNSS_SOLUTION_FORMAT,
            self.gps_week,
            self.tow_ns,
            *self.position_ecef_m,
            *self.velocity_ecef_mps,
            *self.covariance,
            self.fix_type,
            self.satellites,
            self.correction_age_s,
        )

    transaction_bytes = payload_bytes

    @classmethod
    def from_payload_bytes(cls, payload: bytes) -> "GnssSolution":
        if len(payload) != struct.calcsize(GNSS_SOLUTION_FORMAT):
            raise ValueError("invalid generic GNSS solution payload length")
        values = struct.unpack(GNSS_SOLUTION_FORMAT, payload)
        return cls(values[0], values[1], tuple(values[2:5]), tuple(values[5:8]), tuple(values[8:44]), values[44], values[45], values[46])


@dataclass(frozen=True, slots=True)
class GnssPps:
    gps_week: int
    tow_ns: int
    uncertainty_ns: float
    time_valid: bool = True

    def payload_bytes(self) -> bytes:
        return struct.pack(GNSS_PPS_FORMAT, self.gps_week, self.tow_ns, self.uncertainty_ns, int(self.time_valid))

    transaction_bytes = payload_bytes

    @classmethod
    def from_payload_bytes(cls, payload: bytes) -> "GnssPps":
        if len(payload) != struct.calcsize(GNSS_PPS_FORMAT):
            raise ValueError("invalid GNSS PPS payload length")
        week, tow, uncertainty, valid = struct.unpack(GNSS_PPS_FORMAT, payload)
        return cls(week, tow, uncertainty, bool(valid))


def event_to_json(event: MeasurementEvent) -> dict[str, object]:
    return {
        "format_version": event.format_version,
        "sensor_id": int(event.sensor_id),
        "sequence_number": event.sequence_number,
        "measurement_ticks": event.measurement_ticks,
        "arrival_ticks": event.arrival_ticks,
        "status_flags": int(event.status_flags),
        "payload_hex": event.payload.hex(),
    }


def event_from_json(record: dict[str, object]) -> MeasurementEvent:
    return MeasurementEvent(
        format_version=int(record["format_version"]),
        sensor_id=SensorId(int(record["sensor_id"])),
        sequence_number=int(record["sequence_number"]),
        measurement_ticks=int(record["measurement_ticks"]),
        arrival_ticks=int(record["arrival_ticks"]),
        status_flags=StatusFlag(int(record["status_flags"])),
        payload=bytes.fromhex(str(record["payload_hex"])),
    )


def write_replay(events: Iterable[MeasurementEvent], ndjson_path: Path, binary_path: Path) -> tuple[int, int]:
    count = 0
    transaction_bytes = 0
    with ndjson_path.open("w", encoding="utf-8", newline="\n") as logical, binary_path.open("wb") as binary:
        for event in events:
            logical.write(json.dumps(event_to_json(event), sort_keys=True, separators=(",", ":")) + "\n")
            burst = AdisBurst.from_payload_bytes(event.payload)
            transaction = burst.transaction_bytes()
            binary.write(transaction)
            count += 1
            transaction_bytes += len(transaction)
    return count, transaction_bytes


REPLAY_FILENAMES = {
    SensorId.ADIS16470: "adis16470_bursts.bin",
    SensorId.ADXL375: "adxl375_acquisitions.bin",
    SensorId.BMP581: "bmp581_acquisitions.bin",
    SensorId.GNSS_SOLUTION: "gnss_solutions.bin",
    SensorId.GNSS_PPS: "gnss_pps.bin",
}


def transaction_bytes_for_event(event: MeasurementEvent) -> bytes:
    if event.sensor_id == SensorId.ADIS16470:
        return AdisBurst.from_payload_bytes(event.payload).transaction_bytes()
    if event.sensor_id == SensorId.ADXL375:
        return AdxlAcquisition.from_payload_bytes(event.payload).transaction_bytes()
    if event.sensor_id == SensorId.BMP581:
        return BmpAcquisition.from_payload_bytes(event.payload).transaction_bytes()
    if event.sensor_id == SensorId.GNSS_SOLUTION:
        return GnssSolution.from_payload_bytes(event.payload).transaction_bytes()
    if event.sensor_id == SensorId.GNSS_PPS:
        return GnssPps.from_payload_bytes(event.payload).transaction_bytes()
    raise ValueError(f"no replay codec for sensor {event.sensor_id}")


def write_multi_replay(events: Iterable[MeasurementEvent], ndjson_path: Path, output_dir: Path) -> dict[str, dict[str, int]]:
    ordered = sorted(events, key=lambda item: (item.arrival_ticks, int(item.sensor_id), item.sequence_number))
    handles = {sensor: (output_dir / name).open("wb") for sensor, name in REPLAY_FILENAMES.items() if any(event.sensor_id == sensor for event in ordered)}
    stats = {REPLAY_FILENAMES[sensor]: {"events": 0, "bytes": 0} for sensor in handles}
    try:
        with ndjson_path.open("w", encoding="utf-8", newline="\n") as logical:
            for event in ordered:
                logical.write(json.dumps(event_to_json(event), sort_keys=True, separators=(",", ":")) + "\n")
                transaction = transaction_bytes_for_event(event)
                handles[event.sensor_id].write(transaction)
                record = stats[REPLAY_FILENAMES[event.sensor_id]]
                record["events"] += 1
                record["bytes"] += len(transaction)
    finally:
        for handle in handles.values():
            handle.close()
    return stats


def read_events(path: Path) -> Iterator[MeasurementEvent]:
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                yield event_from_json(json.loads(line))


def read_transactions(path: Path) -> Iterator[AdisBurst]:
    size = ADIS_TRANSACTION_WORDS * 2
    with path.open("rb") as stream:
        while block := stream.read(size):
            if len(block) != size:
                raise ValueError("truncated ADIS transaction stream")
            yield AdisBurst.from_transaction_bytes(block)
