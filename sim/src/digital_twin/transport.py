"""ADIS burst and logical event replay codecs."""

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

