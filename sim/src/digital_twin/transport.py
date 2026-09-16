"""Versioned logical events and active per-sensor replay codecs."""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator

from .types import MeasurementEvent, SensorId, StatusFlag

ADXL_READ_COMMAND = 0xF2
BMP_READ_COMMAND = 0x9D
ADXL_INT_SOURCE_COMMAND = 0xB0
BMP_INT_STATUS_COMMAND = 0xA7


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


def event_to_json(event: MeasurementEvent) -> dict[str, object]:
    return {"format_version": event.format_version, "sensor_id": int(event.sensor_id), "sequence_number": event.sequence_number, "measurement_ticks": event.measurement_ticks, "arrival_ticks": event.arrival_ticks, "status_flags": int(event.status_flags), "payload_hex": event.payload.hex()}


def event_from_json(record: dict[str, object]) -> MeasurementEvent:
    return MeasurementEvent(format_version=int(record["format_version"]), sensor_id=SensorId(int(record["sensor_id"])), sequence_number=int(record["sequence_number"]), measurement_ticks=int(record["measurement_ticks"]), arrival_ticks=int(record["arrival_ticks"]), status_flags=StatusFlag(int(record["status_flags"])), payload=bytes.fromhex(str(record["payload_hex"])))


REPLAY_FILENAMES = {
    SensorId.BNO085_SHTP: "bno085_shtp.bin",
    SensorId.ADXL375: "adxl375_acquisitions.bin",
    SensorId.BMP581: "bmp581_acquisitions.bin",
    SensorId.ZED_F9P_UBX: "zed_f9p_uart.bin",
}


def transaction_bytes_for_event(event: MeasurementEvent) -> bytes:
    if event.sensor_id == SensorId.BNO085_SHTP:
        from .bno085 import BnoShtpPacket
        return BnoShtpPacket.from_bytes(event.payload).transaction_bytes()
    if event.sensor_id == SensorId.ADXL375:
        return AdxlAcquisition.from_payload_bytes(event.payload).transaction_bytes()
    if event.sensor_id == SensorId.BMP581:
        return BmpAcquisition.from_payload_bytes(event.payload).transaction_bytes()
    if event.sensor_id == SensorId.ZED_F9P_UBX:
        from .zed_f9p import UbxFrame
        return UbxFrame.from_bytes(event.payload).transaction_bytes()
    raise ValueError(f"sensor {event.sensor_id} has no byte-stream replay artifact")


def write_multi_replay(events: Iterable[MeasurementEvent], ndjson_path: Path, output_dir: Path) -> dict[str, dict[str, int]]:
    ordered = sorted(events, key=lambda item: (item.arrival_ticks, int(item.sensor_id), item.sequence_number))
    present = {event.sensor_id for event in ordered}
    handles = {sensor: (output_dir / name).open("wb") for sensor, name in REPLAY_FILENAMES.items() if sensor in present}
    stats = {REPLAY_FILENAMES[sensor]: {"events": 0, "bytes": 0} for sensor in handles}
    try:
        with ndjson_path.open("w", encoding="utf-8", newline="\n") as logical:
            for event in ordered:
                logical.write(json.dumps(event_to_json(event), sort_keys=True, separators=(",", ":")) + "\n")
                if event.sensor_id not in handles:
                    continue
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


def read_shtp_transactions(path: Path) -> Iterator[bytes]:
    with path.open("rb") as stream:
        while header := stream.read(4):
            if len(header) != 4:
                raise ValueError("truncated SHTP header")
            length = struct.unpack_from("<H", header)[0] & 0x7FFF
            if length < 4:
                raise ValueError("invalid SHTP packet length")
            cargo = stream.read(length - 4)
            if len(cargo) != length - 4:
                raise ValueError("truncated SHTP stream")
            yield header + cargo


def read_ubx_transactions(path: Path) -> Iterator[bytes]:
    with path.open("rb") as stream:
        while sync := stream.read(2):
            if sync != b"\xb5\x62":
                raise ValueError("invalid UBX sync in replay")
            header = stream.read(4)
            if len(header) != 4:
                raise ValueError("truncated UBX header")
            length = struct.unpack_from("<H", header, 2)[0]
            tail = stream.read(length + 2)
            if len(tail) != length + 2:
                raise ValueError("truncated UBX stream")
            yield sync + header + tail
