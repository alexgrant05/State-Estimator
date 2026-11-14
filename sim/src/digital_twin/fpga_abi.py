"""FPGA ABI v1 codecs shared by digital-twin replay and Linux verification."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable, Iterator

from .generated_abi import (
    ABI_MAJOR,
    BLOCK_MAGIC,
    DMA_BLOCK_BYTES,
    DMA_FLUSH_TICKS,
    DMA_HEADER_BYTES,
    INJECTION_HEADER_BYTES,
    INJECTION_MAGIC,
    MAX_PAYLOAD_BYTES,
    RECORD_ALIGNMENT,
    RECORD_HEADER_BYTES,
    RECORD_MAGIC,
    RECORD_VERSION,
)
from .types import MeasurementEvent, SensorId, StatusFlag

_RECORD = struct.Struct("<IBBHHHIQQIIII")
_BLOCK = struct.Struct("<IHHIIHHIIIIIQQII")
_INJECTION = struct.Struct("<IHHQ")
_KNOWN_STATUS_MASK = (1 << 19) - 1


def crc32c(data: bytes | bytearray | memoryview) -> int:
    """Return CRC32C using the ABI's reflected Castagnoli parameters."""

    value = 0xFFFFFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x82F63B78 if value & 1 else 0)
    return value ^ 0xFFFFFFFF


def _aligned(value: int, alignment: int = RECORD_ALIGNMENT) -> int:
    return (value + alignment - 1) & ~(alignment - 1)


def encode_common_record(event: MeasurementEvent) -> bytes:
    payload = bytes(event.payload)
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError("payload exceeds ABI maximum")
    flags = int(event.status_flags)
    if flags & ~_KNOWN_STATUS_MASK:
        raise ValueError("reserved status bits must be zero")
    total = _aligned(RECORD_HEADER_BYTES + len(payload))
    header = bytearray(
        _RECORD.pack(
            RECORD_MAGIC,
            RECORD_VERSION,
            int(event.sensor_id),
            RECORD_HEADER_BYTES,
            total,
            len(payload),
            event.sequence_number,
            event.measurement_ticks,
            event.arrival_ticks,
            flags,
            0,
            crc32c(payload),
            0,
        )
    )
    struct.pack_into("<I", header, 44, crc32c(header))
    return bytes(header) + payload + bytes(total - RECORD_HEADER_BYTES - len(payload))


def decode_common_record(data: bytes | bytearray | memoryview) -> MeasurementEvent:
    view = memoryview(data)
    if len(view) < RECORD_HEADER_BYTES:
        raise ValueError("truncated common-record header")
    fields = _RECORD.unpack_from(view)
    magic, version, sensor, header_bytes, total, payload_bytes, sequence, measurement, arrival, flags, reserved, payload_crc, header_crc = fields
    if magic != RECORD_MAGIC or version != RECORD_VERSION or header_bytes != RECORD_HEADER_BYTES:
        raise ValueError("common-record identity or version mismatch")
    if total < header_bytes or total % RECORD_ALIGNMENT or total > len(view):
        raise ValueError("invalid common-record length")
    if payload_bytes > MAX_PAYLOAD_BYTES or header_bytes + payload_bytes > total:
        raise ValueError("invalid common-record payload length")
    if reserved or flags & ~_KNOWN_STATUS_MASK:
        raise ValueError("reserved common-record field is nonzero")
    header = bytearray(view[:header_bytes])
    struct.pack_into("<I", header, 44, 0)
    if crc32c(header) != header_crc:
        raise ValueError("common-record header CRC32C mismatch")
    payload = bytes(view[header_bytes : header_bytes + payload_bytes])
    if crc32c(payload) != payload_crc:
        raise ValueError("common-record payload CRC32C mismatch")
    if any(view[header_bytes + payload_bytes : total]):
        raise ValueError("common-record padding must be zero")
    return MeasurementEvent(version, SensorId(sensor), sequence, measurement, arrival, StatusFlag(flags), payload)


@dataclass(frozen=True, slots=True)
class DmaBlock:
    sequence: int
    flags: int
    pl_drops: int
    reset_generation: int
    session_id: int
    records: tuple[MeasurementEvent, ...]


def encode_dma_block(block: DmaBlock) -> bytes:
    encoded = [encode_common_record(record) for record in block.records]
    payload = b"".join(encoded)
    valid = DMA_HEADER_BYTES + len(payload)
    if valid > DMA_BLOCK_BYTES:
        raise ValueError("records do not fit in one DMA block")
    sensor_mask = 0
    for record in block.records:
        sensor_mask |= 1 << int(record.sensor_id)
    first = block.records[0].measurement_ticks if block.records else 0
    last = block.records[-1].measurement_ticks if block.records else 0
    header = bytearray(_BLOCK.pack(BLOCK_MAGIC, ABI_MAJOR, DMA_HEADER_BYTES, block.sequence & 0xFFFFFFFF, block.flags, valid, len(encoded), sensor_mask, block.pl_drops, block.reset_generation, block.session_id, 0, first, last, crc32c(payload), 0))
    struct.pack_into("<I", header, 60, crc32c(header))
    return bytes(header) + payload + bytes(DMA_BLOCK_BYTES - valid)


def decode_dma_block(data: bytes | bytearray | memoryview) -> DmaBlock:
    view = memoryview(data)
    if len(view) != DMA_BLOCK_BYTES:
        raise ValueError("DMA block must be exactly 4096 bytes")
    fields = _BLOCK.unpack_from(view)
    magic, major, header_bytes, sequence, flags, valid, packet_count, sensor_mask, drops, generation, session, reserved, first, last, payload_crc, header_crc = fields
    if magic != BLOCK_MAGIC or major != ABI_MAJOR or header_bytes != DMA_HEADER_BYTES:
        raise ValueError("DMA block identity or version mismatch")
    if not header_bytes <= valid <= DMA_BLOCK_BYTES or reserved:
        raise ValueError("invalid DMA block header")
    header = bytearray(view[:header_bytes])
    struct.pack_into("<I", header, 60, 0)
    if crc32c(header) != header_crc or crc32c(view[header_bytes:valid]) != payload_crc:
        raise ValueError("DMA block CRC32C mismatch")
    records: list[MeasurementEvent] = []
    cursor = header_bytes
    while cursor < valid:
        if cursor + RECORD_HEADER_BYTES > valid:
            raise ValueError("truncated record in DMA block")
        total = struct.unpack_from("<H", view, cursor + 8)[0]
        if total == 0 or cursor + total > valid:
            raise ValueError("invalid record extent in DMA block")
        records.append(decode_common_record(view[cursor : cursor + total]))
        cursor += total
    if len(records) != packet_count:
        raise ValueError("DMA packet count mismatch")
    actual_mask = 0
    for record in records:
        actual_mask |= 1 << int(record.sensor_id)
    if actual_mask != sensor_mask:
        raise ValueError("DMA sensor mask mismatch")
    if records and (records[0].measurement_ticks != first or records[-1].measurement_ticks != last):
        raise ValueError("DMA measurement range mismatch")
    if any(view[valid:]):
        raise ValueError("unused DMA bytes must be zero")
    return DmaBlock(sequence, flags, drops, generation, session, tuple(records))


def build_dma_blocks(events: Iterable[MeasurementEvent], session_id: int, reset_generation: int = 0) -> Iterator[bytes]:
    pending: list[MeasurementEvent] = []
    used = DMA_HEADER_BYTES
    first_arrival = 0
    sequence = 0
    for event in events:
        encoded_size = len(encode_common_record(event))
        timed_out = bool(pending and event.arrival_ticks - first_arrival >= DMA_FLUSH_TICKS)
        if pending and (used + encoded_size > DMA_BLOCK_BYTES or timed_out):
            yield encode_dma_block(DmaBlock(sequence, 0, 0, reset_generation, session_id, tuple(pending)))
            sequence = (sequence + 1) & 0xFFFFFFFF
            pending, used = [], DMA_HEADER_BYTES
        if not pending:
            first_arrival = event.arrival_ticks
        pending.append(event)
        used += encoded_size
    if pending:
        yield encode_dma_block(DmaBlock(sequence, 0, 0, reset_generation, session_id, tuple(pending)))


def encode_replay_item(event: MeasurementEvent, delay_ticks: int) -> bytes:
    if delay_ticks < 0 or delay_ticks > 0xFFFFFFFFFFFFFFFF:
        raise ValueError("delay_ticks must fit uint64")
    return _INJECTION.pack(INJECTION_MAGIC, RECORD_VERSION, INJECTION_HEADER_BYTES, delay_ticks) + encode_common_record(event)


def decode_replay_item(data: bytes | bytearray | memoryview) -> tuple[int, MeasurementEvent]:
    if len(data) < INJECTION_HEADER_BYTES + RECORD_HEADER_BYTES:
        raise ValueError("truncated replay item")
    magic, version, header_bytes, delay = _INJECTION.unpack_from(data)
    if magic != INJECTION_MAGIC or version != RECORD_VERSION or header_bytes != INJECTION_HEADER_BYTES:
        raise ValueError("replay item identity mismatch")
    total = struct.unpack_from("<H", data, INJECTION_HEADER_BYTES + 8)[0]
    if len(data) != INJECTION_HEADER_BYTES + total:
        raise ValueError("replay item length mismatch")
    return delay, decode_common_record(memoryview(data)[INJECTION_HEADER_BYTES:])
