from __future__ import annotations

import struct
from pathlib import Path

import pytest

from digital_twin.fpga_abi import (
    DmaBlock,
    crc32c,
    decode_common_record,
    decode_dma_block,
    decode_replay_item,
    encode_common_record,
    encode_dma_block,
    encode_replay_item,
)
from digital_twin.types import MeasurementEvent, SensorId, StatusFlag


def event(payload: bytes = b"\x01\x02\x03") -> MeasurementEvent:
    return MeasurementEvent(1, SensorId.BNO085_SHTP, 7, 100, 130, StatusFlag.VALID, payload)


def test_standard_crc32c_vector() -> None:
    assert crc32c(b"123456789") == 0xE3069283


def test_common_record_round_trip_and_corruption() -> None:
    encoded = encode_common_record(event())
    assert len(encoded) % 8 == 0
    assert decode_common_record(encoded) == event()
    damaged = bytearray(encoded)
    damaged[48] ^= 1
    with pytest.raises(ValueError, match="payload CRC"):
        decode_common_record(damaged)


def test_dma_block_round_trip() -> None:
    original = DmaBlock(4, 0, 2, 3, 0x12345678, (event(),))
    encoded = encode_dma_block(original)
    assert len(encoded) == 4096
    assert decode_dma_block(encoded) == original


def test_replay_item_round_trip() -> None:
    encoded = encode_replay_item(event(), 99)
    delay, decoded = decode_replay_item(encoded)
    assert delay == 99
    assert decoded == event()


def test_reserved_status_bits_rejected() -> None:
    invalid = event()
    object.__setattr__(invalid, "status_flags", StatusFlag(1 << 20))
    with pytest.raises(ValueError, match="reserved"):
        encode_common_record(invalid)


def test_header_crc_covers_length() -> None:
    encoded = bytearray(encode_common_record(event()))
    struct.pack_into("<H", encoded, 8, len(encoded) + 8)
    with pytest.raises(ValueError):
        decode_common_record(encoded)


def test_canonical_golden_fixtures() -> None:
    golden = Path(__file__).resolve().parents[2] / "spec" / "golden"
    record_bytes = (golden / "common_record_v1.bin").read_bytes()
    record = decode_common_record(record_bytes)
    assert record.sensor_id == SensorId.BNO085_SHTP
    assert encode_common_record(record) == record_bytes
    block_bytes = (golden / "dma_block_v1.bin").read_bytes()
    assert decode_dma_block(block_bytes).records == (record,)
    delay, replay_record = decode_replay_item((golden / "replay_item_v1.bin").read_bytes())
    assert delay == 5000
    assert replay_record == record
