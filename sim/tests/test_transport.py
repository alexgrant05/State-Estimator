import hashlib
import json
from pathlib import Path

from digital_twin.transport import AdisBurst, adis_checksum, event_from_json, event_to_json
from digital_twin.types import MeasurementEvent, SensorId, StatusFlag


def test_datasheet_byte_sum_checksum_and_transaction_shape():
    data = (0x1234, 0x0001, 0xFFFE, 0x7FFF, 0x8000, 0x0102, 0xABCD, 0x00FA, 0xFFFE)
    expected = sum(byte for word in data for byte in ((word >> 8) & 0xFF, word & 0xFF)) & 0xFFFF
    assert adis_checksum(data) == expected == 0x08B4
    burst = AdisBurst.create(data[0], tuple(data[1:4]), tuple(data[4:7]), data[7], data[8])
    assert len(burst.payload_bytes()) == 20
    assert len(burst.transaction_bytes()) * 8 == 176
    assert AdisBurst.from_transaction_bytes(burst.transaction_bytes()) == burst


def test_golden_transaction_fixture():
    fixture = Path(__file__).parent / "fixtures" / "golden_adis_transaction.hex"
    expected = bytes.fromhex(fixture.read_text(encoding="ascii").strip())
    burst = AdisBurst.create(
        0x1234,
        (0x0001, 0xFFFE, 0x7FFF),
        (0x8000, 0x0102, 0xABCD),
        0x00FA,
        0xFFFE,
    )
    assert burst.transaction_bytes() == expected
    assert hashlib.sha256(expected).hexdigest() == "e33ac34a9563e060a0116bc7db99aed8d1097168b47d57a1fdaf9a3d2bc416d0"


def test_logical_event_json_round_trip():
    burst = AdisBurst.create(0, (1, 2, 3), (4, 5, 6), 250, 7)
    event = MeasurementEvent(1, SensorId.ADIS16470, 99, 1000, 18600, StatusFlag.VALID, burst.payload_bytes())
    record = json.loads(json.dumps(event_to_json(event)))
    assert event_from_json(record) == event

