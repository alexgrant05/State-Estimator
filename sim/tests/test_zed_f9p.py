from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from digital_twin.bno085 import Bno085Model
from digital_twin.truth import analytic_truth
from digital_twin.types import SensorId, StatusFlag
from digital_twin.zed_f9p import NAV_CLASS, NAV_COV_ID, NAV_PVT_ID, UbxFrame, ZedF9pModel, ZedMessageAssembler, encode_nav_cov, encode_nav_pvt, encode_tim_tp


def test_reserved_and_active_sensor_ids():
    assert [int(SensorId.ADIS16470), int(SensorId.GNSS_SOLUTION), int(SensorId.GNSS_PPS)] == [1, 4, 5]
    assert [int(SensorId.BNO085_SHTP), int(SensorId.ZED_F9P_UBX), int(SensorId.ZED_F9P_TIMEPULSE)] == [6, 7, 8]


def test_ubx_checksum_and_rejection():
    encoded = encode_tim_tp(86_400_000, 2400).payload_bytes()
    assert UbxFrame.from_bytes(encoded).message_id == 0x01
    damaged = encoded[:-1] + bytes((encoded[-1] ^ 1,))
    with pytest.raises(ValueError, match="checksum"):
        UbxFrame.from_bytes(damaged)


def test_protocol_golden_traces(twin_config):
    fixture_dir = Path(__file__).parent / "fixtures"
    startup = b"".join(packet.payload_bytes() for packet in Bno085Model(twin_config.bno085, twin_config.simulation, 1).startup_trace())
    assert startup == bytes.fromhex((fixture_dir / "golden_bno085_startup.hex").read_text().strip())
    assert encode_tim_tp(86_400_000, 2400).payload_bytes() == bytes.fromhex((fixture_dir / "golden_zed_tim_tp.hex").read_text().strip())


def test_pvt_covariance_assembly_requires_matching_itow(twin_config):
    pvt = encode_nav_pvt(1000, twin_config.launch.latitude_deg, twin_config.launch.longitude_deg, twin_config.launch.elevation_msl_m, np.array([-2.0, 3.0, -4.0]), np.array([1.0, 2.0, 3.0]), np.array([0.1, 0.2, 0.3]), True, True)
    wrong = encode_nav_cov(1200, np.ones(3), np.ones(3))
    correct = encode_nav_cov(1000, np.array([1.0, 2.0, 3.0]), np.array([0.1, 0.2, 0.3]))
    assembler = ZedMessageAssembler(twin_config.launch)
    assert assembler.add(pvt, 2400) is None
    assert assembler.add(wrong, 2400) is None
    fix = assembler.add(correct, 2400)
    assert fix is not None
    assert fix.velocity_enu_mps == pytest.approx([-2.0, 3.0, -4.0])
    assert np.diag(fix.covariance) == pytest.approx([1.0, 4.0, 9.0, 0.01, 0.04, 0.09])


def test_uart_frames_are_serialized_and_rates_are_exact(twin_config):
    events = ZedF9pModel(twin_config.zed_f9p, twin_config.launch, twin_config.simulation, 3).generate(analytic_truth(1.1, elevation_msl_m=twin_config.launch.elevation_msl_m))
    uart = sorted((event for event in events if event.sensor_id == SensorId.ZED_F9P_UBX), key=lambda event: event.arrival_ticks)
    for first, second in zip(uart, uart[1:]):
        assert second.arrival_ticks > first.arrival_ticks
    pvt = [event for event in uart if (lambda frame: frame.message_class == NAV_CLASS and frame.message_id == NAV_PVT_ID)(UbxFrame.from_bytes(event.payload))]
    assert np.all(np.diff([event.measurement_ticks for event in pvt]) == 20_000_000)


def test_four_g_invalidation_and_one_second_recovery(twin_config):
    high = analytic_truth(0.4, navigation_acceleration_mps2=np.array([5.0 * twin_config.simulation.gravity_mps2, 0.0, 0.0]), elevation_msl_m=twin_config.launch.elevation_msl_m)
    low = analytic_truth(1.4, elevation_msl_m=twin_config.launch.elevation_msl_m)
    step = twin_config.simulation.clock_hz // twin_config.simulation.truth_rate_hz
    offset = high[-1].ticks + step
    combined = high + [replace(sample, ticks=sample.ticks + offset) for sample in low]
    events = ZedF9pModel(twin_config.zed_f9p, twin_config.launch, twin_config.simulation, 4).generate(combined)
    pvt = [event for event in events if event.sensor_id == SensorId.ZED_F9P_UBX and UbxFrame.from_bytes(event.payload).message_id == NAV_PVT_ID]
    flags = [bool(event.status_flags & StatusFlag.VALID) for event in sorted(pvt, key=lambda event: event.measurement_ticks)]
    assert any(not value for value in flags)
    assert flags[-1]


def test_nominal_ubx_message_shapes():
    pvt = encode_nav_pvt(1, -10.0, 20.0, 123.0, np.zeros(3), np.ones(3), np.ones(3), True, True)
    cov = encode_nav_cov(1, np.ones(3), np.ones(3))
    assert (pvt.message_class, pvt.message_id, len(pvt.payload)) == (NAV_CLASS, NAV_PVT_ID, 92)
    assert (cov.message_class, cov.message_id, len(cov.payload)) == (NAV_CLASS, NAV_COV_ID, 64)
