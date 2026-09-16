from dataclasses import replace

import numpy as np
import pytest

from digital_twin.bno085 import (
    ACCELEROMETER_REPORT,
    GAME_ROTATION_VECTOR_REPORT,
    UNCALIBRATED_GYROSCOPE_REPORT,
    Bno085Model,
    BnoFaultSchedule,
    BnoShtpPacket,
    decode_event,
    decode_packet,
    encode_sensor_report,
)
from digital_twin.truth import analytic_truth
from digital_twin.types import SensorId, StatusFlag


@pytest.mark.parametrize("value", [0.0, 1.0, -1.0, 8.0, -8.0])
def test_accel_q8_vectors(value):
    packet = BnoShtpPacket(3, 0, encode_sensor_report(ACCELEROMETER_REPORT, 0, 3, np.array([value, 0.0, 0.0])))
    assert decode_packet(packet).vector[0] == pytest.approx(value, abs=0.5 / 256)


@pytest.mark.parametrize("value", [0.0, 1.0, -1.0, 30.0, -30.0])
def test_gyro_q9_vectors(value):
    packet = BnoShtpPacket(3, 0, encode_sensor_report(UNCALIBRATED_GYROSCOPE_REPORT, 0, 3, np.array([value, 0.0, 0.0]), np.zeros(3)))
    assert decode_packet(packet).vector[0] == pytest.approx(value, abs=0.5 / 512)


def test_game_rotation_q14_and_shtp_round_trip():
    quaternion = np.array([0.5, 0.5, 0.5, 0.5])
    packet = BnoShtpPacket(3, 255, encode_sensor_report(GAME_ROTATION_VECTOR_REPORT, 17, 3, quaternion), continuation=True)
    encoded = packet.payload_bytes()
    assert BnoShtpPacket.from_bytes(encoded) == packet
    assert decode_packet(packet).quaternion_body_to_nav == pytest.approx(quaternion, abs=0.5 / (1 << 14))


def test_startup_timing_rates_and_variable_transfer(twin_config):
    truth = analytic_truth(0.13)
    events = Bno085Model(twin_config.bno085, twin_config.simulation, 1).generate(truth)
    assert min(event.measurement_ticks for event in events) == 9_400_000
    by_report = {report: [] for report in (ACCELEROMETER_REPORT, UNCALIBRATED_GYROSCOPE_REPORT, GAME_ROTATION_VECTOR_REPORT)}
    for event in events:
        by_report[decode_event(event, twin_config.bno085).report_id].append(event)
    assert np.all(np.diff([event.measurement_ticks for event in by_report[ACCELEROMETER_REPORT]]) == 200_000)
    assert np.all(np.diff([event.measurement_ticks for event in by_report[UNCALIBRATED_GYROSCOPE_REPORT]]) == 250_000)
    assert np.all(np.diff([event.measurement_ticks for event in by_report[GAME_ROTATION_VECTOR_REPORT]]) == 1_000_000)
    sizes = {len(event.payload) for event in events}
    assert len(sizes) == 3
    assert all(event.arrival_ticks > event.measurement_ticks for event in events)


def test_saturation_and_faults(twin_config):
    truth = analytic_truth(0.12, navigation_acceleration_mps2=np.array([20 * twin_config.simulation.gravity_mps2, 0.0, 0.0]))
    faults = BnoFaultSchedule(packet_loss=frozenset({2}), duplicate_channel_sequence=frozenset({3}), truncate=frozenset({4}), reset=frozenset({5}))
    events = Bno085Model(twin_config.bno085, twin_config.simulation, 2).generate(truth, faults)
    assert any(second.sequence_number - first.sequence_number > 1 for first, second in zip(events, events[1:]))
    assert any(event.status_flags & StatusFlag.SATURATED for event in events)
    assert any(event.status_flags & StatusFlag.DIAGNOSTIC_ERROR for event in events)


def test_deterministic_stream_and_independent_sensor_id(twin_config):
    truth = analytic_truth(0.12)
    first = Bno085Model(twin_config.bno085, twin_config.simulation, 42).generate(truth)
    second = Bno085Model(twin_config.bno085, twin_config.simulation, 42).generate(truth)
    assert first == second
    assert all(event.sensor_id == SensorId.BNO085_SHTP for event in first)


def test_mounting_is_applied_to_vectors_and_diagnostic_attitude(twin_config):
    mounting = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    config = replace(twin_config.bno085, sensor_to_body=mounting)
    truth = analytic_truth(0.11, navigation_acceleration_mps2=np.array([1.0, 0.0, 0.0]))
    decoded = [decode_event(event, config) for event in Bno085Model(config, twin_config.simulation, 9).generate(truth)]
    acceleration = next(report for report in decoded if report.report_id == ACCELEROMETER_REPORT)
    attitude = next(report for report in decoded if report.report_id == GAME_ROTATION_VECTOR_REPORT)
    assert acceleration.vector == pytest.approx([1.0, 0.0, twin_config.simulation.gravity_mps2], abs=0.03)
    assert attitude.quaternion_body_to_nav == pytest.approx([1.0, 0.0, 0.0, 0.0], abs=1e-4)


def test_startup_golden_trace_shape(twin_config):
    trace = Bno085Model(twin_config.bno085, twin_config.simulation, 1).startup_trace()
    assert len(trace) == 7
    assert trace[0].channel == 0
    assert [packet.cargo[1] for packet in trace[-3:]] == [0x01, 0x07, 0x08]
    assert all(len(packet.cargo) == 17 for packet in trace[-3:])
