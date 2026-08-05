from dataclasses import replace

import numpy as np
import pytest

from digital_twin.adis16470 import (
    ACCEL_LSB_PER_G,
    Adis16470Model,
    FaultSchedule,
    decode_accel_mps2,
    decode_event,
    decode_gyro_rps,
    encode_accel_mps2,
    encode_gyro_rps,
)
from digital_twin.transport import AdisBurst
from digital_twin.truth import analytic_truth
from digital_twin.types import StatusFlag


@pytest.mark.parametrize("acceleration_g", [0.0, 1.0, -1.0, 40.0, -40.0])
def test_accel_known_vectors(acceleration_g, twin_config):
    gravity = twin_config.simulation.gravity_mps2
    word, saturated = encode_accel_mps2(acceleration_g * gravity, gravity)
    assert not saturated
    assert decode_accel_mps2(word, gravity) == pytest.approx(acceleration_g * gravity, abs=gravity / ACCEL_LSB_PER_G / 2)


@pytest.mark.parametrize("rate_dps", [0.0, 0.1, -0.1, 2000.0, -2000.0])
def test_gyro_known_vectors(rate_dps):
    word, saturated = encode_gyro_rps(np.radians(rate_dps))
    assert not saturated
    assert decode_gyro_rps(word) == pytest.approx(np.radians(rate_dps), abs=np.radians(0.05))


def test_saturation_is_clipped_and_reported(twin_config):
    truth = analytic_truth(
        0.01,
        navigation_acceleration_mps2=np.array([50.0 * twin_config.simulation.gravity_mps2, 0.0, 0.0]),
    )
    noiseless = replace(twin_config.adis16470, accel_noise_rms_mg=0.0, gyro_noise_rms_dps=0.0, dec_rate=0)
    events = Adis16470Model(noiseless, twin_config.simulation, 1).generate(truth)
    assert any(event.status_flags & StatusFlag.SATURATED for event in events)
    measurement = decode_event(events[0], noiseless, twin_config.simulation)
    assert measurement.accel_body_mps2[0] == pytest.approx(40.0 * twin_config.simulation.gravity_mps2)


@pytest.mark.parametrize("dec_rate", [0, 3, 19, 1999])
def test_exact_decimation_timing(dec_rate, twin_config):
    duration = 2.1 if dec_rate == 1999 else 0.1
    truth = analytic_truth(duration)
    sensor = replace(twin_config.adis16470, dec_rate=dec_rate, accel_noise_rms_mg=0.0, gyro_noise_rms_dps=0.0)
    events = Adis16470Model(sensor, twin_config.simulation, 5).generate(truth)
    expected = (dec_rate + 1) * twin_config.simulation.clock_hz // twin_config.simulation.truth_rate_hz
    assert len(events) >= 2
    assert np.all(np.diff([event.measurement_ticks for event in events]) == expected)
    assert all(event.arrival_ticks - event.measurement_ticks == 17_600 for event in events)


def test_deterministic_seed_and_independent_runs(twin_config):
    truth = analytic_truth(0.1)
    first = Adis16470Model(twin_config.adis16470, twin_config.simulation, 42).generate(truth)
    second = Adis16470Model(twin_config.adis16470, twin_config.simulation, 42).generate(truth)
    third = Adis16470Model(twin_config.adis16470, twin_config.simulation, 43).generate(truth)
    assert first == second
    assert [event.payload for event in first] != [event.payload for event in third]


def test_core_fault_schedule(twin_config):
    truth = analytic_truth(0.1)
    faults = FaultSchedule(
        checksum_corruption=frozenset({2}),
        diagnostic_error=frozenset({3}),
        duplicate_counter=frozenset({4}),
        skipped_counter=frozenset({5}),
        packet_loss=frozenset({6}),
    )
    events = Adis16470Model(twin_config.adis16470, twin_config.simulation, 42).generate(truth, faults)
    by_sequence = {event.sequence_number: event for event in events}
    assert not AdisBurst.from_payload_bytes(by_sequence[2].payload).valid_checksum()
    assert by_sequence[2].status_flags & StatusFlag.CHECKSUM_ERROR
    assert not by_sequence[2].status_flags & StatusFlag.VALID
    assert AdisBurst.from_payload_bytes(by_sequence[3].payload).diag_stat != 0
    assert by_sequence[3].status_flags & StatusFlag.DIAGNOSTIC_ERROR
    assert 6 not in by_sequence


def test_noise_statistics_over_200_seeds(twin_config):
    truth = analytic_truth(0.05)
    readings = []
    for seed in range(200):
        events = Adis16470Model(twin_config.adis16470, twin_config.simulation, seed).generate(truth)
        readings.extend(decode_event(event, twin_config.adis16470, twin_config.simulation).gyro_body_rps[0] for event in events)
    values = np.asarray(readings)
    expected_sigma = np.radians(twin_config.adis16470.gyro_noise_rms_dps) / np.sqrt(twin_config.adis16470.dec_rate + 1)
    assert abs(np.mean(values)) < 3.0 * expected_sigma / np.sqrt(len(values)) + np.radians(0.05)
    assert np.std(values, ddof=1) == pytest.approx(expected_sigma, rel=0.25)
