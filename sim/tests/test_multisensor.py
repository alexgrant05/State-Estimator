from dataclasses import replace

import numpy as np
import pytest

from digital_twin.adis16470 import Adis16470Model
from digital_twin.adxl375 import Adxl375Model, AdxlFaultSchedule, decode_accel, encode_accel
from digital_twin.bmp581 import Bmp581Model, BmpFaultSchedule, decode_event as decode_bmp, encode_pressure_pa, encode_temperature_c
from digital_twin.eskf import InertialEskf
from digital_twin.frames import rocketpy_initial_quaternion
from digital_twin.geodesy import ecef_to_enu, enu_to_ecef, geodetic_to_ecef
from digital_twin.gnss import GenericGnssModel, GnssFaultSchedule
from digital_twin.pipeline import generate_all_events, schedule_aux_spi
from digital_twin.transport import (
    ADXL_INT_SOURCE_COMMAND,
    ADXL_READ_COMMAND,
    BMP_INT_STATUS_COMMAND,
    BMP_READ_COMMAND,
    AdxlAcquisition,
    BmpAcquisition,
    GnssPps,
    GnssSolution,
    read_events,
    write_multi_replay,
)
from digital_twin.truth import analytic_truth
from digital_twin.types import SensorId, StatusFlag


def _pad_then_acceleration(config, acceleration, duration_s=0.2):
    rate = config.simulation.truth_rate_hz
    quaternion = rocketpy_initial_quaternion(config.launch.rail_inclination_deg, config.launch.rail_heading_deg)
    pad = analytic_truth(
        config.simulation.pad_duration_s - 1.0 / rate,
        initial_quaternion=quaternion,
        elevation_msl_m=config.launch.elevation_msl_m,
    )
    motion = analytic_truth(
        duration_s,
        navigation_acceleration_mps2=acceleration,
        initial_quaternion=quaternion,
        elevation_msl_m=config.launch.elevation_msl_m,
    )
    shift = int(config.simulation.pad_duration_s * config.simulation.clock_hz)
    return pad + [replace(sample, ticks=sample.ticks + shift) for sample in motion]


@pytest.mark.parametrize("acceleration_g", [0.0, 1.0, -1.0, 200.0, -200.0])
def test_adxl_scale_vectors(acceleration_g, twin_config):
    gravity = twin_config.simulation.gravity_mps2
    counts, saturated = encode_accel(acceleration_g * gravity, gravity)
    assert not saturated
    assert abs(decode_accel(counts, gravity) - acceleration_g * gravity) <= gravity / 20.5 / 2 + 1e-12


def test_adxl_timing_and_codec(twin_config):
    truth = analytic_truth(0.02)
    config = replace(twin_config.adxl375, noise_density_mg_sqrt_hz=0.0)
    events = Adxl375Model(config, twin_config.simulation, 12).generate(truth)
    assert np.all(np.diff([event.measurement_ticks for event in events]) == 125_000)
    acquisition = AdxlAcquisition.from_payload_bytes(events[0].payload)
    assert AdxlAcquisition.from_payload_bytes(acquisition.payload_bytes()) == acquisition
    transaction = acquisition.transaction_bytes()
    assert len(transaction) == 9
    assert transaction[0] == ADXL_READ_COMMAND
    assert transaction[-2] == ADXL_INT_SOURCE_COMMAND


def test_bmp_raw_scales_negative_temperature_and_timing(twin_config):
    pressure_raw, saturated = encode_pressure_pa(101_325.0)
    acquisition = BmpAcquisition(encode_temperature_c(-20.0), pressure_raw)
    from digital_twin.types import MeasurementEvent

    event = MeasurementEvent(1, SensorId.BMP581, 0, 0, 1, StatusFlag.VALID, acquisition.payload_bytes())
    decoded = decode_bmp(event)
    assert not saturated
    assert decoded.pressure_pa == pytest.approx(101_325.0, abs=1 / 64)
    assert decoded.temperature_c == pytest.approx(-20.0, abs=1 / 2**16)
    transaction = acquisition.transaction_bytes()
    assert len(transaction) == 9
    assert transaction[0] == BMP_READ_COMMAND
    assert transaction[-2] == BMP_INT_STATUS_COMMAND
    events = Bmp581Model(twin_config.bmp581, twin_config.simulation, 13).generate(analytic_truth(0.1))
    assert np.all(np.diff([value.measurement_ticks for value in events]) == 2_000_000)


def test_gnss_canonical_codec_frames_and_rates(twin_config):
    origin = geodetic_to_ecef(twin_config.launch.latitude_deg, twin_config.launch.longitude_deg, twin_config.launch.elevation_msl_m)
    offset = np.array([10.0, -3.0, 5.0])
    assert ecef_to_enu(enu_to_ecef(offset, twin_config.launch.latitude_deg, twin_config.launch.longitude_deg), twin_config.launch.latitude_deg, twin_config.launch.longitude_deg) == pytest.approx(offset)
    events = GenericGnssModel(twin_config.gnss, twin_config.launch, twin_config.simulation, 14).generate(analytic_truth(1.1, elevation_msl_m=twin_config.launch.elevation_msl_m))
    solutions = [event for event in events if event.sensor_id == SensorId.GNSS_SOLUTION]
    pulses = [event for event in events if event.sensor_id == SensorId.GNSS_PPS]
    assert np.all(np.diff([event.measurement_ticks for event in solutions]) == 10_000_000)
    assert np.all(np.abs(np.diff([event.measurement_ticks for event in pulses]) - 100_000_000) <= 12)
    solution = GnssSolution.from_payload_bytes(solutions[0].payload)
    pulse = GnssPps.from_payload_bytes(pulses[0].payload)
    assert len(solution.covariance) == 36
    assert pulse.time_valid
    assert np.linalg.norm(np.asarray(solution.position_ecef_m) - origin) < 20.0


def test_aux_spi_is_serialized_with_adxl_priority(twin_config):
    truth = analytic_truth(0.0)
    adxl = Adxl375Model(twin_config.adxl375, twin_config.simulation, 15).generate(truth)
    bmp = Bmp581Model(twin_config.bmp581, twin_config.simulation, 15).generate(truth)
    scheduled = schedule_aux_spi(adxl + bmp, twin_config)
    assert scheduled[0].sensor_id == SensorId.ADXL375
    assert scheduled[1].sensor_id == SensorId.BMP581
    assert scheduled[1].arrival_ticks > scheduled[0].arrival_ticks


def test_high_g_handoff_uses_adxl(twin_config):
    adis = replace(twin_config.adis16470, accel_noise_rms_mg=0.0, gyro_noise_rms_dps=0.0)
    adxl = replace(twin_config.adxl375, noise_density_mg_sqrt_hz=0.0)
    config = replace(twin_config, adis16470=adis, adxl375=adxl, bmp581=replace(twin_config.bmp581, enabled=False), gnss=replace(twin_config.gnss, enabled=False))
    truth = _pad_then_acceleration(config, np.array([50.0 * config.simulation.gravity_mps2, 0.0, 0.0]))
    events = generate_all_events(truth, config, 16)
    estimator = InertialEskf(config)
    estimates = estimator.run(events)
    assert estimates
    assert estimator.health["high_g_switches_to_adxl"] == 1
    assert estimator.health["high_g_samples"] > 0
    assert np.all(np.isfinite(estimates[-1].velocity_enu_mps))


def test_delayed_gnss_rewinds_and_pps_synchronizes(twin_config):
    adis = replace(twin_config.adis16470, accel_noise_rms_mg=0.0, gyro_noise_rms_dps=0.0)
    gnss = replace(twin_config.gnss, position_sigma_enu_m=np.full(3, 0.1), velocity_sigma_enu_mps=np.full(3, 0.01), latency_jitter_s=0.0)
    config = replace(twin_config, adis16470=adis, adxl375=replace(twin_config.adxl375, enabled=False), bmp581=replace(twin_config.bmp581, enabled=False), gnss=gnss)
    truth = _pad_then_acceleration(config, np.array([0.2, 0.0, 0.1]), duration_s=0.5)
    events = generate_all_events(truth, config, 17)
    estimator = InertialEskf(config)
    estimates = estimator.run(events)
    assert estimates
    assert estimator.health["rewinds"] > 0
    assert estimator.health["gnss_updates_accepted"] > 0
    assert estimator.health["pps_updates"] >= 2
    assert estimates[-1].gps_time_ns is not None


def test_new_sensor_fault_campaign_is_deterministic_and_counted(twin_config):
    truth = analytic_truth(0.3, elevation_msl_m=twin_config.launch.elevation_msl_m)
    adxl_faults = AdxlFaultSchedule(overrun=frozenset({2}), packet_loss=frozenset({3}), stuck_sample=frozenset({4}))
    bmp_faults = BmpFaultSchedule(invalid_status=frozenset({2}), packet_loss=frozenset({3}), pressure_spike_pa={4: 500.0})
    gnss_faults = GnssFaultSchedule(solution_loss=frozenset({1}), invalid_fix=frozenset({2}), pps_loss=frozenset({0}), additional_latency_s={3: 3.0})
    adxl = Adxl375Model(twin_config.adxl375, twin_config.simulation, 20).generate(truth, adxl_faults)
    bmp = Bmp581Model(twin_config.bmp581, twin_config.simulation, 20).generate(truth, bmp_faults)
    gnss = GenericGnssModel(twin_config.gnss, twin_config.launch, twin_config.simulation, 20).generate(truth, gnss_faults)
    assert any(event.status_flags & StatusFlag.OVERRUN for event in adxl)
    assert 3 not in {event.sequence_number for event in adxl}
    assert any(not event.status_flags & StatusFlag.VALID for event in bmp)
    assert 1 not in {event.sequence_number for event in gnss if event.sensor_id == SensorId.GNSS_SOLUTION}
    assert any(event.status_flags & StatusFlag.FIX_INVALID for event in gnss)
    assert adxl == Adxl375Model(twin_config.adxl375, twin_config.simulation, 20).generate(truth, adxl_faults)


def test_multi_replay_is_deterministic_and_round_trips(tmp_path, twin_config):
    truth = analytic_truth(0.1, elevation_msl_m=twin_config.launch.elevation_msl_m)
    events = generate_all_events(truth, twin_config, 21)
    first = tmp_path / "first"
    second = tmp_path / "second"
    first.mkdir()
    second.mkdir()
    first_stats = write_multi_replay(events, first / "events.ndjson", first)
    second_stats = write_multi_replay(events, second / "events.ndjson", second)
    assert first_stats == second_stats
    assert (first / "events.ndjson").read_bytes() == (second / "events.ndjson").read_bytes()
    assert list(read_events(first / "events.ndjson")) == sorted(events, key=lambda item: (item.arrival_ticks, int(item.sensor_id), item.sequence_number))
    for filename in first_stats:
        assert (first / filename).read_bytes() == (second / filename).read_bytes()
