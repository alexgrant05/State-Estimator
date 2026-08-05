from dataclasses import replace

import numpy as np

from digital_twin.adis16470 import Adis16470Model, FaultSchedule
from digital_twin.eskf import InertialEskf
from digital_twin.frames import rocketpy_initial_quaternion
from digital_twin.truth import analytic_truth


def _pad_then_motion(config, duration_s=1.0, acceleration=None, velocity=None, omega=None):
    rate = config.simulation.truth_rate_hz
    pad = analytic_truth(
        config.simulation.pad_duration_s - 1.0 / rate,
        initial_quaternion=rocketpy_initial_quaternion(
            config.launch.rail_inclination_deg, config.launch.rail_heading_deg
        ),
        elevation_msl_m=config.launch.elevation_msl_m,
    )
    motion = analytic_truth(
        duration_s,
        initial_velocity_mps=np.zeros(3) if velocity is None else velocity,
        navigation_acceleration_mps2=np.zeros(3) if acceleration is None else acceleration,
        initial_quaternion=rocketpy_initial_quaternion(
            config.launch.rail_inclination_deg, config.launch.rail_heading_deg
        ),
        angular_rate_body_rps=np.zeros(3) if omega is None else omega,
        elevation_msl_m=config.launch.elevation_msl_m,
    )
    shift = int(config.simulation.pad_duration_s * config.simulation.clock_hz)
    return pad + [replace(sample, ticks=sample.ticks + shift) for sample in motion]


def _noiseless_config(twin_config):
    adis = replace(
        twin_config.adis16470,
        accel_noise_rms_mg=0.0,
        gyro_noise_rms_dps=0.0,
        accel_bias_mps2=np.zeros(3),
        gyro_bias_rps=np.zeros(3),
    )
    return replace(twin_config, adis16470=adis)


def test_stationary_pad_initialization_and_propagation(twin_config):
    config = _noiseless_config(twin_config)
    rail_q = rocketpy_initial_quaternion(config.launch.rail_inclination_deg, config.launch.rail_heading_deg)
    truth = analytic_truth(11.0, initial_quaternion=rail_q, elevation_msl_m=config.launch.elevation_msl_m)
    events = Adis16470Model(config.adis16470, config.simulation, 1).generate(truth)
    estimates = InertialEskf(config).run(events)
    assert estimates
    final = estimates[-1]
    assert np.linalg.norm(final.position_enu_m) < 1e-6
    assert np.linalg.norm(final.velocity_enu_mps) < 1e-6
    assert abs(np.linalg.norm(final.q_body_to_nav) - 1.0) < 1e-12
    assert np.max(np.abs(final.covariance - final.covariance.T)) < 1e-10
    assert np.min(np.linalg.eigvalsh(final.covariance)) >= -1e-12


def test_constant_acceleration_propagation_within_quantization_bound(twin_config):
    config = _noiseless_config(twin_config)
    acceleration = np.array([1.0, -0.5, 0.25])
    truth = _pad_then_motion(config, acceleration=acceleration)
    events = Adis16470Model(config.adis16470, config.simulation, 2).generate(truth)
    estimates = InertialEskf(config).run(events)
    reference = {sample.ticks: sample for sample in truth}[estimates[-1].state_ticks]
    # Half an accelerometer LSB integrated for one second on three axes.
    acceleration_bound = np.sqrt(3.0) * config.simulation.gravity_mps2 / 800.0 / 2.0
    assert np.linalg.norm(estimates[-1].velocity_enu_mps - reference.velocity_enu_mps) < acceleration_bound
    assert np.linalg.norm(estimates[-1].position_enu_m - reference.position_enu_m) < acceleration_bound


def test_constant_angular_rate_propagation(twin_config):
    config = _noiseless_config(twin_config)
    truth = _pad_then_motion(config, omega=np.radians(np.array([0.0, 0.0, 10.0])))
    events = Adis16470Model(config.adis16470, config.simulation, 3).generate(truth)
    estimates = InertialEskf(config).run(events)
    reference = {sample.ticks: sample for sample in truth}[estimates[-1].state_ticks]
    from digital_twin.frames import attitude_error_deg

    # Bound includes half a 0.1 deg/s output LSB plus the first decimation window.
    assert attitude_error_deg(estimates[-1].q_body_to_nav, reference.q_body_to_nav) < 0.05


def test_constant_velocity_mechanization(twin_config):
    config = _noiseless_config(twin_config)
    requested_velocity = np.array([2.0, -1.0, 0.5])
    truth = _pad_then_motion(config, velocity=requested_velocity)
    events = Adis16470Model(config.adis16470, config.simulation, 5).generate(truth)
    estimator = InertialEskf(config)
    estimates = []
    truth_by_tick = {sample.ticks: sample for sample in truth}
    seeded = False
    for event in events:
        estimate = estimator.process(event)
        if estimate is None:
            continue
        if not seeded:
            # Position/velocity are externally initialized in a real aided system;
            # seed them here to isolate constant-velocity mechanization.
            estimator.position = truth_by_tick[event.measurement_ticks].position_enu_m.copy()
            estimator.velocity = requested_velocity.copy()
            seeded = True
            continue
        estimates.append(estimate)
    reference = truth_by_tick[estimates[-1].state_ticks]
    assert np.linalg.norm(estimates[-1].velocity_enu_mps - reference.velocity_enu_mps) < 1e-6
    assert np.linalg.norm(estimates[-1].position_enu_m - reference.position_enu_m) < 1e-6


def test_faults_are_counted_and_filter_remains_finite(twin_config):
    config = _noiseless_config(twin_config)
    rail_q = rocketpy_initial_quaternion(config.launch.rail_inclination_deg, config.launch.rail_heading_deg)
    truth = analytic_truth(10.2, initial_quaternion=rail_q)
    faults = FaultSchedule(
        checksum_corruption=frozenset({5002}),
        diagnostic_error=frozenset({5003}),
        duplicate_counter=frozenset({5004}),
        packet_loss=frozenset({5005}),
    )
    events = Adis16470Model(config.adis16470, config.simulation, 4).generate(truth, faults)
    estimator = InertialEskf(config)
    estimates = estimator.run(events)
    assert estimator.health["checksum_failures"] == 1
    assert estimator.health["diagnostic_failures"] == 1
    assert estimator.health["sequence_discontinuities"] >= 1
    assert estimator.health["counter_discontinuities"] >= 1
    assert estimates
    assert np.all(np.isfinite(estimates[-1].position_enu_m))
    assert np.all(np.isfinite(estimates[-1].covariance))
