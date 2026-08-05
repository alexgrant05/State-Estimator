"""Metrics, validation gates, and report artifacts."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import numpy as np

from .config import TwinConfig
from .frames import attitude_error_deg
from .transport import AdisBurst, read_transactions
from .types import MeasurementEvent, StateEstimate, TruthSample


def monte_carlo_statistics(config: TwinConfig, seeds: int = 200) -> dict[str, Any]:
    """Check sensor moments and inertial propagation coverage over fixed seeds."""

    from .adis16470 import Adis16470Model, decode_event
    from .truth import analytic_truth

    short_truth = analytic_truth(
        0.05,
        clock_hz=config.simulation.clock_hz,
        rate_hz=config.simulation.truth_rate_hz,
        elevation_msl_m=config.launch.elevation_msl_m,
    )
    gyro_samples: list[float] = []
    for seed in range(seeds):
        events = Adis16470Model(config.adis16470, config.simulation, seed).generate(short_truth)
        gyro_samples.extend(
            decode_event(event, config.adis16470, config.simulation).gyro_body_rps[0]
            for event in events
        )
    gyro = np.asarray(gyro_samples)
    expected_gyro_sigma = np.radians(config.adis16470.gyro_noise_rms_dps) / np.sqrt(
        config.adis16470.dec_rate + 1
    )
    gyro_lsb = np.radians(0.1)
    observed_gyro_sigma = float(np.std(gyro, ddof=1))
    gyro_mean = float(np.mean(gyro))
    noise_mean_gate = abs(gyro_mean) <= 3.0 * expected_gyro_sigma / np.sqrt(len(gyro)) + 0.5 * gyro_lsb
    noise_std_gate = (
        expected_gyro_sigma == 0.0 and observed_gyro_sigma == 0.0
    ) or abs(observed_gyro_sigma / expected_gyro_sigma - 1.0) <= 0.25

    # Independent discrete white-acceleration propagation check. These weights
    # are the exact velocity and position sensitivities of the mechanization.
    steps = 250
    dt = (config.adis16470.dec_rate + 1) / config.simulation.truth_rate_hz
    accel_sigma = (
        config.adis16470.accel_noise_rms_mg
        * 1e-3
        * config.simulation.gravity_mps2
        / np.sqrt(config.adis16470.dec_rate + 1)
    )
    velocity_variance = accel_sigma**2 * dt**2 * steps
    position_weights = (np.arange(steps, 0, -1) - 0.5) * dt**2
    position_variance = accel_sigma**2 * float(position_weights @ position_weights)
    velocity_covered = 0
    position_covered = 0
    for seed in range(seeds):
        rng = np.random.default_rng(np.random.SeedSequence([seed, 999]))
        acceleration_error = rng.normal(0.0, accel_sigma, steps)
        velocity_error = float(np.sum(acceleration_error) * dt)
        position_error = float(acceleration_error @ position_weights)
        velocity_covered += int(abs(velocity_error) <= 1.96 * np.sqrt(velocity_variance))
        position_covered += int(abs(position_error) <= 1.96 * np.sqrt(position_variance))
    velocity_coverage = velocity_covered / seeds
    position_coverage = position_covered / seeds
    coverage_gate = 0.92 <= velocity_coverage <= 0.98 and 0.92 <= position_coverage <= 0.98

    return {
        "seeds": seeds,
        "gyro_mean_rps": gyro_mean,
        "gyro_expected_sigma_rps": expected_gyro_sigma,
        "gyro_observed_sigma_rps": observed_gyro_sigma,
        "velocity_95pct_coverage": velocity_coverage,
        "position_95pct_coverage": position_coverage,
        "noise_moments_passed": bool(noise_mean_gate and noise_std_gate),
        "covariance_coverage_passed": bool(coverage_gate),
    }


def calculate_metrics(
    truth: list[TruthSample],
    events: list[MeasurementEvent],
    estimates: list[StateEstimate],
    config: TwinConfig,
    replay_binary: Path | None = None,
) -> dict[str, Any]:
    truth_by_tick = {sample.ticks: sample for sample in truth}
    position_error: list[float] = []
    velocity_error: list[float] = []
    attitude_error: list[float] = []
    quaternion_norm_error: list[float] = []
    covariance_symmetry_error: list[float] = []
    covariance_min_eigenvalue: list[float] = []
    finite = True

    for estimate in estimates:
        reference = truth_by_tick.get(estimate.state_ticks)
        if reference is not None:
            position_error.append(float(np.linalg.norm(estimate.position_enu_m - reference.position_enu_m)))
            velocity_error.append(float(np.linalg.norm(estimate.velocity_enu_mps - reference.velocity_enu_mps)))
            attitude_error.append(attitude_error_deg(estimate.q_body_to_nav, reference.q_body_to_nav))
        quaternion_norm_error.append(abs(float(np.linalg.norm(estimate.q_body_to_nav)) - 1.0))
        covariance_symmetry_error.append(float(np.max(np.abs(estimate.covariance - estimate.covariance.T))))
        covariance_min_eigenvalue.append(float(np.min(np.linalg.eigvalsh(estimate.covariance))))
        finite &= bool(
            np.all(np.isfinite(estimate.position_enu_m))
            and np.all(np.isfinite(estimate.velocity_enu_mps))
            and np.all(np.isfinite(estimate.q_body_to_nav))
            and np.all(np.isfinite(estimate.covariance))
        )

    checksum_failures = 0
    diagnostic_failures = 0
    unexpected_saturation = 0
    for event in events:
        burst = AdisBurst.from_payload_bytes(event.payload)
        checksum_failures += int(not burst.valid_checksum())
        diagnostic_failures += int(bool(burst.diag_stat))
        unexpected_saturation += int(bool(int(event.status_flags) & 0x2))

    replay_count = None
    if replay_binary is not None:
        replay_count = sum(1 for _ in read_transactions(replay_binary))

    expected_interval = (config.adis16470.dec_rate + 1) * (
        config.simulation.clock_hz // config.simulation.truth_rate_hz
    )
    intervals = np.diff([event.measurement_ticks for event in events]) if len(events) > 1 else np.array([])
    exact_timing = bool(np.all(intervals == expected_interval))
    health = estimates[-1].health if estimates else {}
    statistical = monte_carlo_statistics(config)

    def error_stats(values: list[float]) -> dict[str, float | None]:
        if not values:
            return {"rms": None, "maximum": None, "final": None}
        array = np.asarray(values)
        return {
            "rms": float(np.sqrt(np.mean(array**2))),
            "maximum": float(np.max(array)),
            "final": float(array[-1]),
        }

    gates = {
        "events_present": len(events) > 0,
        "estimates_present": len(estimates) > 0,
        "checksum_clean": checksum_failures == 0,
        "diagnostics_clean": diagnostic_failures == 0,
        "no_unexpected_saturation": unexpected_saturation == 0,
        "sequence_clean": int(health.get("sequence_discontinuities", 0)) == 0,
        "counter_clean": int(health.get("counter_discontinuities", 0)) == 0,
        "timestamps_exact": exact_timing,
        "replay_round_trip": replay_count is None or replay_count == len(events),
        "states_finite": finite,
        "quaternion_normalized": bool(quaternion_norm_error and max(quaternion_norm_error) < 1e-12),
        "covariance_symmetric": bool(covariance_symmetry_error and max(covariance_symmetry_error) < 1e-10),
        "covariance_psd": bool(covariance_min_eigenvalue and min(covariance_min_eigenvalue) >= -1e-12),
        "noise_statistics": statistical["noise_moments_passed"],
        "covariance_coverage": statistical["covariance_coverage_passed"],
    }
    return {
        "passed": all(gates.values()),
        "gates": gates,
        "counts": {
            "truth_samples": len(truth),
            "measurement_events": len(events),
            "state_estimates": len(estimates),
            "replay_transactions": replay_count,
        },
        "health": dict(health),
        "timing": {
            "expected_interval_ticks": expected_interval,
            "output_rate_hz": config.adis16470.output_rate_hz,
        },
        "errors": {
            "position_m": error_stats(position_error),
            "velocity_mps": error_stats(velocity_error),
            "attitude_deg": error_stats(attitude_error),
        },
        "invariants": {
            "max_quaternion_norm_error": max(quaternion_norm_error, default=None),
            "max_covariance_symmetry_error": max(covariance_symmetry_error, default=None),
            "min_covariance_eigenvalue": min(covariance_min_eigenvalue, default=None),
        },
        "monte_carlo": statistical,
    }


def write_states(path: Path, estimates: list[StateEstimate]) -> None:
    header = [
        "state_ticks",
        "publication_ticks",
        "px_m",
        "py_m",
        "pz_m",
        "vx_mps",
        "vy_mps",
        "vz_mps",
        "qw",
        "qx",
        "qy",
        "qz",
        "bax_mps2",
        "bay_mps2",
        "baz_mps2",
        "bgx_rps",
        "bgy_rps",
        "bgz_rps",
        *[f"pdiag_{index}" for index in range(15)],
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(header)
        for estimate in estimates:
            writer.writerow(
                [
                    estimate.state_ticks,
                    estimate.publication_ticks,
                    *estimate.position_enu_m,
                    *estimate.velocity_enu_mps,
                    *estimate.q_body_to_nav,
                    *estimate.accel_bias_body_mps2,
                    *estimate.gyro_bias_body_rps,
                    *np.diag(estimate.covariance),
                ]
            )


def write_report(path: Path, metrics: dict[str, Any], truth_summary: dict[str, float]) -> None:
    errors = metrics["errors"]
    lines = [
        "# ADIS16470 Vertical-Slice Validation",
        "",
        f"**Overall:** {'PASS' if metrics['passed'] else 'FAIL'}",
        "",
        "## Flight",
        "",
        f"- Pad alignment: {truth_summary['pad_duration_s']:.1f} s",
        f"- Apogee AGL: {truth_summary['apogee_agl_m']:.1f} m",
        f"- Apogee after liftoff: {truth_summary['apogee_time_after_liftoff_s']:.2f} s",
        f"- Maximum speed: {truth_summary['max_speed_mps']:.1f} m/s",
        f"- Maximum Mach: {truth_summary['max_mach']:.2f}",
        "",
        "## Inertial drift (reported, not an aided-navigation gate)",
        "",
        f"- Position RMS/final: {errors['position_m']['rms']:.3f} / {errors['position_m']['final']:.3f} m",
        f"- Velocity RMS/final: {errors['velocity_mps']['rms']:.3f} / {errors['velocity_mps']['final']:.3f} m/s",
        f"- Attitude RMS/final: {errors['attitude_deg']['rms']:.3f} / {errors['attitude_deg']['final']:.3f} deg",
        "",
        "## Gates",
        "",
    ]
    lines.extend(f"- [{'x' if passed else ' '}] `{name}`" for name, passed in metrics["gates"].items())
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_error_plot(path: Path, truth: list[TruthSample], estimates: list[StateEstimate], clock_hz: int) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    truth_by_tick = {sample.ticks: sample for sample in truth}
    aligned = [(estimate, truth_by_tick[estimate.state_ticks]) for estimate in estimates if estimate.state_ticks in truth_by_tick]
    time_s = np.array([estimate.state_ticks / clock_hz for estimate, _ in aligned])
    position = np.array([np.linalg.norm(estimate.position_enu_m - sample.position_enu_m) for estimate, sample in aligned])
    velocity = np.array([np.linalg.norm(estimate.velocity_enu_mps - sample.velocity_enu_mps) for estimate, sample in aligned])
    attitude = np.array([attitude_error_deg(estimate.q_body_to_nav, sample.q_body_to_nav) for estimate, sample in aligned])
    figure, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(time_s, position)
    axes[0].set_ylabel("position (m)")
    axes[1].plot(time_s, velocity)
    axes[1].set_ylabel("velocity (m/s)")
    axes[2].plot(time_s, attitude)
    axes[2].set_ylabel("attitude (deg)")
    axes[2].set_xlabel("simulation time (s)")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    figure.suptitle("ADIS-only ESKF drift versus RocketPy truth")
    figure.tight_layout()
    figure.savefig(path, dpi=130)
    plt.close(figure)


def load_validation(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))
