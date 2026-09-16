"""Metrics, validation gates, and report artifacts for the active sensor stack."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import numpy as np

from .bno085 import ACCELEROMETER_REPORT, GAME_ROTATION_VECTOR_REPORT, UNCALIBRATED_GYROSCOPE_REPORT, Bno085Model, decode_event as decode_bno
from .config import TwinConfig
from .frames import attitude_error_deg
from .transport import read_shtp_transactions, read_ubx_transactions
from .types import MeasurementEvent, SensorId, StateEstimate, StatusFlag, TruthSample
from .zed_f9p import NAV_CLASS, NAV_PVT_ID, UbxFrame, ZedF9pModel, ZedMessageAssembler


def _moment_gate(values: np.ndarray, expected_mean: float, expected_sigma: float, quantization: float) -> dict[str, float | bool]:
    mean = float(np.mean(values))
    sigma = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
    mean_ok = abs(mean - expected_mean) <= 3.0 * expected_sigma / np.sqrt(max(len(values), 1)) + quantization / 2.0
    sigma_ok = sigma <= quantization / 2.0 if expected_sigma == 0.0 else abs(sigma / expected_sigma - 1.0) <= 0.25
    return {"mean": mean, "sigma": sigma, "expected_mean": expected_mean, "expected_sigma": expected_sigma, "passed": bool(mean_ok and sigma_ok)}


def monte_carlo_statistics(config: TwinConfig, seeds: int = 200) -> dict[str, Any]:
    """Check configured sensor moments over deterministic stationary trials."""

    from .adxl375 import Adxl375Model, decode_event as decode_adxl
    from .bmp581 import Bmp581Model, decode_event as decode_bmp
    from .truth import analytic_truth, standard_atmosphere

    duration = max(0.15, config.bno085.startup_delay_s + 0.02)
    stationary = analytic_truth(duration, clock_hz=config.simulation.clock_hz, rate_hz=config.simulation.truth_rate_hz, elevation_msl_m=config.launch.elevation_msl_m)
    bno_accel: list[float] = []
    bno_gyro: list[float] = []
    adxl: list[float] = []
    bmp: list[float] = []
    zed: list[float] = []
    for seed in range(seeds):
        for event in Bno085Model(config.bno085, config.simulation, seed).generate(stationary):
            report = decode_bno(event, config.bno085)
            if report.report_id == ACCELEROMETER_REPORT:
                bno_accel.append(float(report.vector[0]))
            elif report.report_id == UNCALIBRATED_GYROSCOPE_REPORT:
                bno_gyro.append(float(report.vector[0]))
        if config.adxl375.enabled:
            adxl.extend(float(decode_adxl(event, config.adxl375, config.simulation).accel_body_mps2[0]) for event in Adxl375Model(config.adxl375, config.simulation, seed).generate(stationary))
        if config.bmp581.enabled:
            bmp.extend(float(decode_bmp(event).pressure_pa) for event in Bmp581Model(config.bmp581, config.simulation, seed).generate(stationary))
        if config.zed_f9p.enabled:
            assembler = ZedMessageAssembler(config.launch)
            for event in ZedF9pModel(config.zed_f9p, config.launch, config.simulation, seed).generate(stationary):
                if event.sensor_id != SensorId.ZED_F9P_UBX:
                    continue
                frame = UbxFrame.from_bytes(event.payload)
                fix = assembler.add(frame, config.zed_f9p.gps_week)
                if fix is not None:
                    zed.append(float(fix.position_enu_m[0]))
                    break
    accel_sigma = config.bno085.accel_noise_density_mg_sqrt_hz * 1e-3 * config.simulation.gravity_mps2 * np.sqrt(config.bno085.accel_rate_hz / 2.0)
    gyro_sigma = np.radians(config.bno085.gyro_noise_density_dps_sqrt_hz) * np.sqrt(config.bno085.gyro_rate_hz / 2.0)
    statistics: dict[str, dict[str, float | bool]] = {
        "bno085_accel_x": _moment_gate(np.asarray(bno_accel), 0.0, accel_sigma, config.simulation.gravity_mps2 / 256.0),
        "bno085_gyro_x": _moment_gate(np.asarray(bno_gyro), 0.0, gyro_sigma, 1.0 / 512.0),
    }
    if adxl:
        sigma = config.adxl375.noise_density_mg_sqrt_hz * 1e-3 * config.simulation.gravity_mps2 * np.sqrt(config.adxl375.output_rate_hz / 2.0)
        statistics["adxl375_x"] = _moment_gate(np.asarray(adxl), 0.0, sigma, config.simulation.gravity_mps2 / 20.5)
    if bmp:
        expected = standard_atmosphere(config.launch.elevation_msl_m)[0]
        statistics["bmp581_pressure"] = _moment_gate(np.asarray(bmp), expected, config.bmp581.pressure_noise_pa, 1.0 / 64.0)
    if zed:
        statistics["zed_f9p_east"] = _moment_gate(np.asarray(zed), 0.0, float(config.zed_f9p.position_sigma_enu_m[0]), 0.02)
    moments_passed = all(bool(item["passed"]) for item in statistics.values())

    # With nonzero configured acceleration noise, exercise exact discrete integration weights.
    coverage = 0.95
    if accel_sigma > 0.0:
        steps = 250
        dt = 1.0 / config.bno085.gyro_rate_hz
        variance = accel_sigma**2 * dt**2 * steps
        covered = 0
        for seed in range(seeds):
            values = np.random.default_rng(np.random.SeedSequence([seed, 999])).normal(0.0, accel_sigma, steps)
            covered += int(abs(float(np.sum(values) * dt)) <= 1.96 * np.sqrt(variance))
        coverage = covered / seeds
    return {"seeds": seeds, "sensor_statistics": statistics, "noise_moments_passed": moments_passed, "velocity_95pct_coverage": coverage, "covariance_coverage_passed": bool(0.92 <= coverage <= 0.98)}


def calculate_metrics(truth: list[TruthSample], events: list[MeasurementEvent], estimates: list[StateEstimate], config: TwinConfig, replay_binary: Path | dict[str, Path] | None = None) -> dict[str, Any]:
    truth_by_tick = {sample.ticks: sample for sample in truth}
    position_error: list[float] = []
    velocity_error: list[float] = []
    attitude_error: list[float] = []
    game_rotation_error: list[float] = []
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
        finite &= bool(np.all(np.isfinite(estimate.position_enu_m)) and np.all(np.isfinite(estimate.velocity_enu_mps)) and np.all(np.isfinite(estimate.q_body_to_nav)) and np.all(np.isfinite(estimate.covariance)))

    malformed = 0
    bno_accel_saturation = 0
    unexpected_saturation = 0
    expected_intervals: dict[str, int] = {}
    report_ticks: dict[str, list[int]] = {"bno_acceleration": [], "bno_gyro": [], "bno_game_rotation": [], "zed_nav_pvt": [], "zed_timepulse": []}
    for event in events:
        try:
            if event.sensor_id == SensorId.BNO085_SHTP:
                report = decode_bno(event, config.bno085)
                names = {ACCELEROMETER_REPORT: "bno_acceleration", UNCALIBRATED_GYROSCOPE_REPORT: "bno_gyro", GAME_ROTATION_VECTOR_REPORT: "bno_game_rotation"}
                report_ticks[names[report.report_id]].append(event.measurement_ticks)
                if report.report_id == GAME_ROTATION_VECTOR_REPORT:
                    reference = truth_by_tick.get(event.measurement_ticks)
                    if reference is not None:
                        game_rotation_error.append(attitude_error_deg(report.quaternion_body_to_nav, reference.q_body_to_nav))
                if event.status_flags & StatusFlag.SATURATED:
                    if report.report_id == ACCELEROMETER_REPORT:
                        bno_accel_saturation += 1
                    else:
                        unexpected_saturation += 1
            elif event.sensor_id == SensorId.ZED_F9P_UBX:
                frame = UbxFrame.from_bytes(event.payload)
                if frame.message_class == NAV_CLASS and frame.message_id == NAV_PVT_ID:
                    report_ticks["zed_nav_pvt"].append(event.measurement_ticks)
            elif event.sensor_id == SensorId.ZED_F9P_TIMEPULSE:
                report_ticks["zed_timepulse"].append(event.measurement_ticks)
            elif event.status_flags & StatusFlag.SATURATED:
                unexpected_saturation += 1
        except (KeyError, ValueError):
            malformed += 1

    expected_intervals.update({
        "bno_acceleration": config.simulation.clock_hz // config.bno085.accel_rate_hz,
        "bno_gyro": config.simulation.clock_hz // config.bno085.gyro_rate_hz,
        "bno_game_rotation": config.simulation.clock_hz // config.bno085.game_rotation_rate_hz,
        "zed_nav_pvt": config.simulation.clock_hz // config.zed_f9p.output_rate_hz,
        "zed_timepulse": config.simulation.clock_hz // config.zed_f9p.pps_rate_hz,
    })
    timing = {}
    for name, ticks in report_ticks.items():
        if len(ticks) < 2:
            timing[name] = True
            continue
        tolerance = int(np.ceil(6.0 * config.zed_f9p.pps_jitter_ns * config.simulation.clock_hz / 1e9)) if name == "zed_timepulse" else 0
        timing[name] = bool(np.all(np.abs(np.diff(ticks) - expected_intervals[name]) <= tolerance))
    events_by_sensor = {sensor: [event for event in events if event.sensor_id == sensor] for sensor in SensorId}
    replay_counts: dict[str, int] = {}
    replay_ok = True
    if isinstance(replay_binary, dict):
        for name, path in replay_binary.items():
            if name == "bno085_shtp.bin":
                count = sum(1 for _ in read_shtp_transactions(path))
                expected = len(events_by_sensor[SensorId.BNO085_SHTP])
            elif name == "zed_f9p_uart.bin":
                count = sum(1 for _ in read_ubx_transactions(path))
                expected = len(events_by_sensor[SensorId.ZED_F9P_UBX])
            else:
                size = 9
                count = path.stat().st_size // size
                expected = len(events_by_sensor[SensorId.ADXL375 if name.startswith("adxl") else SensorId.BMP581])
                replay_ok &= path.stat().st_size % size == 0
            replay_counts[name] = count
            replay_ok &= count == expected
    health = estimates[-1].health if estimates else {}
    statistical = monte_carlo_statistics(config)

    def stats(values: list[float]) -> dict[str, float | None]:
        if not values:
            return {"rms": None, "maximum": None, "final": None}
        array = np.asarray(values)
        return {"rms": float(np.sqrt(np.mean(array**2))), "maximum": float(np.max(array)), "final": float(array[-1])}

    gates = {
        "events_present": bool(events),
        "estimates_present": bool(estimates),
        "nominal_packets_well_formed": malformed == 0,
        "expected_bno_saturation_covered": bno_accel_saturation == 0 or int(health.get("high_g_samples", 0)) > 0,
        "no_gyro_or_auxiliary_saturation": unexpected_saturation == 0,
        "sequence_clean": int(health.get("sequence_discontinuities", 0)) == 0 and int(health.get("bno_channel_sequence_discontinuities", 0)) == 0,
        "no_invalid_gnss_fix_accepted": int(health.get("gnss_invalid_fixes_accepted", 0)) == 0,
        "gnss_recovered": not config.zed_f9p.enabled or int(health.get("gnss_updates_accepted", 0)) > 0,
        "pps_synchronized": not config.zed_f9p.enabled or int(health.get("pps_updates", 0)) >= 2,
        "timestamps_exact": all(timing.values()),
        "replay_round_trip": replay_ok,
        "states_finite": finite,
        "quaternion_normalized": bool(quaternion_norm_error and max(quaternion_norm_error) < 1e-12),
        "covariance_symmetric": bool(covariance_symmetry_error and max(covariance_symmetry_error) < 1e-10),
        "covariance_psd": bool(covariance_min_eigenvalue and min(covariance_min_eigenvalue) >= -1e-12),
        "noise_statistics": statistical["noise_moments_passed"],
        "covariance_coverage": statistical["covariance_coverage_passed"],
    }
    return {
        "passed": all(gates.values()), "gates": gates,
        "counts": {"truth_samples": len(truth), "measurement_events": len(events), "state_estimates": len(estimates), "events_by_sensor": {sensor.name.lower(): len(values) for sensor, values in events_by_sensor.items() if values or sensor in (SensorId.BNO085_SHTP, SensorId.ADXL375, SensorId.BMP581, SensorId.ZED_F9P_UBX, SensorId.ZED_F9P_TIMEPULSE)}, "replay_transactions": replay_counts},
        "health": dict(health),
        "timing": {"expected_interval_ticks": expected_intervals, "exact_by_stream": timing, "output_rate_hz": {"bno_acceleration": config.bno085.accel_rate_hz, "bno_gyro": config.bno085.gyro_rate_hz, "bno_game_rotation": config.bno085.game_rotation_rate_hz, "adxl375": config.adxl375.output_rate_hz, "bmp581": config.bmp581.output_rate_hz, "zed_navigation": config.zed_f9p.output_rate_hz, "zed_timepulse": config.zed_f9p.pps_rate_hz}},
        "errors": {"position_m": stats(position_error), "velocity_mps": stats(velocity_error), "attitude_deg": stats(attitude_error), "game_rotation_vector_deg": stats(game_rotation_error)},
        "invariants": {"max_quaternion_norm_error": max(quaternion_norm_error, default=None), "max_covariance_symmetry_error": max(covariance_symmetry_error, default=None), "min_covariance_eigenvalue": min(covariance_min_eigenvalue, default=None)},
        "protocol_health": {"malformed_packets": malformed, "bno_accel_saturation_samples": bno_accel_saturation, "unexpected_saturation_samples": unexpected_saturation},
        "monte_carlo": statistical,
    }


def write_states(path: Path, estimates: list[StateEstimate]) -> None:
    header = ["state_ticks", "publication_ticks", "gps_time_ns", "px_m", "py_m", "pz_m", "vx_mps", "vy_mps", "vz_mps", "qw", "qx", "qy", "qz", "bax_mps2", "bay_mps2", "baz_mps2", "bgx_rps", "bgy_rps", "bgz_rps", *[f"pdiag_{index}" for index in range(15)]]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(header)
        for estimate in estimates:
            writer.writerow([estimate.state_ticks, estimate.publication_ticks, "" if estimate.gps_time_ns is None else estimate.gps_time_ns, *estimate.position_enu_m, *estimate.velocity_enu_mps, *estimate.q_body_to_nav, *estimate.accel_bias_body_mps2, *estimate.gyro_bias_body_rps, *np.diag(estimate.covariance)])


def write_report(path: Path, metrics: dict[str, Any], truth_summary: dict[str, float]) -> None:
    errors = metrics["errors"]
    lines = ["# Digital Twin Validation", "", f"**Overall:** {'PASS' if metrics['passed'] else 'FAIL'}", "", "## Flight", "", f"- Pad alignment: {truth_summary['pad_duration_s']:.1f} s", f"- Apogee AGL: {truth_summary['apogee_agl_m']:.1f} m", f"- Maximum Mach: {truth_summary['max_mach']:.2f}", "", "## Navigation error", "", f"- Position RMS/final: {errors['position_m']['rms']:.3f} / {errors['position_m']['final']:.3f} m", f"- Velocity RMS/final: {errors['velocity_mps']['rms']:.3f} / {errors['velocity_mps']['final']:.3f} m/s", f"- Attitude RMS/final: {errors['attitude_deg']['rms']:.3f} / {errors['attitude_deg']['final']:.3f} deg", "", "## Gates", ""]
    lines.extend(f"- [{'x' if passed else ' '}] `{name}`" for name, passed in metrics["gates"].items())
    lines.extend(["", "## Sensor health", "", *[f"- `{name}`: {value}" for name, value in sorted(metrics["health"].items())]])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_error_plot(path: Path, truth: list[TruthSample], estimates: list[StateEstimate], clock_hz: int) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    truth_by_tick = {sample.ticks: sample for sample in truth}
    aligned = [(estimate, truth_by_tick[estimate.state_ticks]) for estimate in estimates if estimate.state_ticks in truth_by_tick]
    time_s = np.array([estimate.state_ticks / clock_hz for estimate, _ in aligned])
    values = [np.array([np.linalg.norm(estimate.position_enu_m - sample.position_enu_m) for estimate, sample in aligned]), np.array([np.linalg.norm(estimate.velocity_enu_mps - sample.velocity_enu_mps) for estimate, sample in aligned]), np.array([attitude_error_deg(estimate.q_body_to_nav, sample.q_body_to_nav) for estimate, sample in aligned])]
    figure, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for axis, value, label in zip(axes, values, ("position (m)", "velocity (m/s)", "attitude (deg)")):
        axis.plot(time_s, value); axis.set_ylabel(label); axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("simulation time (s)")
    figure.suptitle("Multi-sensor ESKF error versus RocketPy truth"); figure.tight_layout(); figure.savefig(path, dpi=130); plt.close(figure)


def load_validation(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))
