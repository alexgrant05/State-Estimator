"""Arrival-ordered multi-sensor 15-state ESKF reference."""

from __future__ import annotations

from bisect import bisect_right
from collections import Counter
from dataclasses import dataclass

import numpy as np

from .adis16470 import DecodedAdisMeasurement, decode_event as decode_adis
from .adxl375 import DecodedAdxlMeasurement, decode_event as decode_adxl
from .bmp581 import DecodedBmpMeasurement, decode_event as decode_bmp
from .config import TwinConfig
from .frames import exponential_quaternion, normalize_quaternion, quaternion_from_two_vectors, quaternion_multiply, rotation_matrix, skew
from .geodesy import ecef_from_enu_rotation, geodetic_to_ecef
from .gnss import GPS_WEEK_NS
from .transport import AdisBurst, GnssPps, GnssSolution
from .types import MeasurementEvent, SensorId, StateEstimate, StatusFlag


@dataclass(slots=True)
class _ImuInput:
    ticks: int
    accel_body_mps2: np.ndarray
    gyro_body_rps: np.ndarray
    use_adxl: bool


@dataclass(slots=True)
class _AidInput:
    ticks: int
    kind: SensorId
    measurement: object


@dataclass(slots=True)
class _Snapshot:
    ticks: int
    position: np.ndarray
    velocity: np.ndarray
    quaternion: np.ndarray
    accel_bias: np.ndarray
    gyro_bias: np.ndarray
    covariance: np.ndarray
    angular_rate_body_rps: np.ndarray


class _TimeSync:
    def __init__(self, clock_hz: int):
        self.clock_hz = clock_hz
        self.points: list[tuple[int, int]] = []
        self.slope_ns_per_tick = 1e9 / clock_hz
        self.intercept_ns: float | None = None
        self.max_residual_ns = 0.0

    def add(self, local_ticks: int, gps_time_ns: int) -> None:
        self.points.append((local_ticks, gps_time_ns))
        if len(self.points) > 32:
            self.points.pop(0)
        if len(self.points) == 1:
            self.intercept_ns = gps_time_ns - self.slope_ns_per_tick * local_ticks
            return
        ticks = np.asarray([point[0] for point in self.points], dtype=np.float64)
        times = np.asarray([point[1] for point in self.points], dtype=np.float64)
        centered = ticks - ticks[0]
        slope, offset = np.polyfit(centered, times - times[0], 1)
        self.slope_ns_per_tick = float(slope)
        self.intercept_ns = float(times[0] - slope * centered[0] - slope * ticks[0])
        predicted = self.slope_ns_per_tick * ticks + self.intercept_ns
        self.max_residual_ns = max(self.max_residual_ns, float(np.max(np.abs(predicted - times))))

    def gps_time_ns(self, local_ticks: int) -> int | None:
        if self.intercept_ns is None:
            return None
        return int(round(self.slope_ns_per_tick * local_ticks + self.intercept_ns))


class InertialEskf:
    """ADIS propagation with high-g acceleration selection and BMP/GNSS aiding."""

    def __init__(self, config: TwinConfig):
        self.config = config
        self.health: Counter[str] = Counter()
        self.pad_measurements: list[tuple[int, DecodedAdisMeasurement]] = []
        self.pad_adxl: list[DecodedAdxlMeasurement] = []
        self.pad_pressures: list[float] = []
        self.initialized = False
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.angular_rate_body_rps = np.zeros(3)
        self.adxl_pad_bias = np.zeros(3)
        self.covariance = np.zeros((15, 15))
        self.last_state_ticks: int | None = None
        self.expected_sequences: dict[SensorId, int] = {}
        self.expected_counter: int | None = None
        self.liftoff_ticks = int(round(config.simulation.pad_duration_s * config.simulation.clock_hz))
        self.latest_adxl: tuple[int, DecodedAdxlMeasurement] | None = None
        self.high_g_active = False
        self.high_g_below_since: int | None = None
        self.barometer_reference_pa: float | None = None
        self.imu_history: list[_ImuInput] = []
        self.aid_history: list[_AidInput] = []
        self.pending_aids: list[_AidInput] = []
        self.snapshots: list[_Snapshot] = []
        self.time_sync = _TimeSync(config.simulation.clock_hz)
        self.flight_phase = "PAD"
        self.origin_ecef = geodetic_to_ecef(config.launch.latitude_deg, config.launch.longitude_deg, config.launch.elevation_msl_m)
        self.ecef_from_enu = ecef_from_enu_rotation(config.launch.latitude_deg, config.launch.longitude_deg)

    def _track_sequence(self, event: MeasurementEvent) -> None:
        expected = self.expected_sequences.get(event.sensor_id)
        if expected is not None and event.sequence_number != expected:
            gap = (event.sequence_number - expected) & 0xFFFFFFFF
            self.health["sequence_discontinuities"] += 1
            self.health[f"{event.sensor_id.name.lower()}_sequence_discontinuities"] += 1
            self.health["packets_lost"] += gap
        self.expected_sequences[event.sensor_id] = (event.sequence_number + 1) & 0xFFFFFFFF

    def _decode_adis(self, event: MeasurementEvent) -> DecodedAdisMeasurement | None:
        try:
            burst = AdisBurst.from_payload_bytes(event.payload)
        except ValueError:
            self.health["malformed_packets"] += 1
            return None
        if not burst.valid_checksum():
            self.health["checksum_failures"] += 1
            return None
        if burst.diag_stat:
            self.health["diagnostic_failures"] += 1
            return None
        if self.expected_counter is not None and burst.data_counter != self.expected_counter:
            self.health["counter_discontinuities"] += 1
        self.expected_counter = (burst.data_counter + 1) & 0xFFFF
        if event.status_flags & StatusFlag.SATURATED:
            self.health["saturated_measurements"] += 1
        try:
            return decode_adis(event, self.config.adis16470, self.config.simulation)
        except ValueError:
            self.health["decode_failures"] += 1
            return None

    def _initialize(self) -> None:
        if not self.pad_measurements:
            raise RuntimeError("no valid pad measurements available for initialization")
        accel_mean = np.mean([measurement.accel_body_mps2 for _, measurement in self.pad_measurements], axis=0)
        gyro_mean = np.mean([measurement.gyro_body_rps for _, measurement in self.pad_measurements], axis=0)
        from .frames import rocketpy_initial_quaternion

        rail_q = rocketpy_initial_quaternion(self.config.launch.rail_inclination_deg, self.config.launch.rail_heading_deg)
        expected_up_body = rotation_matrix(rail_q).T @ np.array([0.0, 0.0, 1.0])
        measured_up_body = accel_mean / np.linalg.norm(accel_mean)
        tilt_correction = quaternion_from_two_vectors(measured_up_body, expected_up_body)
        self.quaternion = normalize_quaternion(quaternion_multiply(rail_q, tilt_correction))
        expected_specific_force = rotation_matrix(self.quaternion).T @ np.array([0.0, 0.0, self.config.simulation.gravity_mps2])
        self.accel_bias = accel_mean - expected_specific_force
        self.gyro_bias = gyro_mean
        if self.pad_adxl:
            self.adxl_pad_bias = np.mean([measurement.accel_body_mps2 for measurement in self.pad_adxl], axis=0) - expected_specific_force
        if self.pad_pressures:
            self.barometer_reference_pa = float(np.mean(self.pad_pressures))

        est = self.config.estimator
        diagonal_sigmas = np.r_[
            np.full(3, est.initial_position_sigma_m),
            np.full(3, est.initial_velocity_sigma_mps),
            np.full(3, np.radians(est.initial_attitude_sigma_deg)),
            np.full(3, est.initial_accel_bias_sigma_mps2),
            np.full(3, est.initial_gyro_bias_sigma_rps),
        ]
        self.covariance = np.diag(diagonal_sigmas**2)
        self.last_state_ticks = self.pad_measurements[-1][0]
        self.initialized = True
        self.health["initializations"] += 1
        self.snapshots = [self._snapshot()]

    def _snapshot(self) -> _Snapshot:
        assert self.last_state_ticks is not None
        return _Snapshot(
            self.last_state_ticks,
            self.position.copy(),
            self.velocity.copy(),
            self.quaternion.copy(),
            self.accel_bias.copy(),
            self.gyro_bias.copy(),
            self.covariance.copy(),
            self.angular_rate_body_rps.copy(),
        )

    def _restore(self, snapshot: _Snapshot) -> None:
        self.last_state_ticks = snapshot.ticks
        self.position = snapshot.position.copy()
        self.velocity = snapshot.velocity.copy()
        self.quaternion = snapshot.quaternion.copy()
        self.accel_bias = snapshot.accel_bias.copy()
        self.gyro_bias = snapshot.gyro_bias.copy()
        self.covariance = snapshot.covariance.copy()
        self.angular_rate_body_rps = snapshot.angular_rate_body_rps.copy()

    def _select_acceleration(self, event: MeasurementEvent, adis: DecodedAdisMeasurement) -> tuple[np.ndarray, bool]:
        threshold_enter = self.config.integration.high_g_enter_fraction * 40.0 * self.config.simulation.gravity_mps2
        threshold_exit = self.config.integration.high_g_exit_fraction * 40.0 * self.config.simulation.gravity_mps2
        latest = self.latest_adxl
        adxl_fresh = False
        adxl_accel = None
        if latest is not None:
            age = event.measurement_ticks - latest[0]
            max_age = self.config.integration.high_g_max_age_samples * self.config.simulation.clock_hz // self.config.adxl375.output_rate_hz
            adxl_fresh = 0 <= age <= max_age and not latest[1].saturated
            adxl_accel = latest[1].accel_body_mps2 - self.adxl_pad_bias
        overlap_ok = False
        if adxl_fresh and adxl_accel is not None:
            adis_corrected = adis.accel_body_mps2 - self.accel_bias
            adxl_sigma = self.config.adxl375.noise_density_mg_sqrt_hz * 1e-3 * self.config.simulation.gravity_mps2 * np.sqrt(self.config.adxl375.output_rate_hz / 2.0)
            variance = max(adxl_sigma**2, 1e-12)
            overlap_ok = float((adxl_accel - adis_corrected) @ (adxl_accel - adis_corrected) / variance) <= self.config.integration.overlap_nis_gate
            if not overlap_ok:
                self.health["high_g_overlap_rejections"] += 1

        high = np.max(np.abs(adis.accel_body_mps2)) >= threshold_enter or adis.saturated
        if not self.high_g_active and high:
            if adxl_fresh:
                self.high_g_active = True
                self.high_g_below_since = None
                self.health["high_g_switches_to_adxl"] += 1
            else:
                self.health["high_g_unavailable"] += 1
        elif self.high_g_active:
            if np.max(np.abs(adis.accel_body_mps2)) < threshold_exit and overlap_ok:
                if self.high_g_below_since is None:
                    self.high_g_below_since = event.measurement_ticks
                hold_ticks = int(round(self.config.integration.high_g_exit_hold_s * self.config.simulation.clock_hz))
                if event.measurement_ticks - self.high_g_below_since >= hold_ticks:
                    self.high_g_active = False
                    self.health["high_g_switches_to_adis"] += 1
            else:
                self.high_g_below_since = None
        if self.high_g_active and adxl_fresh and adxl_accel is not None:
            self.health["high_g_samples"] += 1
            return adxl_accel, True
        return adis.accel_body_mps2, False

    def _propagate(self, value: _ImuInput, count: bool = True) -> bool:
        assert self.last_state_ticks is not None
        dt = (value.ticks - self.last_state_ticks) / self.config.simulation.clock_hz
        if dt <= 0.0:
            if count:
                self.health["nonmonotonic_measurements"] += 1
            return False
        accel = value.accel_body_mps2 if value.use_adxl else value.accel_body_mps2 - self.accel_bias
        gyro = value.gyro_body_rps - self.gyro_bias
        self.angular_rate_body_rps = gyro.copy()
        rotation = rotation_matrix(self.quaternion)
        acceleration_nav = rotation @ accel + np.array([0.0, 0.0, -self.config.simulation.gravity_mps2])
        self.position += self.velocity * dt + 0.5 * acceleration_nav * dt * dt
        self.velocity += acceleration_nav * dt
        self.quaternion = normalize_quaternion(quaternion_multiply(self.quaternion, exponential_quaternion(gyro * dt)))

        rate = np.zeros((15, 15))
        rate[0:3, 3:6] = np.eye(3)
        rate[3:6, 6:9] = -rotation @ skew(accel)
        if not value.use_adxl:
            rate[3:6, 9:12] = -rotation
        rate[6:9, 6:9] = -skew(gyro)
        rate[6:9, 12:15] = -np.eye(3)
        transition = np.eye(15) + rate * dt
        if value.use_adxl:
            accel_density = self.config.adxl375.noise_density_mg_sqrt_hz * 1e-3 * self.config.simulation.gravity_mps2
        else:
            accel_density = self.config.adis16470.accel_noise_rms_mg * 1e-3 * self.config.simulation.gravity_mps2 / np.sqrt(600.0)
        gyro_density = np.radians(self.config.adis16470.gyro_noise_rms_dps) / np.sqrt(550.0)
        process = np.zeros((15, 15))
        process[3:6, 3:6] = np.eye(3) * accel_density**2 * dt
        process[6:9, 6:9] = np.eye(3) * gyro_density**2 * dt
        process[9:12, 9:12] = np.eye(3) * self.config.estimator.accel_bias_rw_mps2_sqrt_s**2 * dt
        process[12:15, 12:15] = np.eye(3) * self.config.estimator.gyro_bias_rw_rps_sqrt_s**2 * dt
        self.covariance = transition @ self.covariance @ transition.T + process
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.last_state_ticks = value.ticks
        if count:
            self.health["propagated_samples"] += 1
        return True

    def _inject(self, innovation: np.ndarray, matrix: np.ndarray, covariance: np.ndarray, gate: float) -> bool:
        residual_covariance = matrix @ self.covariance @ matrix.T + covariance
        try:
            inverse = np.linalg.inv(residual_covariance)
        except np.linalg.LinAlgError:
            return False
        nis = float(innovation @ inverse @ innovation)
        if not np.isfinite(nis) or nis > gate:
            return False
        gain = self.covariance @ matrix.T @ inverse
        correction = gain @ innovation
        self.position += correction[0:3]
        self.velocity += correction[3:6]
        self.quaternion = normalize_quaternion(quaternion_multiply(self.quaternion, exponential_quaternion(correction[6:9])))
        self.accel_bias += correction[9:12]
        self.gyro_bias += correction[12:15]
        identity = np.eye(15)
        joseph = identity - gain @ matrix
        self.covariance = joseph @ self.covariance @ joseph.T + gain @ covariance @ gain.T
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        return True

    def _phase_at_current_state(self, ticks: int) -> str:
        if ticks < self.liftoff_ticks:
            return "PAD"
        elapsed = (ticks - self.liftoff_ticks) / self.config.simulation.clock_hz
        if elapsed < self.config.motor["burn_time_s"]:
            return "BOOST"
        return "COAST" if self.velocity[2] > 0.0 else "APOGEE"

    def _update_flight_phase(self) -> None:
        assert self.last_state_ticks is not None
        phase = self._phase_at_current_state(self.last_state_ticks)
        if phase != self.flight_phase:
            self.flight_phase = phase
            self.health[f"phase_entries_{phase.lower()}"] += 1

    @staticmethod
    def _pressure_altitude_m(pressure_pa: float) -> float:
        return 44_330.0 * (1.0 - (pressure_pa / 101_325.0) ** (1.0 / 5.2558797))

    def _apply_aid(self, aid: _AidInput) -> bool:
        if aid.kind == SensorId.BMP581:
            measurement = aid.measurement
            assert isinstance(measurement, DecodedBmpMeasurement)
            if self.barometer_reference_pa is None:
                return False
            if self._phase_at_current_state(aid.ticks) == "BOOST":
                return False
            mach_proxy = np.linalg.norm(self.velocity) / 340.0
            if self.config.integration.baro_transonic_min_mach <= mach_proxy <= self.config.integration.baro_transonic_max_mach:
                return False
            altitude = self._pressure_altitude_m(measurement.pressure_pa) - self._pressure_altitude_m(self.barometer_reference_pa)
            innovation = np.array([altitude - self.position[2]])
            matrix = np.zeros((1, 15))
            matrix[0, 2] = 1.0
            derivative = max(measurement.pressure_pa * self.config.simulation.gravity_mps2 / (287.05287 * (measurement.temperature_c + 273.15)), 1e-6)
            sigma_altitude = self.config.bmp581.pressure_noise_pa / derivative
            return self._inject(innovation, matrix, np.array([[max(sigma_altitude, 0.05) ** 2]]), self.config.integration.baro_nis_gate)
        if aid.kind == SensorId.GNSS_SOLUTION:
            solution = aid.measurement
            assert isinstance(solution, GnssSolution)
            position_enu = self.ecef_from_enu.T @ (np.asarray(solution.position_ecef_m) - self.origin_ecef)
            velocity_enu = self.ecef_from_enu.T @ np.asarray(solution.velocity_ecef_mps)
            lever = self.config.gnss.antenna_lever_arm_body_m
            rotation = rotation_matrix(self.quaternion)
            lever_nav = rotation @ lever
            lever_velocity_body = np.cross(self.angular_rate_body_rps, lever)
            predicted_position = self.position + lever_nav
            predicted_velocity = self.velocity + rotation @ lever_velocity_body
            innovation = np.r_[position_enu - predicted_position, velocity_enu - predicted_velocity]
            matrix = np.zeros((6, 15))
            matrix[:3, :3] = np.eye(3)
            matrix[:3, 6:9] = -rotation @ skew(lever)
            matrix[3:, 3:6] = np.eye(3)
            matrix[3:, 6:9] = -rotation @ skew(lever_velocity_body)
            matrix[3:, 12:15] = rotation @ skew(lever)
            transform = np.zeros((6, 6))
            transform[:3, :3] = self.ecef_from_enu.T
            transform[3:, 3:] = self.ecef_from_enu.T
            covariance = transform @ np.asarray(solution.covariance).reshape(6, 6) @ transform.T
            return self._inject(innovation, matrix, covariance, self.config.integration.gnss_nis_gate)
        return False

    def _rewind_with_aid(self, aid: _AidInput) -> bool:
        assert self.last_state_ticks is not None
        latest_ticks = self.last_state_ticks
        history_ticks = int(round(self.config.integration.history_duration_s * self.config.simulation.clock_hz))
        if aid.ticks < latest_ticks - history_ticks or aid.ticks > latest_ticks:
            self.health["aiding_history_misses"] += 1
            return False
        self.aid_history.append(aid)
        snapshot_ticks = [snapshot.ticks for snapshot in self.snapshots]
        base_index = max(0, bisect_right(snapshot_ticks, aid.ticks - 1) - 1)
        base = self.snapshots[base_index]
        preserved = self.snapshots[: base_index + 1]
        self._restore(base)
        operations: list[tuple[int, int, object]] = []
        operations += [(value.ticks, 0, value) for value in self.imu_history if base.ticks < value.ticks <= latest_ticks]
        operations += [(value.ticks, 1 if value.kind == SensorId.BMP581 else 2, value) for value in self.aid_history if base.ticks < value.ticks <= latest_ticks]
        accepted = False
        for _, kind, value in sorted(operations, key=lambda item: (item[0], item[1])):
            if kind == 0:
                if self._propagate(value, count=False):
                    preserved.append(self._snapshot())
            else:
                result = self._apply_aid(value)
                if value is aid:
                    accepted = result
        if preserved:
            preserved[-1] = self._snapshot()
        self.snapshots = preserved
        self.health["rewinds"] += 1
        return accepted

    def _prune_history(self) -> None:
        assert self.last_state_ticks is not None
        cutoff = self.last_state_ticks - int(round(self.config.integration.history_duration_s * self.config.simulation.clock_hz))
        ticks = [snapshot.ticks for snapshot in self.snapshots]
        base_index = bisect_right(ticks, cutoff) - 1
        if base_index <= 0:
            return
        base_ticks = self.snapshots[base_index].ticks
        self.snapshots = self.snapshots[base_index:]
        self.imu_history = [value for value in self.imu_history if value.ticks > base_ticks]
        self.aid_history = [value for value in self.aid_history if value.ticks > base_ticks]

    def _estimate(self, publication_ticks: int) -> StateEstimate:
        assert self.last_state_ticks is not None
        health = dict(self.health)
        health["pps_sync_max_residual_ns"] = int(round(self.time_sync.max_residual_ns))
        health["pending_aids"] = len(self.pending_aids)
        return StateEstimate(
            state_ticks=self.last_state_ticks,
            publication_ticks=publication_ticks,
            position_enu_m=self.position.copy(),
            velocity_enu_mps=self.velocity.copy(),
            q_body_to_nav=self.quaternion.copy(),
            accel_bias_body_mps2=self.accel_bias.copy(),
            gyro_bias_body_rps=self.gyro_bias.copy(),
            covariance=self.covariance.copy(),
            valid=True,
            health=health,
            gps_time_ns=self.time_sync.gps_time_ns(self.last_state_ticks),
        )

    def _ingest_aid(self, aid: _AidInput, prefix: str) -> bool | None:
        assert self.last_state_ticks is not None
        if aid.ticks > self.last_state_ticks:
            self.pending_aids.append(aid)
            self.health[f"{prefix}_updates_queued"] += 1
            return None
        accepted = self._rewind_with_aid(aid)
        self.health[f"{prefix}_updates_accepted" if accepted else f"{prefix}_updates_rejected"] += 1
        return accepted

    def _drain_pending_aids(self) -> None:
        assert self.last_state_ticks is not None
        ready = [aid for aid in self.pending_aids if aid.ticks <= self.last_state_ticks]
        self.pending_aids = [aid for aid in self.pending_aids if aid.ticks > self.last_state_ticks]
        for aid in sorted(ready, key=lambda value: (value.ticks, int(value.kind))):
            prefix = "bmp" if aid.kind == SensorId.BMP581 else "gnss"
            accepted = self._rewind_with_aid(aid)
            self.health[f"{prefix}_updates_accepted" if accepted else f"{prefix}_updates_rejected"] += 1

    def process(self, event: MeasurementEvent) -> StateEstimate | None:
        self._track_sequence(event)
        # ADIS checksum/diagnostic flags are decoded below so their dedicated
        # health counters remain observable even when VALID is cleared.
        if event.sensor_id != SensorId.ADIS16470 and not event.status_flags & StatusFlag.VALID:
            self.health["invalid_measurements"] += 1
            self.health[f"{event.sensor_id.name.lower()}_invalid_measurements"] += 1
            return None
        if event.sensor_id == SensorId.GNSS_PPS:
            try:
                pps = GnssPps.from_payload_bytes(event.payload)
            except ValueError:
                self.health["pps_decode_failures"] += 1
                return None
            if not pps.time_valid:
                self.health["pps_invalid"] += 1
                return None
            self.time_sync.add(event.measurement_ticks, pps.gps_week * GPS_WEEK_NS + pps.tow_ns)
            self.health["pps_updates"] += 1
            return self._estimate(event.arrival_ticks) if self.initialized else None
        if event.sensor_id == SensorId.ADXL375:
            try:
                measurement = decode_adxl(event, self.config.adxl375, self.config.simulation)
            except ValueError:
                self.health["adxl_decode_failures"] += 1
                return None
            self.latest_adxl = (event.measurement_ticks, measurement)
            if measurement.overrun:
                self.health["adxl_overruns"] += 1
            if event.measurement_ticks < self.liftoff_ticks:
                self.pad_adxl.append(measurement)
                self.health["adxl_pad_samples"] += 1
            return None
        if event.sensor_id == SensorId.BMP581:
            try:
                measurement = decode_bmp(event)
            except ValueError:
                self.health["bmp_decode_failures"] += 1
                return None
            if event.measurement_ticks < self.liftoff_ticks:
                self.pad_pressures.append(measurement.pressure_pa)
                self.health["bmp_pad_samples"] += 1
                return None
            if not self.initialized:
                return None
            elapsed = (event.measurement_ticks - self.liftoff_ticks) / self.config.simulation.clock_hz
            if elapsed < self.config.motor["burn_time_s"]:
                self.health["bmp_updates_rejected"] += 1
                self.health["bmp_phase_rejections"] += 1
                return self._estimate(event.arrival_ticks)
            self._ingest_aid(_AidInput(event.measurement_ticks, event.sensor_id, measurement), "bmp")
            return self._estimate(event.arrival_ticks)
        if event.sensor_id == SensorId.GNSS_SOLUTION:
            try:
                solution = GnssSolution.from_payload_bytes(event.payload)
            except ValueError:
                self.health["gnss_decode_failures"] += 1
                return None
            if event.measurement_ticks < self.liftoff_ticks or not self.initialized:
                self.health["gnss_pad_solutions"] += 1
                return None
            self._ingest_aid(_AidInput(event.measurement_ticks, event.sensor_id, solution), "gnss")
            return self._estimate(event.arrival_ticks)
        if event.sensor_id != SensorId.ADIS16470:
            self.health["unsupported_sensor"] += 1
            return None

        measurement = self._decode_adis(event)
        if measurement is None:
            return None
        if event.measurement_ticks < self.liftoff_ticks:
            self.pad_measurements.append((event.measurement_ticks, measurement))
            self.health["pad_samples"] += 1
            return None
        if not self.initialized:
            self._initialize()
        acceleration, use_adxl = self._select_acceleration(event, measurement)
        imu = _ImuInput(event.measurement_ticks, acceleration, measurement.gyro_body_rps, use_adxl)
        if not self._propagate(imu):
            return None
        self._update_flight_phase()
        self.imu_history.append(imu)
        self.snapshots.append(self._snapshot())
        self._drain_pending_aids()
        self._prune_history()
        return self._estimate(event.arrival_ticks)

    def run(self, events: list[MeasurementEvent]) -> list[StateEstimate]:
        estimates: list[StateEstimate] = []
        for event in sorted(events, key=lambda item: (item.arrival_ticks, int(item.sensor_id), item.sequence_number)):
            estimate = self.process(event)
            if estimate is not None:
                estimates.append(estimate)
        return estimates
