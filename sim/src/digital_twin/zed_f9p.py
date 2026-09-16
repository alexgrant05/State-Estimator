"""ZED-F9P standalone GNSS model with UBX UART and TIMEPULSE output."""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone

import numpy as np

from .config import LaunchConfig, SimulationConfig, ZedF9pConfig
from .frames import rotation_matrix
from .geodesy import ecef_from_enu_rotation, ecef_to_geodetic, geodetic_to_ecef
from .truth import interpolate_truth
from .types import CanonicalGnssFix, MeasurementEvent, SensorId, StatusFlag, TimePulse, TruthSample

UBX_SYNC = b"\xb5\x62"
NAV_CLASS = 0x01
NAV_PVT_ID = 0x07
NAV_COV_ID = 0x36
TIM_CLASS = 0x0D
TIM_TP_ID = 0x01
GPS_WEEK_MS = 604_800_000
TIMEPULSE_FORMAT = "<HIdB"


def ubx_checksum(data: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for value in data:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


@dataclass(frozen=True, slots=True)
class UbxFrame:
    message_class: int
    message_id: int
    payload: bytes

    def payload_bytes(self) -> bytes:
        body = struct.pack("<BBH", self.message_class, self.message_id, len(self.payload)) + self.payload
        return UBX_SYNC + body + bytes(ubx_checksum(body))

    transaction_bytes = payload_bytes

    @classmethod
    def from_bytes(cls, frame: bytes) -> "UbxFrame":
        if len(frame) < 8 or frame[:2] != UBX_SYNC:
            raise ValueError("invalid UBX sync or truncated frame")
        message_class, message_id, length = struct.unpack_from("<BBH", frame, 2)
        if len(frame) != length + 8:
            raise ValueError("UBX frame length does not match header")
        if tuple(frame[-2:]) != ubx_checksum(frame[2:-2]):
            raise ValueError("UBX checksum mismatch")
        return cls(message_class, message_id, frame[6:-2])


@dataclass(frozen=True, slots=True)
class ZedFaultSchedule:
    pvt_loss: frozenset[int] = field(default_factory=frozenset)
    cov_loss: frozenset[int] = field(default_factory=frozenset)
    invalid_fix: frozenset[int] = field(default_factory=frozenset)
    corrupt_checksum: frozenset[int] = field(default_factory=frozenset)
    pps_loss: frozenset[int] = field(default_factory=frozenset)
    additional_latency_s: dict[int, float] = field(default_factory=dict)


def encode_timepulse(value: TimePulse) -> bytes:
    return struct.pack(TIMEPULSE_FORMAT, value.gps_week, value.tow_ms, value.uncertainty_ns, int(value.time_valid))


def decode_timepulse(payload: bytes) -> TimePulse:
    if len(payload) != struct.calcsize(TIMEPULSE_FORMAT):
        raise ValueError("invalid TIMEPULSE payload")
    week, tow_ms, uncertainty, valid = struct.unpack(TIMEPULSE_FORMAT, payload)
    return TimePulse(week, tow_ms, uncertainty, bool(valid))


def encode_nav_pvt(tow_ms: int, latitude_deg: float, longitude_deg: float, height_m: float, velocity_enu_mps: np.ndarray, position_sigma_enu_m: np.ndarray, velocity_sigma_enu_mps: np.ndarray, valid_fix: bool, time_valid: bool, satellites: int = 12, gps_week: int = 0) -> UbxFrame:
    payload = bytearray(92)
    struct.pack_into("<I", payload, 0, tow_ms)
    # The common HPG 1.51 profile uses the current 18-second GPS to UTC offset.
    utc = datetime(1980, 1, 6, tzinfo=timezone.utc) + timedelta(weeks=gps_week, milliseconds=tow_ms) - timedelta(seconds=18)
    struct.pack_into("<HBBBBBB", payload, 4, utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second, 0x03 if time_valid else 0)
    payload[20] = 3 if valid_fix else 0
    payload[21] = 0x01 if valid_fix else 0
    payload[23] = satellites if valid_fix else 0
    struct.pack_into("<iiii", payload, 24, int(round(longitude_deg * 1e7)), int(round(latitude_deg * 1e7)), int(round(height_m * 1000)), int(round(height_m * 1000)))
    struct.pack_into("<II", payload, 40, int(round(max(position_sigma_enu_m[:2]) * 1000)), int(round(position_sigma_enu_m[2] * 1000)))
    east, north, up = np.asarray(velocity_enu_mps)
    struct.pack_into("<iii", payload, 48, int(round(north * 1000)), int(round(east * 1000)), int(round(-up * 1000)))
    struct.pack_into("<i", payload, 60, int(round(np.hypot(east, north) * 1000)))
    struct.pack_into("<I", payload, 68, int(round(max(velocity_sigma_enu_mps) * 1000)))
    return UbxFrame(NAV_CLASS, NAV_PVT_ID, bytes(payload))


def encode_nav_cov(tow_ms: int, position_sigma_enu_m: np.ndarray, velocity_sigma_enu_mps: np.ndarray) -> UbxFrame:
    payload = bytearray(64)
    struct.pack_into("<IBBBB", payload, 0, tow_ms, 0, 1, 1, 0)
    pos = np.asarray(position_sigma_enu_m) ** 2
    vel = np.asarray(velocity_sigma_enu_mps) ** 2
    # UBX covariance order is NN, NE, ND, EE, ED, DD.
    struct.pack_into("<6f", payload, 8, float(pos[1]), 0.0, 0.0, float(pos[0]), 0.0, float(pos[2]))
    struct.pack_into("<6f", payload, 32, float(vel[1]), 0.0, 0.0, float(vel[0]), 0.0, float(vel[2]))
    return UbxFrame(NAV_CLASS, NAV_COV_ID, bytes(payload))


def encode_tim_tp(tow_ms: int, gps_week: int, time_valid: bool = True) -> UbxFrame:
    flags = 0x03 if time_valid else 0
    return UbxFrame(TIM_CLASS, TIM_TP_ID, struct.pack("<IIiHBB", tow_ms, 0, 0, gps_week, flags, 0))


class ZedMessageAssembler:
    """Join NAV-PVT and NAV-COV messages by iTOW."""

    def __init__(self, launch: LaunchConfig):
        self.launch = launch
        self._pvt: dict[int, UbxFrame] = {}
        self._cov: dict[int, UbxFrame] = {}
        self._origin_ecef = geodetic_to_ecef(launch.latitude_deg, launch.longitude_deg, launch.elevation_msl_m)
        self._ecef_from_enu = ecef_from_enu_rotation(launch.latitude_deg, launch.longitude_deg)

    def add(self, frame: UbxFrame, gps_week: int) -> CanonicalGnssFix | None:
        if frame.message_class != NAV_CLASS:
            return None
        if frame.message_id == NAV_PVT_ID and len(frame.payload) == 92:
            tow = struct.unpack_from("<I", frame.payload)[0]
            self._pvt[tow] = frame
        elif frame.message_id == NAV_COV_ID and len(frame.payload) == 64:
            tow = struct.unpack_from("<I", frame.payload)[0]
            self._cov[tow] = frame
        else:
            return None
        if tow not in self._pvt or tow not in self._cov:
            return None
        pvt = self._pvt.pop(tow).payload
        cov = self._cov.pop(tow).payload
        lon, lat, height = struct.unpack_from("<iii", pvt, 24)[:3]
        position_ecef = geodetic_to_ecef(lat * 1e-7, lon * 1e-7, height * 1e-3)
        position = self._ecef_from_enu.T @ (position_ecef - self._origin_ecef)
        vel_n, vel_e, vel_d = struct.unpack_from("<iii", pvt, 48)
        velocity = np.array([vel_e, vel_n, -vel_d], dtype=np.float64) * 1e-3
        pos_nn, pos_ne, pos_nd, pos_ee, pos_ed, pos_dd = struct.unpack_from("<6f", cov, 8)
        vel_nn, vel_ne, vel_nd, vel_ee, vel_ed, vel_dd = struct.unpack_from("<6f", cov, 32)
        p_ned = np.array([[pos_nn, pos_ne, pos_nd], [pos_ne, pos_ee, pos_ed], [pos_nd, pos_ed, pos_dd]])
        v_ned = np.array([[vel_nn, vel_ne, vel_nd], [vel_ne, vel_ee, vel_ed], [vel_nd, vel_ed, vel_dd]])
        ned_to_enu = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        covariance = np.zeros((6, 6), dtype=np.float64)
        covariance[:3, :3] = ned_to_enu @ p_ned @ ned_to_enu.T
        covariance[3:, 3:] = ned_to_enu @ v_ned @ ned_to_enu.T
        valid = bool(pvt[20] >= 3 and pvt[21] & 0x01)
        time_valid = bool(pvt[11] & 0x03 == 0x03)
        return CanonicalGnssFix(gps_week, tow, position, velocity, covariance, valid, time_valid, pvt[23])


class ZedF9pModel:
    def __init__(self, config: ZedF9pConfig, launch: LaunchConfig, simulation: SimulationConfig, seed: int):
        self.config = config
        self.launch = launch
        self.simulation = simulation
        self.solution_rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.ZED_F9P_UBX), 1]))
        self.latency_rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.ZED_F9P_UBX), 2]))
        self.pps_rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.ZED_F9P_TIMEPULSE)]))
        self.outage_rng = np.random.default_rng(np.random.SeedSequence([seed, int(SensorId.ZED_F9P_UBX), 3]))
        self.origin_ecef = geodetic_to_ecef(launch.latitude_deg, launch.longitude_deg, launch.elevation_msl_m)
        self.ecef_from_enu = ecef_from_enu_rotation(launch.latitude_deg, launch.longitude_deg)

    def _gps_epoch(self, ticks: int) -> tuple[int, int]:
        total_ms = int(round(self.config.start_tow_s * 1000 + ticks / self.simulation.clock_hz * 1000))
        return self.config.gps_week + total_ms // GPS_WEEK_MS, total_ms % GPS_WEEK_MS

    def generate(self, truth: list[TruthSample], faults: ZedFaultSchedule | None = None) -> list[MeasurementEvent]:
        if not self.config.enabled or not truth:
            return []
        faults = faults or ZedFaultSchedule()
        events: list[MeasurementEvent] = []
        nav_interval = self.simulation.clock_hz // self.config.output_rate_hz
        position_bias = np.zeros(3)
        velocity_bias = np.zeros(3)
        valid_since: int | None = truth[0].ticks - int(round(self.config.recovery_time_s * self.simulation.clock_hz))
        outage = False
        planned_uart: list[tuple[int, int, StatusFlag, UbxFrame, bool, int]] = []

        def schedule_uart(frame: UbxFrame, ticks: int, ready: int, flags: StatusFlag, corrupt: bool = False) -> None:
            planned_uart.append((ready, ticks, flags, frame, corrupt, len(planned_uart)))

        for sequence, ticks in enumerate(range(truth[0].ticks, truth[-1].ticks + 1, nav_interval)):
            sample = interpolate_truth(truth, ticks)
            dt = 1.0 / self.config.output_rate_hz
            position_bias += self.solution_rng.normal(0.0, self.config.position_bias_rw_m_sqrt_s * np.sqrt(dt), 3)
            velocity_bias += self.solution_rng.normal(0.0, self.config.velocity_bias_rw_mps_sqrt_s * np.sqrt(dt), 3)
            specific_force = rotation_matrix(sample.q_body_to_nav).T @ (sample.acceleration_enu_mps2 - np.array([0.0, 0.0, -self.simulation.gravity_mps2]))
            below_limit = np.linalg.norm(specific_force) <= self.config.dynamic_limit_g * self.simulation.gravity_mps2
            if not below_limit:
                valid_since = None
            elif valid_since is None:
                valid_since = ticks
            dynamic_valid = valid_since is not None and ticks - valid_since >= int(round(self.config.recovery_time_s * self.simulation.clock_hz))
            if ticks == truth[0].ticks:
                dynamic_valid = True
            if outage:
                outage = self.outage_rng.random() >= self.config.outage_recovery_probability
            else:
                outage = self.outage_rng.random() < self.config.outage_entry_probability
            valid = dynamic_valid and not outage and sequence not in faults.invalid_fix
            lever = rotation_matrix(sample.q_body_to_nav) @ self.config.antenna_lever_arm_body_m
            lever_velocity = rotation_matrix(sample.q_body_to_nav) @ np.cross(sample.angular_rate_body_rps, self.config.antenna_lever_arm_body_m)
            position_enu = sample.position_enu_m + lever + position_bias + self.solution_rng.normal(0.0, self.config.position_sigma_enu_m)
            velocity_enu = sample.velocity_enu_mps + lever_velocity + velocity_bias + self.solution_rng.normal(0.0, self.config.velocity_sigma_enu_mps)
            lat, lon, height = ecef_to_geodetic(self.origin_ecef + self.ecef_from_enu @ position_enu)
            week, tow_ms = self._gps_epoch(ticks)
            pvt = encode_nav_pvt(tow_ms, lat, lon, height, velocity_enu, self.config.position_sigma_enu_m, self.config.velocity_sigma_enu_mps, valid, True, gps_week=week)
            cov = encode_nav_cov(tow_ms, self.config.position_sigma_enu_m, self.config.velocity_sigma_enu_mps)
            latency = max(0.0, self.latency_rng.normal(self.config.latency_mean_s, self.config.latency_jitter_s) + faults.additional_latency_s.get(sequence, 0.0))
            ready = ticks + int(round(latency * self.simulation.clock_hz))
            flags = StatusFlag.VALID if valid else StatusFlag.FIX_INVALID
            if sequence not in faults.pvt_loss:
                schedule_uart(pvt, ticks, ready, flags, sequence in faults.corrupt_checksum)
            if sequence not in faults.cov_loss:
                schedule_uart(cov, ticks, ready, flags)

        pps_interval = self.simulation.clock_hz // self.config.pps_rate_hz
        for sequence, nominal_ticks in enumerate(range(truth[0].ticks, truth[-1].ticks + 1, pps_interval)):
            time_s = nominal_ticks / self.simulation.clock_hz
            error_ns = self.config.clock_offset_ns + self.config.clock_drift_ppm * 1e3 * time_s + self.pps_rng.normal(0.0, self.config.pps_jitter_ns)
            edge = max(0, nominal_ticks + int(round(error_ns * self.simulation.clock_hz / 1e9)))
            week, tow_ms = self._gps_epoch(nominal_ticks)
            pulse = TimePulse(week, tow_ms, self.config.pps_jitter_ns, True)
            if sequence not in faults.pps_loss:
                events.append(MeasurementEvent(1, SensorId.ZED_F9P_TIMEPULSE, sequence, edge, edge, StatusFlag.VALID, encode_timepulse(pulse)))
                schedule_uart(encode_tim_tp(tow_ms, week), nominal_ticks, nominal_ticks, StatusFlag.VALID)
        uart_free = 0
        for event_sequence, (ready, ticks, flags, frame, corrupt, _) in enumerate(sorted(planned_uart, key=lambda item: (item[0], item[5]))):
            data = frame.payload_bytes()
            if corrupt:
                data = data[:-1] + bytes((data[-1] ^ 0x01,))
                flags |= StatusFlag.CHECKSUM_ERROR
            transfer = int(np.ceil(len(data) * 10 / self.config.uart_baud * self.simulation.clock_hz))
            arrival = max(ready, uart_free) + transfer
            uart_free = arrival
            events.append(MeasurementEvent(1, SensorId.ZED_F9P_UBX, event_sequence, ticks, arrival, flags, data))
        return events
