"""RocketPy and analytic ground-truth sources."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from .config import TwinConfig
from .frames import normalize_quaternion, quaternion_multiply, rocketpy_initial_quaternion
from .types import TruthSample


def _ticks(index: int, clock_hz: int, rate_hz: int) -> int:
    return index * (clock_hz // rate_hz)


def standard_atmosphere(altitude_msl_m: float) -> tuple[float, float, float]:
    """Return ISA pressure, temperature, and density through the lower stratosphere."""

    altitude = float(np.clip(altitude_msl_m, -500.0, 20_000.0))
    if altitude <= 11_000.0:
        temperature = 288.15 - 0.0065 * altitude
        pressure = 101_325.0 * (temperature / 288.15) ** 5.2558797
    else:
        temperature = 216.65
        pressure_11 = 22_632.06
        pressure = pressure_11 * np.exp(-9.80665 * (altitude - 11_000.0) / (287.05287 * temperature))
    density = pressure / (287.05287 * temperature)
    return float(pressure), float(temperature), float(density)


def interpolate_truth(samples: list[TruthSample], ticks: int) -> TruthSample:
    """Interpolate a 2000 Hz truth stream at an arbitrary hardware tick."""

    if not samples or ticks < samples[0].ticks or ticks > samples[-1].ticks:
        raise ValueError("requested truth tick is outside the available interval")
    tick_step = samples[1].ticks - samples[0].ticks if len(samples) > 1 else 1
    lower = min((ticks - samples[0].ticks) // tick_step, len(samples) - 1)
    if samples[lower].ticks == ticks or lower == len(samples) - 1:
        return samples[lower]
    upper = lower + 1
    a, b = samples[lower], samples[upper]
    fraction = (ticks - a.ticks) / (b.ticks - a.ticks)
    q0, q1 = a.q_body_to_nav, b.q_body_to_nav
    dot = float(q0 @ q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        quaternion = normalize_quaternion(q0 + fraction * (q1 - q0))
    else:
        angle = np.arccos(np.clip(dot, -1.0, 1.0))
        quaternion = (
            np.sin((1.0 - fraction) * angle) * q0 + np.sin(fraction * angle) * q1
        ) / np.sin(angle)

    def lerp(first, second):
        return (1.0 - fraction) * first + fraction * second

    return TruthSample(
        ticks=ticks,
        position_enu_m=lerp(a.position_enu_m, b.position_enu_m),
        velocity_enu_mps=lerp(a.velocity_enu_mps, b.velocity_enu_mps),
        acceleration_enu_mps2=lerp(a.acceleration_enu_mps2, b.acceleration_enu_mps2),
        q_body_to_nav=quaternion,
        angular_rate_body_rps=lerp(a.angular_rate_body_rps, b.angular_rate_body_rps),
        altitude_msl_m=float(lerp(a.altitude_msl_m, b.altitude_msl_m)),
        ambient_pressure_pa=float(lerp(a.ambient_pressure_pa, b.ambient_pressure_pa)),
        ambient_temperature_k=float(lerp(a.ambient_temperature_k, b.ambient_temperature_k)),
        air_density_kgpm3=float(lerp(a.air_density_kgpm3, b.air_density_kgpm3)),
        mach=float(lerp(a.mach, b.mach)),
    )


def analytic_truth(
    duration_s: float,
    clock_hz: int = 100_000_000,
    rate_hz: int = 2000,
    gravity_mps2: float = 9.80665,
    initial_position_m: np.ndarray | None = None,
    initial_velocity_mps: np.ndarray | None = None,
    navigation_acceleration_mps2: np.ndarray | None = None,
    initial_quaternion: np.ndarray | None = None,
    angular_rate_body_rps: np.ndarray | None = None,
    elevation_msl_m: float = 0.0,
) -> list[TruthSample]:
    """Generate a constant-acceleration, constant-body-rate analytic trajectory."""

    del gravity_mps2  # Gravity is intentionally absent from kinematic acceleration.
    p0 = np.zeros(3) if initial_position_m is None else np.asarray(initial_position_m, dtype=np.float64)
    v0 = np.zeros(3) if initial_velocity_mps is None else np.asarray(initial_velocity_mps, dtype=np.float64)
    accel = np.zeros(3) if navigation_acceleration_mps2 is None else np.asarray(navigation_acceleration_mps2, dtype=np.float64)
    q0 = np.array([1.0, 0.0, 0.0, 0.0]) if initial_quaternion is None else normalize_quaternion(initial_quaternion)
    omega = np.zeros(3) if angular_rate_body_rps is None else np.asarray(angular_rate_body_rps, dtype=np.float64)
    count = int(np.floor(duration_s * rate_hz)) + 1
    samples: list[TruthSample] = []
    omega_norm = np.linalg.norm(omega)
    for index in range(count):
        time_s = index / rate_hz
        if omega_norm == 0.0:
            q = q0.copy()
        else:
            half = 0.5 * omega_norm * time_s
            delta = np.array([np.cos(half), *(np.sin(half) * omega / omega_norm)])
            q = normalize_quaternion(quaternion_multiply(q0, delta))
        position = p0 + v0 * time_s + 0.5 * accel * time_s * time_s
        velocity = v0 + accel * time_s
        pressure, temperature, density = standard_atmosphere(elevation_msl_m + float(position[2]))
        speed_of_sound = np.sqrt(1.4 * 287.05287 * temperature)
        samples.append(
            TruthSample(
                ticks=_ticks(index, clock_hz, rate_hz),
                position_enu_m=position,
                velocity_enu_mps=velocity,
                acceleration_enu_mps2=accel.copy(),
                q_body_to_nav=q,
                angular_rate_body_rps=omega.copy(),
                altitude_msl_m=elevation_msl_m + float(position[2]),
                ambient_pressure_pa=pressure,
                ambient_temperature_k=temperature,
                air_density_kgpm3=density,
                mach=float(np.linalg.norm(velocity) / speed_of_sound),
            )
        )
    return samples


def _as_vector(function: Callable[[np.ndarray], np.ndarray], times: np.ndarray) -> np.ndarray:
    return np.asarray(function(times), dtype=np.float64).reshape(-1)


def generate_andromeda_truth(config: TwinConfig) -> tuple[list[TruthSample], dict[str, float]]:
    """Run RocketPy and prepend a stationary pad-alignment interval."""

    from rocketpy import Environment, Flight, Rocket, SolidMotor

    launch = config.launch
    motor_cfg = config.motor
    rocket_cfg = config.rocket
    simulation = config.simulation
    dt = 1.0 / simulation.truth_rate_hz

    environment = Environment(
        latitude=launch.latitude_deg,
        longitude=launch.longitude_deg,
        elevation=launch.elevation_msl_m,
    )
    environment.set_atmospheric_model(type="standard_atmosphere")
    motor = SolidMotor(
        thrust_source=motor_cfg["total_impulse_ns"] / motor_cfg["burn_time_s"],
        burn_time=motor_cfg["burn_time_s"],
        dry_mass=motor_cfg["dry_mass_kg"],
        dry_inertia=tuple(motor_cfg["dry_inertia_kgm2"]),
        nozzle_radius=motor_cfg["nozzle_radius_m"],
        throat_radius=motor_cfg["throat_radius_m"],
        grain_number=motor_cfg["grain_number"],
        grain_density=motor_cfg["grain_density_kgm3"],
        grain_outer_radius=motor_cfg["grain_outer_radius_m"],
        grain_initial_inner_radius=motor_cfg["grain_initial_inner_radius_m"],
        grain_initial_height=motor_cfg["grain_initial_height_m"],
        grain_separation=motor_cfg["grain_separation_m"],
        grains_center_of_mass_position=motor_cfg["grains_center_of_mass_position_m"],
        center_of_dry_mass_position=motor_cfg["center_of_dry_mass_position_m"],
        nozzle_position=motor_cfg["nozzle_position_m"],
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )
    rocket = Rocket(
        radius=rocket_cfg["radius_m"],
        mass=rocket_cfg["dry_mass_kg"],
        inertia=tuple(rocket_cfg["inertia_kgm2"]),
        power_off_drag=rocket_cfg["power_off_drag"],
        power_on_drag=rocket_cfg["power_on_drag"],
        center_of_mass_without_motor=rocket_cfg["center_of_mass_without_motor_m"],
        coordinate_system_orientation="tail_to_nose",
    )
    rocket.add_motor(motor, position=rocket_cfg["motor_position_m"])
    rocket.add_nose(
        length=rocket_cfg["nose_length_m"],
        kind=rocket_cfg["nose_kind"],
        position=rocket_cfg["nose_position_m"],
    )
    rocket.add_trapezoidal_fins(
        n=rocket_cfg["fin_count"],
        root_chord=rocket_cfg["fin_root_chord_m"],
        tip_chord=rocket_cfg["fin_tip_chord_m"],
        span=rocket_cfg["fin_span_m"],
        position=rocket_cfg["fin_position_m"],
        cant_angle=rocket_cfg["fin_cant_angle_deg"],
    )
    rocket.add_tail(
        top_radius=rocket_cfg["tail_top_radius_m"],
        bottom_radius=rocket_cfg["tail_bottom_radius_m"],
        length=rocket_cfg["tail_length_m"],
        position=rocket_cfg["tail_position_m"],
    )
    flight = Flight(
        rocket=rocket,
        environment=environment,
        rail_length=launch.rail_length_m,
        inclination=launch.rail_inclination_deg,
        heading=launch.rail_heading_deg,
        terminate_on_apogee=True,
        max_time=120,
        max_time_step=0.05,
        verbose=False,
    )

    flight_times = np.arange(0.0, float(flight.apogee_time) + 0.5 * dt, dt)
    flight_times = np.minimum(flight_times, float(flight.apogee_time))
    x = _as_vector(flight.x, flight_times)
    y = _as_vector(flight.y, flight_times)
    z_msl = _as_vector(flight.z, flight_times)
    vx = _as_vector(flight.vx, flight_times)
    vy = _as_vector(flight.vy, flight_times)
    vz = _as_vector(flight.vz, flight_times)
    ax = _as_vector(flight.ax, flight_times)
    ay = _as_vector(flight.ay, flight_times)
    az = _as_vector(flight.az, flight_times)
    e0 = _as_vector(flight.e0, flight_times)
    e1 = _as_vector(flight.e1, flight_times)
    e2 = _as_vector(flight.e2, flight_times)
    e3 = _as_vector(flight.e3, flight_times)
    w1 = _as_vector(flight.w1, flight_times)
    w2 = _as_vector(flight.w2, flight_times)
    w3 = _as_vector(flight.w3, flight_times)
    pressure = np.asarray([standard_atmosphere(value)[0] for value in z_msl])
    temperature = np.asarray([standard_atmosphere(value)[1] for value in z_msl])
    density = pressure / (287.05287 * temperature)
    speed = np.sqrt(vx * vx + vy * vy + vz * vz)
    mach = speed / np.sqrt(1.4 * 287.05287 * temperature)

    pad_count = int(round(simulation.pad_duration_s * simulation.truth_rate_hz))
    initial_q = rocketpy_initial_quaternion(launch.rail_inclination_deg, launch.rail_heading_deg)
    samples: list[TruthSample] = []
    pad_pressure, pad_temperature, pad_density = standard_atmosphere(launch.elevation_msl_m)
    for index in range(pad_count):
        samples.append(
            TruthSample(
                ticks=_ticks(index, simulation.clock_hz, simulation.truth_rate_hz),
                position_enu_m=np.zeros(3),
                velocity_enu_mps=np.zeros(3),
                acceleration_enu_mps2=np.zeros(3),
                q_body_to_nav=initial_q.copy(),
                angular_rate_body_rps=np.zeros(3),
                altitude_msl_m=launch.elevation_msl_m,
                ambient_pressure_pa=pad_pressure,
                ambient_temperature_k=pad_temperature,
                air_density_kgpm3=pad_density,
                mach=0.0,
            )
        )

    for flight_index, _ in enumerate(flight_times):
        index = pad_count + flight_index
        samples.append(
            TruthSample(
                ticks=_ticks(index, simulation.clock_hz, simulation.truth_rate_hz),
                position_enu_m=np.array([x[flight_index], y[flight_index], z_msl[flight_index] - launch.elevation_msl_m]),
                velocity_enu_mps=np.array([vx[flight_index], vy[flight_index], vz[flight_index]]),
                acceleration_enu_mps2=np.array([ax[flight_index], ay[flight_index], az[flight_index]]),
                q_body_to_nav=normalize_quaternion(np.array([e0[flight_index], e1[flight_index], e2[flight_index], e3[flight_index]])),
                angular_rate_body_rps=np.array([w1[flight_index], w2[flight_index], w3[flight_index]]),
                altitude_msl_m=float(z_msl[flight_index]),
                ambient_pressure_pa=float(pressure[flight_index]),
                ambient_temperature_k=float(temperature[flight_index]),
                air_density_kgpm3=float(density[flight_index]),
                mach=float(mach[flight_index]),
            )
        )

    summary = {
        "pad_duration_s": simulation.pad_duration_s,
        "liftoff_ticks": pad_count * (simulation.clock_hz // simulation.truth_rate_hz),
        "apogee_time_after_liftoff_s": float(flight.apogee_time),
        "apogee_agl_m": float(flight.apogee - launch.elevation_msl_m),
        "max_speed_mps": float(flight.max_speed),
        "max_mach": float(flight.max_mach_number),
    }
    return samples, summary
