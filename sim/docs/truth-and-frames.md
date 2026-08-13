# Truth, Frames, and Time

## Navigation frame

The navigation frame is launch-centered ENU:

- `x`: east
- `y`: north
- `z`: up
- position origin: launch rail location

RocketPy altitude remains mean sea level, or MSL. The conversion is:

```text
position_enu_m[2] = rocketpy_z_msl - launch_elevation_msl_m
altitude_msl_m    = rocketpy_z_msl
```

GNSS solutions use WGS84 ECEF. Position is transformed relative to the launch
site; velocity is rotated without an origin translation.

## Body frame and attitude

Quaternions are scalar-first `[w, x, y, z]` and rotate body-frame vectors into
ENU. If `R(q)` is the corresponding direction cosine matrix:

```text
v_nav  = R(q) @ v_body
v_body = R(q).T @ v_nav
```

RocketPy is configured with the rocket axis running tail to nose and the motor
axis running nozzle to combustion chamber. The initial rail quaternion follows
RocketPy's zero-spin 3-1-3 convention using rail inclination and heading.

Every sensor mounting matrix maps sensor axes to body axes. Sensor generation
uses its transpose to map ideal body measurements into sensor coordinates.
Decoding applies the mounting matrix to return body coordinates.

## Specific force

Accelerometers measure specific force, not navigation-frame kinematic
acceleration. With ENU gravity `g_nav = [0, 0, -g]`:

```text
specific_force_body = R(q).T @ (acceleration_enu - g_nav)
```

A stationary, upright accelerometer therefore measures positive one g upward in
its body frame. The estimator reconstructs navigation acceleration with:

```text
acceleration_enu = R(q) @ specific_force_body + g_nav
```

## Hardware time

Simulation time starts at the beginning of pad alignment. The default clock is
100 MHz, and all event timestamps are unsigned integer clock ticks. Truth is
generated at exactly 2000 Hz, so the default sample spacing is 50,000 ticks.
Liftoff begins after `pad_duration_s`, which is 10 seconds by default.

`measurement_ticks` is the sensor data-ready or physical measurement epoch.
`arrival_ticks` is when the complete acquisition is available to the consumer.

## RocketPy truth

`generate_andromeda_truth` builds the environment, motor, rocket geometry, and
launch rail from TOML. RocketPy terminates at apogee. Its continuous outputs are
sampled at 2000 Hz and preceded by a stationary pad segment with fixed rail
attitude and local atmospheric values.

The model uses RocketPy's standard atmosphere and a constant-thrust motor based
on configured total impulse divided by burn time. This is a reference vehicle
model, not a measured motor thrust curve.

## Analytic truth

`analytic_truth` generates exact constant navigation acceleration and constant
body angular-rate cases. It supports configurable initial position, velocity,
attitude, and MSL elevation. Tests use it to isolate mechanization, timing,
quantization, and covariance behavior from RocketPy.

## Atmosphere and interpolation

The local atmosphere helper implements ISA behavior from -500 m through
20,000 m, including the lower troposphere and lower stratosphere. It returns
pressure, temperature, and density.

Vector quantities use linear interpolation. Attitude uses shortest-path
spherical interpolation, with normalized linear interpolation for nearly
identical quaternions.
