# Configuration

`config/andromeda.toml` loads into frozen typed dataclasses. Unknown or missing
typed keys fail at startup.

## Main sections

- `[simulation]`: 100 MHz clock, required 2000 Hz truth, 10 second pad window,
  gravity, and default seed.
- `[launch]`, `[motor]`, `[rocket]`: RocketPy vehicle and launch site inputs.
- `[bno085]`: enable, 3 MHz maximum SPI clock, 94 ms startup, 500/400/100 Hz
  report rates, calibration errors, optional filters, delays, and mounting.
- `[adxl375]`: enable, rate, shared SPI clock, noise, calibration errors, and
  mounting.
- `[bmp581]`: rate, oversampling, noise, bias, and transonic disturbance.
- `[zed_f9p]`: 5 Hz navigation, 1 PPS, GPS epoch, ENU errors, latency, clock
  behavior, lever arm, outages, 4 g limit, one second recovery, UART baud, and
  protocol profile.
- `[integration]`: rewind history, high-g hysteresis and hold, freshness,
  overlap NIS, GNSS NIS, and barometer gates.
- `[estimator]`: bias process noise and initial covariance values.

All vector values contain three entries. Mounting matrices are proper
orthonormal 3 by 3 rotations. Active report rates must divide the hardware
clock exactly. The BNO limits are 500 Hz acceleration, 400 Hz gyro, 400 Hz game
rotation vector, and 3 MHz SPI. ADXL is limited to 800 Hz. GNSS history must
cover nominal latency plus three jitter standard deviations.

Undocumented BNO noise, vehicle mounting errors, lever arms, and related bench
values default to zero. Configuration bytes are hashed into every run manifest.
