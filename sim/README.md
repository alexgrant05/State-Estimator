# Digital Twin Simulation

This directory contains the installable Python simulation and validation
package for the Cornell Rocketry launch-to-apogee state estimator.

```text
10 s pad alignment + Andromeda RocketPy truth at 2000 Hz
    -> ADIS16470 + ADXL375 + BMP581 + generic GNSS/PPS
    -> timestamped events on a simulated 100 MHz timebase
    -> merged arrival stream and deterministic sensor replays
    -> high-g selection + delayed 15-state ESKF fusion
    -> states, plots, metrics, manifest, and pass/fail gates
```

## Implemented behavior

- Launch-centered ENU navigation with separate MSL altitude and WGS84 ECEF
  conversion for GNSS.
- Scalar-first body-to-navigation quaternions and configurable sensor mounting.
- ADIS16470 at 500 Hz by default, with exact 176-bit burst transactions.
- ADXL375 at 800 Hz, with a pre-saturation hysteretic handoff from ADIS.
- BMP581 at 50 Hz, with raw pressure and temperature registers, pad-pressure
  calibration, flight-phase suppression, and innovation gating.
- Generic GNSS at 10 Hz and PPS at 1 Hz, with configurable noise, latency,
  clock error, correlated outages, covariance, and antenna lever arm.
- Dedicated ADIS transport plus deterministic shared-SPI arbitration for ADXL
  and BMP.
- Measurement and arrival timestamps for every event.
- Two-second state history for delayed GNSS and barometer rewind/replay.
- Independent deterministic random streams and sensor-specific fault injection.
- Versioned logical events, per-sensor binary replays, hashes, and validation.

The estimator remains float64 with 15 error states: position, velocity,
attitude, ADIS accelerometer bias, and ADIS gyro bias. ADXL calibration
uncertainty is added to process noise while ADXL is active.

## Install and run

Python 3.10 through 3.13 is supported. RocketPy is pinned to 1.12.1.

```powershell
cd sim
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.\.venv\Scripts\python.exe -m pip install --no-deps -e .

.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m digital_twin run `
  --config config\andromeda.toml `
  --seed 42 `
  --output outputs\andromeda-all-sensors-seed-42
.\.venv\Scripts\python.exe -m digital_twin validate `
  --run outputs\andromeda-all-sensors-seed-42
```

The run command refuses to overwrite a non-empty directory. Any failed gate
returns a nonzero status.

## Configuration and outputs

`config/andromeda.toml` controls vehicle truth, sensor rates, mounts, errors,
transport timing, high-g thresholds, aiding gates, history length, and GNSS
behavior. Generic or vehicle-specific values are marked uncalibrated in the run
manifest until bench data replaces them.

Each run produces:

- `events.ndjson`: merged logical events in arrival order.
- `adis16470_bursts.bin`: exact ADIS transactions.
- `adxl375_acquisitions.bin` and `bmp581_acquisitions.bin`: register-level SPI
  acquisitions including separate status reads.
- `gnss_solutions.bin` and `gnss_pps.bin`: versioned generic receiver packets.
- `states.csv`: state, covariance diagonal, local epoch, publication epoch, and
  synchronized GPS time.
- `validation.json`, `validation.md`, and `errors.png`.
- `manifest.json`: seed, versions, source/configuration hashes, artifact hashes,
  flight summary, and calibration status.

The generic `GnssReceiverAdapter` isolates simulated receiver messages from the
canonical ECEF solution used by the estimator.

## Verification

The current suite has 43 tests covering frames, WGS84 conversions, quaternion
physics, raw sensor scales, codecs, timing, shared-bus ordering, deterministic
replay, high-g handoff, delayed aiding, PPS synchronization, fault campaigns,
and covariance invariants. Statistical checks use 200 fixed seeds.

The reference seed-42 Andromeda run passes all integration gates with 54,693
events, 1,362 delayed rewinds, and zero history misses. Its reported RMS errors
are 0.149 m position, 0.092 m/s velocity, and 1.121 degrees attitude. These are
reference results, not final flight acceptance limits.

## Simulation scope

The simulation covers the stationary pad period through apogee. Descent,
fixed-point behavior, calibrated sensor parameters, and final acceptance limits
are outside the current simulation scope. Generic and vehicle-specific defaults
remain clearly identified in each run manifest.
