# Multi-Sensor Digital Twin

This package is the Python reference implementation for the launch-to-apogee
state-estimation path:

```text
10 s pad + Andromeda RocketPy truth at 2000 Hz
    -> ADIS16470 + ADXL375 + BMP581 + generic GNSS/PPS
    -> timestamped events on the 100 MHz hardware clock
    -> merged arrival-order replay and shared auxiliary-SPI scheduling
    -> high-g selection + delayed 15-state ESKF fusion
    -> deterministic artifacts, plots, metrics, and pass/fail gates
```

The old digital-twin repository is not required at runtime and is not modified.

## Reference environment

Python 3.10 through 3.13 is supported. RocketPy is pinned to 1.12.1; the full
reference environment is recorded in `requirements-lock.txt`.

```powershell
cd sim
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.\.venv\Scripts\python.exe -m pip install --no-deps -e .
```

## Run and verify

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m digital_twin run `
  --config config\andromeda.toml `
  --seed 42 `
  --output outputs\andromeda-all-sensors-seed-42
.\.venv\Scripts\python.exe -m digital_twin validate `
  --run outputs\andromeda-all-sensors-seed-42
```

The run command refuses to overwrite a non-empty directory. Validation failure
returns a nonzero status.

## Conventions and default sensor profile

- Navigation is launch-centered east-north-up (ENU); MSL altitude is retained
  separately and GNSS crosses the estimator boundary in WGS84 ECEF.
- Quaternions are scalar-first and rotate body vectors into navigation.
- Sensor mounting rotations are always applied.
- Time is a 64-bit 100 MHz counter beginning at pad alignment.
- ADIS16470 defaults to 500 Hz (`DEC_RATE=3`) on dedicated 1 MHz SPI.
- ADXL375 defaults to 800 Hz and BMP581 to 50 Hz on serialized auxiliary SPI.
- Generic GNSS defaults to 10 Hz with a separate 1 Hz PPS event.
- GNSS position/velocity solutions retain measurement and arrival epochs. The
  ESKF rewinds within a two-second history, updates at measurement time, and
  repropagates to the publication epoch.
- ADIS acceleration is primary. ADXL takes over at 85% of the ADIS range and
  returns after a 75% threshold, hold interval, freshness check, and overlap
  consistency gate.
- BMP establishes pad pressure, is suppressed during boost/transonic flight,
  and uses pressure/NIS updates in coast.

All rates, errors, gates, mounts, lever arms, transport timing, and latency are
configuration-driven. Vehicle- and receiver-specific defaults remain explicitly
uncalibrated until the deferred review and bench-calibration steps are completed.

## Replay artifacts

Each run contains:

- `events.ndjson`: merged versioned logical events in arrival order.
- `adis16470_bursts.bin`: exact 176-bit ADIS transactions.
- `adxl375_acquisitions.bin`: deterministic ADXL register acquisitions.
- `bmp581_acquisitions.bin`: deterministic BMP register acquisitions.
- `gnss_solutions.bin` and `gnss_pps.bin`: versioned generic receiver packets.
- `states.csv`, `validation.json`, `validation.md`, and `errors.png`.
- `manifest.json`: configuration/source/artifact hashes, versions, seed, flight
  summary, and explicit calibration status.

The common FPGA/R5F packet envelope is deliberately not frozen. A future exact
receiver implements `GnssReceiverAdapter` and translates its wire messages into
the stable `GnssSolution`/`GnssPps` contracts.

## Verification

The suite covers frame and WGS84 conversions, quaternion physics, all raw
scales/codecs, exact timestamp spacing, shared-bus arbitration, deterministic
replay, high-g handoff, delayed GNSS rewind, PPS synchronization, sensor fault
campaigns, covariance invariants, 200-seed noise/coverage statistics, and one
complete Andromeda pad-to-apogee integration run.
