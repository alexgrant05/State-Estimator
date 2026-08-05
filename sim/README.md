# ADIS16470 Digital Twin

This directory contains the Python reference path for the first state-estimator
vertical slice:

```text
10 s stationary pad + Andromeda RocketPy truth at 2000 Hz
    -> ADIS16470 corruption, decimation, quantization, and burst read
    -> versioned events on a 100 MHz hardware timebase
    -> ADIS-only 15-state ESKF propagation
    -> deterministic replay, metrics, plots, and pass/fail gates
```

The old `cornell-rocketry-live-state-estimation-main` checkout is not required
at runtime and is not modified by this package.

## Reference environment

Python 3.10 through 3.13 is supported. The checked reference run uses Python
3.10.11, RocketPy 1.12.1, and the exact versions in
`requirements-lock.txt`.

```powershell
cd sim
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.\.venv\Scripts\python.exe -m pip install --no-deps -e .
```

For development without the lock file:

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[test]"
```

## Run and verify

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m digital_twin run `
  --config config\andromeda.toml `
  --seed 42 `
  --output outputs\andromeda-seed-42
.\.venv\Scripts\python.exe -m digital_twin validate `
  --run outputs\andromeda-seed-42
```

The run command refuses to overwrite a non-empty output directory. A failed
validation gate returns a nonzero exit code.

## Conventions

- Navigation coordinates are launch-centered east-north-up (ENU).
- Altitude MSL is stored independently from the local up coordinate.
- Quaternions are scalar-first and rotate body vectors into ENU.
- RocketPy body axes are retained; the body z axis points tail-to-nose.
- Sensor mounting always passes through the configured `sensor_to_body`
  rotation.
- Timestamps are unsigned 100 MHz counter ticks from the start of pad
  alignment. Liftoff is at 10 seconds by default.
- `DEC_RATE` accepts 0–1999. The output rate is
  `2000 / (DEC_RATE + 1)` SPS; the default is 500 SPS.

## Replay and reports

Each successful run creates:

- `events.ndjson` — versioned logical measurement envelopes, including
  measurement/arrival ticks, sequence, status, and response payload
- `adis16470_bursts.bin` — concatenated 176-bit, MSB-first transactions
  containing command `0x6800`, the ten response words, and checksum
- `states.csv` — state, biases, publication epoch, and covariance diagonal
- `validation.json` / `validation.md` — machine and human-readable gates
- `errors.png` — ADIS-only inertial drift against RocketPy truth
- `manifest.json` — seed, configuration/source hashes, dependency versions,
  flight summary, and artifact SHA-256 hashes

The binary file models the exact ADIS transaction, not the future common
multi-sensor FPGA packet envelope.

## What is verified

The test and integration gates cover frames and gravity sign, quaternion math,
two's-complement register scaling, checksum vectors, counter wrapping,
decimation timing, deterministic replay, stationary/constant-motion
mechanization, saturation, checksum/diagnostic/counter/loss faults, covariance
health, and 200-seed noise/covariance coverage.

Position, velocity, and attitude drift are reported but are not full-navigation
acceptance gates until barometer and GNSS aiding are implemented.

