# Digital Twin

This directory contains the installable Python reference simulation for the
Cornell Rocketry state estimator. It models launch-to-apogee truth, sensor
measurements, acquisition timing, replay files, delayed sensor fusion, and
validation.

```text
10 s pad alignment + RocketPy truth at 2000 Hz
    -> ADIS16470, ADXL375, BMP581, and generic GNSS/PPS
    -> 100 MHz timestamped measurement events
    -> arrival-ordered sensor replay
    -> high-g selection and delayed 15-state ESKF
    -> states, metrics, plots, hashes, and validation gates
```

## Quick start

Python 3.10 through 3.13 is supported. Linux is the reference environment.

```bash
cd sim
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
.venv/bin/python -m pip install --no-deps -e .
.venv/bin/python -m pytest -q

.venv/bin/python -m digital_twin run \
  --config config/andromeda.toml \
  --seed 42 \
  --output outputs/andromeda-seed-42

.venv/bin/python -m digital_twin validate \
  --run outputs/andromeda-seed-42
```

On Windows, replace `.venv/bin/python` with `.venv\Scripts\python.exe`. The
`run` command refuses to overwrite a non-empty output directory. A failed
validation gate returns a nonzero status.

## Documentation

- [Architecture](docs/architecture.md): subsystem boundaries and data flow.
- [Setup and CLI](docs/setup-and-cli.md): installation, commands, and exit codes.
- [Configuration](docs/configuration.md): every TOML section and constraint.
- [Truth and frames](docs/truth-and-frames.md): RocketPy conversion, ENU, time,
  quaternions, atmosphere, and analytic truth.
- [ADIS16470](docs/sensors/adis16470.md): inertial model and exact burst format.
- [ADXL375](docs/sensors/adxl375.md): high-g model and acquisition format.
- [BMP581](docs/sensors/bmp581.md): pressure model and acquisition format.
- [GNSS and PPS](docs/sensors/gnss.md): receiver-neutral model and adapter boundary.
- [Events and replay](docs/events-and-replay.md): logical schema, timing, ordering,
  binary files, and reproducibility.
- [Estimator](docs/estimator.md): initialization, propagation, high-g selection,
  aiding, rewind/replay, covariance, and health counters.
- [Validation and testing](docs/validation-and-testing.md): test suite, metrics,
  gates, fault campaigns, and statistical checks.
- [Artifacts](docs/artifacts.md): files produced by a run and how to inspect them.
- [Development guide](docs/development.md): adding sensors, codecs, tests, and
  calibrated hardware values.

## Current scope

The simulation covers the stationary pad period through apogee. It uses
float64 estimation and a generic GNSS wire format. Descent, the final FPGA to
R5F packet envelope, fixed-point behavior, receiver-specific GNSS framing, and
flight acceptance limits remain outside the current scope. Values that require
bench or vehicle calibration are identified in each run manifest.
