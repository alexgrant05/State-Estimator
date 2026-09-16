# Digital Twin

This directory contains the installable Python simulation and reference
15-state ESKF for the Cornell Rocketry state estimator.

```text
10 s pad alignment + Andromeda RocketPy truth at 2000 Hz
    -> BNO085, ADXL375, BMP581, and ZED-F9P
    -> 100 MHz timestamped hardware events
    -> arrival-ordered replay and delayed aiding
    -> states, metrics, plots, hashes, and validation gates
```

## Active sensor stack

- BNO085 on dedicated 3 MHz SPI mode 3: acceleration at 500 Hz, uncalibrated
  gyro and bias at 400 Hz, and diagnostic game rotation vector at 100 Hz.
- ADXL375 on the auxiliary SPI bus: 800 Hz high-g acceleration. The estimator
  enters ADXL mode at 85 percent of the BNO range and returns below 75 percent
  after a 25 ms hold and consistency check.
- BMP581 on the auxiliary SPI bus: 50 Hz pressure and temperature aiding.
- ZED-F9P on UART1 at 460800 baud: UBX NAV-PVT and NAV-COV at 5 Hz, UBX TIM-TP,
  and a separate 1 PPS TIMEPULSE edge.

The BNO game rotation vector is diagnostic only. GNSS begins prelocked and runs
standalone. Fixes are rejected above 4 g and recover after one continuous
second below 4 g. PPS time validity remains independent of position validity.

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

On Windows, replace `.venv/bin/python` with `.venv\Scripts\python.exe`. The run
command refuses to overwrite a nonempty directory. A failed validation gate
returns a nonzero status.

## Output

Each run contains:

- `events.ndjson`, the complete logical event stream including TIMEPULSE edges
- `bno085_shtp.bin`, exact SHTP transactions
- `adxl375_acquisitions.bin` and `bmp581_acquisitions.bin`
- `zed_f9p_uart.bin`, exact UBX frames
- `states.csv`, `validation.json`, `validation.md`, and `errors.png`
- `manifest.json`, schema version 3 with configuration, protocol settings,
  calibration status, source revision, dependency versions, and hashes

## Documentation

- [Architecture](docs/architecture.md)
- [Configuration](docs/configuration.md)
- [Truth and frames](docs/truth-and-frames.md)
- [BNO085](docs/sensors/bno085.md)
- [ADXL375](docs/sensors/adxl375.md)
- [BMP581](docs/sensors/bmp581.md)
- [ZED-F9P](docs/sensors/zed-f9p.md)
- [Events and replay](docs/events-and-replay.md)
- [Estimator](docs/estimator.md)
- [Validation and testing](docs/validation-and-testing.md)
- [Setup and CLI](docs/setup-and-cli.md)
- [Artifacts](docs/artifacts.md)
- [Development](docs/development.md)

## Scope

The current simulation covers pad alignment through apogee using float64. RTK,
correction services, descent, fixed-point behavior, the final FPGA to R5F packet
envelope, and flight acceptance limits remain out of scope.

Legacy ADIS16470 decoding is available only in `digital_twin.legacy` for old
replay inspection. It is not used by generation, estimation, or validation.
