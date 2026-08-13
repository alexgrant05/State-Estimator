# Architecture

## Purpose

The digital twin is the executable reference for sensor behavior, timestamp
semantics, replay formats, estimator behavior, and validation. It is designed
to provide inputs and expected outputs for later RTL and R5F implementations.

## Pipeline

1. `truth.py` generates 2000 Hz launch-to-apogee truth and prepends the pad
   alignment interval.
2. Each sensor model converts truth into a raw payload and a
   `MeasurementEvent` using an independent deterministic random stream.
3. `pipeline.py` schedules ADXL375 and BMP581 transactions on their shared SPI
   bus, then sorts all sensors by arrival time.
4. `transport.py` writes the versioned NDJSON event stream and one binary replay
   file per sensor.
5. `eskf.py` decodes the replayed events, initializes on the pad, propagates on
   ADIS data-ready epochs, selects ADXL acceleration when required, and applies
   delayed barometer and GNSS updates.
6. `validation.py` compares estimates with truth, checks invariants and replay
   structure, runs fixed-seed statistics, and writes reports.
7. `cli.py` records configuration, dependency, source revision, and artifact
   hashes in the run manifest.

## Package map

| Module | Responsibility |
| --- | --- |
| `types.py` | Shared typed contracts, sensor IDs, and status flags |
| `config.py` | TOML loading, typed configuration, and cross-field checks |
| `frames.py` | Quaternion and ENU frame operations |
| `geodesy.py` | WGS84 geodetic, ECEF, and ENU conversions |
| `truth.py` | RocketPy truth, analytic truth, interpolation, and atmosphere |
| `adis16470.py` | Low-g IMU physics, quantization, faults, and decoding |
| `adxl375.py` | High-g accelerometer physics, faults, and decoding |
| `bmp581.py` | Pressure and temperature physics, faults, and decoding |
| `gnss.py` | Generic ECEF solution, PPS, latency, outage, and adapter model |
| `pipeline.py` | Multi-sensor generation and auxiliary SPI scheduling |
| `transport.py` | Logical events and exact per-sensor binary codecs |
| `eskf.py` | Arrival-ordered 15-state ESKF and delayed updates |
| `validation.py` | Metrics, gates, statistics, state CSV, report, and plot |
| `cli.py` | `run` and `validate` commands plus manifest generation |

## Stable boundaries

The main boundaries are typed objects rather than CSV column positions:

- `TruthSample` connects truth generation to sensor models.
- `MeasurementEvent` connects sensor models, transport, replay, and estimation.
- Sensor codec objects connect logical payloads to exact binary transactions.
- `StateEstimate` connects the estimator to validation and output writers.

Measurement time and arrival time are intentionally separate. Estimator state
evolves at measurement epochs, while estimates are published at packet arrival
epochs. This distinction is required for transport delays and delayed aiding.

## Determinism

Each subsystem derives its random generator from the run seed and its sensor ID.
GNSS further separates solution noise, latency, outage, and PPS streams. Adding
random draws to one sensor therefore does not change another sensor's replay.
Determinism assumes the same configuration, dependency versions, and source
revision, all of which are recorded in `manifest.json`.
