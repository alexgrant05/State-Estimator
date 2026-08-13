# Run Artifacts

A successful `digital_twin run` writes all artifacts into the requested output
directory. The directory must be empty or absent before the run.

## Logical and binary sensor data

| Artifact | Purpose |
| --- | --- |
| `events.ndjson` | Complete arrival-ordered logical event stream with timestamps and status |
| `adis16470_bursts.bin` | Exact 22-byte ADIS SPI transactions |
| `adxl375_acquisitions.bin` | Exact modeled 9-byte ADXL acquisitions |
| `bmp581_acquisitions.bin` | Exact modeled 9-byte BMP acquisitions |
| `gnss_solutions.bin` | Generic 352-byte canonical GNSS solution records |
| `gnss_pps.bin` | Generic 19-byte PPS records |

Binary files are raw fixed-size record concatenations without headers. Use the
NDJSON file for timing, status, sequence, and cross-sensor ordering.

## State output

`states.csv` contains one row for each published estimate. Columns are:

- state and publication ticks
- synchronized GPS nanoseconds, blank before PPS synchronization
- ENU position `px_m`, `py_m`, `pz_m`
- ENU velocity `vx_mps`, `vy_mps`, `vz_mps`
- scalar-first quaternion `qw`, `qx`, `qy`, `qz`
- accelerometer bias `bax_mps2`, `bay_mps2`, `baz_mps2`
- gyro bias `bgx_rps`, `bgy_rps`, `bgz_rps`
- covariance diagonal `pdiag_0` through `pdiag_14`

The CSV does not contain the off-diagonal covariance. Use the in-process
`StateEstimate` objects when full covariance analysis is required.

## Validation output

- `validation.json`: machine-readable gates, counts, timing, errors, invariants,
  health, and Monte Carlo statistics.
- `validation.md`: concise human-readable flight summary, errors, gates, and
  health counters.
- `errors.png`: position, velocity, and attitude error versus simulation time.

`digital_twin validate` reads only `validation.json` and reports its saved gate
results.

## Manifest

`manifest.json` has schema version 2 and records:

- actual seed
- Git source revision when available
- absolute configuration source path and SHA-256
- digital twin, Python, NumPy, Matplotlib, and RocketPy versions
- flight truth summary
- calibration status and deferred hardware values
- size and SHA-256 for events, states, validation JSON, and binary replays
- overall pass result

The plot, Markdown report, and manifest itself are not currently included in the
artifact hash map.

## Comparing deterministic runs

For the same source, dependency versions, configuration bytes, and seed, logical
events and binary replay should be byte-identical. Compare the SHA-256 entries
in `manifest.json`. State and validation files should also match under the same
numeric environment, but replay equality is the primary sensor golden check.

Do not reuse an output directory. Use names that encode configuration and seed,
for example `outputs/andromeda-all-sensors-seed-42`.
