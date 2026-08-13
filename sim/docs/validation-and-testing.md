# Validation and Testing

## Test suite

Run from `sim/`:

```bash
python -m pytest -q
```

The focused test files are:

| File | Coverage |
| --- | --- |
| `test_frames.py` | Quaternion algebra, orthonormality, vector alignment, rail attitude |
| `test_adis16470.py` | Scale vectors, saturation, decimation timing, deterministic streams, faults, statistics |
| `test_transport.py` | Checksum, exact golden transaction, event JSON round trip |
| `test_eskf.py` | Pad initialization, stationary and analytic propagation, fault stability |
| `test_multisensor.py` | ADXL, BMP, GNSS, shared SPI, high-g handoff, delayed aiding, PPS, replay, faults |

Tests use analytic truth when exact expected motion is required and the full
Andromeda configuration for integration behavior.

## Run-time metrics

`calculate_metrics` compares estimates with truth at matching state ticks and
reports RMS, maximum, and final norm error for:

- position in meters
- velocity in meters per second
- attitude in degrees

These values are reported for characterization. They are not currently hard
navigation-accuracy gates because final calibration and receiver selection are
unfinished.

## Integration gates

Every normal run evaluates:

| Gate | Pass condition |
| --- | --- |
| `events_present` | At least one logical event exists |
| `estimates_present` | At least one state estimate exists |
| `checksum_clean` | No ADIS checksum failures in nominal replay |
| `diagnostics_clean` | No ADIS diagnostic errors in nominal replay |
| `no_unexpected_saturation` | No sensor event reports saturation |
| `sequence_clean` | Estimator saw no logical sequence discontinuity |
| `counter_clean` | Estimator saw no ADIS counter discontinuity |
| `aiding_history_clean` | No delayed aid fell outside retained history |
| `aiding_updates_present` | Enabled GNSS and BMP each produced an accepted update |
| `pps_synchronized` | Enabled GNSS produced at least two accepted PPS points |
| `timestamps_exact` | Per-sensor measurement spacing matches configured rate |
| `replay_round_trip` | Binary record sizes and counts match logical events |
| `states_finite` | All nominal state and covariance values are finite |
| `quaternion_normalized` | Maximum norm error is below `1e-12` |
| `covariance_symmetric` | Maximum asymmetry is below `1e-10` |
| `covariance_psd` | Minimum eigenvalue is at least `-1e-12` |
| `noise_statistics` | Fixed-seed sensor moment checks pass |
| `covariance_coverage` | Position and velocity empirical coverage each fall from 92 to 98 percent |

Overall pass requires every gate to pass.

## Timing checks

Expected measurement intervals are calculated in hardware ticks from each
configured output rate. ADIS uses its exact decimation interval. All intervals
must match exactly except PPS, which allows a six-sigma tick tolerance derived
from configured jitter.

## Statistical checks

Each run performs a 200-seed short stationary campaign by default:

- ADIS X gyro mean and standard deviation after decimation
- ADXL X acceleration mean and standard deviation when enabled
- BMP pressure mean error and standard deviation when enabled
- GNSS east-position mean and standard deviation when enabled

Mean gates include three standard errors plus quantization allowance where
applicable. Standard deviations must be within 25 percent of configured values.

An independent discrete white-acceleration propagation experiment evaluates
nominal 95 percent position and velocity coverage over the same fixed seeds.
Both empirical coverages must be within 92 through 98 percent.

## Fault campaigns

Unit and integration tests inject deterministic:

- ADIS checksum, diagnostic, duplicate counter, skipped counter, and loss faults
- ADXL overrun, loss, and stuck data
- BMP invalid status, loss, and pressure spikes
- GNSS invalid fixes, loss, PPS loss, outage, and added latency

Tests confirm rejection, health accounting, deterministic output, and finite
filter behavior. Faulted runs are not expected to pass nominal clean-data gates.

## Golden replay

`tests/fixtures/golden_adis_transaction.hex` stores a compact exact ADIS
transaction, and its JSON companion stores expected decoded fields. The test
computes the checksum independently, verifies transaction shape, and checks
round-trip decoding.

## CI expectations

CI should install from `requirements-lock.txt`, install the package editable
without dependency resolution, and run the complete test suite. A later
hardware parity job should replay the same committed binary fixtures through
RTL and R5F decoders and compare canonical events and estimator inputs.
