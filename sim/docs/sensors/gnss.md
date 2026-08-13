# GNSS and PPS Model

## Role and replacement boundary

The current model provides receiver-neutral navigation solutions and PPS events.
`GnssReceiverAdapter` defines the stable decoder boundary:

```python
decode_solution(payload: bytes) -> GnssSolution
decode_pps(payload: bytes) -> GnssPps
```

When the receiver is selected, add an adapter for its wire format while keeping
the canonical ECEF solution, event timing, and estimator interface unchanged.

## Navigation solution

The default rate is 10 Hz. For each solution the model:

1. Interpolates truth at the solution epoch.
2. Applies antenna position and rotational velocity lever arms.
3. Adds ENU position and velocity bias random walks.
4. Adds independent Gaussian ENU noise.
5. Converts position and velocity to WGS84 ECEF.
6. Rotates the configured 6 by 6 ENU covariance into ECEF.
7. Assigns GPS week and time of week.
8. Adds nonnegative Gaussian transport latency.

Solution noise, latency, and outage state use separate deterministic random
streams. The measurement epoch is the solution epoch; arrival is measurement
plus sampled latency.

## Generic solution payload

The current little-endian payload is 352 bytes:

| Field | Type |
| --- | --- |
| GPS week | unsigned 16-bit |
| Time of week | unsigned 64-bit nanoseconds |
| ECEF position | three float64 values in meters |
| ECEF velocity | three float64 values in m/s |
| Covariance | 36 float64 row-major values |
| Fix type | unsigned 8-bit |
| Satellites | unsigned 8-bit |
| Correction age | float32 seconds |

Valid generic fixes report fix type 3 and 12 satellites. Invalid fixes report
zero for both and set `FIX_INVALID` instead of `VALID`.

## Outages and faults

The model supports correlated outages with per-solution entry and recovery
probabilities. An optional acceleration threshold can force outage state.
`GnssFaultSchedule` adds deterministic solution loss, invalid fixes, PPS loss,
and additional per-solution latency.

Outage and loss omit solution events while sequence numbers continue.

## PPS

The default PPS rate is 1 Hz. GPS time is computed at the nominal PPS epoch.
The local measurement edge includes clock offset, clock drift, and Gaussian
jitter. PPS has no additional transport delay in the current model, so
measurement and arrival ticks are equal.

The little-endian 19-byte payload contains GPS week, time-of-week nanoseconds,
float64 uncertainty in nanoseconds, and one validity byte.

## Time synchronization

The estimator retains up to 32 `(local_ticks, gps_time_ns)` pairs. One point
sets the intercept using the nominal clock slope. Two or more points fit slope
and intercept by least squares, allowing clock drift estimation. Each published
state includes synchronized GPS nanoseconds when a PPS solution exists. Maximum
fit residual is exposed through estimator health.

## Estimator update

GNSS ECEF position, velocity, and covariance are transformed back to ENU. The
measurement model includes antenna position and rotational velocity lever arms.
A six-dimensional NIS gate accepts or rejects the update. Delayed solutions are
applied at their measurement epoch through state rewind and IMU replay.

## Receiver integration checklist

When an exact receiver is selected:

1. Preserve `GnssSolution` and `GnssPps` as canonical decoded objects.
2. Implement receiver framing, checksum, units, validity, and covariance rules.
3. Replace the generic binary replay codec with exact recorded wire messages.
4. Add receiver latency, navigation epoch, PPS alignment, outage, and startup
   measurements from bench testing.
5. Add golden decode vectors and corrupted-frame tests.
6. Keep measurement time distinct from byte-complete arrival time.
