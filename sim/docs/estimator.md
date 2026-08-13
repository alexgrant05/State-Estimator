# 15-State Error-State Kalman Filter

## Nominal state and error ordering

The nominal state contains:

- ENU position, 3 values
- ENU velocity, 3 values
- scalar-first body-to-navigation quaternion, 4 values
- body-frame accelerometer bias, 3 values
- body-frame gyro bias, 3 values

The 15-element error state and covariance ordering is:

```text
[position(3), velocity(3), attitude_error(3), accel_bias(3), gyro_bias(3)]
```

All calculations use NumPy float64.

## Arrival-ordered processing

Events are consumed in arrival order. Sequence continuity is tracked separately
for every sensor. ADIS also tracks its wrapping 16-bit hardware counter.

The state is propagated using `measurement_ticks`, not arrival time. Every
published `StateEstimate` contains:

- `state_ticks`: epoch represented by the nominal state
- `publication_ticks`: arrival epoch that triggered publication
- nominal state and covariance
- validity and accumulated health counters
- synchronized GPS nanoseconds when PPS synchronization is available

## Pad initialization

Valid ADIS, ADXL, and BMP measurements before liftoff are collected. On the first
valid post-liftoff ADIS event, initialization performs:

1. Mean ADIS accelerometer and gyro calculation.
2. Rail attitude calculation from configured inclination and heading.
3. Tilt correction that aligns measured acceleration with expected rail up.
4. Yaw retention from rail heading.
5. Gyro bias initialization from mean stationary gyro.
6. Accelerometer bias initialization from measured minus expected gravity.
7. ADXL pad bias initialization using the same expected specific force.
8. Barometer reference pressure initialization from the pad mean.
9. Diagonal covariance initialization from configured standard deviations.

Position and velocity begin at zero. Initialization fails if no valid pad ADIS
samples exist.

## Inertial propagation

At every valid, increasing ADIS measurement epoch:

```text
gyro = gyro_measurement - gyro_bias
accel = selected_acceleration - applicable_bias
a_nav = R(q) @ accel + [0, 0, -g]
p = p + v*dt + 0.5*a_nav*dt^2
v = v + a_nav*dt
q = normalize(q * exp_quaternion(gyro*dt))
```

ADIS accelerometer bias is removed in low-g mode. ADXL data has its pad bias
removed before selection, so the stored ESKF ADIS bias is not subtracted again.

Covariance uses a first-order state transition and additive process covariance.
The matrix is explicitly symmetrized after propagation.

## High-g selection

ADXL is considered fresh when its epoch is no later than the current ADIS epoch,
its age is within `high_g_max_age_samples` ADXL periods, and it is not saturated.

Entry into high-g mode requires ADIS saturation or an ADIS axis above the entry
fraction of 40 g, plus a fresh ADXL sample. Return to ADIS requires:

- all ADIS axes below the lower exit threshold
- acceptable ADIS versus ADXL overlap NIS
- continuous satisfaction for `high_g_exit_hold_s`

This hysteresis prevents rapid source toggling. ADIS gyro remains active in both
modes.

## Measurement updates

The common injection step computes innovation covariance, NIS, Kalman gain, and
state correction. Attitude correction is applied with an exponential quaternion.
Covariance uses the Joseph form and is symmetrized.

### BMP581

Pressure is converted to standard pressure altitude and referenced to mean pad
pressure. The scalar measurement observes ENU up position. Updates are rejected
during boost and in the configured transonic Mach proxy band, then protected by
the barometer NIS gate.

### GNSS

ECEF position, velocity, and covariance are transformed to ENU. Predicted
antenna position and velocity include the configured body lever arm and current
angular rate. The six-dimensional update observes position and velocity plus
attitude and gyro-bias effects from the lever arm. It uses the GNSS NIS gate.

## Delayed aiding and rewind/replay

The filter keeps IMU inputs, accepted aid inputs, and state snapshots for
`history_duration_s`. When a delayed aid arrives:

1. Reject it if its measurement epoch is outside retained history or ahead of
   the current propagated state.
2. Restore the latest snapshot before the aid epoch.
3. Merge stored IMU and aid operations by measurement epoch.
4. Apply IMU first, BMP second, and GNSS third at equal epochs.
5. Apply the new update at its original measurement epoch.
6. Replay to the latest state and replace the affected snapshots.

Aid that arrives before propagation reaches its measurement epoch is queued and
drained after a later IMU propagation. Old history is pruned while retaining a
base snapshot.

## Flight phases

- `PAD`: before configured liftoff
- `BOOST`: from liftoff through configured motor burn time
- `COAST`: after burnout while vertical velocity is positive
- `APOGEE`: after vertical velocity becomes nonpositive

Phase changes are counted in health telemetry. BMP logic uses phase and Mach
proxy suppression independently.

## Time synchronization

PPS events fit GPS nanoseconds as a linear function of local ticks. Up to 32
points are retained. Published states expose the fitted GPS epoch and health
reports maximum residual.

## Rejection and health behavior

Malformed, checksum-failed, diagnostic-failed, invalid, out-of-history, gated,
and nonmonotonic inputs are rejected without applying a state update. Health is
a dynamic counter map. Important families include:

- per-sensor sequence discontinuities and inferred packet loss
- ADIS checksum, diagnostic, counter, saturation, and decode failures
- ADXL overruns, freshness failures, overlap rejection, and source switches
- BMP and GNSS accepted, rejected, queued, and history-missed updates
- PPS updates, invalid messages, decode failures, and fit residual
- propagation count, rewinds, initialization, and phase entries

Validation reads these counters rather than relying only on navigation error.

## Current limitations

The covariance discretization is first order. Earth rotation, Coriolis, gravity
variation, coning/sculling compensation, fixed-point arithmetic, and descent
logic are not modeled. The current process-noise conversion includes reference
bandwidth constants and should be revisited when flight sensor filtering is
frozen.
