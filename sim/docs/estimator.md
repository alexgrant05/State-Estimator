# 15-State Error-State Kalman Filter

The float64 nominal state contains ENU position and velocity, a scalar-first
body-to-navigation quaternion, accelerometer bias, and gyro bias. The covariance
ordering is position, velocity, attitude error, accelerometer bias, gyro bias.

Events are consumed by arrival time but state propagation uses measurement
time. Pad initialization averages BNO acceleration and gyro, preserves rail
heading for yaw, estimates tilt from gravity, calculates both BNO and ADXL pad
bias, and records mean pad pressure.

The BNO reports are asynchronous. A gyro report creates a propagation epoch
using the newest valid BNO acceleration. Missing, future, or older than two
acceleration periods is rejected and counted. The game rotation vector is never
injected; validation compares it to truth as a diagnostic.

When any BNO acceleration axis reaches 85 percent of 8 g or saturates, the
estimator switches to the newest valid ADXL sample. Return requires all BNO axes
below 75 percent, acceptable overlap consistency, and 25 ms continuously below
the threshold. BNO gyro remains active in both modes.

BMP pressure is converted to pad-relative altitude and gated during boost and
the transonic band. ZED NAV-PVT and NAV-COV are joined by iTOW, converted from
geodetic and NED to launch-centered ENU, checked for `gnssFixOK`, then applied
at their navigation epoch. Delayed BMP and GNSS updates restore a retained
snapshot and replay inertial history to the latest epoch.

TIMEPULSE edges are paired with UBX-TIM-TP before updating the local tick to GPS
time fit. Position validity does not control time validity.

Health counters cover logical and SHTP sequence gaps, malformed packets,
asynchronous age, saturation and source switching, invalid GNSS fixes, PPS
pairing, aiding acceptance, rewind, propagation, and flight phase.
