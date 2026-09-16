# Validation and Testing

Run the suite with:

```bash
cd sim
.venv/bin/python -m pytest -q
```

The tests cover quaternion and ENU physics, BNO Q8/Q9/Q14 encoding, physical
quantization and saturation, SHTP lengths and continuation, report rates,
startup traffic, ADXL and BMP scales, UBX checksums and signed fields, PVT and
covariance assembly, UART serialization, PPS pairing, deterministic replay,
fault rejection, analytic ESKF trajectories, and covariance invariants.

Golden fixtures cover current BNO startup traffic and a ZED TIM-TP frame.

Every run checks:

- active events and estimates are present
- nominal SHTP and UBX packets decode cleanly
- any expected BNO saturation interval is covered by ADXL
- gyro and auxiliary sensors do not saturate unexpectedly
- logical and SHTP sequences are continuous
- invalid GNSS fixes are never applied and valid fixes recover
- PPS is paired and synchronized
- report spacing and replay round trips are correct
- states are finite, quaternion norm error is below `1e-12`, covariance
  asymmetry is below `1e-10`, and minimum eigenvalue is at least `-1e-12`
- configured sensor statistics and nominal covariance coverage pass

The fixed 200-seed campaign evaluates BNO acceleration and gyro, ADXL, BMP, and
ZED configured moments. Zero-valued bench placeholders are tested as
deterministic quantities. Nonzero process noise uses a 92 to 98 percent gate for
nominal 95 percent empirical coverage.

The Andromeda integration run covers pad through apogee and reports position,
velocity, ESKF attitude, and diagnostic game-vector errors without imposing the
old unaided navigation limits.
