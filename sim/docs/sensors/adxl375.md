# ADXL375 Model

## Role

The ADXL375 supplies high-range specific force when ADIS acceleration approaches
its 40 g limit. It shares the auxiliary SPI bus with the BMP581.

## Sampling and timing

- Default output rate: 800 Hz
- Maximum supported rate: 800 Hz
- Measurement timestamp: scheduled data-ready epoch
- Unsheduled model arrival: measurement plus a 72-bit acquisition
- Final arrival: assigned by shared SPI arbitration
- Default SPI clock: 5 MHz

The output rate must divide the 100 MHz hardware clock exactly. Truth is
interpolated to each ADXL measurement epoch.

## Measurement chain

The model reconstructs body specific force, rotates it into sensor axes, applies
small-angle misalignment, per-axis scale error, bias random walk, constant bias,
and Gaussian white noise. Noise standard deviation is derived from density as:

```text
sigma = density * gravity * sqrt(output_rate_hz / 2)
```

Independent random state is derived from `[run_seed, ADXL375 sensor ID]`.

## Raw scaling and acquisition

- Scale: 20.5 LSB/g
- Modeled range: plus or minus 200 g
- Axis encoding: signed little-endian 16-bit counts
- Logical payload: X, Y, Z counts plus interrupt source, 7 bytes
- Binary transaction: read command `0xF2`, six data bytes, status command
  `0xB0`, and interrupt source, 9 bytes
- Default interrupt source: `0x80`

Clipping sets `SATURATED`.

## Fault injection

`AdxlFaultSchedule` supports:

- overrun status
- packet loss
- repeated or stuck acquisition

An overrun sets `OVERRUN` but leaves the payload valid. Packet loss omits the
event while sequence numbering continues.

## High-g selection

The estimator stores the newest ADXL measurement and removes its pad-estimated
bias. ADXL is eligible only when it is recent and not saturated. Entry occurs
when any ADIS acceleration axis exceeds `high_g_enter_fraction * 40 g` or ADIS
reports saturation. Exit requires ADIS below the lower threshold, acceptable
ADIS to ADXL overlap NIS, and the configured hold time.

When ADXL is active, its configured noise density drives acceleration process
noise. ADIS gyro remains the angular-rate source.

## Calibration status

The default mounting is identity, and bias, random walk, scale, and misalignment
are placeholders. The final configuration should use bench calibration at the
selected bandwidth and installed vehicle orientation.
