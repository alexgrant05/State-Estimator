# Architecture

The digital twin is the executable reference for later FPGA and R5F work.

1. `truth.py` creates 2000 Hz launch-centered ENU truth with a 10 second pad
   segment and RocketPy flight through apogee.
2. `bno085.py`, `adxl375.py`, `bmp581.py`, and `zed_f9p.py` create raw sensor
   packets using independent deterministic random streams.
3. `pipeline.py` serializes the shared ADXL and BMP SPI bus and merges all
   events by arrival time. BNO uses dedicated SPI. ZED uses UART1 and a separate
   TIMEPULSE edge.
4. `transport.py` writes NDJSON metadata and exact sensor byte streams.
5. `eskf.py` initializes on the pad, propagates on BNO gyro epochs using the
   newest BNO or ADXL acceleration, and applies delayed BMP and GNSS updates.
6. `validation.py` checks protocol, physics, timing, replay, estimator
   invariants, statistical behavior, and truth error.

The stable boundaries are `TruthSample`, `MeasurementEvent`, decoded inertial
reports, `CanonicalGnssFix`, `TimePulse`, and `StateEstimate`. Measurement time
and arrival time remain separate throughout the pipeline.

Active wire objects are `BnoShtpPacket`, `AdxlAcquisition`, `BmpAcquisition`,
and `UbxFrame`.
