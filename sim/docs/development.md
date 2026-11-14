# Development Guide

Keep typed boundaries, integer hardware ticks, separate measurement and arrival
epochs, raw payloads at transport boundaries, and deterministic random streams.
Unknown hardware errors remain explicit zero-valued calibration inputs.

When adding or changing a sensor:

1. Allocate a new ID without reusing reserved values.
2. Add typed configuration and validation.
3. Generate `MeasurementEvent` objects from truth.
4. Implement exact framing, integrity checks, and bus timing.
5. Add replay writing and truncation-safe reading.
6. Decode into an estimator-facing canonical type.
7. Add scale, timing, golden, fault, deterministic, statistical, and integration
   tests.
8. Update the manifest protocol metadata and simulation documentation.

ZED estimator logic depends on `CanonicalGnssFix`, not raw UBX offsets. A later
module suffix or protocol profile should change only the receiver codec and
configuration, unless its information content changes.

The next parity layer should feed the committed SHTP, auxiliary SPI, and UBX
replays into RTL and R5F decoders and compare the resulting canonical events.
