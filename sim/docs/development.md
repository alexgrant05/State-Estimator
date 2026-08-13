# Digital Twin Development Guide

## Design rules

- Keep truth, measurement, and state interfaces typed.
- Keep measurement and arrival epochs separate.
- Use integer hardware ticks for all event timing.
- Keep sensor payloads raw and units explicit at decode boundaries.
- Give every random subsystem a deterministic seed namespace.
- Treat unknown hardware errors as uncalibrated zero values, not invented
  probability distributions.
- Add focused analytic tests before relying on a RocketPy integration run.
- Preserve backward compatibility or increment the relevant schema version.

## Add or modify a sensor

1. Add a `SensorId` value without renumbering existing IDs.
2. Add a typed configuration dataclass, loader defaults, and validation.
3. Implement model generation from `TruthSample` to `MeasurementEvent`.
4. Derive random state from the run seed and sensor ID. Use extra namespace
   integers when noise, latency, outage, or clock streams should be independent.
5. Implement an exact payload codec and binary transaction codec.
6. Add the replay filename, record writer, and record-size validation.
7. Add the model to `generate_all_events` and model bus contention if shared.
8. Add estimator decoding, validity handling, health counters, and update logic.
9. Add scale vectors, timing, codec round trips, faults, determinism, statistics,
   and integration tests.
10. Document calibration status and the hardware evidence required to replace
    each placeholder.

## Replace the generic GNSS receiver

Keep `GnssSolution` and `GnssPps` as canonical estimator inputs. Implement a new
`GnssReceiverAdapter` that converts receiver bytes and metadata into those
objects. Exact receiver replay should preserve byte framing and integrity checks,
while logical events preserve the solution epoch and byte-complete arrival.

Update golden vectors for valid, invalid, truncated, checksum-failed, week
rollover, delayed, and outage cases. Do not couple the ESKF directly to a vendor
message layout.

## Add a configuration field

Add the field to the appropriate frozen dataclass, TOML reference file, and
loader defaults if the section is optional. Add dimensional and cross-field
validation in `load_config`. Update `configuration.md` with units, meaning,
allowed range, and whether the value is calibrated.

Configuration hashing uses the original TOML bytes, so any change creates a new
run identity even when the resulting numeric behavior is unchanged.

## Add a validation gate

Add the measured value to `calculate_metrics`, then add a named boolean under
`gates`. The gate name becomes public output used by the CLI and CI, so choose a
stable, specific name. Add both passing and intentionally failing tests. If the
gate depends on hardware calibration that does not yet exist, report the metric
without gating it.

## Testing layers

1. Pure math and frame unit tests.
2. Raw scale, endian, checksum, and clipping vectors.
3. Exact rate and timestamp tests.
4. Encode and decode round trips.
5. Analytic trajectory propagation and update tests.
6. Deterministic fault campaigns.
7. Fixed-seed statistical checks.
8. Full Andromeda launch-to-apogee integration.
9. Future RTL and R5F replay parity.

## Documentation update checklist

When behavior changes, update:

- the relevant subsystem page
- `configuration.md` for parameters or constraints
- `events-and-replay.md` for schema, timing, or payload changes
- `artifacts.md` for output changes
- `validation-and-testing.md` for tests, metrics, or gates
- the documentation index in `sim/README.md`

Documentation should describe current implemented behavior separately from
planned behavior and hardware calibration work.
