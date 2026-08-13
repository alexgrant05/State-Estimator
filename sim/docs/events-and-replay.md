# Events, Timing, and Replay

## MeasurementEvent schema

Every sensor output uses format version 1:

| Field | Meaning |
| --- | --- |
| `format_version` | Logical schema version, currently exactly 1 |
| `sensor_id` | Numeric member of `SensorId` |
| `sequence_number` | Wrapping unsigned 32-bit sensor sequence |
| `measurement_ticks` | Physical sample or data-ready epoch |
| `arrival_ticks` | Epoch when the complete payload is available |
| `status_flags` | Bit mask from `StatusFlag` |
| `payload` | Sensor-specific raw bytes |

Construction rejects unsupported versions, negative measurement time, arrival
before measurement, and sequence values outside unsigned 32-bit range.

## Sensor IDs

| Value | Sensor |
| ---: | --- |
| 1 | ADIS16470 |
| 2 | ADXL375 |
| 3 | BMP581 |
| 4 | GNSS solution |
| 5 | GNSS PPS |

## Status flags

| Bit | Name | Meaning |
| ---: | --- | --- |
| 0 | `VALID` | Payload is eligible for normal consumption |
| 1 | `SATURATED` | At least one encoded channel clipped |
| 2 | `DIAGNOSTIC_ERROR` | Sensor diagnostic or invalid status |
| 3 | `CHECKSUM_ERROR` | Known checksum corruption |
| 4 | `SEQUENCE_DISCONTINUITY` | Reserved logical continuity indication |
| 5 | `PACKET_LOSS` | Reserved explicit loss indication |
| 6 | `OVERRUN` | Acquisition overrun |
| 7 | `STALE` | Data is older than allowed |
| 8 | `FIX_INVALID` | Navigation solution has no valid fix |
| 9 | `TIME_INVALID` | Time solution is invalid |
| 10 | `OUT_OF_ORDER` | Event arrived outside expected order |

The current models express packet loss by omitting events, so the estimator
detects it from sequence gaps. Several flags are reserved for future RTL packet
metadata and are not currently generated.

## NDJSON event stream

`events.ndjson` has one compact JSON object per line. The payload is represented
as lowercase hexadecimal under `payload_hex`. Keys are sorted when written so
identical inputs produce byte-identical files.

Example shape:

```json
{"arrival_ticks":1002200,"format_version":1,"measurement_ticks":1000000,"payload_hex":"...","sensor_id":1,"sequence_number":5,"status_flags":1}
```

Events are ordered by:

1. `arrival_ticks`
2. numeric `sensor_id`
3. `sequence_number`

The ESKF uses the same ordering after reading the file.

## Transport timing

ADIS uses a dedicated SPI bus. Its arrival is data-ready plus the exact 176-bit
transaction duration.

ADXL and BMP share the auxiliary SPI bus. Scheduling sorts pending acquisitions
by measurement time, gives ADXL priority for simultaneous requests, and starts
each transaction at the later of its measurement epoch or bus-free epoch. Each
modeled auxiliary acquisition occupies 72 serial bits.

GNSS solution latency is sampled independently for each solution. PPS arrival
equals its jittered measurement edge in the current model.

## Binary replay files

Binary replay contains sensor transactions only. Timing and status metadata
remain in NDJSON until the common FPGA to R5F envelope is frozen.

| File | Record size | Contents |
| --- | ---: | --- |
| `adis16470_bursts.bin` | 22 bytes | Command plus ten big-endian response words |
| `adxl375_acquisitions.bin` | 9 bytes | Read command, XYZ data, status command, status |
| `bmp581_acquisitions.bin` | 9 bytes | Read command, temperature, pressure, status command, status |
| `gnss_solutions.bin` | 352 bytes | Generic little-endian canonical solution |
| `gnss_pps.bin` | 19 bytes | Generic little-endian PPS record |

Only files for enabled sensors with at least one event are created.

## Replay verification

The run pipeline serializes and rereads logical events before estimation. The
validator checks that every binary file is an integer number of records and that
its record count matches the logical events for that sensor. ADIS tests also
verify exact transaction decode, checksum behavior, and a committed golden
transaction fixture.

## Format evolution

Changing a payload without changing the logical event envelope does not require
a new event format version. Any incompatible change to event fields or their
meaning must increment `format_version` and keep an explicit legacy decoder or
fail clearly. The future common packet envelope should carry schema version,
sensor ID, sequence, measurement epoch, arrival or publication metadata, status,
payload length, and an envelope integrity check.
