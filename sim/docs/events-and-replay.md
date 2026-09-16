# Events, Timing, and Replay

Every output uses `MeasurementEvent` format version 1 with sensor ID, wrapping
sequence, 100 MHz measurement ticks, completion or arrival ticks, status flags,
and raw payload bytes.

| ID | Meaning |
| ---: | --- |
| 1 | Reserved legacy decoder |
| 2 | ADXL375 acquisition |
| 3 | BMP581 acquisition |
| 4 | Reserved legacy decoder |
| 5 | Reserved legacy decoder |
| 6 | BNO085 SHTP packet |
| 7 | ZED-F9P UBX frame |
| 8 | ZED-F9P TIMEPULSE edge |

`events.ndjson` contains every event in arrival order. Binary replay contains
only physical byte streams:

| File | Contents |
| --- | --- |
| `bno085_shtp.bin` | Variable-length SHTP transactions |
| `adxl375_acquisitions.bin` | Nine-byte modeled SPI acquisitions |
| `bmp581_acquisitions.bin` | Nine-byte modeled SPI acquisitions |
| `zed_f9p_uart.bin` | Variable-length UBX frames |

TIMEPULSE remains in NDJSON because it is an edge, not a byte stream. SHTP and
UBX replay readers use their embedded lengths and reject truncation, bad sync,
or invalid checksums. The future common FPGA to R5F envelope remains deferred.

Arrival timing includes each bus clock, complete transaction length, receiver
latency where applicable, and serialization against other traffic on the same
bus. Identical configuration, seed, dependencies, and source revision produce
byte-identical events and replay files.
