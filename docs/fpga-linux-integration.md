# FPGA-to-Linux Integration Contract

ABI v1 carries the active BNO085, ADXL375, BMP581, ZED-F9P UBX, and ZED-F9P
TIMEPULSE streams. Incompatible layouts or semantics require a new ABI major.
The canonical source is `spec/fpga_linux_abi.yaml`; generated files must never
be edited directly.

## Data paths

Every sensor controller submits one logical event to the common packetizer.
The packetizer atomically fans the complete record into two independent paths:

1. The 64 KiB BRAM ring is guaranteed for the R5F. It never overwrites unread
   data. A full ring drops the newest R5F copy and raises a degraded or fatal
   condition.
2. The Linux mirror is best effort. It uses 256 coherent 4096-byte blocks. A
   stalled mirror drops only the Linux copy and records that loss in the next
   block. It can never stall the flight path.

TIMEPULSE is a zero-payload common record. Its synchronized edge timestamp is
paired in software with UBX TIM-TP without stepping the PL counter.

## Time and ordering

The PL owns an unsigned 64-bit 100 MHz counter. A hard PL reset clears it. Soft
datapath and sensor resets preserve it. `measurement_ticks` is the synchronized
sensor edge or protocol epoch. `arrival_ticks` is common-record commit time.
Asynchronous inputs incur two to three positive PL-clock cycles of timestamp
latency.

Live 64-bit AXI-Lite registers use low-word-first reads. Reading the low word
latches the high word until that high word is read. Software compares wrapping
sequences and timestamps with wrapping arithmetic.

## Access, reset, and lock rules

AXI-Lite accesses are aligned, little-endian, 32-bit transactions. Undefined
offsets, partial writes, unaligned accesses, and writes to read-only registers
return `SLVERR` and set `ACCESS_VIOLATION`.

A soft reset requires `RST!` followed by `RESET_CONTROL=1` within 16 PL clocks.
It flushes queues, restarts controllers, increments `RESET_GENERATION`, and
emits one health record while preserving time and diagnostic counters. A hard
reset also clears time, session, and counters. Sensor resets affect only their
controller and emit a health record.

Configuration lock is write-once until hard reset. Once set, reconfiguration,
replay, counter clearing, and destructive reset controls fail. Flight software
sets this bit before arming.

## Interrupt ownership

Custom interrupts are active-high level signals and stay asserted until the
W1C source is cleared. Linux owns global health, TIMEPULSE, DMA S2MM, and replay
MM2S sources. The R5F owns the BRAM watermark. Device-tree-assigned Linux IRQ
numbers are outside the ABI.

## Replay safety

Replay items retain their original measurement and arrival timestamps. The
engine waits each relative delay before release. Injection is accepted only
when physical acquisition is stopped, test mode is enabled, the core is
unlocked, and no fatal condition exists. Malformed or disallowed input raises a
sticky error and is never forwarded.

## Verification sequence

Board acceptance proceeds in this order:

1. Identify magic, ABI major, ABI hash, build ID, and source hashes.
2. Run walking-bit and access-rule register tests.
3. Measure the PL timebase for ten seconds and require 100 ppm accuracy.
4. Force and clear each test-mode interrupt.
5. Replay the generated golden records and compare every output byte.
6. Apply Linux backpressure and verify only Linux loss counters increment.
7. Starve the R5F and verify its independent fatal or degraded response.
8. Check soft reset, per-sensor reset, and permanent configuration lock.
9. Run a one-hour soak with no unexpected drop, malformed record, stuck IRQ,
   or non-monotonic timestamp.

Reports retain JSON, JUnit XML, raw blocks, decoded events, register snapshots,
kernel logs, ABI and build identifiers, XSA and bitstream hashes, and replay
hashes.
