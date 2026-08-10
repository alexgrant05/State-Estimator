# RTL Development

This directory contains synthesizable SystemVerilog, constraints, and
self-checking module testbenches.

## Layout

- `rtl/common/`: reusable clock, event, buffering, and arbitration blocks.
- `rtl/sensors/`: sensor controllers and protocol-specific logic added later.
- `rtl/transport/`: packet builders and processor-facing buffers added later.
- `constraints/`: timing and board constraints.
- `tb/common/`: one self-checking testbench per common module.

## Tool setup

Windows development uses MSYS2 UCRT64 Verilator, GCC, and GNU Make. Python runs
the test manifest and works around the repository path containing spaces.

```powershell
powershell -ExecutionPolicy Bypass -File tools\install_rtl_tools.ps1
```

The current reference versions are Verilator 5.036, GCC 15.1, and GNU Make
4.4.1. Linux CI installs Verilator from the Ubuntu package repository.

## Tests

From the repository root:

```powershell
python tools\run_rtl_tests.py --lint-only
python tools\run_rtl_tests.py
python tools\run_rtl_tests.py --list
python tools\run_rtl_tests.py --test sync_fifo
python tools\run_rtl_tests.py --test async_event_capture --trace
```

Each `tb_*.sv` file is compiled into an independent executable. Tests are
self-checking and fail through `$fatal`. On Windows, build products and optional
`waveform.vcd` files are placed under
`%TEMP%\cornell_state_estimation_rtl\<test-name>` because GNU Make cannot build
inside the repository path containing spaces. Set `RTL_BUILD_ROOT` to override
that location.

## Common modules

- `timebase_counter`: parameterized free-running counter, configured as 64 bits
  at the system level.
- `async_event_capture`: multi-stage synchronization, rising-edge timestamp
  capture, ready/valid delivery, and saturating overflow accounting.
- `sync_fifo`: parameterized ready/valid FIFO with simultaneous full pop/push,
  level reporting, and overflow/underflow status.
- `fixed_priority_arbiter`: one-hot combinational grant with requester zero as
  the highest priority. ADXL can use requester zero on the auxiliary SPI bus.

All modules use synchronous active-low reset, `logic` ports, nonblocking
sequential assignments, `always_ff` or `always_comb`, and `default_nettype none`.
Module tests cover reset, nominal operation, backpressure, overflow, underflow,
ordering, simultaneous transactions, and priority behavior.

## Next RTL modules

1. Compose event capture with a timestamp FIFO and sequence counter.
2. Implement a generic configurable SPI transaction engine.
3. Verify SPI modes, divider timing, transfer lengths, and timeout recovery.
4. Implement the dedicated ADIS16470 burst controller and checksum checker.
5. Implement ADXL375 and BMP581 controllers plus auxiliary-bus scheduling.
6. Define the common packet writer and processor-facing ring buffer.
7. Add PPS capture, UART receive, and receiver-specific GNSS framing.

New synthesizable files must be registered in `build.tcl`, and every standalone
module must receive a focused Verilator test before integration.
