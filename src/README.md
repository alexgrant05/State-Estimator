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

The setup script supports Ubuntu and Debian, Fedora, Arch Linux, macOS with
Homebrew, and Windows through an MSYS2 UCRT64 terminal.

```bash
bash tools/install_rtl_tools.sh
```

Linux CI is the reference environment and uses the same setup script. Python
runs the test manifest on every platform. Windows developers should run setup
from an MSYS2 UCRT64 terminal. The runner places Windows build products under
the system temporary directory to support repository paths containing spaces.

## Tests

Run these commands from the repository root:

```bash
python3 tools/run_rtl_tests.py --lint-only
python3 tools/run_rtl_tests.py
python3 tools/run_rtl_tests.py --list
python3 tools/run_rtl_tests.py --test sync_fifo
python3 tools/run_rtl_tests.py --test async_event_capture --trace
```

Use `python` instead of `python3` when that is the local Python 3 command. Each
`tb_*.sv` file becomes an independent executable. Tests are self-checking and
fail through `$fatal`. Verilator warnings are fatal and are not suppressed
except for the expected multiple-top warning caused by compiling the shared RTL
source set for each focused test.

Set `RTL_BUILD_ROOT` to override the build directory. Optional `waveform.vcd`
files are stored under the selected test build directory.

## Common modules

- `timebase_counter`: parameterized free-running system time counter.
- `async_event_capture`: synchronized rising-edge timestamp capture with
  ready/valid delivery and saturating overflow accounting.
- `sync_fifo`: ready/valid FIFO with simultaneous full pop/push, level
  reporting, and overflow/underflow status.
- `fixed_priority_arbiter`: one-hot combinational grant with requester zero as
  the highest priority.

All modules use synchronous active-low reset, `logic` ports, nonblocking
sequential assignments, `always_ff` or `always_comb`, and `default_nettype none`.

## Next RTL modules

1. Compose event capture with a timestamp FIFO and sequence counter.
2. Implement a configurable SPI transaction engine.
3. Verify SPI modes, divider timing, transfer lengths, and timeout recovery.
4. Implement the ADIS16470 burst controller and checksum checker.
5. Implement ADXL375 and BMP581 controllers plus auxiliary-bus scheduling.
6. Define the common packet writer and processor-facing ring buffer.
7. Add PPS capture, UART receive, and receiver-specific GNSS framing.

New synthesizable files must be registered in `build.tcl`, and every standalone
module must receive a focused Verilator test before integration.
