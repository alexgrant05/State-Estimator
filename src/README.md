# RTL Development

This directory contains synthesizable SystemVerilog, constraints, and
self-checking module testbenches.

## Layout

- `rtl/common/`: reusable clock, event, buffering, and arbitration blocks.
- `rtl/bringup/`: temporary board-level hardware checks.
- `rtl/sensors/`: sensor controllers and protocol-specific logic added later.
- `rtl/transport/`: packet builders and processor-facing buffers added later.
- `constraints/`: timing and board constraints.
- `tb/common/`: one self-checking testbench per common module.
- `tb/bringup/`: self-checking board bring-up module tests.

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

With Vivado's `bin` directory on `PATH`, also run the LED tests in four-state
XSim, which detects unknown startup values that two-state simulation can hide:

```text
python tools/run_led_xsim.py
```

This checks normal reset/restart and startup with reset inactive from time zero,
with both immediate and delayed clocks. Logs stay under `.rtl_build/led_xsim/`.
Test the XSDB orchestration without connecting to hardware using:

```text
xsdb tools/tests/test_xsdb_led_bringup.tcl
```

These mocks check sequencing, paths containing spaces, missing/ambiguous targets,
initialization failure, disabled clock, and lost configuration. They do not
replace programming and measuring the physical board.

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
- `led_blinker`: parameterized 1 Hz KR260 bring-up indicator driven by the
  100 MHz PS PL clock.

All modules use synchronous active-low reset, explicit SystemVerilog port
types, nonblocking sequential assignments, `always_ff` or `always_comb`, and
`default_nettype none`.

## Next RTL modules

1. Compose event capture with a timestamp FIFO and sequence counter.
2. Implement a configurable SPI transaction engine.
3. Verify SPI modes, divider timing, transfer lengths, and timeout recovery.
4. Implement the BNO085 SHTP SPI controller and interrupt handling.
5. Implement ADXL375 and BMP581 controllers plus auxiliary-bus scheduling.
6. Define the common packet writer and processor-facing ring buffer.
7. Add ZED-F9P PPS capture, UART receive, and UBX framing.

New synthesizable files must be registered in `build.tcl`, and every standalone
module must receive a focused Verilator test before integration.

## KR260 LED bring-up

The KR260 does not provide a user LED directly controlled by PL logic. Connect
an external LED to Pmod 1, connector J2:

```text
J2 pin 1 -> 330 ohm resistor -> LED anode (+)
LED cathode (-) -> verified J2 pin 9 (GND)
```

The constraint maps `led_0` to FPGA ball H12, LVCMOS33, 4 mA drive, slow slew.
`DRIVE 4` is an output drive-strength setting, not a current limiter; retain the
series resistor. Both counter and LED initialize to zero because this bring-up
design ties reset inactive. It still requires PS initialization to supply the
approximately 100 MHz `pl_clk0`. Programming a `.bit` in Hardware Manager does
not apply the PS clock preset.

### 1. Verify the LED circuit

With power disconnected, identify J2 and its pin-1 marker. Do not count pins
using a generic Pmod drawing: connector and module numbering may differ. Check
continuity from the intended pin 9 to a known board ground, measure the resistor
(approximately 330 ohms, preferably with one end disconnected), and use diode
mode to identify the LED anode and cathode.

AMD's [KR260 connector example](https://xilinx.github.io/kria-apps-docs/kr260/build/html/docs/gps_1588_ptp/src/app_deployment.html)
identifies J2-10 as GND and J2-12 as 3.3 V. Disconnect the signal lead from J2-1,
power the board, and verify approximately 3.3 V across J2-12 and J2-10. Power
down before moving connections; test `J2-12 -> resistor -> anode`, with cathode
on J2-10. If it does not light, fix polarity/connections or replace the LED.
Restore the signal lead to J2-1 with power off after this test.

### 2. Prove the physical output with static-high

Run these commands from the repository root, with Vivado on PATH:

```text
vivado -mode batch -source tools/vivado_check_jtag.tcl
vivado -mode batch -source tools/vivado_led_static_test.tcl
vivado -mode batch -source tools/vivado_program.tcl -tclargs ".rtl_build/kr260_led_static_high.bit"
```

Require `PROGRAM:DONE=1` and `PROGRAM:DONE_AFTER_5S=1`. A successful build alone
does not mean the diagnostic was programmed. If zero targets are detected,
check board power, the USB data cable on J4, and the FTDI driver.

Measure J2-1 against verified ground, unloaded first and then with the LED
connected (power down to change wiring). Expected unloaded voltage is about
3.3 V; the LED should light steadily when connected.

| Observation | Next action |
| --- | --- |
| About 3.3 V unloaded, LED stays dark | Check LED polarity, resistor and jumper continuity; repeat the independent LED test. |
| Voltage collapses with LED attached | Check for a short, wrong resistor value or excessive load. |
| Low unloaded voltage despite stable DONE | Recheck physical pin numbering, selected bitstream, ground and I/O power. Do not proceed to clock debugging. |
| Steady LED and correct unloaded voltage | Proceed to PS initialization and the blinking design. |

### 3. Initialize the PS and run the blinker

Rebuild using the project README commands. The build prints `BUILD:BITSTREAM`
and `BUILD:PSU_INIT`; keep these two artifacts from the same build together.
It resynthesizes the LED module reference before the top-level
build, so an old out-of-context checkpoint cannot hide RTL changes. It also
verifies that the implemented LED and counter registers have zero INIT.
On a cold-started bench board without a running OS, use:

```text
xsdb tools/xsdb_led_bringup.tcl "vivado/State-Estimation/State-Estimation.runs/impl_1/state_est_bd_wrapper.bit" "vivado/State-Estimation/State-Estimation.runs/impl_1/state_est_bd_wrapper.psu_init.tcl"
```

Both paths are individual quoted arguments; this also supports absolute paths
containing spaces. The script requires one K26 and one PSU target. It programs
PL, sources and runs the matching generated `psu_init`, waits one second, removes
PS/PL isolation, waits one second, then releases PS/PL reset, following
[AMD UG1725](https://docs.amd.com/r/en-US/ug1725-xsdb-reference-guide/Debugging-Applications-on-Zynq-UltraScale-MPSoC).
This initializes PS peripherals and memory as well as clocks, so do not run it
over a live operating system. Neither this script nor the static programmer
writes flash or installs Linux.

Require `BRINGUP:SUCCESS`, `BRINGUP:CONFIGURED=1`, and
`BRINGUP:CONFIGURED_AFTER_5S=1`. Configuration checks use XSDB's end-of-startup
status. `BRINGUP:PL0_REF_CTRL` reads back the PL0 clock enable, not its physical
frequency. A DONE or configuration check alone does not prove a running clock.

Observe roughly 0.5 seconds on and 0.5 seconds off for at least one minute.
Repeat from a cold power cycle and rerun the same initialization sequence.
An ordinary multimeter may average the blinking voltage; use the static test
for voltage checks. Record those bench results before marking bring-up verified.
