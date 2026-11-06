# Tang Nano 20K bring-up board

A cheap, macOS-friendly hardware test bed for the GNSS SCC, built entirely
with the open-source oss-cad-suite flow (Yosys, nextpnr, openFPGALoader).
This is **not** the flight target. Nothing here is part of the KR260 Vivado
project (`build.tcl` only collects `src/rtl/`), and the repo CI does not run
these tests yet.

## Current state: bring-up step 1

`tangnano20k_top` blinks LED0 once per second and sends
`tang nano hello XX\r\n` (XX = hex message counter) at 115200 baud over the
board's USB-serial bridge. LED1 toggles once per message. This proves the
toolchain, the pin constraints, and the FPGA-to-host UART path before any GNSS
logic is attached.

## Commands

Run these from the repository root after `source ~/oss-cad-suite/environment`:

```bash
python3 boards/tangnano20k/tangnano.py test     # Verilator lint + testbenches
python3 boards/tangnano20k/tangnano.py build    # bitstream -> boards/tangnano20k/build/
python3 boards/tangnano20k/tangnano.py load     # load into SRAM (lost on power cycle)
python3 boards/tangnano20k/tangnano.py flash    # write to flash (survives power cycle)
python3 boards/tangnano20k/tangnano.py monitor  # print UART output; --port to override
```

`build` prints device utilization and the post-route maximum clock frequency.
Those timing numbers are for the Gowin GW2A chip and say nothing about the
KR260's UltraScale+ fabric.

## Layout

- `rtl/`: board-only RTL. `uart_tx` is a generic 8N1 transmitter; `tangnano20k_top` is the board top level.
- `tb/`: self-checking Verilator testbenches, one per module.
- `tangnano20k.cst`: pin constraints.

## Pins

| Port | Pin | Notes |
| --- | --- | --- |
| `clk_27m` | 4 | On-board 27 MHz oscillator |
| `uart_tx` | 69 | FPGA to USB-serial bridge (BL616). Confirmed on hardware |
| `led_n[5:0]` | 15-20 | Active-low on-board LEDs |

## Notes

- The power-on reset relies on the FPGA loading flip-flop initial values at
  configuration. It is the only intentional declaration initializer in the
  design; see the comment in `tangnano20k_top.sv`.
- Verilator build products go to the system temp directory because
  Verilator's `make` step breaks on repository paths that contain spaces.
  Set `RTL_BUILD_ROOT` to override.
