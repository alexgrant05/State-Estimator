# Tang Nano 20K bring-up board

A cheap, macOS-friendly hardware test bed for the GNSS SCC, built entirely
with the open-source oss-cad-suite flow (Yosys, nextpnr, openFPGALoader).
This is **not** the flight target. Nothing here is part of the KR260 Vivado
project (`build.tcl` only collects `src/rtl/`), and the repo CI does not run
these tests yet.

## Current state: raw SPI bytes to the host

`tangnano20k_top` runs the real `gnss_scc` against a ZED-F9P breakout and sends
the raw bytes received over SPI to the host at 115200 baud, through the board's
USB-serial bridge. `raw_byte_tap` removes the idle filler so the stream fits.
Nothing is decoded on the FPGA yet: this stage proves the SPI wiring and timing
before anything downstream of the SPI engine is trusted.

The receiver keeps whatever configuration it already has (`start_sequence` is
tied low). At factory defaults it sends NMEA text and no UBX over SPI, so
readable `$G...` sentences are the expected first output.

## Commands

Run these from the repository root after `source ~/oss-cad-suite/environment`:

```bash
python3 boards/tangnano20k/tangnano.py test     # Verilator lint + testbenches
python3 boards/tangnano20k/tangnano.py build    # bitstream -> boards/tangnano20k/build/
python3 boards/tangnano20k/tangnano.py load     # load into SRAM (lost on power cycle)
python3 boards/tangnano20k/tangnano.py flash    # write to flash (survives power cycle)
python3 boards/tangnano20k/tangnano.py monitor  # print UART output; --port to override
```

`monitor` prints text as-is and other bytes as `<hex>`, and collapses long 0xFF
runs into a `[32 x ff]` marker. `build` prints device utilization and the
post-route maximum clock frequency. Those timing numbers are for the Gowin GW2A
chip and say nothing about the KR260's UltraScale+ fabric.

`build` and `test` generate an all-zero placeholder for `config_rom.mem` and
`config_rom_last.mem`. They are never played, because `start_sequence` is low;
they only need to exist until the real config ROM is generated.

## Wiring

All Tang Nano 20K banks are 3.3 V (Sipeed pinout diagram). Connect the grounds,
and power each board from its own USB port. Do not connect the two 3.3 V rails.

| Tang Nano pin | Signal | ZED-F9P pin | SparkFun pad |
| --- | --- | --- | --- |
| 73 | `gnss_sclk` (out) | SCL / SPI clock | SCL |
| 74 | `gnss_mosi` (out) | RX / SPI MOSI | RX/MOSI |
| 75 | `gnss_miso` (in, pull-up) | TX / SPI MISO | TX/MISO |
| 76 | `gnss_cs_n` (out) | SDA / SPI_CS_N | SDA |
| 86 | `gnss_timepulse` (in, pull-down) | TIMEPULSE | PPS |
| any GND | ground | GND | GND |

**The receiver must be in SPI mode.** D_SEL is read at startup and must be low.
On the SparkFun GPS-RTK2 that means bridging the DSEL solder jumper on the back
of the board, which disables UART1 and I2C; USB keeps working. Power-cycle the
receiver after changing it. Other breakouts may label this differently.

The other Tang Nano pins in use are `clk_27m` (4), `uart_tx` (69, confirmed on
hardware), and `led_n[5:0]` (15-20, active-low).

## LEDs

| LED | Meaning |
| --- | --- |
| 0 | Heartbeat, toggles once per second |
| 1 | Toggles every 64 bytes sent to the host |
| 2 | `raw_byte_tap` FIFO overflowed (sticky); the stream has lost bytes |
| 3 | `link_timeout_sticky`. Counts UBX frames only, so it comes on about 7 s after power-up while the receiver sends only NMEA |
| 4 | Toggles on each TIMEPULSE edge. The SparkFun guide says PPS starts at 1 Hz once the receiver has a position lock |

## Bench checks

**Before connecting the receiver** (no logic analyzer needed):

1. With `gnss_miso` unconnected, the pull-up reads as idle, so `monitor` stays
   silent while LED0 blinks. LED3 comes on after about 7 s.
2. Jumper pin 75 to GND. `monitor` should flood with `<00>`, LED1 should look lit
   (at this rate it toggles at about 90 Hz, too fast to see), and LED2 should
   latch on, because the flood outruns the UART. This proves the
   MISO input, the SPI engine's sampling, the FIFO, and the UART path. Remove the
   jumper and reload the bitstream to clear LED2.
3. Optional, with a multimeter on DC volts: pin 73 (SCLK) reads about 1.6 V (a
   50% square wave averaged), pin 74 (MOSI) about 3.3 V, and pin 76 (CS) about 0 V.

**With the receiver connected:** expect NMEA sentences in `monitor`. To compare
the two interfaces, the breakout's USB port can stay connected to a GNSS tool on
the Mac at the same time. If the stream is garbage, suspect the D_SEL strap first,
then the wiring, then the CS-held-low-forever polling behavior.

## Known bench quirk: bit-shifted output after a wiring glitch

If `monitor` shows repeating non-text bytes such as `H<8e><9c>` instead of
`$GN`, the data is probably correct but shifted by some number of bits. It has
been seen with a loose connection. `gnss_scc` holds CS low while polling, so
nothing realigns the receiver's byte framing after a clock glitch. Seat the
wires firmly and run `load`, which restarts the FPGA. The cause is likely but
has not been deliberately reproduced, and the fix in `gnss_spi_engine` is open.

## Known bench quirk: pin 86 idles high

With nothing connected, pin 86 measured about 2.6 V on the first board tested,
even though the pull-down is set in the bitstream. The cause is unknown. The
pin reads as high when reset ends, and `async_event_capture` starts by assuming
it is low, so it counts one false rising edge and LED4 comes on after every load.
The input itself works: jumpering pin 86 to GND and releasing it flips LED4.

The same capture logic would record a false PPS timestamp in any design where
TIMEPULSE is high as reset ends. That question is still open.

## Layout

- `rtl/`: board-only RTL. `uart_tx` is a generic 8N1 transmitter; `raw_byte_tap`
  filters and buffers the raw byte stream; `tangnano20k_top` is the board top level.
- `tb/`: self-checking Verilator testbenches, one per module.
- `tangnano20k.cst`: pin constraints.

## Notes

- The power-on reset relies on the FPGA loading flip-flop initial values at
  configuration. It is the only intentional declaration initializer in the
  design; see the comment in `tangnano20k_top.sv`.
- `raw_byte_tap` passes up to 32 consecutive 0xFF bytes, then drops the rest. A
  message with a longer 0xFF run would be shortened in this debug stream only;
  the path into `ubx_frame_sync` is unaffected.
- Verilator build products go to the system temp directory because
  Verilator's `make` step breaks on repository paths that contain spaces.
  Set `RTL_BUILD_ROOT` to override.
