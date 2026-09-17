# Cornell Rocketry State Estimation

State-estimation platform for Cornell Rocketry targeting the AMD Kria KR260
with a K26 SOM. The repository combines the Vivado hardware design, sensor
interfaces, a Python digital twin, and the reference 15-state error-state
Kalman filter used to validate behavior before flight-hardware deployment.

## Repository layout

- `build.tcl`: regenerates the Vivado project and block design.
- `src/rtl/`: custom Verilog and SystemVerilog sources.
- `src/constraints/`: board and timing constraints.
- `src/tb/`: RTL testbenches.
- `sim/`: installable Python digital twin, estimator, replay tools, and tests.
- `vivado/`: generated Vivado project, excluded from source control.

## Tentative hardware and software partition

| Platform | Responsibility | Current state |
| --- | --- | --- |
| Programmable logic | Sensor buses, 100 MHz timestamps, event capture, buffering, and packet movement | Common timebase, asynchronous capture, FIFO, and arbitration modules implemented |
| Cortex R5F | Deterministic sensor decoding, source selection, time synchronization, and 15-state ESKF | Python reference complete; embedded port not started |
| Linux on Cortex A53 | Configuration, logging, telemetry, health reporting, and operator tools | Planned |
| Python digital twin | Truth, sensor protocols, deterministic replay, estimator reference, faults, and validation | Active and passing |

## Current reference pipeline

```text
10 s pad alignment + Andromeda RocketPy truth at 2000 Hz
    -> BNO085 + ADXL375 + BMP581 + ZED-F9P
    -> 100 MHz timestamped events and deterministic sensor replays
    -> shared-bus scheduling and high-g acceleration selection
    -> delayed 15-state ESKF fusion with rewind/replay
    -> states, metrics, plots, manifest, and pass/fail gates
```

Implemented digital-twin behavior includes:

- Launch-centered ENU navigation, MSL altitude, and WGS84 ECEF GNSS handling.
- BNO085 calibrated acceleration at 500 Hz, uncalibrated gyro at 400 Hz, and
  diagnostic game rotation vector at 100 Hz over SHTP and SPI mode 3.
- ADXL375 at 800 Hz with hysteretic handoff before BNO acceleration saturation.
- BMP581 at 50 Hz with raw register output, pad calibration, flight-phase
  suppression, transonic disturbance modeling, and innovation gating.
- ZED-F9P standalone GNSS at 5 Hz with UBX NAV-PVT, NAV-COV, TIM-TP, and a
  separate 1 PPS TIMEPULSE event.
- Dedicated BNO085 SPI and deterministic shared-SPI arbitration for ADXL and BMP.
- Separate measurement and arrival epochs plus two seconds of estimator history
  for delayed aiding updates.
- Deterministic fault injection, binary replay, artifact hashing, and 200-seed
  statistical validation.
- Exact replay artifacts for BNO085 SHTP, ADXL375 and BMP581 SPI acquisitions,
  and ZED-F9P UART. TIMEPULSE edges remain in the logical NDJSON stream.

See [sim/README.md](sim/README.md) for sensor conventions, configuration,
artifacts, detailed verification, and simulation scope.

## RTL development

The initial RTL layer contains a 64-bit timebase, asynchronous event timestamp
capture, a ready/valid synchronous FIFO, a fixed-priority arbiter, and a KR260
LED bring-up module. Each module has an independent self-checking Verilator
test.

```bash
bash tools/install_rtl_tools.sh
python3 tools/run_rtl_tests.py --lint-only
python3 tools/run_rtl_tests.py
```

The setup script supports common Linux distributions, macOS with Homebrew, and
Windows through MSYS2 UCRT64. Linux CI is the reference environment. See
[src/README.md](src/README.md) for module conventions and targeted test commands.

## Digital-twin setup

Python 3.10 through 3.13 is supported. The reference environment uses the
versions pinned in `sim/requirements-lock.txt`.

```bash
cd sim
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
.venv/bin/python -m pip install --no-deps -e .

.venv/bin/python -m pytest -q
.venv/bin/python -m digital_twin run \
  --config config/andromeda.toml \
  --seed 42 \
  --output outputs/andromeda-seed-42
.venv/bin/python -m digital_twin validate \
  --run outputs/andromeda-seed-42
```

On Windows, replace `.venv/bin/python` with `.venv\Scripts\python.exe`.

The current suite has 49 passing tests. The seed-42 Andromeda pad-to-apogee run
passes all 17 validation gates with 74,735 measurement events, 13,794 published
states, and 1,211 delayed rewinds. Current RMS errors are 0.286 m position,
0.088 m/s velocity, and 0.821 degrees attitude. These are simulation reference
results, not final flight limits.

The current Andromeda trajectory peaks near 3.89 g, so it does not naturally
cross the BNO085 handoff or ZED-F9P invalid-fix thresholds. Dedicated analytic
tests cover BNO saturation, ADXL handoff, 4 g GNSS invalidation, and recovery.

## Vivado setup

```bash
vivado -mode batch -source build.tcl
vivado -mode batch -source tools/vivado_build.tcl
```

The first command regenerates `vivado/State-Estimation/` for the KR260. The
second validates the block design, creates its HDL wrapper, runs synthesis and
implementation, and writes the bitstream and a matching PS initialization script.
It checks timing, DRC, and the LED/counter configuration-time initial values.
With the powered board connected to J4 using a micro-USB data cable, check JTAG:

```bash
vivado -mode batch -source tools/vivado_check_jtag.tcl
```

The check requires exactly one K26 in the JTAG chain. First complete the
[static LED and multimeter checks](src/README.md#kr260-led-bring-up). Then, on
a cold-started bench board without a running OS, initialize the blinking design:

```text
xsdb tools/xsdb_led_bringup.tcl "vivado/State-Estimation/State-Estimation.runs/impl_1/state_est_bd_wrapper.bit" "vivado/State-Estimation/State-Estimation.runs/impl_1/state_est_bd_wrapper.psu_init.tcl"
```

This programs the PL, initializes the PS clocks, removes PS/PL isolation, and
releases PS/PL reset. It checks configuration immediately and five seconds after
initialization, and reads back the PL0 clock enable. A successful FPGA `DONE`
readback alone does **not** prove that the PS-provided clock is running.
`tools/vivado_program.tcl` only programs PL and is suitable for the clock-free
static diagnostic; it does not initialize the PS. These JTAG operations are
volatile and do not modify QSPI. Generated Vivado files are not required by the
Python simulation.

For the physical bring-up check, connect J2 pin 1 through a 330 ohm resistor to
the LED anode, then connect the LED cathode to verified J2 pin 9 ground. Verify
connector numbering and ground continuity before powering the circuit. The LED
should complete one blink cycle per second after PS initialization. The KR260's
onboard LEDs are power or PS-managed status indicators and are not repurposed.

## Project status

- [x] KR260 project skeleton and Zynq UltraScale+ PS block design.
- [x] Verilator lint, per-module simulation, and RTL CI workflow.
- [x] Common timebase, event capture, FIFO, and arbitration modules.
- [x] KR260 JTAG, PL clock, and external Pmod LED bring-up design and diagnostics.
- [ ] Confirm static-high voltage and repeatable 1 Hz blinking on the physical board.
- [x] BNO085 SHTP model, asynchronous reports, and inertial propagation.
- [x] ADXL375 model and high-g transition logic.
- [x] BMP581 pressure and temperature model with aided updates.
- [x] ZED-F9P UBX model, TIMEPULSE, latency, outages, time sync, and delayed fusion.
- [x] Merged multi-sensor logical events and per-sensor binary replays.
- [x] Fault campaigns, 200-seed statistics, and Andromeda integration gates.
- [ ] Replace generic and placeholder values with measured flight-hardware data.
- [ ] Confirm the ZED-F9P module suffix and freeze its configuration profile.
- [ ] Freeze the common FPGA-to-R5F packet envelope.
- [ ] Implement sensor acquisition, timestamping, GNSS UART, and PPS capture RTL.
- [ ] Port the selector, time sync, ESKF, and delayed replay to Cortex R5F.
- [ ] Require Python, RTL, and R5F replay parity.
- [ ] Complete bench calibration, hardware-in-the-loop testing, and flight gates.

Descent, fixed-point estimator behavior, and the final packet envelope remain
deferred until the receiver and hardware interfaces are finalized.
