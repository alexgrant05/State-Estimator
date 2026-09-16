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

See [sim/README.md](sim/README.md) for sensor conventions, configuration,
artifacts, detailed verification, and simulation scope.

## RTL development

The initial common RTL layer contains a 64-bit timebase, asynchronous event
timestamp capture, a ready/valid synchronous FIFO, and a fixed-priority arbiter.
Each module has an independent self-checking Verilator test.

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

```powershell
cd sim
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.\.venv\Scripts\python.exe -m pip install --no-deps -e .

.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m digital_twin run `
  --config config\andromeda.toml `
  --seed 42 `
  --output outputs\andromeda-all-sensors-seed-42
.\.venv\Scripts\python.exe -m digital_twin validate `
  --run outputs\andromeda-all-sensors-seed-42
```

The suite covers sensor scales, protocol framing, timestamps, fault handling,
deterministic replay, estimator invariants, and the seed-42 Andromeda run.
These checks are reference engineering gates, not final flight limits.

## Vivado setup

```bash
vivado -mode batch -source build.tcl
vivado -mode batch -source tools/vivado_build.tcl
```

The first command regenerates `vivado/State-Estimation/` for the KR260. The
second validates the block design, creates its HDL wrapper, runs synthesis and
implementation, and writes the bitstream. With the powered board connected to
the KR260 micro-USB JTAG/UART port, program the PL with:

```bash
vivado -mode batch -source tools/vivado_check_jtag.tcl
vivado -mode batch -source tools/vivado_program.tcl -tclargs \
  vivado/State-Estimation/State-Estimation.runs/impl_1/state_est_bd_wrapper.bit
```

The check requires exactly one K26 in the JTAG chain. Programming also reads
back the FPGA `DONE` bit and fails if configuration did not complete. This JTAG
operation is volatile and does not modify the board's QSPI flash. Generated
Vivado files are not required by the Python simulation.

## Project status

- [x] KR260 project skeleton and Zynq UltraScale+ PS block design.
- [x] Verilator lint, per-module simulation, and RTL CI workflow.
- [x] Common timebase, event capture, FIFO, and arbitration modules.
- [x] BNO085 SHTP model, asynchronous reports, and inertial propagation.
- [x] ADXL375 model and high-g transition logic.
- [x] BMP581 pressure and temperature model with aided updates.
- [x] ZED-F9P UBX model, TIMEPULSE, latency, outages, time sync, and delayed fusion.
- [x] Merged multi-sensor logical events and per-sensor binary replays.
- [x] Fault campaigns, 200-seed statistics, and Andromeda integration gates.
<<<<<<< HEAD
- [ ] Replace generic and placeholder values with measured flight-hardware data.
- [ ] Confirm the ZED-F9P module suffix and freeze its configuration profile.
=======
- [x] Replace generic and placeholder values with measured flight-hardware data.
- [ ] Select the exact GNSS receiver and add its wire-format adapter.
>>>>>>> cba1e7c176a9081daf245b35a3f0121600f3ec7b
- [ ] Freeze the common FPGA-to-R5F packet envelope.
- [ ] Implement sensor acquisition, timestamping, GNSS UART, and PPS capture RTL.
- [ ] Port the selector, time sync, ESKF, and delayed replay to Cortex R5F.
- [ ] Require Python, RTL, and R5F replay parity.
- [ ] Complete bench calibration, hardware-in-the-loop testing, and flight gates.

Descent, fixed-point estimator behavior, and the final packet envelope remain
deferred until the receiver and hardware interfaces are finalized.
