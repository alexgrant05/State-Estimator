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
    -> ADIS16470 + ADXL375 + BMP581 + generic GNSS/PPS
    -> 100 MHz timestamped events and deterministic sensor replays
    -> shared-bus scheduling and high-g acceleration selection
    -> delayed 15-state ESKF fusion with rewind/replay
    -> states, metrics, plots, manifest, and pass/fail gates
```

Implemented digital-twin behavior includes:

- Launch-centered ENU navigation, MSL altitude, and WGS84 ECEF GNSS handling.
- ADIS16470 at 500 Hz with exact 176-bit burst transactions.
- ADXL375 at 800 Hz with hysteretic handoff before ADIS saturation.
- BMP581 at 50 Hz with raw register output, pad calibration, flight-phase
  suppression, transonic disturbance modeling, and innovation gating.
- Generic GNSS at 10 Hz and PPS at 1 Hz with covariance, latency, clock error,
  antenna lever arm, correlated outages, and a replaceable receiver adapter.
- Dedicated ADIS SPI and deterministic shared-SPI arbitration for ADXL and BMP.
- Separate measurement and arrival epochs plus two seconds of estimator history
  for delayed aiding updates.
- Deterministic fault injection, binary replay, artifact hashing, and 200-seed
  statistical validation.

See [sim/README.md](sim/README.md) for sensor conventions, configuration,
artifacts, detailed verification, and the remaining porting sequence.

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

The current suite has 43 passing tests. The reference seed-42 Andromeda run
passes all integration gates with 54,693 events, 1,362 delayed rewinds, zero
history misses, and RMS errors of 0.149 m position, 0.092 m/s velocity, and
1.121 degrees attitude. These are reference results, not final flight limits.

## Vivado setup

```powershell
vivado -mode batch -source build.tcl
```

This regenerates `vivado/State-Estimation/` for the KR260. Generated Vivado
files are not required by the Python simulation.

## Project status

- [x] KR260 project skeleton and Zynq UltraScale+ PS block design.
- [x] ADIS16470 model, exact replay, and inertial propagation.
- [x] ADXL375 model and high-g transition logic.
- [x] BMP581 pressure and temperature model with aided updates.
- [x] Generic GNSS/PPS model, latency, outages, time sync, and delayed fusion.
- [x] Merged multi-sensor logical events and per-sensor binary replays.
- [x] Fault campaigns, 200-seed statistics, and Andromeda integration gates.
- [ ] Replace generic and placeholder values with measured flight-hardware data.
- [ ] Select the exact GNSS receiver and add its wire-format adapter.
- [ ] Freeze the common FPGA-to-R5F packet envelope.
- [ ] Implement sensor acquisition, timestamping, GNSS UART, and PPS capture RTL.
- [ ] Port the selector, time sync, ESKF, and delayed replay to Cortex R5F.
- [ ] Require Python, RTL, and R5F replay parity.
- [ ] Complete bench calibration, hardware-in-the-loop testing, and flight gates.

Descent, fixed-point estimator behavior, and the final packet envelope remain
deferred until the receiver and hardware interfaces are finalized.
