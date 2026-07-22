# State Estimator

FPGA-based state estimator for Cornell Rocketry, targeting the AMD Kria KR260
(K26 SOM + Robotics Starter Kit carrier card). Integrates with a Python
digital twin for simulation and validation before flight.

## Repository layout

- `build.tcl` — Regenerates the Vivado project + block design from scratch
- `src/rtl/` — Custom Verilog/SystemVerilog sources
- `src/constraints/` — XDC constraint files
- `src/tb/` — Testbenches
- `vivado/` — Generated Vivado project (gitignored, not tracked)

## Setup

```bash
cd State-Estimator
vivado -mode batch -source build.tcl
```

Rebuilds the project under `vivado/State-Estimation/`, configured for the
KR260 with the Zynq UltraScale+ PS block design and board preset applied.

## Status

- [x] Project skeleton + KR260 board configuration
- [x] Zynq UltraScale+ PS block design (board preset applied)
- [ ] Sensor interface configuration (SPI/I2C/UART for IMU, baro, GNSS)
- [ ] Timestamping/data-integrity coprocessor RTL
- [ ] Connection automation (PS <-> PL wiring) once coprocessor RTL exists
- [ ] Sensor-interface XDC constraints
- [ ] Integration with Python digital twin
- [ ] Build/flash workflow (still TBD)
- [ ] Linux bring-up on target (after hardware flow is solid)
