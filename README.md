# State Estimator

State-estimation platform for Cornell Rocketry, targeting the AMD Kria KR260
(K26 SOM + Robotics Starter Kit carrier). The repository contains both the
Vivado hardware skeleton and the Python reference digital twin used to validate
sensor timing, packets, and estimator behavior before hardware deployment.

## Repository layout

- `build.tcl` — regenerates the Vivado project and block design
- `src/rtl/` — custom Verilog/SystemVerilog sources
- `src/constraints/` — XDC constraints
- `src/tb/` — RTL testbenches
- `sim/` — installable Python digital twin and validation suite
- `vivado/` — generated Vivado project (ignored)

## Digital twin

The first implemented slice is:

```text
Andromeda RocketPy truth
    -> ADIS16470 model
    -> 100 MHz timestamped logical events + exact burst transactions
    -> 15-state inertial ESKF
    -> validation report
```

See [`sim/README.md`](sim/README.md) for setup, commands, conventions, output
artifacts, and acceptance gates.

## Vivado setup

```powershell
vivado -mode batch -source build.tcl
```

This rebuilds `vivado/State-Estimation/` for the KR260 board. Digital-twin work
does not modify or depend on the generated Vivado project.

## Status

- [x] KR260 project skeleton and Zynq UltraScale+ PS block design
- [x] ADIS16470 Python vertical slice and replay format
- [x] Event-driven inertial ESKF and layered validation
- [ ] ADXL375 model and high-g transition logic
- [ ] BMP581 pressure/temperature model
- [ ] GNSS navigation, PPS, latency, and outage model
- [ ] Common multi-sensor FPGA packet envelope
- [ ] Sensor acquisition and timestamping RTL
- [ ] Cortex R5F estimator port and replay comparison
