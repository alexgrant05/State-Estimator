# Linux and Rust Verification

Linux is the configuration, logging, diagnostics, and verification control
plane. The Cortex R5F remains the flight estimator and consumes the guaranteed
BRAM record ring. Linux receives a best-effort DMA mirror that cannot apply
backpressure to sensor acquisition or the R5F path.

## Rust workspace

The Edition 2021 workspace contains:

- `se-abi`: `no_std`, endian-safe records, DMA blocks, replay items, CRC32C,
  version checks, and generated constants
- `se-device`: guarded AXI-Lite access, Linux UIO mapping, DMA ioctls, and a
  read-only coherent-ring mapping
- `se-hwtest`: board identification, register, timebase, interrupt, and reset
  checks
- `se-logger`: strict DMA-block validation and lossless raw logging
- `se-replay`: conversion of canonical records into gated replay items

Build and test on the host with:

```bash
python3 tools/generate_fpga_abi.py --check
cargo fmt --manifest-path linux/rust/Cargo.toml --all -- --check
cargo clippy --manifest-path linux/rust/Cargo.toml --all-targets -- -D warnings
RUSTFLAGS=-Dwarnings cargo test --manifest-path linux/rust/Cargo.toml
```

For AArch64 deployment, install the Rust target, build, and copy the tools to
the board over removable media or Ethernet when available:

```bash
rustup target add aarch64-unknown-linux-gnu
cargo build --manifest-path linux/rust/Cargo.toml --release \
  --target aarch64-unknown-linux-gnu
```

The cross-linker must come from the PetaLinux SDK. Native compilation on the
board is also possible after adding Rust to the image.

## Hardware test CLI

The bring-up UIO path supports read-only identification, register snapshots,
the ten-second timebase test, test-mode interrupt forcing, and keyed soft reset:

```bash
se-hwtest --uio /dev/uio0 identify
se-hwtest --uio /dev/uio0 registers
se-hwtest --uio /dev/uio0 timebase
se-hwtest --uio /dev/uio0 interrupts
se-hwtest --uio /dev/uio0 reset
se-hwtest --uio /dev/uio0 --device /dev/state-estimator dma
se-hwtest --uio /dev/uio0 report
se-hwtest --uio /dev/uio0 lock
```

`identify` verifies `CORE_MAGIC`, ABI major, and ABI hash before any mutating
command is allowed. The DMA command verifies the kernel ring geometry, and the
report command emits a machine-readable health snapshot. The lock command is
intentionally irreversible until a hard PL reset. Replay and overflow commands
return an error until their remaining RTL engines are connected. This prevents
a partial implementation from being mistaken for a passing hardware test.

## Kernel boundary

`kernel/state_estimator_dma.c` is the minimal conventional kernel shim. It owns
both DMA channels, allocates the coherent 1 MiB ring, provides read-only `mmap`
and `poll`, and restricts start, stop, and reset ioctls to privileged callers.
Protocol decoding and project policy stay in Rust.

`petalinux/system-user.dtsi` contains disabled templates for UIO bring-up and
the final driver. They are mutually exclusive. The driver node still needs the
Vivado-generated DMA and interrupt phandles before it can be enabled.

## Current integration status

The AXI-Lite control bank is integrated into `build.tcl` at `0xA000_0000` and
is reachable through `M_AXI_HPM0_FPD`. R5F BRAM, AXI DMA, the block packer,
replay engine, and sensor banks remain to be connected as their RTL producers
land. Their addresses and behavior are already frozen by ABI v1.

See [the generated ABI reference](../docs/fpga-linux-abi.md) and
[the integration contract](../docs/fpga-linux-integration.md).
