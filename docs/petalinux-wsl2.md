# PetaLinux 2026.1 on WSL2

This repository uses Vivado 2026.1, so its matching Linux build environment is
PetaLinux 2026.1. PetaLinux tools and projects must remain in the native WSL
ext4 filesystem. Do not install or build under `/mnt/c`.

## Current host layout

- Tool installation: `~/tools/petalinux/2026.1`
- Projects: `~/work/petalinux`
- Installer cache: `~/.cache/petalinux/installers`
- Generated XSA: `vivado/export/state_estimator_kr260.xsa`

The Ubuntu prerequisites, `en_US.UTF-8` locale, Bash `/bin/sh`, and native
Linux directories are prepared by the host setup. Verify them at any time:

```bash
cd "/mnt/c/Rocketry 26-27/State-Estimation"
bash tools/petalinux/check_host.sh
```

## Install the tools

Download the 2026.1 installer from the AMD Embedded Software download page in
Windows. AMD account login and license acceptance can require a browser. Then
run the following in Ubuntu, adjusting the filename if AMD changes its build
suffix:

```bash
cd "/mnt/c/Rocketry 26-27/State-Estimation"
bash tools/petalinux/install_2026_1.sh \
  /mnt/c/Users/0agra/Downloads/petalinux-v2026.1-final-installer.run
```

The script copies the installer to ext4, verifies AMD's published MD5 checksum,
installs only the AArch64 eSDK required by the K26, and verifies the resulting
commands. The installer itself runs as the normal `agra` user and prompts for
AMD license acceptance.

Start a new WSL shell after installation, then load the environment:

```bash
cd "/mnt/c/Rocketry 26-27/State-Estimation"
source tools/petalinux/env.sh
echo "$PETALINUX"
```

## Export hardware and create the project

Run Vivado from Windows. The build now exports an implemented XSA containing
the bitstream:

```powershell
vivado -mode batch -source build.tcl
vivado -mode batch -source tools/vivado_build.tcl
```

Then create the PetaLinux project in ext4 from Ubuntu:

```bash
cd "/mnt/c/Rocketry 26-27/State-Estimation"
bash tools/petalinux/create_project.sh
cd ~/work/petalinux/state-estimator-linux
petalinux-build
```

The helper refuses to overwrite an existing project. Pass an XSA and project
name as arguments if a separate build is needed.

## WSL notes

- USB access to a board from WSL is not required for image creation. Vivado and
  JTAG can remain on Windows.
- Do not apply the Ubuntu 24.04 AppArmor user-namespace workaround unless a
  build reports a `uid_map` error.
- The first build downloads a substantial Yocto source and sstate set and can
  take a long time.
- PetaLinux configuration and generated build directories stay in ext4. Only
  source-controlled recipes, configuration fragments, and scripts should be
  copied back into this repository.

## Board bring-up checkpoint

The current image boots PetaLinux 2026.1 from SD and exposes the control-bank
UIO node after loading the generic UIO platform driver:

```bash
sudo modprobe uio_pdrv_genirq of_id=generic-uio
cat /sys/class/uio/uio*/name
```

The expected node name is `state-estimator-uio@a0000000`. Do not access
`0xA0000000` from Linux until the PL has been loaded with the matching
bitstream and the PS-to-PL path has passed the JTAG smoke test.

For the next bench session, leave Linux powered off and run these commands in
Windows from the repository root:

```powershell
& 'C:\AMDDesignTools\2026.1\Vitis\bin\xsdb.bat' `
  'tools/xsdb_led_bringup.tcl' `
  'vivado/State-Estimation/State-Estimation.runs/impl_1/state_est_bd_wrapper.bit' `
  'vivado/State-Estimation/State-Estimation.runs/impl_1/state_est_bd_wrapper.psu_init.tcl'

& 'C:\AMDDesignTools\2026.1\Vitis\bin\xsdb.bat' `
  'tools/xsdb_abi_smoke_test.tcl'
```

The first command performs a cold JTAG PS and PL bring-up. The second checks
the ABI magic, ABI version, declared clock rate, scratch register, and live
timebase. A passing smoke test ends with `ABI_SMOKE:SUCCESS`. Reboot with SW2
after the bench test to return to the SD Linux image.

Current checkpoint:

- The Vivado build passes synthesis, implementation, timing, DRC, and
  bitstream generation.
- Cold JTAG initialization reports `BRINGUP:SUCCESS` and enables PL0 clock.
- Linux boots and the device tree creates the expected generic UIO node.
- The PL loader applies the generated AXI fabric reset configuration before
  releasing PS-to-PL isolation.
- The reset controller holds both active-low reset inputs inactive-high.
- The JTAG ABI smoke test passes core magic, ABI version, clock declaration,
  scratch read/write, and monotonic timebase checks.
- Linux UIO access passes at `0xA0000000`; autonomous PL loading during boot
  remains to be tested.

## Autonomous PL boot

The SD boot partition uses `uEnv.txt`, which is already imported by the
generated PetaLinux boot script. The repository version loads `system.bit`,
enables the 100 MHz PL0 clock, and releases the AXI fabric-interface resets
before `image.ub` starts Linux.

After updating the project from the current XSA and running `petalinux-build`,
stage the complete boot-partition file set with:

```bash
cd "/mnt/c/Rocketry 26-27/State-Estimation"
bash tools/petalinux/update_project.sh
cd ~/work/petalinux/state-estimator-linux
petalinux-build
cd "/mnt/c/Rocketry 26-27/State-Estimation"
bash tools/petalinux/stage_sd_boot.sh
```

Copy every file from `outputs/kr260-sd-boot` to the SD boot partition. Verify
the copy with `SHA256SUMS`; do not mix files from different builds.
