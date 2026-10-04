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
