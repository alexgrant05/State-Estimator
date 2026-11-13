#!/usr/bin/env bash
set -euo pipefail

firmware_dir=/lib/firmware/xilinx/state-estimator-pl
bitstream="$firmware_dir/state-estimator-pl.bin"

if [[ ! -e /sys/class/fpga_manager/fpga0 ]]; then
    echo "ZynqMP FPGA Manager is unavailable" >&2
    exit 1
fi
if [[ ! -f "$bitstream" ]]; then
    echo "PL firmware is missing: $bitstream" >&2
    exit 1
fi

fpgautil -b "$bitstream"

# Enable PL0 at 100 MHz and release the generated AXI fabric resets. The FPGA
# Manager performs PL power and isolation operations through platform firmware.
devmem 0xFF5E00C0 32 0x01010A00
fpd_reset=$(devmem 0xFD1A0100 32)
lpd_reset=$(devmem 0xFF5E023C 32)
devmem 0xFD1A0100 32 $((fpd_reset & 0xFFFFE07F))
devmem 0xFF5E023C 32 $((lpd_reset & 0xFFF7FFFF))

# Apply the generated PS-to-PL fabric reset pulse.
gpio_mask=$(devmem 0xFF0A002C 32)
devmem 0xFF0A002C 32 $(((gpio_mask & 0x0000FFFF) | 0x80000000))
devmem 0xFF0A0344 32 0x80000000
devmem 0xFF0A0348 32 0x80000000
devmem 0xFF0A0054 32 0x80000000
devmem 0xFF0A0054 32 0x00000000
devmem 0xFF0A0054 32 0x80000000

modprobe uio_pdrv_genirq of_id=generic-uio

magic=$(devmem 0xA0000000 32)
if [[ "${magic^^}" != "0X54534553" ]]; then
    echo "State-estimator CORE_MAGIC mismatch: $magic" >&2
    exit 1
fi

echo "State-estimator PL loaded and ABI identity verified"
