# FPGA-to-Linux ABI v1

This file is generated from `spec/fpga_linux_abi.yaml`. Do not edit it directly.

Schema SHA-256: `0464018e401bc6c78a00dfce9234dabf024c8a780a530b3ec1e47161437014e9`

## Address map

| Region | Base | Size |
| --- | ---: | ---: |
| control | `0xA0000000` | 64 KiB |
| r5_bram_ring | `0xA0010000` | 64 KiB |
| axi_dma | `0xA0020000` | 64 KiB |
| reserved | `0xA0030000` | 64 KiB |

## Core registers

All accesses are aligned, little-endian 32-bit AXI-Lite transactions.

| Offset | Register | Access |
| ---: | --- | --- |
| `0x0000` | `CORE_MAGIC` | RO |
| `0x0004` | `ABI_VERSION` | RO |
| `0x0008` | `CORE_VERSION` | RO |
| `0x000C` | `CAPABILITIES` | RO |
| `0x0010` | `CLOCK_HZ` | RO |
| `0x0014` | `SENSOR_PRESENT` | RO |
| `0x0018` | `BUILD_ID0` | RO |
| `0x001C` | `BUILD_ID1` | RO |
| `0x0020` | `BUILD_ID2` | RO |
| `0x0024` | `BUILD_ID3` | RO |
| `0x0028` | `ABI_HASH_LO` | RO |
| `0x002C` | `ABI_HASH_HI` | RO |
| `0x0030` | `SESSION_ID` | RW |
| `0x0034` | `RESET_GENERATION` | RO |
| `0x0038` | `CONTROL` | RW |
| `0x003C` | `STATUS` | RO |
| `0x0040` | `ERROR_STATUS` | W1C |
| `0x0044` | `ERROR_ENABLE` | RW |
| `0x0048` | `IRQ_STATUS` | W1C |
| `0x004C` | `IRQ_ENABLE` | RW |
| `0x0050` | `IRQ_FORCE` | WO |
| `0x0054` | `IRQ_COALESCE_BLOCKS` | RW |
| `0x0058` | `IRQ_COALESCE_TICKS` | RW |
| `0x005C` | `SCRATCH` | RW |
| `0x0060` | `TIME_NOW_LO` | RO |
| `0x0064` | `TIME_NOW_HI` | RO |
| `0x0068` | `LAST_PPS_TICKS_LO` | RO |
| `0x006C` | `LAST_PPS_TICKS_HI` | RO |
| `0x0070` | `PPS_COUNT` | RO |
| `0x0074` | `TIME_STATUS` | RO |
| `0x0080` | `PACKET_COUNT` | RO |
| `0x0084` | `R5_DROP_COUNT` | RO |
| `0x0088` | `LINUX_DROP_COUNT` | RO |
| `0x008C` | `DMA_BLOCK_COUNT` | RO |
| `0x0090` | `BACKPRESSURE_COUNT` | RO |
| `0x0094` | `HOST_OVERRUN_COUNT` | RO |
| `0x0098` | `EVENT_OVERFLOW_COUNT` | RO |
| `0x009C` | `PACKET_OVERFLOW_COUNT` | RO |
| `0x00A0` | `LAST_FATAL` | RO |
| `0x00A4` | `RESERVED_COUNTER` | RO |
| `0x00A8` | `RESET_KEY` | WO |
| `0x00AC` | `RESET_CONTROL` | WO |

## DMA mirror bank at 0x0100

| Relative offset | Register | Access |
| ---: | --- | --- |
| `0x00` | `STATUS` | RO |
| `0x04` | `BLOCK_SEQUENCE` | RO |
| `0x08` | `PL_DROP_COUNT` | RO |
| `0x0C` | `BLOCK_COUNT` | RO |
| `0x10` | `CURRENT_VALID_BYTES` | RO |
| `0x14` | `FLUSH` | WO |
| `0x18` | `HOST_CONSUMER_BLOCK` | RW |
| `0x1C` | `HOST_OVERRUN_COUNT` | RO |

## R5F ring bank at 0x0200

| Relative offset | Register | Access |
| ---: | --- | --- |
| `0x00` | `STATUS` | RO |
| `0x04` | `PRODUCER_OFFSET` | RO |
| `0x08` | `CONSUMER_OFFSET` | RW |
| `0x0C` | `USED_BYTES` | RO |
| `0x10` | `WATERMARK_BYTES` | RW |
| `0x14` | `DROP_COUNT` | RO |
| `0x18` | `RESET` | WO |

## Replay bank at 0x0300

| Relative offset | Register | Access |
| ---: | --- | --- |
| `0x00` | `STATUS` | RO |
| `0x04` | `CONTROL` | RW |
| `0x08` | `ACCEPTED_COUNT` | RO |
| `0x0C` | `REJECTED_COUNT` | RO |
| `0x10` | `LAST_ERROR` | RO |
| `0x14` | `RELEASED_COUNT` | RO |

## Per-sensor bank

| Relative offset | Register | Access |
| ---: | --- | --- |
| `0x00` | `IDENTITY` | RO |
| `0x04` | `CONTROL` | RW |
| `0x08` | `HEALTH` | RO |
| `0x0C` | `ERROR_STATUS` | W1C |
| `0x10` | `ERROR_ENABLE` | RW |
| `0x14` | `EVENT_COUNT` | RO |
| `0x18` | `PACKET_COUNT` | RO |
| `0x1C` | `DROP_COUNT` | RO |
| `0x20` | `LAST_MEASUREMENT_LO` | RO |
| `0x24` | `LAST_MEASUREMENT_HI` | RO |
| `0x28` | `LAST_ARRIVAL_LO` | RO |
| `0x2C` | `LAST_ARRIVAL_HI` | RO |
| `0x30` | `LAST_SEQUENCE` | RO |
| `0x34` | `NATIVE_STATUS` | RO |
| `0x38` | `BUS_CLOCK_HZ` | RO |
| `0x3C` | `REPORT_RATES` | RO |
| `0x40` | `TIMEOUT_TICKS` | RW |
| `0x44` | `CONFIG_HASH_LO` | RO |
| `0x48` | `CONFIG_HASH_HI` | RO |

## Sensor identifiers

| ID | Name |
| ---: | --- |
| 0 | `system_health` |
| 1 | `legacy_adis16470` |
| 2 | `adxl375` |
| 3 | `bmp581` |
| 4 | `legacy_gnss_solution` |
| 5 | `legacy_gnss_pps` |
| 6 | `bno085_shtp` |
| 7 | `zed_f9p_ubx` |
| 8 | `zed_f9p_timepulse` |

## Common record

Header: 48 bytes. Maximum payload: 3584 bytes. Records are 8-byte aligned and never cross a DMA block.

| Offset | Type | Field |
| ---: | --- | --- |
| 0 | `u32` | `magic` |
| 4 | `u8` | `format_version` |
| 5 | `u8` | `sensor_id` |
| 6 | `u16` | `header_bytes` |
| 8 | `u16` | `total_bytes` |
| 10 | `u16` | `payload_bytes` |
| 12 | `u32` | `sequence` |
| 16 | `u64` | `measurement_ticks` |
| 24 | `u64` | `arrival_ticks` |
| 32 | `u32` | `status_flags` |
| 36 | `u32` | `reserved` |
| 40 | `u32` | `payload_crc32c` |
| 44 | `u32` | `header_crc32c` |

## DMA block

The coherent ring contains 256 blocks of 4096 bytes.

| Offset | Type | Field |
| ---: | --- | --- |
| 0 | `u32` | `magic` |
| 4 | `u16` | `abi_major` |
| 6 | `u16` | `header_bytes` |
| 8 | `u32` | `sequence` |
| 12 | `u32` | `flags` |
| 16 | `u16` | `valid_bytes` |
| 18 | `u16` | `packet_count` |
| 20 | `u32` | `sensor_mask` |
| 24 | `u32` | `pl_drops` |
| 28 | `u32` | `reset_generation` |
| 32 | `u32` | `session_id` |
| 36 | `u32` | `reserved` |
| 40 | `u64` | `first_measurement_ticks` |
| 48 | `u64` | `last_measurement_ticks` |
| 56 | `u32` | `payload_crc32c` |
| 60 | `u32` | `header_crc32c` |

CRC32C uses reflected polynomial `0x82F63B78`, initial value `0xFFFFFFFF`, and final XOR `0xFFFFFFFF`.
