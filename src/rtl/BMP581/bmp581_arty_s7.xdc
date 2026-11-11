## ============================================================================
## bmp581_arty_top on the Digilent Arty S7-25 (Rev. E)
## Pins taken from Digilent's Arty-S7-25-Master.xdc.
##
## BMP581 on Pmod JA (top row = pins 1-6, bottom row = pins 7-12):
##   JA1  L17  bmp_cs_n  -> BMP581 CSB        JA7   M16  bmp_int  <- BMP581 INT
##   JA2  L18  bmp_mosi  -> BMP581 SDI        JA8   (unused)
##   JA3  M14  bmp_miso  <- BMP581 SDO        JA9   (unused)
##   JA4  N14  bmp_sclk  -> BMP581 SCK        JA10  (unused)
##   JA5  GND  -> BMP581 GND                  JA11  GND
##   JA6  3.3V -> BMP581 VDD and VDDIO        JA12  3.3V
## ============================================================================

## 12 MHz system clock
set_property -dict { PACKAGE_PIN F14   IOSTANDARD LVCMOS33 } [get_ports { clk }]; #IO_L13P_T2_MRCC_15 Sch=uclk
create_clock -add -name sys_clk_pin -period 83.333 -waveform {0 41.667} [get_ports { clk }];

## BTN0 = reset (active high)
set_property -dict { PACKAGE_PIN G15   IOSTANDARD LVCMOS33 } [get_ports { btn_rst }]; #IO_L18N_T2_A23_15 Sch=btn[0]

## LEDs LD2..LD5
set_property -dict { PACKAGE_PIN E18   IOSTANDARD LVCMOS33 } [get_ports { led[0] }]; #IO_L16N_T2_A27_15 Sch=led[2]
set_property -dict { PACKAGE_PIN F13   IOSTANDARD LVCMOS33 } [get_ports { led[1] }]; #IO_L17P_T2_A26_15 Sch=led[3]
set_property -dict { PACKAGE_PIN E13   IOSTANDARD LVCMOS33 } [get_ports { led[2] }]; #IO_L17N_T2_A25_15 Sch=led[4]
set_property -dict { PACKAGE_PIN H15   IOSTANDARD LVCMOS33 } [get_ports { led[3] }]; #IO_L18P_T2_A24_15 Sch=led[5]

## BMP581 on Pmod JA
set_property -dict { PACKAGE_PIN L17   IOSTANDARD LVCMOS33 } [get_ports { bmp_cs_n }]; #IO_L4P_T0_D04_14 Sch=ja_p[1]
set_property -dict { PACKAGE_PIN L18   IOSTANDARD LVCMOS33 } [get_ports { bmp_mosi }]; #IO_L4N_T0_D05_14 Sch=ja_n[1]
set_property -dict { PACKAGE_PIN M14   IOSTANDARD LVCMOS33 } [get_ports { bmp_miso }]; #IO_L5P_T0_D06_14 Sch=ja_p[2]
set_property -dict { PACKAGE_PIN N14   IOSTANDARD LVCMOS33 } [get_ports { bmp_sclk }]; #IO_L5N_T0_D07_14 Sch=ja_n[2]
set_property -dict { PACKAGE_PIN M16   IOSTANDARD LVCMOS33 } [get_ports { bmp_int  }]; #IO_L7P_T1_D09_14 Sch=ja_p[3]

## INT floats until the controller enables it during bring-up: hold it low.
set_property PULLTYPE PULLDOWN [get_ports { bmp_int }]
## SDO is not driven until the sensor switches to SPI: keep MISO defined.
set_property PULLTYPE PULLUP [get_ports { bmp_miso }]

## The SPI pins and LEDs leave the chip through registers; nothing here is timed
## against an external device, so cut them from timing analysis.
set_false_path -to   [get_ports { bmp_cs_n bmp_mosi bmp_sclk led[*] }]
set_false_path -from [get_ports { bmp_miso bmp_int btn_rst }]

## Configuration options (from the Digilent master file, use for all designs)
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
## Required on the Arty S7 even if SW3 is unused (see the Digilent master file).
set_property INTERNAL_VREF 0.675 [get_iobanks 34]
