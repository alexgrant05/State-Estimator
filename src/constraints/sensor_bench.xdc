## ============================================================================
## Sensor test bench - constraints for the Digilent Arty S7-25 (Rev. E)
## Port names match sensor_bench_top. Same pins as adxl375_rig.xdc (ADXL375,
## JC) and bno085_rig.xdc (BNO085, JB); the Pico link on JD is shared.
##
## Header plan for the 4-sensor bench:
##   JA  BMP581      slot 2     (same pins as rtl/BMP581/bmp581_arty_s7.xdc)
##   JB  BNO085      slot 1     (wiring: see bno085_rig.xdc)
##   JC  ADXL375     slot 0     (wiring: see adxl375_rig.xdc)
##   JD  Pico 2 W link (JD1 CS, JD2 data, JD4 SCLK, JD5 GND); JD3, JD7-10 free
##
##   BMP581 on Pmod JA:
##     JA1 L17 bmp_cs_n -> CSB    JA2 L18 bmp_mosi -> SDI    JA3 M14 bmp_miso <- SDO
##     JA4 N14 bmp_sclk -> SCK    JA7 M16 bmp_int  <- INT    JA5 GND, JA6 3.3 V -> VDD, VDDIO
##
##   rst_btn = BTN0   LEDs LD2..LD5 = led[0..3]:
##     LD2 ADXL375 streaming   LD3 BNO085 streaming   LD4 BMP581 streaming
##     LD5 a sensor has a fault now, or has needed a recovery since reset
## ============================================================================

## ---- Clock: 100 MHz oscillator (matches sensor_bench_top's CLK_HZ default) ----
set_property -dict { PACKAGE_PIN R2  IOSTANDARD SSTL135 } [get_ports { clk }]; #IO_L12P_T1_MRCC_34 Sch=ddr3_clk[200]
create_clock -add -name sys_clk_pin -period 10.000 -waveform {0 5.000} [get_ports { clk }];

## ---- Reset: BTN0, active high (RST_ACTIVE_LOW = 0), synchronized in the RTL ----
set_property -dict { PACKAGE_PIN G15 IOSTANDARD LVCMOS33 } [get_ports { rst_btn }]; #IO_L18N_T2_A23_15 Sch=btn[0]
set_false_path -from [get_ports rst_btn]

## ---- ADXL375 on Pmod JC ----
set_property -dict { PACKAGE_PIN U15 IOSTANDARD LVCMOS33 } [get_ports { acc_cs_n }]; #IO_L18P_T2_A12_D28_14 Sch=jc1
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports { acc_mosi }]; #IO_L18N_T2_A11_D27_14 Sch=jc2
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33 } [get_ports { acc_miso }]; #IO_L15P_T2_DQS_RDWR_B_14 Sch=jc3
set_property -dict { PACKAGE_PIN U18 IOSTANDARD LVCMOS33 } [get_ports { acc_sclk }]; #IO_L15N_T2_DQS_DOUT_CSO_B_14 Sch=jc4
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports { acc_int1 }]; #IO_L16P_T2_CSI_B_14 Sch=jc7
set_property PULLTYPE PULLDOWN [get_ports acc_int1]
set_property PULLTYPE PULLDOWN [get_ports acc_miso]

## ---- BNO085 on Pmod JB ----
set_property -dict { PACKAGE_PIN P17 IOSTANDARD LVCMOS33 } [get_ports { bno_cs_n   }]; #IO_L9P_T1_DQS_14 Sch=jb_p[1]
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVCMOS33 } [get_ports { bno_mosi   }]; #IO_L9N_T1_DQS_D13_14 Sch=jb_n[1]
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 } [get_ports { bno_miso   }]; #IO_L10P_T1_D14_14 Sch=jb_p[2]
set_property -dict { PACKAGE_PIN T18 IOSTANDARD LVCMOS33 } [get_ports { bno_sclk   }]; #IO_L10N_T1_D15_14 Sch=jb_n[2]
set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33 } [get_ports { bno_intn   }]; #IO_L11P_T1_SRCC_14 Sch=jb_p[3]
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 } [get_ports { bno_wake_n }]; #IO_L11N_T1_SRCC_14 Sch=jb_n[3]
set_property -dict { PACKAGE_PIN N15 IOSTANDARD LVCMOS33 } [get_ports { bno_rst_n  }]; #IO_L12P_T1_MRCC_14 Sch=jb_p[4]
set_property PULLTYPE PULLUP [get_ports bno_intn]
set_property PULLTYPE PULLUP [get_ports bno_miso]

## ---- BMP581 on Pmod JA ----
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports { bmp_cs_n }]; #IO_L4P_T0_D04_14 Sch=ja_p[1]
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports { bmp_mosi }]; #IO_L4N_T0_D05_14 Sch=ja_n[1]
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports { bmp_miso }]; #IO_L5P_T0_D06_14 Sch=ja_p[2]
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports { bmp_sclk }]; #IO_L5N_T0_D07_14 Sch=ja_n[2]
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports { bmp_int  }]; #IO_L7P_T1_D09_14 Sch=ja_p[3]
## INT floats until the controller enables it during bring-up: hold it low.
set_property PULLTYPE PULLDOWN [get_ports bmp_int]
## SDO is not driven until the sensor switches to SPI: keep MISO defined.
set_property PULLTYPE PULLUP [get_ports bmp_miso]

## ---- Pico 2 W on Pmod JD ----
set_property -dict { PACKAGE_PIN V15 IOSTANDARD LVCMOS33 } [get_ports { pico_cs_n }]; #IO_L20N_T3_A07_D23_14 Sch=jd1
set_property -dict { PACKAGE_PIN U12 IOSTANDARD LVCMOS33 } [get_ports { pico_mosi }]; #IO_L21P_T3_DQS_14 Sch=jd2
set_property -dict { PACKAGE_PIN T12 IOSTANDARD LVCMOS33 } [get_ports { pico_sclk }]; #IO_L22P_T3_A05_D21_14 Sch=jd4

## ---- SPI outputs (same choices as the single-sensor files) ----
set_property IOB TRUE  [get_ports {acc_cs_n acc_mosi bno_mosi bmp_cs_n bmp_mosi pico_cs_n pico_mosi}]
set_property SLEW SLOW [get_ports {acc_cs_n acc_sclk acc_mosi bno_cs_n bno_sclk bno_mosi bno_wake_n bno_rst_n bmp_cs_n bmp_sclk bmp_mosi pico_cs_n pico_sclk pico_mosi}]
set_property DRIVE 8   [get_ports {acc_cs_n acc_sclk acc_mosi bno_cs_n bno_sclk bno_mosi bno_wake_n bno_rst_n bmp_cs_n bmp_sclk bmp_mosi pico_cs_n pico_sclk pico_mosi}]

## ---- SPI timing: margins come from the RTL (see the single-sensor files) ----
set_false_path -from [get_ports {acc_miso acc_int1 bno_miso bno_intn bmp_miso bmp_int}]
set_false_path -to   [get_ports {acc_cs_n acc_sclk acc_mosi bno_cs_n bno_sclk bno_mosi bno_wake_n bno_rst_n bmp_cs_n bmp_sclk bmp_mosi pico_cs_n pico_sclk pico_mosi}]

## ---- LEDs (board labels LD2..LD5) ----
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports { led[0] }]; #IO_L16N_T2_A27_15 Sch=led[2]
set_property -dict { PACKAGE_PIN F13 IOSTANDARD LVCMOS33 } [get_ports { led[1] }]; #IO_L17P_T2_A26_15 Sch=led[3]
set_property -dict { PACKAGE_PIN E13 IOSTANDARD LVCMOS33 } [get_ports { led[2] }]; #IO_L17N_T2_A25_15 Sch=led[4]
set_property -dict { PACKAGE_PIN H15 IOSTANDARD LVCMOS33 } [get_ports { led[3] }]; #IO_L18P_T2_A24_15 Sch=led[5]
set_false_path -to [get_ports {led[*]}]

## ---- Configuration options (from Digilent's master XDC) ----
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]

## Bank 34 runs at 1.35 V and needs an internal VREF (the SSTL135 clock input).
set_property INTERNAL_VREF 0.675 [get_iobanks 34]
