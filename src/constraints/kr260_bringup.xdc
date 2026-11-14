# KR260 Pmod 1 (J2), pin 1. Connect through a 330 ohm resistor to the
# LED anode, then connect the LED cathode to J2 pin 9 (GND).
set_property PACKAGE_PIN H12 [get_ports led_0]
set_property IOSTANDARD LVCMOS33 [get_ports led_0]
set_property DRIVE 4 [get_ports led_0]
set_property SLEW SLOW [get_ports led_0]
