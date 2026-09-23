# Verify that one KR260 K26 device is visible and report its configuration state.
# Usage: vivado -mode batch -source tools/vivado_check_jtag.tcl

open_hw_manager
connect_hw_server -allow_non_jtag

set hw_targets [get_hw_targets -quiet]
if {[llength $hw_targets] != 1} {
  disconnect_hw_server
  close_hw_manager
  error "Expected exactly one JTAG hardware target, found [llength $hw_targets]. Check KR260 power, the J4 micro-USB data cable, and the FTDI driver."
}
current_hw_target [lindex $hw_targets 0]
open_hw_target

set k26_devices [get_hw_devices -quiet -filter {PART == "xck26"}]
if {[llength $k26_devices] != 1} {
  close_hw_target
  disconnect_hw_server
  close_hw_manager
  error "Expected exactly one xck26 device, found [llength $k26_devices]."
}

set device [lindex $k26_devices 0]
current_hw_device $device
refresh_hw_device -update_hw_probes false $device

set done_property "REGISTER.IR.BIT05_DONE"
set done_value [get_property $done_property $device]
puts "JTAG:SUCCESS"
puts "JTAG:DEVICE=[get_property NAME $device]"
puts "JTAG:PART=[get_property PART $device]"
puts "JTAG:DONE=$done_value"

close_hw_target
disconnect_hw_server
close_hw_manager
