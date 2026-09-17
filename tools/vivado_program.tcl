# Program the KR260 PL over JTAG. This is volatile and does not modify QSPI.
# Usage: vivado -mode batch -source tools/vivado_program.tcl -tclargs path/to/file.bit

if {$argc != 1} {
  error "Usage: vivado -mode batch -source tools/vivado_program.tcl -tclargs <bitstream.bit>"
}

set bit_file [file normalize [lindex $argv 0]]
if {![file exists $bit_file]} {
  error "Bitstream not found: $bit_file"
}

open_hw_manager
connect_hw_server -allow_non_jtag

set hw_targets [get_hw_targets -quiet]
if {[llength $hw_targets] != 1} {
  disconnect_hw_server
  close_hw_manager
  error "Refusing to program: expected exactly one JTAG hardware target, found [llength $hw_targets]. Check KR260 power, the J4 micro-USB data cable, and the FTDI driver."
}
current_hw_target [lindex $hw_targets 0]
open_hw_target

set k26_devices [get_hw_devices -quiet -filter {PART == "xck26"}]
if {[llength $k26_devices] != 1} {
  close_hw_target
  disconnect_hw_server
  close_hw_manager
  error "Refusing to program: expected exactly one xck26 device, found [llength $k26_devices]."
}

set device [lindex $k26_devices 0]
current_hw_device $device
refresh_hw_device -update_hw_probes false $device
set_property PROGRAM.FILE $bit_file $device
program_hw_devices $device
refresh_hw_device -update_hw_probes false $device

set done_property "REGISTER.IR.BIT05_DONE"
set done_value [get_property $done_property $device]
if {$done_value ne "1"} {
  close_hw_target
  disconnect_hw_server
  close_hw_manager
  error "Programming command returned, but the K26 DONE bit is '$done_value'."
}

# Catch a board reset or platform manager clearing the PL immediately after
# configuration. A successful programming command alone does not prove that
# the design remains loaded long enough to run.
after 5000
refresh_hw_device -update_hw_probes false $device
set stable_done_value [get_property $done_property $device]
if {$stable_done_value ne "1"} {
  close_hw_target
  disconnect_hw_server
  close_hw_manager
  error "The K26 DONE bit fell to '$stable_done_value' five seconds after programming. The board reset or another platform component cleared the PL."
}

puts "PROGRAM:SUCCESS"
puts "PROGRAM:DEVICE=[get_property NAME $device]"
puts "PROGRAM:BITSTREAM=$bit_file"
puts "PROGRAM:DONE=$done_value"
puts "PROGRAM:DONE_AFTER_5S=$stable_done_value"

close_hw_target
disconnect_hw_server
close_hw_manager
