# Build a minimal clock-free diagnostic bitstream that drives KR260 J2 pin 1 high.
# This deliberately excludes the processing system so it can distinguish an I/O
# or wiring problem from a missing PS-provided PL clock.

set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set build_dir [file join $repo_dir .rtl_build kr260_led_static_high]
set output_bit [file join $repo_dir .rtl_build kr260_led_static_high.bit]
set rtl_file [file join $repo_dir src rtl bringup led_static_high.sv]
set xdc_file [file join $repo_dir src constraints kr260_bringup.xdc]

file mkdir $build_dir
create_project -in_memory -part xck26-sfvc784-2LV-c
add_files -norecurse [list $rtl_file]
read_xdc [list $xdc_file]
set_property top led_static_high [current_fileset]

# A constant-only diagnostic intentionally has no internal logic and drives its
# sole port with a constant. Suppress only those two expected messages, then
# promote every other Vivado warning to an error.
set_msg_config -id {Synth 8-3330} -suppress
set_msg_config -id {Synth 8-3917} -suppress
set_msg_config -severity WARNING -new_severity ERROR

synth_design -top led_static_high -part xck26-sfvc784-2LV-c
opt_design
place_design
route_design

set failing_paths [get_timing_paths -quiet -delay_type max -slack_lesser_than 0 -max_paths 1]
if {[llength $failing_paths] > 0} {
  error "Static LED diagnostic has a failing timing path."
}

set fatal_drc_violations {}
foreach violation [get_drc_violations -quiet] {
  set severity [get_property SEVERITY $violation]
  if {$severity eq "Error" || $severity eq "Critical Warning"} {
    lappend fatal_drc_violations $violation
  }
}
if {[llength $fatal_drc_violations] > 0} {
  error "Static LED diagnostic has [llength $fatal_drc_violations] fatal DRC violations."
}

write_bitstream -force $output_bit
puts "STATIC_LED_BUILD:SUCCESS"
puts "STATIC_LED_BUILD:BITSTREAM=[file normalize $output_bit]"
