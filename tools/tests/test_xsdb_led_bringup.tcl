# Host-only tests: all hardware commands run as mocks in a child interpreter.
# Usage: xsdb tools/tests/test_xsdb_led_bringup.tcl
set repo_dir [file dirname [file dirname [file dirname [file normalize [info script]]]]]
set fixture_dir [file join $repo_dir .rtl_build {xsdb test fixtures}]
file mkdir $fixture_dir
set bit_file [file join $fixture_dir {test design.bit}]
set init_file [file join $fixture_dir {test psu_init.tcl}]
set stream [open $bit_file w]
close $stream
set stream [open $init_file w]
puts $stream {
    set generated_global_data present
    proc psu_init {} {
        if {$::generated_global_data ne "present"} { error "Incorrect source scope" }
        lappend ::events init
        if {$::fail_init} { error "PS initialization failed" }
    }
    proc psu_ps_pl_isolation_removal {} { lappend ::events isolation }
    proc psu_ps_pl_reset_config {} { lappend ::events reset }
}
close $stream

proc run_case {name overrides expected_error expected_events} {
    global repo_dir bit_file init_file
    set child [interp create]
    interp eval $child {
        set events {}
        set output {}
        set k26_count 1
        set psu_count 1
        set fail_init 0
        set clock_enabled 1
        set lose_config 0
        set initial_config 1
        set state_reads 0
        proc connect {} { lappend ::events connect }
        proc disconnect {} { lappend ::events disconnect }
        proc jtag {args} { return [lrepeat $::k26_count {name xck26}] }
        proc targets {args} {
            if {[lindex $args 0] eq "-set"} {
                lappend ::events select_psu
                return
            }
            return [lrepeat $::psu_count {name PSU}]
        }
        proc fpga {args} {
            if {[lindex $args 0] eq "-file"} {
                if {[lindex $args 1] ne [lindex $::argv 0]} { error "Broken bitstream path" }
                lappend ::events program
                return
            }
            incr ::state_reads
            lappend ::events configured
            if {!$::initial_config || ($::lose_config && $::state_reads == 2)} {
                return "FPGA is not configured"
            }
            return "FPGA is configured"
        }
        proc mrd {args} {
            if {$args ne {-value 0xFF5E00C0}} { error "Unexpected register read" }
            lappend ::events clock_read
            return [expr {$::clock_enabled ? 0x01010A00 : 0}]
        }
        proc after {milliseconds} { lappend ::events delay_$milliseconds }
        proc puts {args} { lappend ::output [lindex $args end] }
        proc exit {code} { error "EXIT:$code" }
    }
    interp eval $child [list set argc 2]
    interp eval $child [list set argv [list $bit_file $init_file]]
    interp eval $child $overrides
    set failed [catch {
        interp eval $child [list source [file join $repo_dir tools xsdb_led_bringup.tcl]]
    } message]
    set events [interp eval $child {set events}]
    set output [interp eval $child {join $output \n}]
    interp delete $child
    if {$expected_error eq ""} {
        if {$failed || ![string match "*BRINGUP:SUCCESS*" $output]} {
            error "$name: unexpected failure: $message; $output"
        }
    } elseif {!$failed || $message ne "EXIT:1" || ![string match "*$expected_error*" $output]} {
        error "$name: expected '$expected_error', got '$message'; $output"
    }
    if {$events ne $expected_events} {
        error "$name: wrong sequence: $events (expected $expected_events)"
    }
    puts "PASS $name"
}

if {[catch {
    set prefix {connect select_psu program configured}
    set init_events [concat $prefix {init delay_1000 isolation delay_1000 reset clock_read}]
    run_case success {} {} [concat $init_events {delay_5000 configured disconnect}]
    run_case missing_input {lset argv 0 [file join [file dirname [lindex $argv 0]] missing.bit]} \
        {Missing or unreadable input} {}
    run_case no_board {set k26_count 0; set psu_count 0} \
        {Expected exactly one K26 and one PSU} {connect disconnect}
    run_case multiple_boards {set k26_count 2; set psu_count 2} \
        {Expected exactly one K26 and one PSU} {connect disconnect}
    run_case not_configured {set initial_config 0} \
        {not configured immediately} [concat $prefix {disconnect}]
    run_case init_failure {set fail_init 1} \
        {PS initialization failed} [concat $prefix {init disconnect}]
    run_case clock_disabled {set clock_enabled 0} \
        {CLKACT is clear} [concat $init_events {disconnect}]
    run_case configuration_lost {set lose_config 1} \
        {configuration was lost} [concat $init_events {delay_5000 configured disconnect}]
} message]} {
    puts stderr $message
    exit 1
}
puts "PASS 8 XSDB bring-up script tests (mock hardware)"
