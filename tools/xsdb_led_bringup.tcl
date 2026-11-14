# Volatile KR260 bring-up, including the PS clock configuration omitted by
# Vivado Hardware Manager. Run on a cold-started bench board, not live Linux.
# Usage: xsdb tools/xsdb_led_bringup.tcl <design.bit> <matching_psu_init.tcl>

if {$argc != 2} {
    puts stderr "Usage: xsdb tools/xsdb_led_bringup.tcl <design.bit> <matching_psu_init.tcl>"
    exit 1
}
set bit_file [file normalize [lindex $argv 0]]
set init_file [file normalize [lindex $argv 1]]

# Use top-level source: generated psu_init.tcl defines global data and procs.
set connected 0
if {[catch {
    foreach input_file [list $bit_file $init_file] {
        if {![file isfile $input_file] || ![file readable $input_file]} {
            error "Missing or unreadable input: $input_file"
        }
    }
    source $init_file
    foreach command {psu_init psu_ps_pl_isolation_removal psu_ps_pl_reset_config} {
        if {[llength [info commands $command]] != 1} {
            error "Initialization script does not define $command: $init_file"
        }
    }

    connect
    set connected 1
    set k26_targets [jtag targets -target-properties -filter {name =~ "*xck26*"}]
    set psu_targets [targets -target-properties -filter {name == "PSU"}]
    if {[llength $k26_targets] != 1 || [llength $psu_targets] != 1} {
        error "Expected exactly one K26 and one PSU; found [llength $k26_targets] K26 and [llength $psu_targets] PSU targets. Check power, J4 data cable, drivers, and disconnect other boards."
    }
    targets -set -filter {name == "PSU"}
    puts "BRINGUP:BITSTREAM=$bit_file"
    puts "BRINGUP:PSU_INIT=$init_file"
    fpga -file $bit_file
    if {[fpga -state] ne "FPGA is configured"} {
        error "FPGA is not configured immediately after programming."
    }
    puts "BRINGUP:CONFIGURED=1"

    # AMD UG1725 sequence. Configuring PL alone does not apply PS presets.
    psu_init
    after 1000
    psu_ps_pl_isolation_removal
    after 1000
    psu_ps_pl_reset_config
    puts "BRINGUP:PS_INITIALIZED"

    # Read back the PL0 clock enable; this does not measure physical frequency.
    set pl0_ctrl [mrd -value 0xFF5E00C0]
    if {($pl0_ctrl & 0x01000000) == 0} {
        error "PL0_REF_CTRL.CLKACT is clear after PS initialization: $pl0_ctrl"
    }
    puts "BRINGUP:PL0_REF_CTRL=[format 0x%08X $pl0_ctrl]"
    after 5000
    if {[fpga -state] ne "FPGA is configured"} {
        error "FPGA configuration was lost after PS initialization."
    }
    puts "BRINGUP:CONFIGURED_AFTER_5S=1"
    puts "BRINGUP:SUCCESS"
} message]} {
    if {$connected} { catch {disconnect} }
    puts stderr "BRINGUP:ERROR=$message"
    exit 1
}
disconnect
