# Load the PL while Linux is running without reinitializing DDR or PS clocks.
# Usage: xsdb tools/xsdb_linux_pl_load.tcl <design.bit> <matching_psu_init.tcl>

if {$argc != 2} {
    puts stderr "Usage: xsdb tools/xsdb_linux_pl_load.tcl <design.bit> <matching_psu_init.tcl>"
    exit 1
}

set bit_file [file normalize [lindex $argv 0]]
set init_file [file normalize [lindex $argv 1]]
set connected 0

if {[catch {
    foreach input_file [list $bit_file $init_file] {
        if {![file isfile $input_file] || ![file readable $input_file]} {
            error "Missing or unreadable input: $input_file"
        }
    }

    # The generated handoff supplies mask_write and the two PS-to-PL helper
    # procedures. Do not call psu_init here because Linux already owns DDR,
    # PLLs, peripherals, and the rest of the processing system.
    source $init_file
    foreach command {mask_write init_ps psu_ps_pl_isolation_removal psu_ps_pl_reset_config} {
        if {[llength [info commands $command]] != 1} {
            error "Initialization script does not define $command: $init_file"
        }
    }
    if {![info exists psu_afi_config]} {
        error "Initialization script does not define psu_afi_config: $init_file"
    }

    connect
    set connected 1
    set k26_targets [jtag targets -target-properties -filter {name =~ "*xck26*"}]
    set psu_targets [targets -target-properties -filter {name == "PSU"}]
    if {[llength $k26_targets] != 1 || [llength $psu_targets] != 1} {
        error "Expected exactly one K26 and one PSU; found [llength $k26_targets] K26 and [llength $psu_targets] PSU targets."
    }

    targets -set -filter {name == "PSU"}
    puts "LINUX_PL_LOAD:BITSTREAM=$bit_file"
    puts "LINUX_PL_LOAD:PSU_HANDOFF=$init_file"
    fpga -file $bit_file
    if {[fpga -state] ne "FPGA is configured"} {
        error "FPGA is not configured immediately after programming."
    }

    # Enable only PL0_REF_CLK using the value generated for this XSA. This is
    # the clock connected to the AXI SmartConnect and register bank.
    mask_write 0xFF5E00C0 0x013F3F07 0x01010A00
    set pl0_ctrl [mrd -value 0xFF5E00C0]
    if {($pl0_ctrl & 0x01000000) == 0} {
        error "PL0_REF_CTRL.CLKACT is clear: $pl0_ctrl"
    }
    puts "LINUX_PL_LOAD:PL0_REF_CTRL=[format 0x%08X $pl0_ctrl]"

    # Release the generated AXI fabric-interface resets. Without this step the
    # PL clock can run and the bitstream can remain configured while every HPM
    # transaction still times out.
    init_ps [subst {$psu_afi_config}]

    # Power the PL, remove PS-to-PL isolation, and issue the generated fabric
    # reset pulse. These operations do not reconfigure DDR or Linux clocks.
    psu_ps_pl_isolation_removal
    psu_ps_pl_reset_config
    after 1000

    if {[fpga -state] ne "FPGA is configured"} {
        error "FPGA configuration was lost after PS-to-PL activation."
    }
    puts "LINUX_PL_LOAD:CONFIGURED=1"
    puts "LINUX_PL_LOAD:SUCCESS"
} message]} {
    if {$connected} {
        catch {disconnect}
    }
    puts stderr "LINUX_PL_LOAD:ERROR=$message"
    exit 1
}

disconnect
