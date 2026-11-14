# Read-only and scratch-register smoke test for the FPGA-to-Linux ABI.
# Run after configuring the matching bitstream and PS handoff.

set connected 0
if {[catch {
    connect
    set connected 1

    set psu_targets [targets -target-properties -filter {name == "PSU"}]
    if {[llength $psu_targets] != 1} {
        error "Expected exactly one PSU target; found [llength $psu_targets]."
    }
    targets -set -filter {name == "PSU"}

    set base 0xA0000000
    # The standalone debugger does not import the XSA address map. Force is
    # required for PL AXI ranges even when the physical access is valid.
    set magic [mrd -force -value [expr {$base + 0x0000}]]
    set abi_version [mrd -force -value [expr {$base + 0x0004}]]
    set clock_hz [mrd -force -value [expr {$base + 0x0010}]]

    if {$magic != 0x54534553} {
        error [format "CORE_MAGIC mismatch: got 0x%08X" $magic]
    }
    if {$abi_version != 0x00010000} {
        error [format "ABI_VERSION mismatch: got 0x%08X" $abi_version]
    }
    if {$clock_hz != 100000000} {
        error "CLOCK_HZ mismatch: got $clock_hz"
    }

    set scratch_address [expr {$base + 0x005C}]
    foreach pattern {0xA5A55A5A 0x5A5AA5A5} {
        mwr -force $scratch_address $pattern
        set observed [mrd -force -value $scratch_address]
        if {$observed != $pattern} {
            error [format "SCRATCH mismatch: wrote 0x%08X, read 0x%08X" $pattern $observed]
        }
    }

    # Reading TIME_NOW_LO latches TIME_NOW_HI until it is read.
    set time_lo_address [expr {$base + 0x0060}]
    set time_hi_address [expr {$base + 0x0064}]
    set time0_lo [mrd -force -value $time_lo_address]
    set time0_hi [mrd -force -value $time_hi_address]
    after 100
    set time1_lo [mrd -force -value $time_lo_address]
    set time1_hi [mrd -force -value $time_hi_address]
    set time0 [expr {($time0_hi << 32) | $time0_lo}]
    set time1 [expr {($time1_hi << 32) | $time1_lo}]
    if {$time1 <= $time0} {
        error "TIME_NOW is not monotonic: $time0 then $time1"
    }

    puts [format "ABI_SMOKE:CORE_MAGIC=0x%08X" $magic]
    puts [format "ABI_SMOKE:ABI_VERSION=0x%08X" $abi_version]
    puts "ABI_SMOKE:CLOCK_HZ=$clock_hz"
    puts [format "ABI_SMOKE:TIME_DELTA_TICKS=%u" [expr {$time1 - $time0}]]
    puts "ABI_SMOKE:SCRATCH=PASS"
    puts "ABI_SMOKE:SUCCESS"
} message]} {
    if {$connected} {
        catch {disconnect}
    }
    puts stderr "ABI_SMOKE:ERROR=$message"
    exit 1
}

disconnect
