use std::env;
use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use se_abi::*;
use se_device::{Device, RegisterIo, UioRegisters};

fn usage() {
    eprintln!("usage: se-hwtest [--uio /dev/uioN] [--device /dev/state-estimator] <identify|registers|timebase|interrupts|dma|replay|overflow|reset|lock|report> [arguments]");
}

fn read64(registers: &impl RegisterIo, low: usize, high: usize) -> Result<u64, Box<dyn Error>> {
    let lo = registers.read32(low)?;
    let hi = registers.read32(high)?;
    Ok((u64::from(hi) << 32) | u64::from(lo))
}

#[cfg(target_os = "linux")]
fn dma_report(path: &str) -> Result<se_device::dma::RingInfo, Box<dyn Error>> {
    let dma = se_device::dma::DmaDevice::open(path)?;
    let info = dma.ring_info()?;
    if info.block_bytes as usize != DMA_BLOCK_BYTES || info.block_count as usize != DMA_BLOCK_COUNT
    {
        return Err("kernel DMA ring geometry does not match ABI v1".into());
    }
    Ok(info)
}

#[cfg(not(target_os = "linux"))]
fn dma_report(_path: &str) -> Result<(), Box<dyn Error>> {
    Err("DMA device tests are available only on Linux".into())
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args: Vec<String> = env::args().skip(1).collect();
    let mut uio = "/dev/uio0".to_owned();
    let mut dma_path = "/dev/state-estimator".to_owned();
    while args
        .first()
        .is_some_and(|argument| argument.starts_with("--"))
    {
        if args.len() < 2 {
            usage();
            return Err("missing option value".into());
        }
        match args[0].as_str() {
            "--uio" => uio = args[1].clone(),
            "--device" => dma_path = args[1].clone(),
            _ => {
                usage();
                return Err("unknown option".into());
            }
        }
        args.drain(0..2);
    }
    let Some(command) = args.first().map(String::as_str) else {
        usage();
        return Err("missing command".into());
    };
    let registers = UioRegisters::open(&uio, CONTROL_SIZE)?;
    let mut device = Device::new(registers);
    let identity = device.identify()?;
    match command {
        "identify" => println!(
            "{{\"passed\":true,\"abi_major\":{},\"abi_minor\":{},\"abi_hash\":\"{:08x}{:08x}\"}}",
            identity.abi_major, identity.abi_minor, identity.hash_lo, identity.hash_hi
        ),
        "registers" => {
            for (name, offset) in [
                ("CORE_MAGIC", REG_CORE_MAGIC),
                ("ABI_VERSION", REG_ABI_VERSION),
                ("CORE_VERSION", REG_CORE_VERSION),
                ("CAPABILITIES", REG_CAPABILITIES),
                ("STATUS", REG_STATUS),
                ("ERROR_STATUS", REG_ERROR_STATUS),
                ("IRQ_STATUS", REG_IRQ_STATUS),
                ("RESET_GENERATION", REG_RESET_GENERATION),
            ] {
                println!("{name:20} 0x{:08x}", device.registers().read32(offset)?);
            }
        }
        "timebase" => {
            let start_ticks = read64(device.registers(), REG_TIME_NOW_LO, REG_TIME_NOW_HI)?;
            let start = Instant::now();
            thread::sleep(Duration::from_secs(10));
            let elapsed = start.elapsed();
            let end_ticks = read64(device.registers(), REG_TIME_NOW_LO, REG_TIME_NOW_HI)?;
            let measured = (end_ticks.wrapping_sub(start_ticks) as f64) / elapsed.as_secs_f64();
            let ppm = (measured - CLOCK_HZ as f64).abs() * 1_000_000.0 / CLOCK_HZ as f64;
            println!(
                "{{\"passed\":{},\"measured_hz\":{measured:.3},\"error_ppm\":{ppm:.3}}}",
                ppm <= 100.0
            );
            if ppm > 100.0 {
                return Err("timebase is outside 100 ppm gate".into());
            }
        }
        "interrupts" => {
            device
                .registers_mut()
                .write32(REG_IRQ_FORCE, IRQ_GLOBAL_HEALTH)?;
            let observed = device.registers().read32(REG_IRQ_STATUS)? & IRQ_GLOBAL_HEALTH != 0;
            device
                .registers_mut()
                .write32(REG_IRQ_STATUS, IRQ_GLOBAL_HEALTH)?;
            println!("{{\"passed\":{observed},\"source\":\"global_health\"}}");
            if !observed {
                return Err("forced interrupt was not observed".into());
            }
        }
        "reset" => {
            let before_time = read64(device.registers(), REG_TIME_NOW_LO, REG_TIME_NOW_HI)?;
            let before_generation = device.registers().read32(REG_RESET_GENERATION)?;
            device
                .registers_mut()
                .write32(REG_RESET_KEY, RESET_KEY as u32)?;
            device.registers_mut().write32(REG_RESET_CONTROL, 1)?;
            let after_generation = device.registers().read32(REG_RESET_GENERATION)?;
            let after_time = read64(device.registers(), REG_TIME_NOW_LO, REG_TIME_NOW_HI)?;
            let passed =
                after_generation == before_generation.wrapping_add(1) && after_time >= before_time;
            println!("{{\"passed\":{passed},\"generation_before\":{before_generation},\"generation_after\":{after_generation}}}");
            if !passed {
                return Err("soft-reset invariant failed".into());
            }
        }
        "lock" => {
            let before = device.registers().read32(REG_CONTROL)?;
            device
                .registers_mut()
                .write32(REG_CONTROL, before | CONTROL_CONFIGURATION_LOCK)?;
            let after = device.registers().read32(REG_CONTROL)?;
            let passed = after & CONTROL_CONFIGURATION_LOCK != 0;
            println!(
                "{{\"passed\":{passed},\"control_before\":{before},\"control_after\":{after}}}"
            );
            if !passed {
                return Err("configuration lock did not latch".into());
            }
        }
        "dma" => {
            #[cfg(target_os = "linux")]
            {
                let info = dma_report(&dma_path)?;
                let available = info.producer_block.wrapping_sub(info.consumer_block);
                println!(
                    "{{\"passed\":true,\"block_bytes\":{},\"block_count\":{},\"producer_block\":{},\"consumer_block\":{},\"available_blocks\":{},\"host_overruns\":{},\"reset_generation\":{}}}",
                    info.block_bytes,
                    info.block_count,
                    info.producer_block,
                    info.consumer_block,
                    available,
                    info.host_overruns,
                    info.reset_generation
                );
            }
            #[cfg(not(target_os = "linux"))]
            dma_report(&dma_path)?;
        }
        "report" => {
            let status = device.registers().read32(REG_STATUS)?;
            let errors = device.registers().read32(REG_ERROR_STATUS)?;
            let reset_generation = device.registers().read32(REG_RESET_GENERATION)?;
            let time_ticks = read64(device.registers(), REG_TIME_NOW_LO, REG_TIME_NOW_HI)?;
            println!(
                "{{\"passed\":{},\"abi_major\":{},\"abi_minor\":{},\"abi_hash\":\"{:08x}{:08x}\",\"status\":{},\"errors\":{},\"reset_generation\":{},\"time_ticks\":{}}}",
                errors == 0,
                identity.abi_major,
                identity.abi_minor,
                identity.hash_lo,
                identity.hash_hi,
                status,
                errors,
                reset_generation,
                time_ticks
            );
            if errors != 0 {
                return Err("hardware report contains sticky errors".into());
            }
        }
        "replay" | "overflow" => {
            return Err(
                "command requires the replay or fault-injection RTL, which is not connected yet"
                    .into(),
            )
        }
        _ => {
            usage();
            return Err("unknown command".into());
        }
    }
    Ok(())
}
