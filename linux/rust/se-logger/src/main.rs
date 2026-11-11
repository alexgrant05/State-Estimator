use std::env;
use std::fs::{self, File};
use std::io::{Read, Write};
use std::path::PathBuf;

use se_abi::{decode_dma_block, ABI_HASH_HEX, DMA_BLOCK_BYTES};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err("usage: se-logger <DMA block input> <output directory>".into());
    }
    let output = PathBuf::from(&args[2]);
    fs::create_dir_all(&output)?;
    let mut input = File::open(&args[1])?;
    let mut raw = File::create(output.join("dma_blocks.bin"))?;
    let mut events = File::create(output.join("events.ndjson"))?;
    let mut block_bytes = [0_u8; DMA_BLOCK_BYTES];
    let mut blocks = 0_u64;
    let mut records = 0_u64;
    loop {
        let mut read = 0;
        while read < block_bytes.len() {
            let count = input.read(&mut block_bytes[read..])?;
            if count == 0 {
                break;
            }
            read += count;
        }
        if read == 0 {
            break;
        }
        if read != DMA_BLOCK_BYTES {
            return Err("truncated DMA block".into());
        }
        let block = decode_dma_block(&block_bytes)?;
        raw.write_all(&block_bytes)?;
        for record in block.records {
            writeln!(events, "{{\"sensor_id\":{},\"sequence_number\":{},\"measurement_ticks\":{},\"arrival_ticks\":{},\"status_flags\":{},\"payload_hex\":\"{}\"}}", record.sensor_id, record.sequence, record.measurement_ticks, record.arrival_ticks, record.status_flags, hex(record.payload))?;
            records += 1;
        }
        blocks += 1;
    }
    let mut manifest = File::create(output.join("manifest.json"))?;
    writeln!(
        manifest,
        "{{\"abi_hash\":\"{ABI_HASH_HEX}\",\"blocks\":{blocks},\"records\":{records}}}"
    )?;
    println!("validated and logged {blocks} blocks containing {records} records");
    Ok(())
}

fn hex(data: &[u8]) -> String {
    const TABLE: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(data.len() * 2);
    for byte in data {
        output.push(TABLE[(byte >> 4) as usize] as char);
        output.push(TABLE[(byte & 0xF) as usize] as char);
    }
    output
}
