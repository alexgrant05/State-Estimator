use std::env;
use std::fs;

use se_abi::{decode_record, encode_replay_item};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 4 {
        return Err("usage: se-replay <common_records.bin> <delay_ticks> <injection.bin>".into());
    }
    let input = fs::read(&args[1])?;
    let delay: u64 = args[2].parse()?;
    let mut cursor = 0;
    let mut output = Vec::new();
    let mut count = 0;
    while cursor < input.len() {
        let (record, consumed) = decode_record(&input[cursor..])?;
        output.extend_from_slice(&encode_replay_item(
            &record,
            if count == 0 { 0 } else { delay },
        )?);
        cursor += consumed;
        count += 1;
    }
    fs::write(&args[3], output)?;
    println!("encoded {count} replay items");
    Ok(())
}
