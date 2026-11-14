#![no_std]

extern crate alloc;

use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

pub mod generated;
pub use generated::*;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Truncated,
    BadMagic,
    UnsupportedVersion,
    BadLength,
    ReservedNonzero,
    BadHeaderCrc,
    BadPayloadCrc,
    BadPadding,
    TooLarge,
    CountMismatch,
    SensorMaskMismatch,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}
impl core::error::Error for Error {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CommonRecord<'a> {
    pub sensor_id: u8,
    pub sequence: u32,
    pub measurement_ticks: u64,
    pub arrival_ticks: u64,
    pub status_flags: u32,
    pub payload: &'a [u8],
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnedRecord {
    pub sensor_id: u8,
    pub sequence: u32,
    pub measurement_ticks: u64,
    pub arrival_ticks: u64,
    pub status_flags: u32,
    pub payload: Vec<u8>,
}

impl OwnedRecord {
    #[must_use]
    pub fn as_borrowed(&self) -> CommonRecord<'_> {
        CommonRecord {
            sensor_id: self.sensor_id,
            sequence: self.sequence,
            measurement_ticks: self.measurement_ticks,
            arrival_ticks: self.arrival_ticks,
            status_flags: self.status_flags,
            payload: &self.payload,
        }
    }
}

fn u16_at(data: &[u8], offset: usize) -> Result<u16, Error> {
    let bytes: [u8; 2] = data
        .get(offset..offset + 2)
        .ok_or(Error::Truncated)?
        .try_into()
        .map_err(|_| Error::Truncated)?;
    Ok(u16::from_le_bytes(bytes))
}

fn u32_at(data: &[u8], offset: usize) -> Result<u32, Error> {
    let bytes: [u8; 4] = data
        .get(offset..offset + 4)
        .ok_or(Error::Truncated)?
        .try_into()
        .map_err(|_| Error::Truncated)?;
    Ok(u32::from_le_bytes(bytes))
}

fn u64_at(data: &[u8], offset: usize) -> Result<u64, Error> {
    let bytes: [u8; 8] = data
        .get(offset..offset + 8)
        .ok_or(Error::Truncated)?
        .try_into()
        .map_err(|_| Error::Truncated)?;
    Ok(u64::from_le_bytes(bytes))
}

fn put_u16(data: &mut [u8], offset: usize, value: u16) {
    data[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_u32(data: &mut [u8], offset: usize, value: u32) {
    data[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_u64(data: &mut [u8], offset: usize, value: u64) {
    data[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

#[must_use]
pub fn crc32c(data: &[u8]) -> u32 {
    let mut value = 0xFFFF_FFFF_u32;
    for byte in data {
        value ^= u32::from(*byte);
        for _ in 0..8 {
            value = (value >> 1) ^ if value & 1 == 1 { 0x82F6_3B78 } else { 0 };
        }
    }
    value ^ 0xFFFF_FFFF
}

fn aligned(value: usize) -> usize {
    (value + RECORD_ALIGNMENT - 1) & !(RECORD_ALIGNMENT - 1)
}

pub fn encode_record(record: &CommonRecord<'_>) -> Result<Vec<u8>, Error> {
    if record.payload.len() > MAX_PAYLOAD_BYTES || record.status_flags & !0x0007_FFFF != 0 {
        return Err(Error::TooLarge);
    }
    let total = aligned(RECORD_HEADER_BYTES + record.payload.len());
    let mut output = vec![0_u8; total];
    put_u32(&mut output, 0, RECORD_MAGIC as u32);
    output[4] = RECORD_VERSION as u8;
    output[5] = record.sensor_id;
    put_u16(&mut output, 6, RECORD_HEADER_BYTES as u16);
    put_u16(&mut output, 8, total as u16);
    put_u16(&mut output, 10, record.payload.len() as u16);
    put_u32(&mut output, 12, record.sequence);
    put_u64(&mut output, 16, record.measurement_ticks);
    put_u64(&mut output, 24, record.arrival_ticks);
    put_u32(&mut output, 32, record.status_flags);
    put_u32(&mut output, 40, crc32c(record.payload));
    output[RECORD_HEADER_BYTES..RECORD_HEADER_BYTES + record.payload.len()]
        .copy_from_slice(record.payload);
    let header_crc = crc32c(&output[..RECORD_HEADER_BYTES]);
    put_u32(&mut output, 44, header_crc);
    Ok(output)
}

pub fn decode_record(data: &[u8]) -> Result<(CommonRecord<'_>, usize), Error> {
    if data.len() < RECORD_HEADER_BYTES {
        return Err(Error::Truncated);
    }
    if u32_at(data, 0)? != RECORD_MAGIC as u32 {
        return Err(Error::BadMagic);
    }
    if data[4] != RECORD_VERSION as u8 {
        return Err(Error::UnsupportedVersion);
    }
    let header = usize::from(u16_at(data, 6)?);
    let total = usize::from(u16_at(data, 8)?);
    let payload_len = usize::from(u16_at(data, 10)?);
    if header != RECORD_HEADER_BYTES
        || total < header
        || total % RECORD_ALIGNMENT != 0
        || total > data.len()
        || payload_len > MAX_PAYLOAD_BYTES
        || header + payload_len > total
    {
        return Err(Error::BadLength);
    }
    if u32_at(data, 36)? != 0 || u32_at(data, 32)? & !0x0007_FFFF != 0 {
        return Err(Error::ReservedNonzero);
    }
    let mut header_copy = [0_u8; RECORD_HEADER_BYTES];
    header_copy.copy_from_slice(&data[..RECORD_HEADER_BYTES]);
    header_copy[44..48].fill(0);
    if crc32c(&header_copy) != u32_at(data, 44)? {
        return Err(Error::BadHeaderCrc);
    }
    let payload = &data[header..header + payload_len];
    if crc32c(payload) != u32_at(data, 40)? {
        return Err(Error::BadPayloadCrc);
    }
    if data[header + payload_len..total]
        .iter()
        .any(|value| *value != 0)
    {
        return Err(Error::BadPadding);
    }
    Ok((
        CommonRecord {
            sensor_id: data[5],
            sequence: u32_at(data, 12)?,
            measurement_ticks: u64_at(data, 16)?,
            arrival_ticks: u64_at(data, 24)?,
            status_flags: u32_at(data, 32)?,
            payload,
        },
        total,
    ))
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DmaBlock<'a> {
    pub sequence: u32,
    pub flags: u32,
    pub valid_bytes: u16,
    pub packet_count: u16,
    pub sensor_mask: u32,
    pub pl_drops: u32,
    pub reset_generation: u32,
    pub session_id: u32,
    pub first_measurement_ticks: u64,
    pub last_measurement_ticks: u64,
    pub records: Vec<CommonRecord<'a>>,
}

pub fn decode_dma_block(data: &[u8]) -> Result<DmaBlock<'_>, Error> {
    if data.len() != DMA_BLOCK_BYTES {
        return Err(Error::BadLength);
    }
    if u32_at(data, 0)? != BLOCK_MAGIC as u32 {
        return Err(Error::BadMagic);
    }
    if u16_at(data, 4)? != ABI_MAJOR {
        return Err(Error::UnsupportedVersion);
    }
    let header = usize::from(u16_at(data, 6)?);
    let valid = usize::from(u16_at(data, 16)?);
    if header != DMA_HEADER_BYTES
        || valid < header
        || valid > DMA_BLOCK_BYTES
        || u32_at(data, 36)? != 0
    {
        return Err(Error::BadLength);
    }
    let mut header_copy = [0_u8; DMA_HEADER_BYTES];
    header_copy.copy_from_slice(&data[..DMA_HEADER_BYTES]);
    header_copy[60..64].fill(0);
    if crc32c(&header_copy) != u32_at(data, 60)? {
        return Err(Error::BadHeaderCrc);
    }
    if crc32c(&data[header..valid]) != u32_at(data, 56)? {
        return Err(Error::BadPayloadCrc);
    }
    if data[valid..].iter().any(|value| *value != 0) {
        return Err(Error::BadPadding);
    }
    let mut records = Vec::new();
    let mut cursor = header;
    let mut mask = 0_u32;
    while cursor < valid {
        let (record, consumed) = decode_record(&data[cursor..valid])?;
        if record.sensor_id < 32 {
            mask |= 1_u32 << record.sensor_id;
        }
        records.push(record);
        cursor += consumed;
    }
    if records.len() != usize::from(u16_at(data, 18)?) {
        return Err(Error::CountMismatch);
    }
    if mask != u32_at(data, 20)? {
        return Err(Error::SensorMaskMismatch);
    }
    Ok(DmaBlock {
        sequence: u32_at(data, 8)?,
        flags: u32_at(data, 12)?,
        valid_bytes: valid as u16,
        packet_count: u16_at(data, 18)?,
        sensor_mask: mask,
        pl_drops: u32_at(data, 24)?,
        reset_generation: u32_at(data, 28)?,
        session_id: u32_at(data, 32)?,
        first_measurement_ticks: u64_at(data, 40)?,
        last_measurement_ticks: u64_at(data, 48)?,
        records,
    })
}

pub fn encode_replay_item(record: &CommonRecord<'_>, delay_ticks: u64) -> Result<Vec<u8>, Error> {
    let encoded = encode_record(record)?;
    let mut output = vec![0_u8; INJECTION_HEADER_BYTES + encoded.len()];
    put_u32(&mut output, 0, INJECTION_MAGIC as u32);
    put_u16(&mut output, 4, RECORD_VERSION as u16);
    put_u16(&mut output, 6, INJECTION_HEADER_BYTES as u16);
    put_u64(&mut output, 8, delay_ticks);
    output[INJECTION_HEADER_BYTES..].copy_from_slice(&encoded);
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture<'a>(payload: &'a [u8]) -> CommonRecord<'a> {
        CommonRecord {
            sensor_id: SENSOR_BNO085_SHTP,
            sequence: 7,
            measurement_ticks: 100,
            arrival_ticks: 130,
            status_flags: STATUS_VALID,
            payload,
        }
    }

    #[test]
    fn standard_crc_vector() {
        assert_eq!(crc32c(b"123456789"), 0xE306_9283);
    }

    #[test]
    fn record_round_trip() {
        let input = fixture(&[1, 2, 3]);
        let encoded = encode_record(&input).unwrap();
        let (decoded, consumed) = decode_record(&encoded).unwrap();
        assert_eq!(decoded, input);
        assert_eq!(consumed, encoded.len());
    }

    #[test]
    fn malformed_inputs_never_panic() {
        for length in 0..80 {
            assert!(decode_record(&vec![0xA5; length]).is_err());
        }
    }

    #[test]
    fn canonical_golden_fixtures_decode() {
        let record = include_bytes!("../../../../spec/golden/common_record_v1.bin");
        let (decoded, consumed) = decode_record(record).unwrap();
        assert_eq!(decoded.sensor_id, SENSOR_BNO085_SHTP);
        assert_eq!(consumed, record.len());
        let block = include_bytes!("../../../../spec/golden/dma_block_v1.bin");
        assert_eq!(decode_dma_block(block).unwrap().records.len(), 1);
    }
}
