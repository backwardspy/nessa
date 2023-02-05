#![warn(
    clippy::pedantic,
    clippy::nursery,
    clippy::cargo,
    clippy::unwrap_used,
    clippy::expect_used
)]
use nessa6502::{addressing, instruction::INSTRUCTIONS};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("unexpected end of stream")]
    EndOfStream,
}

/// Disassemble a single instruction from the given byte iterator.
///
/// # Errors
///
/// Returns an error if the iterator runs out of bytes too early.
pub fn disassemble_instruction<ByteIter>(mut bytes: ByteIter) -> Result<String, Error>
where
    ByteIter: Iterator<Item = u8>,
{
    let opcode = bytes.next().ok_or(Error::EndOfStream)?;
    let instruction = &INSTRUCTIONS[opcode as usize];

    let next = bytes
        .take(instruction.size as usize - 1)
        .collect::<Vec<_>>();

    let mut bytes_str = String::new();
    bytes_str.push_str(&format!("{opcode:02X} "));
    for byte in &next {
        bytes_str.push_str(&format!("{byte:02X} "));
    }

    let mut result = String::new();
    result.push_str(&format!("{bytes_str: <9}"));
    result.push_str(&format!("{:?}", instruction.name));

    let address = match instruction.size {
        1 => 0,
        2 => u16::from(next[0]),
        3 => u16::from_le_bytes([next[0], next[1]]),
        _ => unreachable!(),
    };

    let address = match instruction.mode {
        addressing::Mode::Immediate => format!("#${address:<2X} ({address})"),
        addressing::Mode::ZeroPage => format!("${address:02X}"),
        addressing::Mode::ZeroPageX => format!("${address:02X},X"),
        addressing::Mode::ZeroPageY => format!("${address:02X},Y"),
        addressing::Mode::AbsoluteX => format!("${address:04X},X"),
        addressing::Mode::AbsoluteY => format!("${address:04X},Y"),
        addressing::Mode::Indirect => format!("(${address:04X})"),
        addressing::Mode::IndirectX => format!("(${address:02X},X)"),
        addressing::Mode::IndirectY => format!("(${address:02X}),Y"),
        addressing::Mode::Implied => String::new(),
        addressing::Mode::Accumulator => "A".to_string(),
        addressing::Mode::Absolute | addressing::Mode::Relative => format!("${address:04X}"),
    };

    if !address.is_empty() {
        result.push_str(&format!(" {address}"));
    }

    Ok(result)
}
