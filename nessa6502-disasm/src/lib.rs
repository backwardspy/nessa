#![warn(
    clippy::pedantic,
    clippy::nursery,
    clippy::cargo,
    clippy::unwrap_used,
    clippy::expect_used
)]
use nessa6502::{
    addressing,
    cpu::CPU,
    instruction::{self, INSTRUCTIONS},
};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("unexpected end of stream")]
    EndOfStream,
}

struct BusIter<'a> {
    cpu: &'a CPU,
    pc: u16,
}

impl Iterator for BusIter<'_> {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        let value = self.cpu.bus.read(self.pc);
        self.pc += 1;
        Some(value)
    }
}

/// Disassemble a single instruction from the given byte iterator.
///
/// # Errors
///
/// Returns an error if the iterator runs out of bytes too early.
pub fn disassemble_instruction(cpu: &CPU) -> Result<String, Error> {
    let mut next_bytes = BusIter {
        cpu,
        pc: cpu.reg.pc,
    };
    let opcode = next_bytes.next().ok_or(Error::EndOfStream)?;
    let instruction = &INSTRUCTIONS[opcode as usize];
    let next = next_bytes
        .take(instruction.size as usize - 1)
        .collect::<Vec<_>>();

    let mut bytes_str = String::new();
    bytes_str.push_str(&format!("{opcode:02X} "));
    for byte in &next {
        bytes_str.push_str(&format!("{byte:02X} "));
    }

    let mut result = String::new();
    result.push_str(&format!("{bytes_str: <9} "));
    result.push_str(&format!("{:?}", instruction.name));

    let address = match instruction.size {
        1 => 0,
        2 => u16::from(next[0]),
        3 => u16::from_le_bytes([next[0], next[1]]),
        _ => unreachable!(),
    };

    let value = cpu.bus.read(address);

    let show_value = !matches!(
        instruction.name,
        instruction::Name::JMP | instruction::Name::JSR
    );

    let address = match instruction.mode {
        addressing::Mode::Immediate => format!("#${address:02X}"),
        addressing::Mode::ZeroPage => {
            if show_value {
                format!("${address:02X} = {value:02X}")
            } else {
                format!("${address:02X}")
            }
        }
        addressing::Mode::ZeroPageX => format!("${address:02X},X"),
        addressing::Mode::ZeroPageY => format!("${address:02X},Y"),
        addressing::Mode::Absolute => {
            if show_value {
                format!("${address:04X} = {value:02X}")
            } else {
                format!("${address:04X}")
            }
        }
        addressing::Mode::AbsoluteX => format!("${address:04X},X"),
        addressing::Mode::AbsoluteY => format!("${address:04X},Y"),
        addressing::Mode::Indirect => format!("(${address:04X})"),
        addressing::Mode::IndirectX => format!("(${address:02X},X)"),
        addressing::Mode::IndirectY => format!("(${address:02X}),Y"),
        addressing::Mode::Implied => String::new(),
        addressing::Mode::Accumulator => "A".to_string(),
        addressing::Mode::Relative => {
            format!(
                "${:04X}",
                cpu.reg.pc + u16::from(instruction.size) + address
            )
        }
    };

    if !address.is_empty() {
        result.push_str(&format!(" {address}"));
    }

    Ok(result)
}
