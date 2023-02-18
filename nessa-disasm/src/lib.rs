#![warn(
    clippy::pedantic,
    clippy::nursery,
    clippy::cargo,
    clippy::unwrap_used,
    clippy::expect_used
)]
use nessa_cpu::{
    addressing,
    instruction::{self, Instruction, INSTRUCTIONS},
    MemoryAccess, CPU,
};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("unexpected end of stream")]
    EndOfStream,
}

struct MemIter<'a, M>
where
    M: MemoryAccess,
{
    cpu: &'a CPU<M>,
    pc: u16,
}

impl<'a, M> MemIter<'a, M>
where
    M: MemoryAccess,
{
    const fn new(cpu: &'a CPU<M>) -> Self {
        Self {
            cpu,
            pc: cpu.reg.pc,
        }
    }
}

impl<M> Iterator for MemIter<'_, M>
where
    M: MemoryAccess,
{
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        let value = self.cpu.read8_ro(self.pc);
        self.pc += 1;
        Some(value)
    }
}

fn offset_8<M>(cpu: &CPU<M>, addr: u16, offset: u8) -> (u8, u8)
where
    M: MemoryAccess,
{
    let offset_addr = addr.to_le_bytes()[0].wrapping_add(offset);
    let value = cpu.read8_ro(u16::from(offset_addr));
    (offset_addr, value)
}

fn offset_16<M>(cpu: &CPU<M>, addr: u16, offset: u8) -> (u16, u8)
where
    M: MemoryAccess,
{
    let offset_addr = addr.wrapping_add(u16::from(offset));
    let value = cpu.read8_ro(offset_addr);
    (offset_addr, value)
}

fn format_address<M>(cpu: &CPU<M>, instruction: &Instruction, address: u16) -> String
where
    M: MemoryAccess,
{
    let show_value = !matches!(
        instruction.name,
        instruction::Name::JMP | instruction::Name::JSR
    );
    let value = cpu.read8_ro(address);

    match instruction.mode {
        addressing::Mode::Immediate => format!("#${address:02X}"),
        addressing::Mode::ZeroPage => {
            if show_value {
                format!("${address:02X} = {value:02X}")
            } else {
                format!("${address:02X}")
            }
        }
        addressing::Mode::ZeroPageX => {
            let (offset_address, value) = offset_8(cpu, address, cpu.reg.x);
            format!("${address:02X},X @ {offset_address:02X} = {value:02X}")
        }
        addressing::Mode::ZeroPageY => {
            let (offset_address, value) = offset_8(cpu, address, cpu.reg.y);
            format!("${address:02X},Y @ {offset_address:02X} = {value:02X}")
        }
        addressing::Mode::Absolute => {
            if show_value {
                format!("${address:04X} = {value:02X}")
            } else {
                format!("${address:04X}")
            }
        }
        addressing::Mode::AbsoluteX => {
            let (offset_address, value) = offset_16(cpu, address, cpu.reg.x);
            format!("${address:04X},X @ {offset_address:04X} = {value:02X}")
        }
        addressing::Mode::AbsoluteY => {
            let (offset_address, value) = offset_16(cpu, address, cpu.reg.y);
            format!("${address:04X},Y @ {offset_address:04X} = {value:02X}",)
        }
        addressing::Mode::Indirect => {
            let value16 = cpu.read16_wrap_ro(address);
            format!("(${address:04X}) = {value16:04X}")
        }
        addressing::Mode::IndirectX => {
            if show_value {
                let offset_address = address.to_le_bytes()[0].wrapping_add(cpu.reg.x);
                let resolved = cpu.read16_zp_ro(offset_address);
                let value = cpu.read8_ro(resolved);
                format!("(${address:02X},X) @ {offset_address:02X} = {resolved:04X} = {value:02X}",)
            } else {
                format!("(${address:02X},X)")
            }
        }
        addressing::Mode::IndirectY => {
            if show_value {
                let address = address.to_le_bytes()[0];
                let resolved = cpu.read16_zp_ro(address).wrapping_add(u16::from(cpu.reg.y));
                let value = cpu.read8_ro(resolved);
                format!(
                    "(${address:02X}),Y = {:04X} @ {resolved:04X} = {value:02X}",
                    cpu.read16_zp_ro(address)
                )
            } else {
                format!("(${address:02X}),Y")
            }
        }
        addressing::Mode::Implied => String::new(),
        addressing::Mode::Accumulator => "A".to_string(),
        addressing::Mode::Relative => {
            #[allow(clippy::cast_possible_wrap)]
            let offset = address.to_le_bytes()[0] as i8;
            #[allow(clippy::cast_sign_loss)]
            let address = cpu
                .reg
                .pc
                .wrapping_add(u16::from(instruction.size))
                .wrapping_add(offset as u16);
            format!("${address:04X}",)
        }
    }
}

/// Disassemble a single instruction from the given byte iterator.
///
/// # Errors
///
/// Returns an error if the iterator runs out of bytes too early.
pub fn disassemble_instruction<M>(cpu: &CPU<M>) -> Result<String, Error>
where
    M: MemoryAccess,
{
    let mut next_bytes = MemIter::new(cpu);
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
    result.push_str(&format!("{bytes_str: <9}"));
    result.push_str(&format!(
        "{}{:?}",
        if instruction.undocumented { '*' } else { ' ' },
        instruction.name
    ));

    let address = match instruction.size {
        1 => 0,
        2 => u16::from(next[0]),
        3 => u16::from_le_bytes([next[0], next[1]]),
        _ => unreachable!(),
    };

    let address = format_address(cpu, instruction, address);

    if !address.is_empty() {
        result.push_str(&format!(" {address}"));
    }

    Ok(result)
}
