use crate::{
    addressing,
    instruction::{self, INSTRUCTIONS},
    mem::Bus,
    Error,
};
use bitflags::bitflags;
use tracing::debug;

const PPU_COLUMNS: u32 = 341;
const PPU_ROWS: u32 = 262;

bitflags! {
    pub struct Status: u8 {
        const CARRY = 0b0000_0001;
        const ZERO = 0b0000_0010;
        const INTERRUPT_DISABLE = 0b0000_0100;
        const DECIMAL_MODE = 0b0000_1000;
        const BREAK = 0b0001_0000;
        const UNUSED = 0b0010_0000;
        const OVERFLOW = 0b0100_0000;
        const NEGATIVE = 0b1000_0000;
    }
}

// This struct is used to represent the registers of the CPU.
//
// The names of the registers are based on the 6502. The `a` register is the
// accumulator, `x` and `y` are the X and Y index registers, respectively. The
// `status` register is the status register, which contains the current state of
// the CPU. The `pc` register is the program counter, which contains the memory
// address of the next instruction to be executed.
//
// The `Registers` struct is used by the `Cpu` struct.
pub struct Registers {
    pub a: u8,
    pub x: u8,
    pub y: u8,

    pub status: Status,
    pub pc: u16,
    pub sp: u8,
}

// This is the CPU struct. It contains a Registers struct, which contains the
// values of all of the registers in the CPU.
pub struct CPU {
    pub reg: Registers,
    pub is_running: bool,
    pub bus: Bus,
    pub cycles: u32,
    // temp until ppu is implemented
    pub ppu_x: u32,
    pub ppu_y: u32,
}

impl CPU {
    /// Create a new CPU.
    #[must_use]
    pub const fn new(bus: Bus) -> Self {
        Self {
            reg: Registers {
                a: 0,
                x: 0,
                y: 0,
                status: Status::empty(),
                pc: 0,
                sp: 0,
            },
            is_running: false,
            bus,
            cycles: 0,
            ppu_x: 0,
            ppu_y: 0,
        }
    }

    /// Reset the CPU.
    pub fn reset(&mut self) {
        self.reg.a = 0;
        self.reg.x = 0;
        self.reg.y = 0;
        self.reg.status = Status::UNUSED | Status::INTERRUPT_DISABLE;
        self.reg.pc = self.read16(0xFFFC);
        self.push16(self.reg.pc);
        self.push8(self.reg.status.bits());

        // tasty magic startup numbers
        self.cycles = 7;
        self.ppu_x = 21;
        self.ppu_y = 0;

        self.is_running = true;

        debug!("cpu reset: pc = 0x{:04X}", self.reg.pc);
    }

    /// Execute a program by repeatedly calling the `step` function until `is_running` is false.
    ///
    /// # Errors
    ///
    /// This function will return an error if the program contains an unknown opcode.
    pub fn execute(&mut self) -> Result<(), Error> {
        while self.is_running {
            self.step()?;
        }

        Ok(())
    }

    /// Execute a single instruction.
    ///
    /// # Errors
    ///
    /// This function will return an error if the program contains an unknown opcode.
    pub fn step(&mut self) -> Result<(), Error> {
        let prev_cycles = self.cycles;

        let code = self.next8();
        let instr = &INSTRUCTIONS[code as usize];

        match instr.name {
            instruction::Name::BRK => self.is_running = false,
            instruction::Name::NOP => (),
            instruction::Name::LDA => self.lda(instr.mode),
            instruction::Name::STA => self.sta(instr.mode),
            instruction::Name::LDX => self.ldx(instr.mode),
            instruction::Name::STX => self.stx(instr.mode),
            instruction::Name::LDY => self.ldy(instr.mode),
            instruction::Name::STY => self.sty(instr.mode),
            instruction::Name::TAX => self.tax(),
            instruction::Name::TXA => self.txa(),
            instruction::Name::TAY => self.tay(),
            instruction::Name::TYA => self.tya(),
            instruction::Name::TXS => self.txs(),
            instruction::Name::TSX => self.tsx(),
            instruction::Name::BIT => self.bit(instr.mode),
            instruction::Name::INC if instr.mode == addressing::Mode::Accumulator => self.inc_a(),
            instruction::Name::INC => self.inc(instr.mode),
            instruction::Name::DEC if instr.mode == addressing::Mode::Accumulator => self.dec_a(),
            instruction::Name::DEC => self.dec(instr.mode),
            instruction::Name::INX => self.inx(),
            instruction::Name::INY => self.iny(),
            instruction::Name::DEX => self.dex(),
            instruction::Name::DEY => self.dey(),
            instruction::Name::JSR => self.jsr(instr.mode),
            instruction::Name::RTS => self.rts(),
            instruction::Name::ROL if instr.mode == addressing::Mode::Accumulator => self.rol_a(),
            instruction::Name::ROL => self.rol(instr.mode),
            instruction::Name::ASL => self.asl(instr.mode),
            instruction::Name::LSR if instr.mode == addressing::Mode::Accumulator => self.lsr_a(),
            instruction::Name::LSR => self.lsr(instr.mode),
            instruction::Name::SEC => self.reg.status.insert(Status::CARRY),
            instruction::Name::CLC => self.reg.status.remove(Status::CARRY),
            instruction::Name::SEI => self.reg.status.insert(Status::INTERRUPT_DISABLE),
            instruction::Name::CLI => self.reg.status.remove(Status::INTERRUPT_DISABLE),
            instruction::Name::SED => self.reg.status.insert(Status::DECIMAL_MODE),
            instruction::Name::CLD => self.reg.status.remove(Status::DECIMAL_MODE),
            instruction::Name::CLV => self.reg.status.remove(Status::OVERFLOW),
            instruction::Name::ORA => self.ora(instr.mode),
            instruction::Name::AND => self.and(instr.mode),
            instruction::Name::EOR => self.eor(instr.mode),
            instruction::Name::ADC => self.adc(instr.mode),
            instruction::Name::SBC => self.sbc(instr.mode),
            instruction::Name::CMP => self.cmp(instr.mode),
            instruction::Name::CPX => self.cpx(instr.mode),
            instruction::Name::CPY => self.cpy(instr.mode),
            instruction::Name::JMP => self.jmp(instr.mode),
            instruction::Name::PHA => self.pha(),
            instruction::Name::PLA => self.pla(),
            instruction::Name::PHP => self.php(),
            instruction::Name::PLP => self.plp(),
            instruction::Name::BEQ => self.beq(instr.mode),
            instruction::Name::BNE => self.bne(instr.mode),
            instruction::Name::BPL => self.bpl(instr.mode),
            instruction::Name::BMI => self.bmi(instr.mode),
            instruction::Name::BCC => self.bcc(instr.mode),
            instruction::Name::BCS => self.bcs(instr.mode),
            instruction::Name::BVC => self.bvc(instr.mode),
            instruction::Name::BVS => self.bvs(instr.mode),
            _ => return Err(Error::UnimplementedOpcode(instr.name)),
        }

        self.cycles += u32::from(instr.cycles);

        // add a cycle if a page was crossed (and the intruction takes longer)
        if instr.cycles_page_crossed > 0 {
            // FIXME: nasty hack for now
            self.reg.pc -= u16::from(instr.size) - 1;
            let pc = self.reg.pc;
            let address = self.resolve_address(instr.mode);
            if address & 0xFF00 != pc & 0xFF00 {
                self.cycles += u32::from(instr.cycles_page_crossed);
            }
        }

        self.ppu_x += (self.cycles - prev_cycles) * 3;
        if self.ppu_x >= PPU_COLUMNS {
            self.ppu_x -= PPU_COLUMNS;
            self.ppu_y += 1;
            if self.ppu_y > PPU_ROWS {
                self.ppu_y = 0;
            }
        }

        Ok(())
    }

    fn resolve_address(&mut self, mode: addressing::Mode) -> u16 {
        match mode {
            addressing::Mode::Immediate => self.reg.pc,
            addressing::Mode::ZeroPage => u16::from(self.next8()),
            addressing::Mode::ZeroPageX => {
                let address = self.next8();
                u16::from(address.wrapping_add(self.reg.x))
            }
            addressing::Mode::ZeroPageY => {
                let address = self.next8();
                u16::from(address.wrapping_add(self.reg.y))
            }
            addressing::Mode::Absolute => self.next16(),
            addressing::Mode::AbsoluteX => {
                let address = self.next16();
                address.wrapping_add(u16::from(self.reg.x))
            }
            addressing::Mode::AbsoluteY => {
                let address = self.next16();
                address.wrapping_add(u16::from(self.reg.y))
            }
            addressing::Mode::Indirect => {
                let address = self.next16();
                self.read16(address)
            }
            addressing::Mode::IndirectX => {
                let address = self.next8();
                let address = address.wrapping_add(self.reg.x);
                self.read16(u16::from(address))
            }
            addressing::Mode::IndirectY => {
                let address = self.next8();
                let address = self.read16(u16::from(address));
                address.wrapping_add(u16::from(self.reg.y))
            }
            addressing::Mode::Relative => {
                #[allow(clippy::cast_possible_wrap)]
                let offset = self.next8() as i8;
                #[allow(clippy::cast_sign_loss)]
                self.reg.pc.wrapping_add(offset as u16)
            }
            addressing::Mode::Implied | addressing::Mode::Accumulator => unreachable!(),
        }
    }

    fn next8(&mut self) -> u8 {
        let value = self.read8(self.reg.pc);
        self.reg.pc += 1;
        value
    }

    fn next16(&mut self) -> u16 {
        let value = self.read16(self.reg.pc);
        self.reg.pc += 2;
        value
    }

    fn read8(&self, address: u16) -> u8 {
        self.bus.read(address)
    }

    fn write8(&mut self, address: u16, value: u8) {
        self.bus.write(address, value);
    }

    fn read16(&self, address: u16) -> u16 {
        let lo = self.read8(address);
        let hi = self.read8(address.wrapping_add(1));
        u16::from_le_bytes([lo, hi])
    }

    fn push8(&mut self, value: u8) {
        self.write8(0x0100 + u16::from(self.reg.sp), value);
        self.reg.sp = self.reg.sp.wrapping_sub(1);
    }

    fn push16(&mut self, value: u16) {
        let [lo, hi] = value.to_le_bytes();
        self.push8(lo);
        self.push8(hi);
    }

    fn pop8(&mut self) -> u8 {
        self.reg.sp = self.reg.sp.wrapping_add(1);
        self.read8(0x0100 + u16::from(self.reg.sp))
    }

    fn pop16(&mut self) -> u16 {
        let hi = self.pop8();
        let lo = self.pop8();
        u16::from_le_bytes([lo, hi])
    }

    fn read_arg(&mut self, mode: addressing::Mode) -> u8 {
        let address = self.resolve_address(mode);
        let value = self.read8(address);
        if mode == addressing::Mode::Immediate {
            self.reg.pc += 1;
        }
        value
    }

    fn set_flags(&mut self, value: u8) {
        self.reg.status.set(Status::ZERO, value == 0);
        self.reg
            .status
            .set(Status::NEGATIVE, value & 0b1000_0000 == 0b1000_0000);
    }
}

/// Instruction implementations.
impl CPU {
    fn lda(&mut self, mode: addressing::Mode) {
        let value = self.read_arg(mode);
        self.reg.a = value;
        self.set_flags(self.reg.a);
    }

    fn sta(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        self.write8(address, self.reg.a);
        self.set_flags(self.reg.a);
    }

    fn ldx(&mut self, mode: addressing::Mode) {
        let value = self.read_arg(mode);
        self.reg.x = value;
        self.set_flags(self.reg.x);
    }

    fn stx(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        self.write8(address, self.reg.x);
        self.set_flags(self.reg.x);
    }

    fn ldy(&mut self, mode: addressing::Mode) {
        let value = self.read_arg(mode);
        self.reg.y = value;
        self.set_flags(self.reg.y);
    }

    fn sty(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        self.write8(address, self.reg.y);
        self.set_flags(self.reg.y);
    }

    fn tax(&mut self) {
        self.reg.x = self.reg.a;
        self.set_flags(self.reg.x);
    }

    fn txa(&mut self) {
        self.reg.a = self.reg.x;
        self.set_flags(self.reg.a);
    }

    fn tay(&mut self) {
        self.reg.y = self.reg.a;
        self.set_flags(self.reg.y);
    }

    fn tya(&mut self) {
        self.reg.a = self.reg.y;
        self.set_flags(self.reg.a);
    }

    fn txs(&mut self) {
        self.reg.sp = self.reg.x;
    }

    fn tsx(&mut self) {
        self.reg.x = self.reg.sp;
        self.set_flags(self.reg.x);
    }

    fn bit(&mut self, mode: addressing::Mode) {
        let value = self.read_arg(mode);
        self.reg.status.set(Status::ZERO, value & self.reg.a == 0);
        self.reg
            .status
            .set(Status::OVERFLOW, value & 0b0100_0000 == 0b0100_0000);
        self.reg
            .status
            .set(Status::NEGATIVE, value & 0b1000_0000 == 0b1000_0000);
    }

    fn inc_a(&mut self) {
        self.reg.a = self.reg.a.wrapping_add(1);
        self.set_flags(self.reg.a);
    }

    fn inc(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        let value = self.read8(address).wrapping_add(1);
        self.write8(address, value);
        self.set_flags(value);
    }

    fn dec_a(&mut self) {
        self.reg.a = self.reg.a.wrapping_sub(1);
        self.set_flags(self.reg.a);
    }

    fn dec(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        let value = self.read8(address).wrapping_sub(1);
        self.write8(address, value);
        self.set_flags(value);
    }

    fn inx(&mut self) {
        self.reg.x = self.reg.x.wrapping_add(1);
        self.set_flags(self.reg.x);
    }

    fn iny(&mut self) {
        self.reg.y = self.reg.y.wrapping_add(1);
        self.set_flags(self.reg.y);
    }

    fn dex(&mut self) {
        self.reg.x = self.reg.x.wrapping_sub(1);
        self.set_flags(self.reg.x);
    }

    fn dey(&mut self) {
        self.reg.y = self.reg.y.wrapping_sub(1);
        self.set_flags(self.reg.y);
    }

    fn jsr(&mut self, mode: addressing::Mode) {
        self.push16(self.reg.pc.wrapping_add(1));
        let address = self.resolve_address(mode);
        self.reg.pc = address;
    }

    fn rts(&mut self) {
        let address = self.pop16();
        self.reg.pc = address.wrapping_add(1);
    }

    fn rol_a(&mut self) {
        let mut value = self.reg.a;
        let overflow = value & 0b1000_0000 == 0b1000_0000;
        value <<= 1;
        if self.reg.status.contains(Status::CARRY) {
            value |= 1;
        }
        self.reg.a = value;
        self.reg.status.set(Status::CARRY, overflow);
        self.set_flags(self.reg.a);
    }

    fn rol(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        let mut value = self.read8(address);
        let overflow = value & 0b1000_0000 == 0b1000_0000;
        value <<= 1;
        if self.reg.status.contains(Status::CARRY) {
            value |= 1;
        }
        self.write8(address, value);
        self.reg.status.set(Status::CARRY, overflow);
        self.set_flags(value);
    }

    fn asl(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        let mut value = self.read8(address);
        self.reg
            .status
            .set(Status::CARRY, value & 0b1000_0000 == 0b1000_0000);
        value <<= 1;
        self.write8(address, value);
        self.set_flags(value);
    }

    fn lsr_a(&mut self) {
        let mut value = self.reg.a;
        self.reg.status.set(Status::CARRY, value & 1 == 1);
        value >>= 1;
        self.reg.a = value;
        self.set_flags(value);
    }

    fn lsr(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        let mut value = self.read8(address);
        self.reg.status.set(Status::CARRY, value & 1 == 1);
        value >>= 1;
        self.write8(address, value);
        self.set_flags(value);
    }

    fn ora(&mut self, mode: addressing::Mode) {
        let value = self.read_arg(mode);
        self.reg.a |= value;
        self.set_flags(self.reg.a);
    }

    fn and(&mut self, mode: addressing::Mode) {
        let value = self.read_arg(mode);
        self.reg.a &= value;
        self.set_flags(self.reg.a);
    }

    fn eor(&mut self, mode: addressing::Mode) {
        let value = self.read_arg(mode);
        self.reg.a ^= value;
        self.set_flags(self.reg.a);
    }

    fn do_add(&mut self, reg: u16, value: u16) {
        let mut result = reg + value;
        if self.reg.status.contains(Status::CARRY) {
            result += 1;
        }
        self.reg.status.set(Status::CARRY, result > 0xFF);
        self.reg.status.set(
            Status::OVERFLOW,
            (reg ^ result) & (value ^ result) & 0x80 != 0,
        );
        self.reg.a = result.to_le_bytes()[0];
        self.set_flags(self.reg.a);
    }

    fn adc(&mut self, mode: addressing::Mode) {
        let a = u16::from(self.reg.a);
        let value = u16::from(self.read_arg(mode));
        self.do_add(a, value);
    }

    fn sbc(&mut self, mode: addressing::Mode) {
        let a = u16::from(self.reg.a);
        let value = u16::from(self.read_arg(mode).wrapping_neg().wrapping_sub(1));
        self.do_add(a, value);
    }

    fn do_compare(&mut self, reg: u16, value: u16) {
        let result = reg.wrapping_sub(value);
        self.reg.status.set(Status::CARRY, reg >= value);
        self.set_flags(result.to_le_bytes()[0]);
    }

    fn cmp(&mut self, mode: addressing::Mode) {
        let value = u16::from(self.read_arg(mode));
        let a = u16::from(self.reg.a);
        self.do_compare(a, value);
    }

    fn cpx(&mut self, mode: addressing::Mode) {
        let value = u16::from(self.read_arg(mode));
        let x = u16::from(self.reg.x);
        self.do_compare(x, value);
    }

    fn cpy(&mut self, mode: addressing::Mode) {
        let value = u16::from(self.read_arg(mode));
        let y = u16::from(self.reg.y);
        self.do_compare(y, value);
    }

    fn jmp(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        self.reg.pc = address;
    }

    fn pha(&mut self) {
        self.push8(self.reg.a);
    }

    fn pla(&mut self) {
        self.reg.a = self.pop8();
        self.set_flags(self.reg.a);
    }

    fn php(&mut self) {
        self.push8((self.reg.status | Status::BREAK).bits());
    }

    fn plp(&mut self) {
        self.reg.status = Status::from_bits_truncate(self.pop8()) | Status::UNUSED;
        self.reg.status.remove(Status::BREAK);
    }

    fn add_branch_cycles(&mut self, address: u16) {
        self.cycles += 1;
        let page_crossed = (self.reg.pc & 0xFF00) != (address & 0xFF00);
        if page_crossed {
            self.cycles += 1;
        }
    }

    fn beq(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        if self.reg.status.contains(Status::ZERO) {
            self.reg.pc = address;
            self.add_branch_cycles(address);
        }
    }

    fn bne(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        if !self.reg.status.contains(Status::ZERO) {
            self.reg.pc = address;
            self.add_branch_cycles(address);
        }
    }

    fn bpl(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        if !self.reg.status.contains(Status::NEGATIVE) {
            self.reg.pc = address;
            self.add_branch_cycles(address);
        }
    }

    fn bmi(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        if self.reg.status.contains(Status::NEGATIVE) {
            self.reg.pc = address;
            self.add_branch_cycles(address);
        }
    }

    fn bcc(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        if !self.reg.status.contains(Status::CARRY) {
            self.reg.pc = address;
            self.add_branch_cycles(address);
        }
    }

    fn bcs(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        if self.reg.status.contains(Status::CARRY) {
            self.reg.pc = address;
            self.add_branch_cycles(address);
        }
    }

    fn bvc(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        if !self.reg.status.contains(Status::OVERFLOW) {
            self.reg.pc = address;
            self.add_branch_cycles(address);
        }
    }

    fn bvs(&mut self, mode: addressing::Mode) {
        let address = self.resolve_address(mode);
        if self.reg.status.contains(Status::OVERFLOW) {
            self.reg.pc = address;
            self.add_branch_cycles(address);
        }
    }
}
