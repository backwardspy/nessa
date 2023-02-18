#![warn(
    clippy::pedantic,
    clippy::nursery,
    clippy::cargo,
    clippy::unwrap_used,
    clippy::expect_used
)]
use bitflags::bitflags;
use nessa_rom::{Mirroring, ROM};
use tracing::warn;

pub struct ShiftRegister {
    bytes: [u8; 2],
    index: usize,
}

impl ShiftRegister {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            bytes: [0; 2],
            index: 0,
        }
    }

    pub fn reset(&mut self) {
        self.index = 0;
    }

    pub fn write(&mut self, byte: u8) {
        self.bytes[self.index] = byte;
        self.index = (self.index + 1) % 2;
    }

    pub fn increment(&mut self, n: u16) {
        let mut value = u16::from_be_bytes(self.bytes);
        value += n;
        value &= 0x3FFF;
        self.bytes = value.to_be_bytes();
    }

    #[must_use]
    pub const fn read(&self) -> u16 {
        u16::from_be_bytes(self.bytes)
    }
}

impl Default for ShiftRegister {
    fn default() -> Self {
        Self::new()
    }
}

bitflags! {
    pub struct Control: u8 {
        const NAMETABLE_BASE = 0b0000_0011;
        const VRAM_INCREMENT = 0b0000_0100;
        const SPRITE_PATTERN_TABLE = 0b0000_1000;
        const BACKGROUND_PATTERN_TABLE = 0b0001_0000;
        const SPRITE_SIZE = 0b0010_0000;
        const MASTER_SLAVE_SELECT = 0b0100_0000;
        const NMI_ENABLE = 0b1000_0000;
    }
}

impl Control {
    #[must_use]
    pub const fn new() -> Self {
        Self::empty()
    }

    #[must_use]
    pub fn nametable_address(&self) -> u16 {
        match self.bits & Self::NAMETABLE_BASE.bits {
            0b00 => 0x2000,
            0b01 => 0x2400,
            0b10 => 0x2800,
            0b11 => 0x2C00,
            _ => unreachable!(),
        }
    }

    #[must_use]
    pub const fn vram_increment(&self) -> u16 {
        if self.contains(Self::VRAM_INCREMENT) {
            32
        } else {
            1
        }
    }
}

impl Default for Control {
    fn default() -> Self {
        Self::new()
    }
}

pub struct PPU {
    pub palette_ram: [u8; 0x20],
    pub vram: [u8; 0x2000],
    pub oam: [u8; 0x100],
    pub address: ShiftRegister,
    pub control: Control,
    buffer: u8,
}

impl PPU {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            palette_ram: [0; 0x20],
            vram: [0; 0x2000],
            oam: [0; 0x100],
            address: ShiftRegister::new(),
            control: Control::new(),
            buffer: 0,
        }
    }

    fn increment_address(&mut self) {
        self.address.increment(self.control.vram_increment());
    }

    const fn mirror_address(address: u16, mirroring: Mirroring) -> u16 {
        let address = (address & 0x2FFF) - 0x2000;
        let table = address / 0x400;
        match (mirroring, table) {
            (Mirroring::Vertical, 2 | 3) | (Mirroring::Horizontal, 3) => address - 0x800,
            (Mirroring::Horizontal, 1 | 2) => address - 0x400,
            _ => address,
        }
    }
}

impl nessa_mem::PPU for PPU {
    fn read(&mut self, rom: &ROM) -> u8 {
        let address = self.address.read();
        self.increment_address();
        match address {
            0x0000..=0x1FFF => {
                let value = self.buffer;
                self.buffer = rom.chr_rom[address as usize];
                value
            }
            0x2000..=0x2FFF => {
                let value = self.buffer;
                self.buffer = self.vram[Self::mirror_address(address, rom.mirroring) as usize];
                value
            }
            0x3000..=0x3EFF => {
                warn!("unexpected PPU read at address {address:#04X}");
                0
            }
            // mirrors of 0x3F00, 0x3F04, 0x3F08, 0x3F0C
            0x3F10 | 0x3F14 | 0x3F18 | 0x3F1C => {
                let address = address - 0x10;
                self.palette_ram[(address - 0x3F00) as usize]
            }
            0x3F00..=0x3FFF => self.palette_ram[(address - 0x3F00) as usize],
            _ => {
                warn!("out of bounds PPU read at address {address:#04X}");
                0
            }
        }
    }

    fn write_data(&mut self, value: u8, rom: &ROM) {
        let address = self.address.read();
        match address {
            0x0000..=0x1FFF => warn!("attempted PPU write to character ROM"),
            0x2000..=0x2FFF => {
                let address = Self::mirror_address(address, rom.mirroring);
                self.vram[address as usize] = value;
            }
            0x3000..=0x3EFF => warn!("unexpected PPU write at address {address:#04X}"),
            // mirrors of 0x3F00, 0x3F04, 0x3F08, 0x3F0C
            0x3F10 | 0x3F14 | 0x3F18 | 0x3F1C => {
                let address = address - 0x10;
                self.palette_ram[(address - 0x3F00) as usize] = value;
            }
            0x3F00..=0x3FFF => self.palette_ram[(address - 0x3F00) as usize] = value,
            _ => warn!("out of bounds PPU write at address {address:#04X}"),
        }
        self.increment_address();
    }

    fn write_ctrl(&mut self, value: u8) {
        self.control = Control::from_bits_truncate(value);
    }

    fn write_addr(&mut self, value: u8) {
        self.address.write(value);
    }
}

impl Default for PPU {
    fn default() -> Self {
        Self::new()
    }
}
