#![warn(
    clippy::pedantic,
    clippy::nursery,
    clippy::cargo,
    clippy::unwrap_used,
    clippy::expect_used
)]
use nessa_cpu::MemoryAccess;
use nessa_rom::ROM;
use tracing::{error, warn};

const RAM_START: u16 = 0x0000;
const RAM_END: u16 = 0x1FFF;
const PPU_START: u16 = 0x2000;
const PPU_END: u16 = 0x3FFF;
const ROM_START: u16 = 0x8000;

const RAM_SIZE: usize = 0x0800;

const RAM_MASK: u16 = 0x07FF;
const PPU_MASK: u16 = 0x2007;

pub trait PPU {
    fn read(&mut self, rom: &ROM) -> u8;
    fn write_data(&mut self, value: u8, rom: &ROM);
    fn write_ctrl(&mut self, value: u8);
    fn write_addr(&mut self, value: u8);
}

pub struct Bus<P>
where
    P: PPU,
{
    pub ram: [u8; RAM_SIZE],
    pub rom: ROM,
    pub ppu: P,
}

impl<P> Bus<P>
where
    P: PPU,
{
    #[must_use]
    pub const fn new(rom: ROM, ppu: P) -> Self {
        Self {
            ram: [0; RAM_SIZE],
            rom,
            ppu,
        }
    }

    fn read_rom(&self, address: u16) -> u8 {
        if self.rom.prg_rom.len() == 0x4000 {
            let address = address % 0x4000;
            self.rom.prg_rom[address as usize]
        } else {
            self.rom.prg_rom[address as usize]
        }
    }
}

impl<P> MemoryAccess for Bus<P>
where
    P: PPU,
{
    fn read_ro(&self, address: u16) -> u8 {
        match address {
            RAM_START..=RAM_END => {
                let address = address & RAM_MASK;
                self.ram[address as usize]
            }
            PPU_START..=PPU_END => {
                warn!("attempted read-only ppu access: {address:04X}");
                0
            }
            ROM_START..=0xFFFF => self.read_rom(address - ROM_START),
            _ => {
                warn!("out of bounds read-only access: {address:04X}");
                0
            }
        }
    }

    fn read(&mut self, address: u16) -> u8 {
        match address {
            RAM_START..=RAM_END => {
                let address = address & RAM_MASK;
                self.ram[address as usize]
            }
            0x2000 | 0x2001 | 0x2003 | 0x2005 | 0x2006 | 0x4014 => {
                warn!("read from write-only PPU address: {address:04X}");
                0
            }
            0x2007 => self.ppu.read(&self.rom),
            0x2008..=PPU_END => {
                let mirror_address = address & PPU_MASK;
                self.read(mirror_address)
            }
            ROM_START..=0xFFFF => self.read_rom(address - ROM_START),
            _ => {
                warn!("out of bounds read: {address:04X}");
                0
            }
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        match address {
            RAM_START..=RAM_END => {
                let address = address & RAM_MASK;
                self.ram[address as usize] = value;
            }
            0x2000 => self.ppu.write_ctrl(value),
            0x2006 => self.ppu.write_addr(value),
            0x2007 => self.ppu.write_data(value, &self.rom),
            0x2008..=PPU_END => {
                let mirror_address = address & PPU_MASK;
                self.write(mirror_address, value);
            }
            ROM_START..=0xFFFF => {
                error!("attempted write to ROM: {address:04X} = {value:02X}");
            }
            _ => {
                warn!("out of bounds write: {address:04X} = {value:02X}");
            }
        }
    }
}
