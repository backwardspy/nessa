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
    fn read(&mut self, addr: u16, rom: &ROM) -> u8;
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
            PPU_START..=PPU_END => {
                let mirror_address = address & PPU_MASK;
                self.ppu.read(mirror_address, &self.rom)
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
            PPU_START..=PPU_END => {
                let mirror_address = address & PPU_MASK;
                warn!("ppu write: {address:04X} ({mirror_address:04X}) = {value:02X}");
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
