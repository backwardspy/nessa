mod ines;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("invalid magic number")]
    InvalidMagic,
    #[error("unsupported mapper: {0}")]
    UnsupportedMapper(u8),
    #[error("unexpected end of file")]
    EndOfFile,
}

#[derive(Debug)]
pub enum Mapper {
    NROM,
}

#[derive(Debug)]
pub enum Mirroring {
    Horizontal,
    Vertical,
    FourScreen,
}

pub struct ROM {
    pub mapper: Mapper,
    pub mirroring: Mirroring,
    pub trainer: Option<Vec<u8>>,
    pub prg_rom: Vec<u8>,
    pub chr_rom: Vec<u8>,
}

impl ROM {
    pub fn new() -> Self {
        ROM {
            mapper: Mapper::NROM,
            mirroring: Mirroring::Horizontal,
            trainer: None,
            prg_rom: Vec::new(),
            chr_rom: Vec::new(),
        }
    }
}

impl Default for ROM {
    fn default() -> Self {
        Self::new()
    }
}
