use std::io::{Cursor, Read};

use byteorder::{LittleEndian, ReadBytesExt};

use crate::{Error, Mapper, Mirroring, ROM};

impl ROM {
    /// Create a `ROM` from an `iNES` file
    ///
    /// # Errors
    ///
    /// Returns an error if the file is not a valid `iNES` file or if the mapper is not supported
    pub fn from_ines(ines: &[u8]) -> Result<Self, Error> {
        let mut cursor = Cursor::new(ines);
        let magic = cursor
            .read_u32::<LittleEndian>()
            .map_err(|_| Error::EndOfFile)?;
        if magic != 0x1A53_454E {
            return Err(Error::InvalidMagic);
        }

        let mut rom = Self::new();

        let mapper_id = (ines[7] & 0xf0) | (ines[6] >> 4);
        rom.mapper = match mapper_id {
            0 => Mapper::NROM,
            _ => return Err(Error::UnsupportedMapper(mapper_id)),
        };

        // set mirroring
        rom.mirroring = match (ines[6] & 0b1 != 0, ines[6] & 0b1000 != 0) {
            (false, false) => Mirroring::Horizontal,
            (false, true) => Mirroring::Vertical,
            (true, _) => Mirroring::FourScreen,
        };

        cursor.set_position(0x10);

        // load trainer if any
        if ines[6] & 0b100 != 0 {
            let mut trainer = vec![0; 512];
            cursor
                .read_exact(&mut trainer)
                .map_err(|_| Error::EndOfFile)?;
        }

        rom.prg_rom = vec![0; ines[4] as usize * 0x4000];
        cursor
            .read_exact(&mut rom.prg_rom)
            .map_err(|_| Error::EndOfFile)?;

        rom.chr_rom = vec![0; ines[5] as usize * 0x2000];
        cursor
            .read_exact(&mut rom.chr_rom)
            .map_err(|_| Error::EndOfFile)?;

        Ok(rom)
    }
}
