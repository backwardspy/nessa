use thiserror::Error;

use crate::instruction;

#[derive(Error, Debug)]
pub enum Error {
    #[error("unimplemented operation: {0}")]
    UnimplementedOpcode(instruction::Name),
}
