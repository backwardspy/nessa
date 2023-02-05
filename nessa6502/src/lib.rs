#![warn(
    clippy::pedantic,
    clippy::nursery,
    clippy::cargo,
    clippy::unwrap_used,
    clippy::expect_used
)]
pub mod addressing;
pub mod cpu;
mod errors;
pub use errors::Error;
pub mod instruction;
