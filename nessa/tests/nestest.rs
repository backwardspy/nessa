use nessa6502::{
    cpu::{Status, CPU},
    mem::Bus,
};
use nessa6502_disasm::disassemble_instruction;
use nessa_rom::ROM;
use tracing::{trace, warn};

#[test]
fn test_nestest_rom() {
    color_eyre::install().unwrap();
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::TRACE)
        .init();

    let rom = ROM::from_ines(include_bytes!("../../rom/nestest.nes"))
        .expect("failed to load nestest rom");
    let mut cpu = CPU::new(Bus::new(rom));
    cpu.reset();
    cpu.reg.pc = 0xC000;

    let nestest_lines = include_str!("../../log/nestest.log").lines();
    for nestest_line in nestest_lines {
        let disasm = disassemble_instruction(&cpu).expect("disassembly failed");

        let line = format!(
            "{:04X}  {disasm:<42}A:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X} PPU:{:3},{:3} CYC:{}",
            cpu.reg.pc,
            cpu.reg.a,
            cpu.reg.x,
            cpu.reg.y,
            cpu.reg.status,
            cpu.reg.sp,
            cpu.ppu_y,
            cpu.ppu_x,
            cpu.cycles,
            disasm = disasm,
        );

        let nestest_status =
            Status::from_bits_truncate(u8::from_str_radix(&nestest_line[65..67], 16).unwrap());
        let nessa_status = cpu.reg.status;
        let diff = nestest_status.difference(nessa_status);
        if !diff.is_empty() {
            warn!("nestest has flags we don't: {diff:?}");
        }
        let diff = nessa_status.difference(nestest_status);
        if !diff.is_empty() {
            warn!("nessa has flags nestest doesn't: {diff:?}");
        }

        assert_eq!(line, nestest_line,);

        trace!("{line}");
        cpu.step().expect("step failed");
    }
}
