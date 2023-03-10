use crate::addressing::Mode;

#[allow(clippy::upper_case_acronyms)]
#[derive(Debug, Clone, Copy)]
pub enum Name {
    BRK,
    ORA,
    KIL,
    SLO,
    NOP,
    ASL,
    PHP,
    ANC,
    BPL,
    CLC,
    JSR,
    AND,
    RLA,
    BIT,
    ROL,
    PLP,
    BMI,
    SEC,
    RTI,
    EOR,
    SRE,
    LSR,
    PHA,
    ALR,
    JMP,
    BVC,
    CLI,
    RTS,
    ADC,
    RRA,
    ROR,
    PLA,
    ARR,
    BVS,
    SEI,
    STA,
    SAX,
    STY,
    STX,
    DEY,
    TXA,
    XAA,
    BCC,
    AHX,
    TYA,
    TXS,
    TAS,
    SHY,
    SHX,
    LDY,
    LDA,
    LDX,
    LAX,
    TAY,
    TAX,
    BCS,
    CLV,
    TSX,
    LAS,
    CPY,
    CMP,
    DCP,
    DEC,
    INY,
    DEX,
    AXS,
    BNE,
    CLD,
    CPX,
    SBC,
    ISB,
    INC,
    INX,
    BEQ,
    SED,
}

pub struct Instruction {
    pub name: Name,
    pub size: u8,
    pub undocumented: bool,
    pub cycles: u8,
    pub cycles_page_crossed: u8,
    pub mode: Mode,
}

impl Instruction {
    #[must_use]
    pub const fn new(
        name: Name,
        size: u8,
        undocumented: bool,
        cycles: u8,
        cycles_page_crossed: u8,
        mode: Mode,
    ) -> Self {
        Self {
            name,
            size,
            undocumented,
            cycles,
            cycles_page_crossed,
            mode,
        }
    }
}

pub const INSTRUCTIONS: &[Instruction] = &[
    Instruction::new(Name::BRK, 1, false, 7, 0, Mode::Implied), // 0x00
    Instruction::new(Name::ORA, 2, false, 6, 0, Mode::IndirectX),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::SLO, 2, true, 8, 0, Mode::IndirectX),
    Instruction::new(Name::NOP, 2, true, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::ORA, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::ASL, 2, false, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::SLO, 2, true, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::PHP, 1, false, 3, 0, Mode::Implied),
    Instruction::new(Name::ORA, 2, false, 2, 0, Mode::Immediate),
    Instruction::new(Name::ASL, 1, false, 2, 0, Mode::Accumulator),
    Instruction::new(Name::ANC, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::NOP, 3, true, 4, 0, Mode::Absolute),
    Instruction::new(Name::ORA, 3, false, 4, 0, Mode::Absolute),
    Instruction::new(Name::ASL, 3, false, 6, 0, Mode::Absolute),
    Instruction::new(Name::SLO, 3, true, 6, 0, Mode::Absolute),
    Instruction::new(Name::BPL, 2, false, 2, 1, Mode::Relative), // 0x10
    Instruction::new(Name::ORA, 2, false, 5, 1, Mode::IndirectY),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::SLO, 2, true, 8, 0, Mode::IndirectY),
    Instruction::new(Name::NOP, 2, true, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::ORA, 2, false, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::ASL, 2, false, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::SLO, 2, true, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::CLC, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::ORA, 3, false, 4, 1, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 1, true, 2, 0, Mode::Implied),
    Instruction::new(Name::SLO, 3, true, 7, 0, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 3, true, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::ORA, 3, false, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::ASL, 3, false, 7, 0, Mode::AbsoluteX),
    Instruction::new(Name::SLO, 3, true, 7, 0, Mode::AbsoluteX),
    Instruction::new(Name::JSR, 3, false, 6, 0, Mode::Absolute), // 0x20
    Instruction::new(Name::AND, 2, false, 6, 0, Mode::IndirectX),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::RLA, 2, true, 8, 0, Mode::IndirectX),
    Instruction::new(Name::BIT, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::AND, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::ROL, 2, false, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::RLA, 2, true, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::PLP, 1, false, 4, 0, Mode::Implied),
    Instruction::new(Name::AND, 2, false, 2, 0, Mode::Immediate),
    Instruction::new(Name::ROL, 1, false, 2, 0, Mode::Accumulator),
    Instruction::new(Name::ANC, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::BIT, 3, false, 4, 0, Mode::Absolute),
    Instruction::new(Name::AND, 3, false, 4, 0, Mode::Absolute),
    Instruction::new(Name::ROL, 3, false, 6, 0, Mode::Absolute),
    Instruction::new(Name::RLA, 3, true, 6, 0, Mode::Absolute),
    Instruction::new(Name::BMI, 2, false, 2, 1, Mode::Relative), // 0x30
    Instruction::new(Name::AND, 2, false, 5, 1, Mode::IndirectY),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::RLA, 2, true, 8, 0, Mode::IndirectY),
    Instruction::new(Name::NOP, 2, true, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::AND, 2, false, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::ROL, 2, false, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::RLA, 2, true, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::SEC, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::AND, 3, false, 4, 1, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 1, true, 2, 0, Mode::Implied),
    Instruction::new(Name::RLA, 3, true, 7, 0, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 3, true, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::AND, 3, false, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::ROL, 3, false, 7, 0, Mode::AbsoluteX),
    Instruction::new(Name::RLA, 3, true, 7, 0, Mode::AbsoluteX),
    Instruction::new(Name::RTI, 1, false, 6, 0, Mode::Implied), // 0x40
    Instruction::new(Name::EOR, 2, false, 6, 0, Mode::IndirectX),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::SRE, 2, true, 8, 0, Mode::IndirectX),
    Instruction::new(Name::NOP, 2, true, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::EOR, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::LSR, 2, false, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::SRE, 2, true, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::PHA, 1, false, 3, 0, Mode::Implied),
    Instruction::new(Name::EOR, 2, false, 2, 0, Mode::Immediate),
    Instruction::new(Name::LSR, 1, false, 2, 0, Mode::Accumulator),
    Instruction::new(Name::ALR, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::JMP, 3, false, 3, 0, Mode::Absolute),
    Instruction::new(Name::EOR, 3, false, 4, 0, Mode::Absolute),
    Instruction::new(Name::LSR, 3, false, 6, 0, Mode::Absolute),
    Instruction::new(Name::SRE, 3, true, 6, 0, Mode::Absolute),
    Instruction::new(Name::BVC, 2, false, 2, 1, Mode::Relative), // 0x50
    Instruction::new(Name::EOR, 2, false, 5, 1, Mode::IndirectY),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::SRE, 2, true, 8, 0, Mode::IndirectY),
    Instruction::new(Name::NOP, 2, true, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::EOR, 2, false, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::LSR, 2, false, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::SRE, 2, true, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::CLI, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::EOR, 3, false, 4, 1, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 1, true, 2, 0, Mode::Implied),
    Instruction::new(Name::SRE, 3, true, 7, 0, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 3, true, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::EOR, 3, false, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::LSR, 3, false, 7, 0, Mode::AbsoluteX),
    Instruction::new(Name::SRE, 3, true, 7, 0, Mode::AbsoluteX),
    Instruction::new(Name::RTS, 1, false, 6, 0, Mode::Implied), // 0x60
    Instruction::new(Name::ADC, 2, false, 6, 0, Mode::IndirectX),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::RRA, 2, true, 8, 0, Mode::IndirectX),
    Instruction::new(Name::NOP, 2, true, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::ADC, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::ROR, 2, false, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::RRA, 2, true, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::PLA, 1, false, 4, 0, Mode::Implied),
    Instruction::new(Name::ADC, 2, false, 2, 0, Mode::Immediate),
    Instruction::new(Name::ROR, 1, false, 2, 0, Mode::Accumulator),
    Instruction::new(Name::ARR, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::JMP, 3, false, 5, 0, Mode::Indirect),
    Instruction::new(Name::ADC, 3, false, 4, 1, Mode::Absolute),
    Instruction::new(Name::ROR, 3, false, 6, 0, Mode::Absolute),
    Instruction::new(Name::RRA, 3, true, 6, 0, Mode::Absolute),
    Instruction::new(Name::BVS, 2, false, 2, 1, Mode::Relative), // 0x70
    Instruction::new(Name::ADC, 2, false, 5, 1, Mode::IndirectY),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::RRA, 2, true, 8, 0, Mode::IndirectY),
    Instruction::new(Name::NOP, 2, true, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::ADC, 2, false, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::ROR, 2, false, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::RRA, 2, true, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::SEI, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::ADC, 3, false, 4, 1, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 1, true, 2, 0, Mode::Implied),
    Instruction::new(Name::RRA, 3, true, 7, 0, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 3, true, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::ADC, 3, false, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::ROR, 3, false, 7, 0, Mode::AbsoluteX),
    Instruction::new(Name::RRA, 3, true, 7, 0, Mode::AbsoluteX),
    Instruction::new(Name::NOP, 2, true, 2, 0, Mode::Immediate), // 0x80
    Instruction::new(Name::STA, 2, false, 6, 0, Mode::IndirectX),
    Instruction::new(Name::NOP, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::SAX, 2, true, 6, 0, Mode::IndirectX),
    Instruction::new(Name::STY, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::STA, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::STX, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::SAX, 2, true, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::DEY, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::NOP, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::TXA, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::XAA, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::STY, 3, false, 4, 1, Mode::Absolute),
    Instruction::new(Name::STA, 3, false, 4, 0, Mode::Absolute),
    Instruction::new(Name::STX, 3, false, 4, 1, Mode::Absolute),
    Instruction::new(Name::SAX, 3, true, 4, 1, Mode::Absolute),
    Instruction::new(Name::BCC, 2, false, 2, 1, Mode::Relative), // 0x90
    Instruction::new(Name::STA, 2, false, 6, 0, Mode::IndirectY),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::AHX, 2, true, 6, 0, Mode::IndirectY),
    Instruction::new(Name::STY, 2, false, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::STA, 2, false, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::STX, 2, false, 4, 0, Mode::ZeroPageY),
    Instruction::new(Name::SAX, 2, true, 4, 0, Mode::ZeroPageY),
    Instruction::new(Name::TYA, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::STA, 3, false, 5, 0, Mode::AbsoluteY),
    Instruction::new(Name::TXS, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::TAS, 3, true, 5, 1, Mode::AbsoluteY),
    Instruction::new(Name::SHY, 3, true, 5, 1, Mode::AbsoluteX),
    Instruction::new(Name::STA, 3, false, 5, 0, Mode::AbsoluteX),
    Instruction::new(Name::SHX, 3, true, 5, 1, Mode::AbsoluteY),
    Instruction::new(Name::AHX, 3, true, 5, 1, Mode::AbsoluteY),
    Instruction::new(Name::LDY, 2, false, 2, 0, Mode::Immediate), // 0xA0
    Instruction::new(Name::LDA, 2, false, 6, 0, Mode::IndirectX),
    Instruction::new(Name::LDX, 2, false, 2, 0, Mode::Immediate),
    Instruction::new(Name::LAX, 2, true, 6, 0, Mode::IndirectX),
    Instruction::new(Name::LDY, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::LDA, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::LDX, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::LAX, 2, true, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::TAY, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::LDA, 2, false, 2, 0, Mode::Immediate),
    Instruction::new(Name::TAX, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::LAX, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::LDY, 3, false, 4, 1, Mode::Absolute),
    Instruction::new(Name::LDA, 3, false, 4, 1, Mode::Absolute),
    Instruction::new(Name::LDX, 3, false, 4, 1, Mode::Absolute),
    Instruction::new(Name::LAX, 3, true, 4, 1, Mode::Absolute),
    Instruction::new(Name::BCS, 2, false, 2, 1, Mode::Relative), // 0xB0
    Instruction::new(Name::LDA, 2, false, 5, 1, Mode::IndirectY),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::LAX, 2, true, 5, 1, Mode::IndirectY),
    Instruction::new(Name::LDY, 2, false, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::LDA, 2, false, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::LDX, 2, false, 4, 0, Mode::ZeroPageY),
    Instruction::new(Name::LAX, 2, true, 4, 0, Mode::ZeroPageY),
    Instruction::new(Name::CLV, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::LDA, 3, false, 4, 1, Mode::AbsoluteY),
    Instruction::new(Name::TSX, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::LAS, 3, true, 4, 1, Mode::AbsoluteY),
    Instruction::new(Name::LDY, 3, false, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::LDA, 3, false, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::LDX, 3, false, 4, 1, Mode::AbsoluteY),
    Instruction::new(Name::LAX, 3, true, 4, 1, Mode::AbsoluteY),
    Instruction::new(Name::CPY, 2, false, 2, 0, Mode::Immediate), // 0xC0
    Instruction::new(Name::CMP, 2, false, 6, 0, Mode::IndirectX),
    Instruction::new(Name::NOP, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::DCP, 2, true, 8, 0, Mode::IndirectX),
    Instruction::new(Name::CPY, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::CMP, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::DEC, 2, false, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::DCP, 2, true, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::INY, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::CMP, 2, false, 2, 0, Mode::Immediate),
    Instruction::new(Name::DEX, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::AXS, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::CPY, 3, false, 4, 1, Mode::Absolute),
    Instruction::new(Name::CMP, 3, false, 4, 1, Mode::Absolute),
    Instruction::new(Name::DEC, 3, false, 6, 0, Mode::Absolute),
    Instruction::new(Name::DCP, 3, true, 6, 0, Mode::Absolute),
    Instruction::new(Name::BNE, 2, false, 2, 1, Mode::Relative), // 0xD0
    Instruction::new(Name::CMP, 2, false, 5, 1, Mode::IndirectY),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::DCP, 2, true, 8, 0, Mode::IndirectY),
    Instruction::new(Name::NOP, 2, true, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::CMP, 2, false, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::DEC, 2, false, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::DCP, 2, true, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::CLD, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::CMP, 3, false, 4, 1, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 1, true, 2, 0, Mode::Implied),
    Instruction::new(Name::DCP, 3, true, 7, 0, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 3, true, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::CMP, 3, false, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::DEC, 3, false, 7, 0, Mode::AbsoluteX),
    Instruction::new(Name::DCP, 3, true, 7, 0, Mode::AbsoluteX),
    Instruction::new(Name::CPX, 2, false, 2, 0, Mode::Immediate), // 0xE0
    Instruction::new(Name::SBC, 2, false, 6, 0, Mode::IndirectX),
    Instruction::new(Name::NOP, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::ISB, 2, true, 8, 0, Mode::IndirectX),
    Instruction::new(Name::CPX, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::SBC, 2, false, 3, 0, Mode::ZeroPage),
    Instruction::new(Name::INC, 2, false, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::ISB, 2, true, 5, 0, Mode::ZeroPage),
    Instruction::new(Name::INX, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::SBC, 2, false, 2, 0, Mode::Immediate),
    Instruction::new(Name::NOP, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::SBC, 2, true, 2, 0, Mode::Immediate),
    Instruction::new(Name::CPX, 3, false, 4, 1, Mode::Absolute),
    Instruction::new(Name::SBC, 3, false, 4, 1, Mode::Absolute),
    Instruction::new(Name::INC, 3, false, 6, 0, Mode::Absolute),
    Instruction::new(Name::ISB, 3, true, 6, 0, Mode::Absolute),
    Instruction::new(Name::BEQ, 2, false, 2, 1, Mode::Relative), // 0xF0
    Instruction::new(Name::SBC, 2, false, 5, 1, Mode::IndirectY),
    Instruction::new(Name::KIL, 1, true, 0, 0, Mode::Implied),
    Instruction::new(Name::ISB, 2, true, 8, 0, Mode::IndirectY),
    Instruction::new(Name::NOP, 2, true, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::SBC, 2, false, 4, 0, Mode::ZeroPageX),
    Instruction::new(Name::INC, 2, false, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::ISB, 2, true, 6, 0, Mode::ZeroPageX),
    Instruction::new(Name::SED, 1, false, 2, 0, Mode::Implied),
    Instruction::new(Name::SBC, 3, false, 4, 1, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 1, true, 2, 0, Mode::Implied),
    Instruction::new(Name::ISB, 3, true, 7, 0, Mode::AbsoluteY),
    Instruction::new(Name::NOP, 3, true, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::SBC, 3, false, 4, 1, Mode::AbsoluteX),
    Instruction::new(Name::INC, 3, false, 7, 0, Mode::AbsoluteX),
    Instruction::new(Name::ISB, 3, true, 7, 0, Mode::AbsoluteX),
];
