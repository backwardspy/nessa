use std::{fs, path::PathBuf, rc::Rc};

use clap::Parser;
use color_eyre::Result;
use nessa6502::{cpu::CPU, mem::Bus};
use nessa6502_disasm::disassemble_instruction;
use nessa_rom::ROM;
use speedy2d::{
    color::Color,
    font::{Font, FormattedTextBlock, TextLayout, TextOptions},
    shape::Rectangle,
    window::{KeyScancode, VirtualKeyCode, WindowHandler, WindowHelper},
    Graphics2D, Window,
};
use tracing::{error, info, trace};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    rom_file: PathBuf,
}

struct ByteReader<'a> {
    cpu: &'a CPU,
    address: u16,
}

impl Iterator for ByteReader<'_> {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        let byte = self.cpu.bus.read(self.address);
        self.address += 1;
        Some(byte)
    }
}

struct EmulatorWindow {
    started: bool,
    start_text: Rc<FormattedTextBlock>,
    cpu: CPU,
}

impl EmulatorWindow {
    fn new(cpu: CPU, font: Font) -> Self {
        Self {
            started: false,
            start_text: font.layout_text("press SPACE to begin", 16.0, TextOptions::new()),
            cpu,
        }
    }

    fn step(&mut self) -> Result<()> {
        let reader = ByteReader {
            cpu: &self.cpu,
            address: self.cpu.reg.pc,
        };
        let disasm = disassemble_instruction(reader)?;
        trace!("{:04X}: {}", self.cpu.reg.pc, disasm);

        // FIXME: this is just to make snake work
        self.cpu.bus.write(0xFE, rand::random());

        self.cpu.step()?;

        Ok(())
    }
}

impl WindowHandler for EmulatorWindow {
    fn on_key_down(
        &mut self,
        _helper: &mut WindowHelper,
        virtual_key_code: Option<VirtualKeyCode>,
        _scancode: KeyScancode,
    ) {
        match virtual_key_code {
            Some(VirtualKeyCode::Space) => {
                self.started = true;
            }

            // FIXME: this is just to make snake work
            Some(VirtualKeyCode::W) => {
                self.cpu.bus.write(0xFF, 0x77);
            }
            Some(VirtualKeyCode::A) => {
                self.cpu.bus.write(0xFF, 0x61);
            }
            Some(VirtualKeyCode::S) => {
                self.cpu.bus.write(0xFF, 0x73);
            }
            Some(VirtualKeyCode::D) => {
                self.cpu.bus.write(0xFF, 0x64);
            }

            _ => {}
        }
    }

    fn on_draw(&mut self, helper: &mut WindowHelper, graphics: &mut Graphics2D) {
        graphics.clear_screen(Color::BLACK);

        if self.started {
            for _ in 0..100 {
                if let Err(e) = self.step() {
                    error!("step failure: {}", e.to_string());
                    helper.terminate_loop();
                    break;
                }

                if !self.cpu.is_running {
                    info!("CPU halted");
                    helper.terminate_loop();
                    break;
                }
            }
        }

        if self.started {
            for y in 0..32 {
                for x in 0..32 {
                    let pixel = self.cpu.bus.read(0x200 + y * 32 + x);
                    if pixel > 0 {
                        let color = match pixel {
                            0..=63 => Color::from_rgb(243.0 / 255.0, 139.0 / 255.0, 168.0 / 255.0),
                            64..=127 => {
                                Color::from_rgb(166.0 / 255.0, 227.0 / 255.0, 161.0 / 255.0)
                            }
                            128..=191 => {
                                Color::from_rgb(116.0 / 255.0, 199.0 / 255.0, 236.0 / 255.0)
                            }
                            192..=255 => {
                                Color::from_rgb(203.0 / 255.0, 166.0 / 255.0, 247.0 / 255.0)
                            }
                        };
                        graphics.draw_circle(
                            (x as f32 * 16.0 + 8.0, y as f32 * 16.0 + 8.0),
                            8.0,
                            color,
                        );
                    }
                }
            }
        } else {
            graphics.draw_text((1.0, 1.0), Color::WHITE, &self.start_text);
        }

        helper.request_redraw();
    }
}

fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::TRACE)
        .init();

    // setup graphical stuff
    let window = Window::new_centered("Nessa", (512, 512))
        .map_err(|e| color_eyre::eyre::eyre!("failed to create window: {}", e.to_string()))?;
    let font = Font::new(include_bytes!("../font/RecMonoDuotone_Regular_1.084.ttf"))
        .map_err(|e| color_eyre::eyre::eyre!("failed to load font: {}", e.to_string()))?;

    // load rom & init emulator
    let args = Args::parse();
    if !args.rom_file.exists() {
        return Err(color_eyre::eyre::eyre!(
            "the file {:?} does not exist",
            args.rom_file
        ));
    }

    let rom = ROM::from_ines(&fs::read(&args.rom_file)?)?;
    let mut cpu = CPU::new(Bus::new(rom));
    cpu.reset();

    // run emulator
    window.run_loop(EmulatorWindow::new(cpu, font));
}
