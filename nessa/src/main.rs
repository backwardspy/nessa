use std::{collections::HashSet, fs, path::PathBuf};

use clap::{Parser, ValueEnum};
use color_eyre::Result;
use nessa6502::{cpu::CPU, mem::Bus};
use nessa_rom::ROM;
use speedy2d::{
    color::Color,
    font::{Font, TextLayout, TextOptions},
    shape::Rectangle,
    window::{KeyScancode, VirtualKeyCode, WindowHandler, WindowHelper},
    Graphics2D, Window,
};
use tracing::{error, info};

#[derive(ValueEnum, Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Hack {
    Snake,
    Nestest,
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    rom_file: PathBuf,

    #[arg(long)]
    hack: Vec<Hack>,

    #[arg(short, long, default_value = "false")]
    nowait: bool,
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
    paused: bool,
    hacks: HashSet<Hack>,
    font: Font,
    cpu: CPU,
}

impl EmulatorWindow {
    fn new(cpu: CPU, nowait: bool, font: Font, hacks: HashSet<Hack>) -> Self {
        Self {
            paused: !nowait,
            hacks,
            font,
            cpu,
        }
    }

    fn step(&mut self) -> Result<()> {
        if self.hacks.contains(&Hack::Snake) {
            self.cpu.bus.write(0xFE, rand::random());
        }

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
        if let Some(VirtualKeyCode::Space) = virtual_key_code {
            self.paused = !self.paused;
        };

        if self.paused {
            match virtual_key_code {
                Some(VirtualKeyCode::Comma) => {
                    self.step().expect("step failure");
                }
                Some(VirtualKeyCode::Period) => {
                    for _ in 0..100 {
                        self.step().expect("step failure");
                    }
                }
                _ => (),
            }
        }

        if self.hacks.contains(&Hack::Snake) {
            match virtual_key_code {
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
            };
        }
    }

    fn on_draw(&mut self, helper: &mut WindowHelper, graphics: &mut Graphics2D) {
        graphics.clear_screen(Color::BLACK);

        if !self.paused {
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

        graphics.draw_rectangle(
            Rectangle::from_tuples((0.0, 0.0), (512.0, 512.0)),
            Color::from_hex_rgb(0x1e1e2e),
        );
        for y in 0..32 {
            for x in 0..32 {
                let pixel = self.cpu.bus.read(0x200 + y * 32 + x);
                if pixel > 0 {
                    let color = match pixel {
                        0..=63 => Color::from_rgb(243.0 / 255.0, 139.0 / 255.0, 168.0 / 255.0),
                        64..=127 => Color::from_rgb(166.0 / 255.0, 227.0 / 255.0, 161.0 / 255.0),
                        128..=191 => Color::from_rgb(116.0 / 255.0, 199.0 / 255.0, 236.0 / 255.0),
                        192..=255 => Color::from_rgb(203.0 / 255.0, 166.0 / 255.0, 247.0 / 255.0),
                    };
                    graphics.draw_circle(
                        (x as f32 * 16.0 + 8.0, y as f32 * 16.0 + 8.0),
                        8.0,
                        color,
                    );
                }
            }
        }

        if self.paused {
            let start_text =
                self.font
                    .layout_text("press SPACE to unpause", 16.0, TextOptions::new());
            graphics.draw_text((1.0, 1.0), Color::WHITE, &start_text);
        }

        helper.request_redraw();
    }
}

fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::TRACE)
        .init();

    let args = Args::parse();

    // setup graphical stuff
    let window = Window::new_centered("Nessa", (512, 512))
        .map_err(|e| color_eyre::eyre::eyre!("failed to create window: {}", e.to_string()))?;
    let font = Font::new(include_bytes!("../font/RecMonoDuotone_Regular_1.084.ttf"))
        .map_err(|e| color_eyre::eyre::eyre!("failed to load font: {}", e.to_string()))?;

    // load rom & init emulator
    if !args.rom_file.exists() {
        return Err(color_eyre::eyre::eyre!(
            "the file {:?} does not exist",
            args.rom_file
        ));
    }

    let rom = ROM::from_ines(&fs::read(&args.rom_file)?)?;
    let mut cpu = CPU::new(Bus::new(rom));
    cpu.reset();

    if args.hack.contains(&Hack::Nestest) {
        cpu.reg.pc = 0xC000;
    }

    // run emulator
    window.run_loop(EmulatorWindow::new(
        cpu,
        args.nowait,
        font,
        HashSet::from_iter(args.hack),
    ));
}
