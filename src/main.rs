#![allow(dead_code)]

extern crate sdl2;

mod cpu;
mod mmu;
mod ppu;

use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::time::Duration;
use cpu::CPU;
use ppu::PPU;
use std::fs::File;
use std::io::{self, Read, Write};
use std::path::Path;


fn main() {
    // Initialize SDL2
    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();

    let scale_factor = 4;
    let width = 160 * scale_factor;
    let height = 144 * scale_factor;

    let window = video_subsystem.window("Gameboy Emulator", width, height)
        .position_centered()
        .resizable()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().build().unwrap();

    canvas.set_draw_color(Color::RGB(0, 0, 0));
    canvas.clear();
    canvas.present();

    let mut event_pump = sdl_context.event_pump().unwrap();    
    
    // Create Components
    let mut cpu = CPU::new();
    let mut ppu = PPU::new();

    // Load ROM to Buffer, then load buffer to memory
    //let rom = load_rom("roms\\tests\\01.gb");
    let rom = vec![0x00; 0x0100];
    cpu.load_rom(&rom);

    // cpu.log_state(0x00, 0x0100);
    // println!();
    
    'running: loop {
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} |
                Event::KeyDown { keycode: Some(Keycode::Escape), .. }=> {
                    break 'running
                },
                Event::Window { win_event, .. } => {
                    match win_event {
                        sdl2::event::WindowEvent::Resized(width, height) => {
                            canvas.set_logical_size(width as u32, height as u32).unwrap();
                        },
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        //cpu.step();
        ppu.update(1);
        ppu.render(&mut canvas);

        canvas.present();

        ::std::thread::sleep(Duration::from_millis(1000 / 60));
    }
}


fn load_rom<P: AsRef<Path>>(path: P) -> Vec<u8> {
    let mut file = File::open(path).expect("Failed to open ROM file");
    let mut buffer = Vec::new();
    file.read_to_end(&mut buffer).expect("Failed to read ROM file");
    return buffer;
}

#[cfg(debug_assertions)]
fn wait_for_enter() {
    let mut input = String::new();
    io::stdout().flush().unwrap(); // Ensure the prompt is printed before waiting for input
    io::stdin().read_line(&mut input).unwrap();
}