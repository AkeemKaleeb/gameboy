#![allow(dead_code)]

extern crate sdl2;

mod cpu;
mod mmu;
mod ppu;
mod bus;

use sdl2::pixels::Color;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::time::Duration;
use bus::Bus;
use cpu::CPU;
use ppu::PPU;
use mmu::MMU;
use std::fs::File;
use std::io::{self, Read, Write};
use std::path::Path;
use std::cell::RefCell;
use std::rc::Rc;


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
    let mmu: MMU = MMU::new();
    let bus: Rc<RefCell<Bus>> = Bus::new(mmu);
    let mut ppu: PPU = PPU::new(bus.clone(), scale_factor);
    let mut cpu: CPU = CPU::new(bus);

    // Load ROM to Buffer, then load buffer to memory
    let rom = read_rom("roms\\tests\\09.gb");
    let metadata = cpu.load_rom(&rom);
    //print_metadata(&metadata);
    
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
                            let new_width = (width as f32 / scale_factor as f32).round() as u32;
                            let new_height = (height as f32 / scale_factor as f32).round() as u32;
                            canvas.set_logical_size(new_width, new_height).unwrap();
                        },
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        cpu.step();
        wait_for_enter();
        //ppu.update(1, &mut canvas);
        //canvas.present();

        ::std::thread::sleep(Duration::from_millis(1000 / 60));
    }
}


fn read_rom<P: AsRef<Path>>(path: P) -> Vec<u8> {
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

fn print_metadata(metadata: &mmu::RomMetadata) {
    println!("Title: {}", metadata.title);
    println!("Manufacturer Code: {}", metadata.manufacturer_code);
    println!("CGB Flag: {}", metadata.cgb_flag);
    println!("New Licensee Code: {}", metadata.new_licensee_code);
    println!("SGB Flag: {}", metadata.sgb_flag);
    println!("Cartridge Type: {}", metadata.cartridge_type);
    println!("ROM Size: {}", metadata.rom_size);
    println!("RAM Size: {}", metadata.ram_size);
    println!("Destination Code: {}", metadata.destination_code);
    println!("Old Licensee Code: {}", metadata.old_licensee_code);
    println!("Version Number: {}", metadata.version_number);
    println!("Header Checksum: {}", metadata.header_checksum);
    println!("Global Checksum: {}", metadata.global_checksum);
}