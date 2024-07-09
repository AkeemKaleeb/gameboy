/**************************************************************************************************
* File Name         : emu.rs
* Description       : Contains process for starting the emulator and running from code
**************************************************************************************************/

use sdl2;
use sdl2::sys::SDL_Delay;
use sdl2::sys::SDL_Init;
use sdl2::pixels::Color;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::rect::Rect;
use std::path::Path;

use crate::cpu;
use crate::ppu;
use crate::memory;
use crate::interrupts_timers;
use crate::cartridge;

const WIDTH: usize = 160;
const HEIGHT: usize = 144;
const SCALE: usize = 5;

pub struct Emu {
    cpu: cpu::CPU,
    ppu: ppu::PPU,
    memory: memory::Memory,
    timers: interrupts_timers::Timers,
    interrupts: interrupts_timers::Interrupts,
    context: EmuContext,
}

struct EmuContext {
    is_paused: bool,
    is_running: bool,
    ticks: u64,
}

impl Emu {
    pub fn new() -> Self {
        let mut emu: Emu = Emu{
            cpu: cpu::CPU::new(),
            ppu: ppu::PPU::new(),
            memory: memory::Memory::new(),
            timers: interrupts_timers::Timers::new(),
            interrupts: interrupts_timers::Interrupts::new(),
            context: EmuContext::new(),
        };
        emu
    }

    // Emulation Entry Point
    pub fn run(&mut self, cartridge_path: &Path) -> Result<(), String> {
        let mut cartridge = cartridge::Cartridge::new();
        if let Err(err) = cartridge::Cartridge::load(&mut cartridge, &mut self.memory, &cartridge_path) {
            eprintln!("{}", err);
            return Err(err.to_string());
        }
        println!("ROM loaded successfully");


        // Video Render
        // Initialize SDL2 and the video system as well as create a blank screen to be drawn on top of by the PPU
        // Also collect events from the user to be handled in the emulation loop
        let sdl_context = sdl2::init()?;
        let video_subsystem = sdl_context.video()?;
        let window = video_subsystem.window("Gameboy Emu", (WIDTH * SCALE) as u32, (HEIGHT * SCALE) as u32)
            .position_centered()
            .build()
            .expect("could not initialize video subsystem");
        let mut canvas = window.into_canvas().build()
            .expect("could not make a canvas");
        canvas.set_draw_color(Color::RGB(0, 0, 0));
        canvas.clear();
        canvas.present();
        let mut event_pump = sdl_context.event_pump()?;
        
        // Set context variables and begin running emulation
        self.context.is_running = true;
        self.context.is_paused = false;
        self.context.ticks = 0;

        // Main emulation loop
        while self.context.is_running {
            for event in event_pump.poll_iter() {
                match event {
                    Event::Quit { .. } => { self.context.is_running = false; }
                    Event::KeyDown { keycode: Some(Keycode::Escape), .. } => { self.context.is_running = false; }
                    Event::KeyDown { keycode: Some(Keycode::P), .. } => { self.context.is_paused = !self.context.is_paused; }
                    _ => {}
                }
            }

            if !self.context.is_paused {
                //self.cpu.step(&mut self.memory, &mut self.ppu, &mut self.timers, &mut self.interrupts);
                //self.ppu.step(&mut self.memory);
                //self.timers.step(&mut self.memory);
                //self.interrupts.step(&mut self.memory, &mut self.cpu);

                self.context.ticks += 1;

                canvas.set_draw_color(Color::RGB(0, 0, 0));
                canvas.clear();
                canvas.present();
            }

            ::std::thread::sleep(std::time::Duration::from_millis(16));
        }
        Ok(())
    }
}

impl EmuContext {
    pub fn new() -> Self {
        EmuContext {
            is_paused: false,
            is_running: false,
            ticks: 0,
        }
    }
}