/**************************************************************************************************
* File Name         : main.rs
* Description       : Contains the main entry point of the application
**************************************************************************************************/

pub mod cpu;
pub mod ppu;
pub mod memory;
pub mod opcodes;
pub mod io;
pub mod timers;
pub mod interrupts;
pub mod cartridge;
#[cfg(feature = "debug")]
pub mod debugger;
pub mod utils;

fn main() {
    // Initialize components
    let mut cpu = cpu::CPU::new();
    let mut ppu = ppu::PPU::new();
    let mut memory = memory::Memory::new();
    let mut io = io::IO::new();
    let mut timers = timers::Timers::new();
    let mut interrupts = interrupts::Interrupts::new();
    let cartridge = cartridge::Cartridge::load("path/to/rom.gb");

    // Main emulation loop
    loop {
        cpu.step(&mut memory, &mut ppu, &mut io, &mut timers, &mut interrupts);
        ppu.step(&mut memory);
        timers.step(&mut memory);
        interrupts.step(&mut memory, &mut cpu);
        io.step(&mut memory);
        
        // Debugging
        #[cfg(feature = "debug")]
        debugger::debug(&cpu, &memory, &ppu, &timers, &interrupts, &io);
    }
}
