/**************************************************************************************************
* File Name         : main.rs
* Description       : Contains the main entry point of the application
**************************************************************************************************/
#![allow(unused_variables)]
use std::path::Path;

pub mod emu;
pub mod cpu;
pub mod registers;
pub mod ppu;
pub mod memory;
pub mod opcodes;
pub mod interrupts_timers;
pub mod cartridge;

fn main() {
    let rom_path = std::env::args().nth(1).expect("No ROM provided");
    let path = Path::new(&rom_path);
    let mut emu = emu::Emu::new();

    let _ = emu::Emu::run(&mut emu, &path);
}
