#![allow(dead_code)]

mod cpu;
mod mmu;
use cpu::CPU;
use std::fs::File;
use std::io::{self, Read, Write};
use std::path::Path;

fn main() {
    // Create Components
    let mut cpu = CPU::new();

    // Load ROM to Buffer, then load buffer to memory
    let rom = load_rom("roms\\tests\\01.gb");
    cpu.load_rom(&rom);

    cpu.log_state(0x00, 0x0100);
    println!();
    
    loop {
        cpu.step();
        wait_for_enter();
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

#[cfg(debug_assertions)]
fn printcodes(rom: Vec<u8>) {
    let mut file = File::create("test.txt").expect("Failed to create test file");
    let mut count = 0;
    for code in rom.iter() {
        write!(file, "{:02X} ", code).expect("Failed to write to test file");
        count += 1;
        if count % 16 == 0 {
            writeln!(file).expect("Failed to write newline to test file");
        }
    }
    // Ensure the last line ends with a newline if it didn't already
    if count % 16 != 0 {
        writeln!(file).expect("Failed to write final newline to test file");
    }
}