mod cpu;
mod mmu;
use cpu::CPU;
use std::fs::File;
use std::io::Read;
use std::path::Path;

fn main() {
    // Create Components
    let mut cpu = CPU::new();

    // Load ROM to Buffer, then load buffer to memory
    let rom = load_rom("roms\\tests\\ldops.gb");
    cpu.load_rom(&rom);
    
    loop {
        cpu.step();
    }
}

fn load_rom<P: AsRef<Path>>(path: P) -> Vec<u8> {
    let mut file = File::open(path).expect("Failed to open ROM file");
    let mut buffer = Vec::new();
    file.read_to_end(&mut buffer).expect("Failed to read ROM file");
    return buffer;
}