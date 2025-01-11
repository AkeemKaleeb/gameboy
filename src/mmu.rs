pub struct MMU {
    memory: Vec<u8>, 
    /* 
        0000-3FFF: 16KB ROM Bank 00
        4000-7FFF: 16KB ROM Bank 01~NN
        8000-9FFF: 8KB Video RAM (VRAM)
        A000-BFFF: 8KB External RAM
        C000-DFFF: 8KB Work RAM (WRAM)
        E000-FDFF: Mirror of C000~DDFF (ECHO RAM)
        FE00-FE9F: Sprite Attribute Table (OAM)
        FEA0-FEFF: Not Usable
        FF00-FF7F: I/O Ports
        FF80-FFFE: High RAM (HRAM)
        FFFF: Interrupt Enable Register
    */
}

impl MMU {
    pub fn new() -> MMU {
        MMU {
            memory: vec![0; 0x10000], // Initialize 64KB of memory
        }
    }

    /// Read a byte from memory
    pub fn read_byte(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    /// Write a byte to memory
    pub fn write_byte(&mut self, address: u16, value: u8) {
        self.memory[address as usize] = value;
    }

    /// Load ROM into RAM
    pub fn load_rom(&mut self, rom: &Vec<u8>) {
        let rom_size = 0x8000; // ROM size is 32KB
        let copy_size = rom.len().min(rom_size);
        self.memory[..copy_size].copy_from_slice(&rom[..copy_size]);
    }
}
