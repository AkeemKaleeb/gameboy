pub struct MMU {
    rom: Vec<u8>,       // 0000-7FFF: 32KB ROM
    vram: Vec<u8>,      // 8000-9FFF: 8KB Video RAM (VRAM)
    eram: Vec<u8>,      // A000-BFFF: 8KB External RAM
    wram: Vec<u8>,      // C000-DFFF: 8KB Work RAM (WRAM)
    oam: Vec<u8>,       // FE00-FE9F: Sprite Attribute Table (OAM)
    io_ports: Vec<u8>,  // FF00-FF7F: I/O Ports
    hram: Vec<u8>,      // FF80-FFFE: High RAM (HRAM)
    ie_register: u8,    // FFFF: Interrupt Enable Register
}

impl MMU {
    pub fn new() -> MMU {
        MMU {
            rom: vec![0; 0x8000],       // Initialize 32KB of ROM
            vram: vec![0; 0x2000],      // Initialize 8KB of VRAM
            eram: vec![0; 0x2000],      // Initialize 8KB of External RAM
            wram: vec![0; 0x2000],      // Initialize 8KB of Work RAM
            oam: vec![0; 0xA0],         // Initialize Sprite Attribute Table
            io_ports: vec![0; 0x80],    // Initialize I/O Ports
            hram: vec![0; 0x7F],        // Initialize High RAM
            ie_register: 0,             // Initialize Interrupt Enable Register
        }
    }

    /// Read a byte from memory
    pub fn read_byte(&self, address: u16) -> u8 {
        match address {
            0x0000..=0x7FFF => self.rom[address as usize],
            0x8000..=0x9FFF => self.vram[(address - 0x8000) as usize],
            0xA000..=0xBFFF => self.eram[(address - 0xA000) as usize],
            0xC000..=0xDFFF => self.wram[(address - 0xC000) as usize],
            0xFE00..=0xFE9F => self.oam[(address - 0xFE00) as usize],
            0xFF00..=0xFF7F => self.io_ports[(address - 0xFF00) as usize],
            0xFF80..=0xFFFE => self.hram[(address - 0xFF80) as usize],
            0xFFFF => self.ie_register,
            _ => 0, // Not usable or mirrored regions
        }
    }

    /// Write a byte to memory
    pub fn write_byte(&mut self, address: u16, value: u8) {
        match address {
            0x0000..=0x7FFF => self.rom[address as usize] = value,
            0x8000..=0x9FFF => self.vram[(address - 0x8000) as usize] = value,
            0xA000..=0xBFFF => self.eram[(address - 0xA000) as usize] = value,
            0xC000..=0xDFFF => self.wram[(address - 0xC000) as usize] = value,
            0xFE00..=0xFE9F => self.oam[(address - 0xFE00) as usize] = value,
            0xFF00..=0xFF7F => self.io_ports[(address - 0xFF00) as usize] = value,
            0xFF80..=0xFFFE => self.hram[(address - 0xFF80) as usize] = value,
            0xFFFF => self.ie_register = value,
            _ => {}, // Not usable or mirrored regions
        }
    }

    /// Load ROM into RAM
    pub fn load_rom(&mut self, rom: &Vec<u8>) {
        let rom_size = 0x8000; // ROM size is 32KB
        let copy_size = rom.len().min(rom_size);
        self.rom[..copy_size].copy_from_slice(&rom[..copy_size]);
        //println!("ROM loaded: {:?}", &self.rom[..rom_size]);
    }
}
