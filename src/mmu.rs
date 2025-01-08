pub struct MMU {
    rom: [u8; 0x8000],      // ROM Area
    vram: [u8; 0x2000],     // VRAM Area
    eram: [u8; 0x2000],     // External RAM Area
    wram: [u8; 0x2000],     // Working RAM Area
    oam: [u8; 0x00A0],      // OAM Area
    io: [u8; 0x0080],       // I/O Area
    hram: [u8; 0x007F],     // HRAM Area
    ie: u8,                 // Interrupt Enable Register
}

impl MMU {
    pub fn new() -> MMU {
        MMU {
            rom: [0; 0x8000],
            vram: [0; 0x2000],
            eram: [0; 0x2000],
            wram: [0; 0x2000],
            oam: [0; 0x00A0],
            io: [0; 0x0080],
            hram: [0; 0x007F],
            ie: 0,
        }
    }

    // Read a byte from memory
    pub fn read_byte(&self, address: u16) -> u8 {
        match address {
            0x000..=0x7FFF => self.rom[address as usize],
            0x8000..=0x9FFF => self.vram[(address - 0x8000) as usize],
            0xA000..=0xBFFF => self.eram[(address - 0xA000) as usize],
            0xC000..=0xDFFF => self.wram[(address - 0xC000) as usize],
            0xE000..=0xFDFF => self.wram[(address - 0xE000) as usize],      // Echo Ram, mirror of C000-DDFF
            0xFE00..=0xFE9F => self.oam[(address - 0xFE00) as usize],
            0xFF00..=0xFF7F => self.io[(address - 0xFF00) as usize],
            0xFF80..=0xFFFE => self.hram[(address - 0xFF80) as usize],
            0xFFFF => self.ie,
            _ => 0,                                                         // Unusable range
        }
    }

    // Write a byte to memory
    pub fn write_byte(&mut self, address: u16, value: u8) {
        match address {
            0x000..=0x7FFF => self.rom[address as usize] = value,
            0x8000..=0x9FFF => self.vram[(address - 0x8000) as usize] = value,
            0xA000..=0xBFFF => self.eram[(address - 0xA000) as usize] = value,
            0xC000..=0xDFFF => self.wram[(address - 0xC000) as usize] = value,
            0xE000..=0xFDFF => self.wram[(address - 0xE000) as usize] = value,  // Echo Ram, mirror of C000-DDFF
            0xFE00..=0xFE9F => self.oam[(address - 0xFE00) as usize] = value,
            0xFF00..=0xFF7F => self.io[(address - 0xFF00) as usize] = value,
            0xFF80..=0xFFFE => self.hram[(address - 0xFF80) as usize] = value,
            0xFFFF => self.ie = value,
            _ => (),                                                            // Unusable range
        }
    }

    // Load ROM into RAM
    pub fn load_rom(&mut self, rom: &Vec<u8>) {
        let rom_size = self.rom.len();
        let copy_size = rom.len().min(rom_size);
        self.rom[..copy_size].copy_from_slice(&rom[..copy_size]);
    }
}