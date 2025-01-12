pub struct MMU {
    rom: Vec<u8>,       // 0000-7FFF: 32KB ROM
    vram: Vec<u8>,      // 8000-9FFF: 8KB Video RAM (VRAM)
    eram: Vec<u8>,      // A000-BFFF: 8KB External RAM
    wram: Vec<u8>,      // C000-DFFF: 8KB Work RAM (WRAM)
    oam: Vec<u8>,       // FE00-FE9F: Sprite Attribute Table (OAM)
    io_ports: Vec<u8>,  // FF00-FF7F: I/O Ports
    hram: Vec<u8>,      // FF80-FFFE: High RAM (HRAM)
    ie_register: u8,    // FFFF: Interrupt Enable Register

    // Timer Registers
    div: u8,            // Divider Register
    tima: u8,           // Timer Counter
    tma: u8,            // Timer Modulo
    tac: u8,            // Timer Control
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

            div: 0,                     // Initialize Divider Register
            tima: 0,                    // Initialize Timer Counter
            tma: 0,                     // Initialize Timer Modulo
            tac: 0,                     // Initialize Timer Control
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
            0xFF04 => self.div, 
            0xFF05 => self.tima,
            0xFF06 => self.tma,
            0xFF07 => self.tac,
            0xFF08..=0xFF7F => self.io_ports[(address - 0xFF00) as usize],
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
            0xFF04 => self.div = 0, 
            0xFF05 => self.tima = value,
            0xFF06 => self.tma = value,
            0xFF07 => self.tac = value,
            0xFF08..=0xFF7F => self.io_ports[(address - 0xFF00) as usize] = value,
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

    /// Update Timers
    pub fn tick(&mut self, cycles: u32) {
        // Update Divider Register
        self.div = self.div.wrapping_add((cycles / 256) as u8);

        // Update Timer Counter
        if self.tac & 0x04 != 0 {
            let timer_freq = match self.tac & 0x03 {
                0 => 1024,      // 4096 Hz
                1 => 16,        // 262144 Hz
                2 => 64,        // 65536 Hz
                3 => 256,       // 16384 Hz
                _ => unreachable!(),
            };

            if cycles % timer_freq == 0 {
                self.tima = self.tima.wrapping_add(1);
                if self.tima == 0 {
                    self.tima = self.tma;
                    self.request_interrupt(2);
                }
            }
        }
    }

    /// Request an interrupt
    pub fn request_interrupt(&mut self, interrupt: u8) {
        self.io_ports[0x0F] |= 1 << interrupt;
    }
}
