use std::io::{self, Write};
use std::cell::RefCell;
use std::rc::Rc;

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

pub struct RomMetadata {
    pub title: String,
    pub manufacturer_code: String,
    pub cgb_flag: u8,
    pub new_licensee_code: String,
    pub sgb_flag: u8,
    pub cartridge_type: u8,
    pub rom_size: u8,
    pub ram_size: u8,
    pub destination_code: u8,
    pub old_licensee_code: u8,
    pub version_number: u8,
    pub header_checksum: u8,
    pub global_checksum: u16,
}

impl MMU {
    const JOYP: u16 = 0xFF00;
    const SB: u16 = 0xFF01;
    const SC: u16 = 0xFF02;
    const DIV: u16 = 0xFF04;
    const TIMA: u16 = 0xFF05;
    const TMA: u16 = 0xFF06;
    const TAC: u16 = 0xFF07;
    const IF: u16 = 0xFF0F;
    
    pub fn new() -> Rc<RefCell<MMU>> {
        Rc::new(RefCell::new(MMU {
            rom: vec![0; 0x8000],       // Initialize 32KB of ROM
            vram: vec![0; 0x2000],      // Initialize 8KB of VRAM
            eram: vec![0; 0x2000],      // Initialize 8KB of External RAM
            wram: vec![0; 0x2000],      // Initialize 8KB of Work RAM
            oam: vec![0; 0xA0],         // Initialize Sprite Attribute Table
            io_ports: vec![0; 0x80],    // Initialize I/O Ports
            hram: vec![0; 0x7F],        // Initialize High RAM
            ie_register: 0,             // Initialize Interrupt Enable Register
        }))
    }

    pub fn do_step(&mut self, ticks: u8) -> u8 {
        self.tick(1);
        1
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
            0xFF00..=0xFF7F => {
                self.io_ports[(address - 0xFF00) as usize] = value;
                if address == Self::SC && value == 0x81 {
                    self.handle_serial_transfer();
                }
            }    
            0xFF80..=0xFFFE => self.hram[(address - 0xFF80) as usize] = value,
            0xFFFF => self.ie_register = value,
            _ => {}, // Not usable or mirrored regions
        }
    }

    pub fn write_word(&mut self, address: u16, value: u16) {
        let low_byte = value as u8;
        let high_byte = (value >> 8) as u8;
        self.write_byte(address, low_byte);
        self.write_byte(address + 1, high_byte);
    }

    pub fn read_word(&self, address: u16) -> u16 {
        let low_byte = self.read_byte(address) as u16;
        let high_byte = self.read_byte(address + 1) as u16;
        return (high_byte << 8) | low_byte;
    }

    /// Load ROM into RAM
    pub fn load_rom(&mut self, rom: &Vec<u8>) -> RomMetadata {
        let rom_size = 0x8000; // ROM size is 32KB
        let copy_size = rom.len().min(rom_size);
        self.rom[..copy_size].copy_from_slice(&rom[..copy_size]);

        // Parse ROM metadata
        let mut metadata = RomMetadata::new();
        metadata.title = String::from_utf8_lossy(&rom[0x0134..0x0143]).to_string();
        metadata.manufacturer_code = String::from_utf8_lossy(&rom[0x013F..0x0143]).to_string();
        metadata.cgb_flag = rom[0x0143];
        metadata.new_licensee_code = String::from_utf8_lossy(&rom[0x0144..0x0145]).to_string();
        metadata.sgb_flag = rom[0x0146];
        metadata.cartridge_type = rom[0x0147];
        metadata.rom_size = rom[0x0148];
        metadata.ram_size = rom[0x0149];
        metadata.destination_code = rom[0x014A];
        metadata.old_licensee_code = rom[0x014B];
        metadata.version_number = rom[0x014C];
        metadata.header_checksum = rom[0x014D];
        metadata.global_checksum = ((rom[0x014E] as u16) << 8) | (rom[0x014F] as u16);

        // Load Initial Memory Values
        self.write_byte(0xFF05, 0x00);  // TIMA
        self.write_byte(0xFF06, 0x00);  // TMA
        self.write_byte(0xFF07, 0x00);  // TAC
        self.write_byte(0xFF10, 0x80);  // NR10
        self.write_byte(0xFF11, 0xBF);  // NR11
        self.write_byte(0xFF12, 0xF3);  // NR12
        self.write_byte(0xFF14, 0xBF);  // NR14
        self.write_byte(0xFF16, 0x3F);  // NR21
        self.write_byte(0xFF17, 0x00);  // NR22
        self.write_byte(0xFF19, 0xBF);  // NR24
        self.write_byte(0xFF1A, 0x7F);  // NR30
        self.write_byte(0xFF1B, 0xFF);  // NR31
        self.write_byte(0xFF1C, 0x9F);  // NR32
        self.write_byte(0xFF1E, 0xBF);  // NR33
        self.write_byte(0xFF20, 0xFF);  // NR41
        self.write_byte(0xFF21, 0x00);  // NR42
        self.write_byte(0xFF22, 0x00);  // NR43
        self.write_byte(0xFF23, 0xBF);  // NR44
        self.write_byte(0xFF24, 0x77);  // NR50
        self.write_byte(0xFF25, 0xF3);  // NR51
        self.write_byte(0xFF26, 0xF1);  // NR52 - GB, 0xF0 - SGB
        self.write_byte(0xFF40, 0x91);  // LCDC
        self.write_byte(0xFF42, 0x00);  // SCY
        self.write_byte(0xFF43, 0x00);  // SCX
        self.write_byte(0xFF45, 0x00);  // LYC
        self.write_byte(0xFF47, 0xFC);  // BGP
        self.write_byte(0xFF48, 0xFF);  // OBP0
        self.write_byte(0xFF49, 0xFF);  // OBP1
        self.write_byte(0xFF4A, 0x00);  // WY
        self.write_byte(0xFF4B, 0x00);  // WX
        self.write_byte(0xFFFF, 0x00);  // IE

        metadata
    }

    /// Update Timers
    pub fn tick(&mut self, cycles: u32) {
        // Update Divider Register
        self.write_byte(Self::DIV, self.read_byte(Self::DIV).wrapping_add((cycles / 256) as u8));

        // Update Timer Counter
        if self.read_byte(Self::TAC) & 0x04 != 0 {
            let timer_freq = match Self::TAC & 0x03 {
                0 => 1024,      // 4096 Hz
                1 => 16,        // 262144 Hz
                2 => 64,        // 65536 Hz
                3 => 256,       // 16384 Hz
                _ => unreachable!(),
            };

            if cycles % timer_freq == 0 {
                self.write_byte(Self::TIMA, self.read_byte(Self::TIMA.wrapping_add(1)));
                if self.read_byte(Self::TIMA) == 0 {
                    self.write_byte(Self::TIMA, self.read_byte(Self::TMA));
                    self.get_interrupts();
                }
            }
        }
    }

    /// Request an interrupt
    pub fn get_interrupts(&mut self) {
        // TODO: Implement
    }

    fn handle_serial_transfer(&self) {
        print!("{}", self.read_byte(Self::SB) as char);
        io::stdout().flush().unwrap();
        std::process::exit(0);
    }
}

impl RomMetadata {
    pub fn new() -> RomMetadata {
        RomMetadata {
            title: String::new(),
            manufacturer_code: String::new(),
            cgb_flag: 0,
            new_licensee_code: String::new(),
            sgb_flag: 0,
            cartridge_type: 0,
            rom_size: 0,
            ram_size: 0,
            destination_code: 0,
            old_licensee_code: 0,
            version_number: 0,
            header_checksum: 0,
            global_checksum: 0,
        }
    }
}
