/**************************************************************************************************
* File Name         : memory.rs
* Description       : Manages the Gameboy's memory including RAM, ROM, and I/O
**************************************************************************************************/

// Memory map and state
pub struct Memory {
    pub data: [u8; 0xFFFF],
}

impl Memory {
    // Initialize memory
    pub fn new() -> Self {
        Memory {
            data: [0; 0xFFFF],
        }
    }

    // Read and return the data byte at provided memory address
    pub fn read_byte(&self, address:u16) -> u8 {
        self.data[address as usize]
    }

    // Write a byte of data to memory
    pub fn write_byte(&mut self, address: u16, value: u8) {
        self.data[address as usize] = value
    }

    // Read a two byte word
    pub fn read_word(& self, address: u16) -> u16 {
        let low = self.read_byte(address) as u16;
        let high = self.read_byte(address + 1) as u16;
        (high << 8) | low
    }

    pub fn write_word(&mut self, address: u16, value: u16) {
        self.write_byte(address, (value & 0xFF) as u8);
        self.write_byte(address + 1, (value >> 8) as u8);
    }
}