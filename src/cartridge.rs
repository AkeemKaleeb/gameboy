/**************************************************************************************************
* File Name         : cartridge.rs
* Description       : Handles the loading and management of game cartridges (ROMs)
**************************************************************************************************/
#![allow(unused_variables)]

use std::{fs::File, io::Read, path::Path};

use crate::memory;

pub struct Cartridge {
    entry: u8,
    logo: u8,
    title: [char; 16],
    new_lic_code: u16,
    sgb_flag: u8,
    rom_type: u8,
    rom_size: usize,
    ram_size: u8,
    dest_code: u8,
    lic_code: u8,
    version: u8,
    checksum: u8,
    global_checksum: u16,
}

impl Cartridge {
    pub fn new() -> Self {
        let mut cartridge = Cartridge {
            entry: 0,
            logo: 0,
            title: [' '; 16],
            new_lic_code: 0,
            sgb_flag: 0,
            rom_type: 0,
            rom_size: 0,
            ram_size: 0,
            dest_code: 0,
            lic_code: 0,
            version: 0,
            checksum: 0,
            global_checksum: 0,
        };
        cartridge
    }
    pub fn load(&mut self, memory: &mut memory::Memory, path: &Path) -> Result<(), std::io::Error> {
        let mut file = File::open(path)?;                       // Open File in Binary Mode
        let mut buffer: Vec<u8> = Vec::new();                         // Create buffer of bytes   
        self.rom_size = file.read_to_end(&mut buffer)?;     // Read file into buffer



        Ok(())
    }
}