use std::cell::RefCell;
use std::rc::Rc;
use crate::mmu::{RomMetadata, MMU};

pub struct Bus {
    mmu: MMU,
}

impl Bus {
    pub fn new(mmu: MMU) -> Rc<RefCell<Bus>> {
        Rc::new(RefCell::new(Bus { mmu }))
    }

    pub fn read_byte(&self, address: u16) -> u8 {
        self.mmu.read_byte(address)
    }

    pub fn write_byte(&mut self, address: u16, value: u8) {
        self.mmu.write_byte(address, value);
    }

    pub fn load_rom(&mut self, rom: &Vec<u8>) -> RomMetadata {
        return self.mmu.load_rom(rom);
    }

    pub fn tick(&mut self, cycles: u32) {
        self.mmu.tick(cycles);
    }
}