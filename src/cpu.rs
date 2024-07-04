/**************************************************************************************************
* File Name         : cpu.rs
* Description       : Implementation of CPU including registers and main execution loop
**************************************************************************************************/
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_mut)]
#![allow(unused_imports)]

use rand::Rng;
use crate::registers::Registers;
use crate::memory::Memory;
use crate::{interrupts_timers, opcodes};
use crate::ppu::PPU;
use crate::io::IO;
use crate::interrupts_timers::Interrupts;
use crate::interrupts_timers::Timers;


// Components for the gameboy CPU
pub struct CPU {
    pub registers: Registers,           // General Putpose Registers reg_a - reg_l
    pub memory: Memory,                 // Memory
    index: u16,                         // Index Register
    pc: u16,                            // Program Counter
    sp: u16,                            // Stack Pointer
    stack: [u16; 16],                   // Stack
    opcode: u16,                        // Program Opperation Code
    //display: [u8; WIDTH * HEIGHT],      // Display
    //key:[u8; 16],                       // Input keys
    //draw_flag: bool,                    // Determine whether or not to update screen
}

// Implementation for the CPU
impl CPU {
    // Initialization function returning default values for the CPU
    pub fn new() -> Self {
        let mut cpu = CPU {
            registers: Registers::new(),
            index: 0,
            pc: 0x100,
            sp: 0,
            stack: [0; 16],
            memory: Memory::new(),
            opcode: 0,
            //display: [0; WIDTH * HEIGHT],
            //key: [0; 16],
            //draw_flag: false,
        };
        cpu
    }

    pub fn step(&mut self, memory: &mut Memory, ppu: &mut PPU, io: &mut IO, timers: &mut Timers, interrupts: &mut Interrupts) {
        let opcode = self.fetch_byte();
        crate::opcodes::decode_execute(self, opcode);
    }

    pub fn fetch_byte(&mut self) -> u8 {
        let byte = self.memory.read_byte(self.pc);
        self.pc += 1;
        byte
    }

    pub fn fetch_word(&mut self) -> u16 {
        let low_byte = self.fetch_byte() as u16;
        let high_byte = self.fetch_byte() as u16;
        (high_byte << 8) | low_byte
    }
}