/**************************************************************************************************
* File Name         : cpu.rs
* Description       : Implementation of CPU including registers and main execution loop
**************************************************************************************************/
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_mut)]
#![allow(unused_imports)]

use rand::Rng;
use crate::{interrupts_timers::{self, Interrupts, Timers}, memory::Memory, opcodes, ppu::PPU, registers::Registers};

// Components for the gameboy CPU
pub struct CPU {
    pub registers: Registers,           // General Putpose Registers reg_a - reg_l
    pub memory: Memory,                 // Memory
    pub pc: u16,                        // Program Counter
    index: u16,                         // Index Register
    sp: u16,                            // Stack Pointer
    stack: [u16; 16],                   // Stack
    opcode: u16,                        // Program Opperation Code
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
        };
        cpu
    }

    pub fn step(&mut self) {
        let opcode = self.memory.read_byte(self.pc);
        opcodes::decode_execute(self, opcode);
    }
}