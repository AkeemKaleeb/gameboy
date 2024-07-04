/**************************************************************************************************
* File Name         : cpu.rs
* Description       : Implementation of CPU including registers and main execution loop
**************************************************************************************************/

use rand::Rng;
use std::fs::File;
use std::io::Read;
use crate::memory::Memory;
use crate::ppu::PPU;
use crate::io::IO;
use crate::timers::Timers;
use crate::interrupts::Interrupts;

// Screen constants
//const WIDTH: usize = 160;
//const HEIGHT: usize = 144;
//const MAX_SPRITES: u8 = 40;
//const MAX_SPRITES_PER_LINE: u8 = 10;

// Flag register bit positions
const ZERO_FLAG_POSITION: u8 = 7;
const SUB_FLAG_POSITION: u8 = 6;
const HALF_CARRY_FLAG_POSITION: u8 = 5;
const CARRY_FLAG_POSITION: u8 = 4;

#[derive(Debug, Default, Clone, Copy)]
struct FlagsRegister {
    zero: bool,
    subtract: bool,
    half_carry: bool,
    carry: bool,
}

// Implementation of the FlagsRegister
impl FlagsRegister {
    // Function to initialize a new flags register with starting values of false flags
    fn new() -> Self {
        FlagsRegister {
            zero: false,
            subtract: false,
            half_carry: false,
            carry: false,
        }
    }
}

// Converts the flags in the flags register into a single u8 byte
impl std::convert::From<FlagsRegister> for u8 {
    fn from(flag: FlagsRegister) -> u8 {
        (if flag.zero       { 1 } else { 0 }) << ZERO_FLAG_POSITION |
        (if flag.subtract   { 1 } else { 0 }) << SUB_FLAG_POSITION |
        (if flag.half_carry { 1 } else { 0 }) << HALF_CARRY_FLAG_POSITION |
        (if flag.carry      { 1 } else { 0 }) << CARRY_FLAG_POSITION
    }
}

// Converts a single u8 byte into the separate flags in the flags register
impl std::convert::From<u8> for FlagsRegister {
    fn from(byte: u8) -> Self {
        let zero = ((byte >> ZERO_FLAG_POSITION) & 0b1) != 0;
        let subtract = ((byte >> SUB_FLAG_POSITION) & 0b1) != 0;
        let half_carry = ((byte >> HALF_CARRY_FLAG_POSITION) & 0b1) != 0;
        let carry = ((byte >> CARRY_FLAG_POSITION) & 0b1) != 0;

        FlagsRegister {
            zero,
            subtract,
            half_carry,
            carry
        }
    }
}

// General Purpose Registers of the Gameboy
struct Registers {
    reg_a: u8,
    reg_b: u8,
    reg_c: u8,
    reg_d: u8,
    reg_e: u8,
    reg_f: FlagsRegister,
    reg_h: u8,
    reg_l: u8,
}

// Implementation of the general purpose Registers
impl Registers {
    // Function to initialize the registers all to zero
    fn new() -> Self {
        Registers {
            reg_a: 0,
            reg_b: 0,
            reg_c: 0,
            reg_d: 0,
            reg_e: 0,
            reg_f: FlagsRegister::new(),
            reg_h: 0,
            reg_l: 0,
        }
    }

    // Functions to get and set combined, 2-byte registers AF, BC, DE, HL
    // Uses bit manipulation to move the first 8 bits to the front of the 16 bit combined registers
    // Or's the value with the remaining 8 bits of the second half of the value
    // FFFF 0000 | 0000 FFFF = FFFF FFFF

    fn get_af(&self) -> u16 {
        (self.reg_a as u16) << 8
        | u8::from(self.reg_f) as u16
    }
    fn set_af(&mut self, value: u16) {
        self.reg_a = ((value & 0xFF00) >> 8) as u8;
        self.reg_f = FlagsRegister::from((value & 0xFF) as u8);
    }
    fn get_bc(&self) -> u16 {
        (self.reg_b as u16) << 8
        | self.reg_c as u16
    }
    fn set_bc(&mut self, value: u16) {
        self.reg_b = ((value & 0xFF00) >> 8) as u8;
        self.reg_c = (value & 0xFF) as u8;
    }
    fn get_de(&self) -> u16 {
        (self.reg_d as u16) << 8
        | self.reg_e as u16
    }
    fn set_de(&mut self, value: u16) {
        self.reg_d = ((value & 0xFF00) >> 8) as u8;
        self.reg_e = (value & 0xFF) as u8;
    }
    fn get_hl(&self) -> u16 {
        (self.reg_h as u16) << 8
        | self.reg_l as u16
    }
    fn set_hl(&mut self, value: u16) {
        self.reg_h = ((value & 0xFF00) >> 8) as u8;
        self.reg_l = (value & 0xFF) as u8;
    }
}

// Components for the gameboy CPU
pub struct CPU {
    registers: Registers,               // General Putpose Registers reg_a - reg_l
    index: u16,                         // Index Register
    pc: u16,                            // Program Counter
    sp: u16,                            // Stack Pointer
    stack: [u16; 16],                   // Stack
    memory: [u8; 8192],                 // Memory
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
            pc: 0x200,
            sp: 0,
            stack: [0; 16],
            memory: [0; 8192],
            opcode: 0,
            //display: [0; WIDTH * HEIGHT],
            //key: [0; 16],
            //draw_flag: false,
        };
        cpu
    }

    pub fn load_rom(&mut self, path: &str) -> Result<(), std::io::Error> {
        let mut file = File::open(path)?;     // Open File in Binary Mode
        let mut buffer: Vec<u8> = Vec::new();       // Create buffer of bytes   
        file.read_to_end(&mut buffer)?;        // Read file into buffer

        for (i, &byte) in buffer.iter().enumerate() {
            if i + 512 < self.memory.len() {
                self.memory[i + 512] = byte;
            } else {
                eprintln!("ROM is too large to fit in memory.");
                break;
            }
        }
        Ok(())
    }

    pub fn step(&mut self, memory: &mut Memory, ppu: &mut PPU, io: &mut IO, timers: &mut Timers, interrupts: &mut Interrupts) {
        // Fetch, decode, and execute instructions
    }
}