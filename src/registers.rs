/**************************************************************************************************
* File Name         : registers.rs
* Description       : Contains general purpose registers and their implementations
**************************************************************************************************/

// Flag register bit positions
const ZERO_FLAG_POSITION: u8 = 7;
const SUB_FLAG_POSITION: u8 = 6;
const HALF_CARRY_FLAG_POSITION: u8 = 5;
const CARRY_FLAG_POSITION: u8 = 4;

#[derive(Debug, Default, Clone, Copy)]
pub struct FlagsRegister {
    pub zero: bool,
    pub subtract: bool,
    pub half_carry: bool,
    pub carry: bool,
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
pub struct Registers {
    pub reg_a: u8,
    pub reg_b: u8,
    pub reg_c: u8,
    pub reg_d: u8,
    pub reg_e: u8,
    pub reg_f: FlagsRegister,
    pub reg_h: u8,
    pub reg_l: u8,
}

// Implementation of the general purpose Registers
impl Registers {
    // Function to initialize the registers all to zero
    pub fn new() -> Self {
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

    pub fn get_af(&self) -> u16 {
        (self.reg_a as u16) << 8
        | u8::from(self.reg_f) as u16
    }
    pub fn set_af(&mut self, value: u16) {
        self.reg_a = ((value & 0xFF00) >> 8) as u8;
        self.reg_f = FlagsRegister::from((value & 0xFF) as u8);
    }
    pub fn get_bc(&self) -> u16 {
        (self.reg_b as u16) << 8
        | self.reg_c as u16
    }
    pub fn set_bc(&mut self, value: u16) {
        self.reg_b = ((value & 0xFF00) >> 8) as u8;
        self.reg_c = (value & 0xFF) as u8;
    }
    pub fn get_de(&self) -> u16 {
        (self.reg_d as u16) << 8
        | self.reg_e as u16
    }
    pub fn set_de(&mut self, value: u16) {
        self.reg_d = ((value & 0xFF00) >> 8) as u8;
        self.reg_e = (value & 0xFF) as u8;
    }
    pub fn get_hl(&self) -> u16 {
        (self.reg_h as u16) << 8
        | self.reg_l as u16
    }
    pub fn set_hl(&mut self, value: u16) {
        self.reg_h = ((value & 0xFF00) >> 8) as u8;
        self.reg_l = (value & 0xFF) as u8;
    }
}
