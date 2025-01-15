#[derive(Copy, Clone, PartialEq)]
pub enum Register {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
    F,
    HL,
    SP,
    PC,
}

#[derive(Copy, Clone, PartialEq)]
pub enum Flag {
    ZERO,
    SUB,
    HC,
    CARRY,
}

pub struct Reg {
    // 8 Bit Registers
    a: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,

    // 16 Bit Registers
    pub pc: u16,
    pub sp: u16,

    // Flags
    pub zero: bool,
    pub sub: bool,
    pub hc: bool,
    pub carry: bool,
}

impl Reg {
    pub fn new() -> Reg {
        Reg {
            a: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            h: 0,
            l: 0,
            pc: 0,
            sp: 0,
            zero: false,
            sub: false,
            hc: false,
            carry: false,
        }
    }

    /// Get the value of a single register
    pub fn get_register(&self, reg: Register) -> u8 {
        match reg {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::H => self.h,
            Register::L => self.l,
            _ => panic!("Invalid register for get_register"),
        }
    }
   
    /// Set the value of a single register
    pub fn set_register(&mut self, reg: Register, value: u8) {
        match reg {
            Register::A => self.a = value,
            Register::B => self.b = value,
            Register::C => self.c = value,
            Register::D => self.d = value,
            Register::E => self.e = value,
            Register::H => self.h = value,
            Register::L => self.l = value,
            _ => panic!("Invalid register for set_register"),
        }
    }

    /// Return the 16 bit value of the given registers
    pub fn get_double_register(&self, high: Register, low: Register) -> u16 {
        return (self.get_register(high) as u16) << 8 | self.get_register(low) as u16;
    }

    /// Set the 16 bit value of the given registers
    pub fn set_double_register(&mut self, high: Register, low: Register, value: u16) {
        self.set_register(high, (value >> 8) as u8);
        self.set_register(low, value as u8);
    }

    /// Get the value of the flags register
    pub fn get_flags(&self) -> u8 {
        let mut flags = 0;
        if self.zero {
            flags |= 0b1000_0000;
        }
        if self.sub {
            flags |= 0b0100_0000;
        }
        if self.hc {
            flags |= 0b0010_0000;
        }
        if self.carry {
            flags |= 0b0001_0000;
        }
        return flags;
    }

    pub fn get_flag(&self, flag: Flag) -> bool {
        match flag {
            Flag::ZERO => self.zero,
            Flag::SUB => self.sub,
            Flag::HC => self.hc,
            Flag::CARRY => self.carry,
        }
    }

    /// Set the value of the flags register
    pub fn set_flags(&mut self, value: u8) {
        self.zero = value & 0b1000_0000 != 0;
        self.sub = value & 0b0100_0000 != 0;
        self.hc = value & 0b0010_0000 != 0;
        self.carry = value & 0b0001_0000 != 0;
    }

    pub fn set_flag(&mut self, flag: Flag, value: bool) {
        match flag {
            Flag::ZERO => self.zero = value,
            Flag::SUB => self.sub = value,
            Flag::HC => self.hc = value,
            Flag::CARRY => self.carry = value,
        }
    }
}