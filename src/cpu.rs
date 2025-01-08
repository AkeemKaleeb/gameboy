#![allow(dead_code)]
use crate::mmu::MMU;

#[derive(Copy, Clone)]
pub enum Register {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
    HL,
    SP,
    PC,
    ZERO,
    SUB,
    HC,
    CARRY,
}

pub struct CPU {
    // 8 Bit Registers
    reg_a: u8,
    reg_b: u8,
    reg_c: u8,
    reg_d: u8,
    reg_e: u8,
    reg_h: u8,
    reg_l: u8,

    // 16 Bit Registers
    sp: u16,
    pc: u16,

    // Flags
    zero: bool,
    sub: bool,
    hc: bool,
    carry: bool,

    // Memory Management Unit
    mmu: MMU,
}

impl CPU {
    pub fn new() -> CPU {
        CPU {
            reg_a: 0,
            reg_b: 0,
            reg_c: 0,
            reg_d: 0,
            reg_e: 0,
            reg_h: 0,
            reg_l: 0,
            sp: 0,
            pc: 0x0100,         // Gameboy starting address
            zero: false,
            sub: false,
            hc: false,
            carry: false,
            mmu: MMU::new(),
        }
    }

    // Read 8 bit value from register
    pub fn read_register(&self, reg: Register) -> u8 {
        match reg {
            Register::A => self.reg_a,
            Register::B => self.reg_b,
            Register::C => self.reg_c,
            Register::D => self.reg_d,
            Register::E => self.reg_e,
            Register::H => self.reg_h,
            Register::L => self.reg_l,
            Register::HL => panic!("HL is a 16-bit register"),
            Register::SP => panic!("SP is a 16-bit register"),
            Register::PC => panic!("PC is a 16-bit register"),
            _ => panic!("Invalid register"),
        }
    }

    // Read the flag boolean value
    pub fn read_flag(&self, flag: Register) -> bool {
        match flag {
            Register::ZERO => self.zero,
            Register::SUB => self.sub,
            Register::HC => self.hc,
            Register::CARRY => self.carry,
            _ => panic!("Invalid flag register"),
        }
    }
    
    // Write 8 bit value to register
    pub fn write_register(&mut self, reg: Register, value: u8) {
        match reg {
            Register::A => self.reg_a = value,
            Register::B => self.reg_b = value,
            Register::C => self.reg_c = value,
            Register::D => self.reg_d = value,
            Register::E => self.reg_e = value,
            Register::H => self.reg_h = value,
            Register::L => self.reg_l = value,
            Register::HL => panic!("HL is a 16-bit register"),
            Register::SP => panic!("SP is a 16-bit register"),
            Register::PC => panic!("PC is a 16-bit register"),
            Register::ZERO => self.zero = value != 0,
            Register::SUB => self.sub = value != 0,
            Register::HC => self.hc = value != 0,
            Register::CARRY => self.carry = value != 0,
        }
    }
    
    // Write flag boolean value
    pub fn write_flag(&mut self, flag: Register, value: bool) {
        match flag {
            Register::ZERO => self.zero = value,
            Register::SUB => self.sub = value,
            Register::HC => self.hc = value,
            Register::CARRY => self.carry = value,
            _ => panic!("Invalid flag register"),
        }
    }

    // Read 8 bit value from memory at address
    fn read_memory(&self, address: u16) -> u8 {
        return self.mmu.read_byte(address);
    }

    // Write 8 bit value to memory at address
    fn write_memory(&mut self, address: u16, value: u8) {
        self.mmu.write_byte(address, value);
    }

    // Public instruction to send ROM to MMU
    pub fn load_rom(&mut self, rom: &Vec<u8>) {
        self.mmu.load_rom(rom);
    }

    // Function to Run ROM by instruction
    pub fn step(&mut self) {
        let opcode = self.read_memory(self.pc);
        self.execute(opcode);
    }

    // Get OPCODE and execute the appropriate function
    pub fn execute(&mut self, opcode: u8) {
        println!("\nExecuting Opcode: {:02X}", opcode);
        match opcode & 0xF0 {
            0x00 => match opcode & 0x0F {
                0x00 => self.nop(),
                0x01 => self.ldi2(Register::B, Register::C),
                0x02 => self.ldi2(Register::B, Register::C),
                0x03 => self.inc(Register::B, Some(Register::C)),
                0x04 => self.inc(Register::B, None),
                0x05 => self.dec(Register::B, None),
                0x06 => panic!("Opcode not implemented"),
                0x07 => panic!("Opcode not implemented"),
                0x08 => panic!("Opcode not implemented"),
                0x09 => panic!("Opcode not implemented"),
                0x0A => self.lda(Register::B, Register::C, false, false),
                0x0B => self.dec(Register::B, Some(Register::C)),
                0x0C => self.inc(Register::C, None),
                0x0D => self.dec(Register::C, None),
                0x0E => self.ldi1(Register::C),
                0x0F => self.rrca(),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },
            0x10 => match opcode & 0x0F {
                0x00 => panic!("Opcode not implemented"),
                0x01 => self.ldi2(Register::D, Register::E),
                0x02 => panic!("Opcode not implemented"),
                0x03 => self.inc(Register::D, Some(Register::E)),
                0x04 => self.inc(Register::D, None),
                0x05 => self.dec(Register::D, None),
                0x06 => panic!("Opcode not implemented"),
                0x07 => panic!("Opcode not implemented"),
                0x08 => self.jr(),
                0x09 => panic!("Opcode not implemented"),
                0x0A => self.lda(Register::D, Register::E, false, false),
                0x0B => self.dec(Register::D, Some(Register::E)),
                0x0C => self.inc(Register::E, None),
                0x0D => self.dec(Register::E, None),
                0x0E => self.ldi1(Register::E),
                0x0F => self.rra(),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },
            0x20 => match opcode & 0x0F {
                0x00 => self.jrnz(),
                0x01 => self.ldi2(Register::H, Register::L),
                0x02 => self.jrnz(),
                0x03 => self.inc(Register::H, Some(Register::L)),
                0x04 => self.inc(Register::H, None),
                0x05 => self.dec(Register::H, None),
                0x06 => panic!("Opcode not implemented"),
                0x07 => panic!("Opcode not implemented"),
                0x08 => self.jrz(),
                0x09 => panic!("Opcode not implemented"),
                0x0A => self.lda(Register::H, Register::L, true, false),
                0x0B => self.dec(Register::H, Some(Register::L)),
                0x0C => self.inc(Register::L, None),
                0x0D => self.dec(Register::L, None),
                0x0E => self.ldi1(Register::L),
                0x0F => panic!("Opcode not implemented"),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },
            0x30 => match opcode & 0x0F {
                0x00 => self.jrnc(),
                0x01 => self.ldspi(),
                0x02 => panic!("Opcode not implemented"),
                0x03 => self.incsp(),
                0x04 => panic!("Opcode not implemented"),
                0x05 => panic!("Opcode not implemented"),
                0x06 => panic!("Opcode not implemented"),
                0x07 => panic!("Opcode not implemented"),
                0x08 => self.jrc(),
                0x09 => panic!("Opcode not implemented"),
                0x0A => self.lda(Register::H, Register::L, false, true),
                0x0B => self.decsp(),
                0x0C => self.inc(Register::A, None),
                0x0D => self.dec(Register::A, None),
                0x0E => self.ldi1(Register::A),
                0x0F => panic!("Opcode not implemented"),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },

            // LOAD INSTRUCTIONS
            // Load Instructions 1
            0x40 => match opcode & 0x0F {
                0x00 => self.ld(Register::B, Register::B),
                0x01 => self.ld(Register::B, Register::C),
                0x02 => self.ld(Register::B, Register::D),
                0x03 => self.ld(Register::B, Register::E),
                0x04 => self.ld(Register::B, Register::H),
                0x05 => self.ld(Register::B, Register::L),
                0x06 => self.ld(Register::B, Register::HL),
                0x07 => self.ld(Register::B, Register::A),
                0x08 => self.ld(Register::C, Register::B),
                0x09 => self.ld(Register::C, Register::C),
                0x0A => self.ld(Register::C, Register::D),
                0x0B => self.ld(Register::C, Register::E),
                0x0C => self.ld(Register::C, Register::H),
                0x0D => self.ld(Register::C, Register::L),
                0x0E => self.ld(Register::C, Register::HL),
                0x0F => self.ld(Register::C, Register::A),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },
            // Load Instructions 2
            0x50 => match opcode & 0x0F {
                0x00 => self.ld(Register::D, Register::B),
                0x01 => self.ld(Register::D, Register::C),
                0x02 => self.ld(Register::D, Register::D),
                0x03 => self.ld(Register::D, Register::E),
                0x04 => self.ld(Register::D, Register::H),
                0x05 => self.ld(Register::D, Register::L),
                0x06 => self.ld(Register::D, Register::HL),
                0x07 => self.ld(Register::D, Register::A),
                0x08 => self.ld(Register::E, Register::B),
                0x09 => self.ld(Register::E, Register::C),
                0x0A => self.ld(Register::E, Register::D),
                0x0B => self.ld(Register::E, Register::E),
                0x0C => self.ld(Register::E, Register::H),
                0x0D => self.ld(Register::E, Register::L),
                0x0E => self.ld(Register::E, Register::HL),
                0x0F => self.ld(Register::E, Register::A),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },
            // Load Instructions 3
            0x60 => match opcode & 0x0F {
                0x00 => self.ld(Register::H, Register::B),
                0x01 => self.ld(Register::H, Register::C),
                0x02 => self.ld(Register::H, Register::D),
                0x03 => self.ld(Register::H, Register::E),
                0x04 => self.ld(Register::H, Register::H),
                0x05 => self.ld(Register::H, Register::L),
                0x06 => self.ld(Register::H, Register::HL),
                0x07 => self.ld(Register::H, Register::A),
                0x08 => self.ld(Register::L, Register::B),
                0x09 => self.ld(Register::L, Register::C),
                0x0A => self.ld(Register::L, Register::D),
                0x0B => self.ld(Register::L, Register::E),
                0x0C => self.ld(Register::L, Register::H),
                0x0D => self.ld(Register::L, Register::L),
                0x0E => self.ld(Register::L, Register::HL),
                0x0F => self.ld(Register::L, Register::A),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },
            // Load Instructions 4
            0x70 => match opcode & 0x0F {
                0x00 => self.ld(Register::HL, Register::B),
                0x01 => self.ld(Register::HL, Register::C),
                0x02 => self.ld(Register::HL, Register::D),
                0x03 => self.ld(Register::HL, Register::E),
                0x04 => self.ld(Register::HL, Register::H),
                0x05 => self.ld(Register::HL, Register::L),
                0x06 => self.halt(),
                0x07 => self.ld(Register::HL, Register::A),
                0x08 => self.ld(Register::A, Register::B),
                0x09 => self.ld(Register::A, Register::C),
                0x0A => self.ld(Register::A, Register::D),
                0x0B => self.ld(Register::A, Register::E),
                0x0C => self.ld(Register::A, Register::H),
                0x0D => self.ld(Register::A, Register::L),
                0x0E => self.ld(Register::A, Register::HL),
                0x0F => self.ld(Register::A, Register::A),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },

            // MATH INSTRUCTIONS
            // Add and Add Carry Instructions
            0x80 => match opcode & 0x0F {
                0x00 => self.add(Register::B),
                0x01 => self.add(Register::C),
                0x02 => self.add(Register::D),
                0x03 => self.add(Register::E),
                0x04 => self.add(Register::H),
                0x05 => self.add(Register::L),
                0x06 => self.add(Register::HL),
                0x07 => self.add(Register::A),
                0x08 => self.adc(Register::B),
                0x09 => self.adc(Register::C),
                0x0A => self.adc(Register::D),
                0x0B => self.adc(Register::E),
                0x0C => self.adc(Register::H),
                0x0D => self.adc(Register::L),
                0x0E => self.adc(Register::HL),
                0x0F => self.adc(Register::A),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },

            // Subtract and Subtract Carry Instructions
            0x90 => match opcode & 0x0F {
                0x00 => self.sub(Register::B),
                0x01 => self.sub(Register::C),
                0x02 => self.sub(Register::D),
                0x03 => self.sub(Register::E),
                0x04 => self.sub(Register::H),
                0x05 => self.sub(Register::L),
                0x06 => self.sub(Register::HL),
                0x07 => self.sub(Register::A),
                0x08 => self.sbc(Register::B),
                0x09 => self.sbc(Register::C),
                0x0A => self.sbc(Register::D),
                0x0B => self.sbc(Register::E),
                0x0C => self.sbc(Register::H),
                0x0D => self.sbc(Register::L),
                0x0E => self.sbc(Register::HL),
                0x0F => self.sbc(Register::A),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },

            // AND and XOR Instructions
            0xA0 => match opcode & 0x0F {
                0x00 => self.and(Register::B),
                0x01 => self.and(Register::C),
                0x02 => self.and(Register::D),
                0x03 => self.and(Register::E),
                0x04 => self.and(Register::H),
                0x05 => self.and(Register::L),
                0x06 => self.and(Register::HL),
                0x07 => self.and(Register::A),
                0x08 => self.xor(Register::B),
                0x09 => self.xor(Register::C),
                0x0A => self.xor(Register::D),
                0x0B => self.xor(Register::E),
                0x0C => self.xor(Register::H),
                0x0D => self.xor(Register::L),
                0x0E => self.xor(Register::HL),
                0x0F => self.xor(Register::A),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },

            // OR and Compare Instructions
            0xB0 => match opcode & 0x0F {
                0x00 => self.or(Register::B),
                0x01 => self.or(Register::C),
                0x02 => self.or(Register::D),
                0x03 => self.or(Register::E),
                0x04 => self.or(Register::H),
                0x05 => self.or(Register::L),
                0x06 => self.or(Register::HL),
                0x07 => self.or(Register::A),
                0x08 => self.cp(Register::B),
                0x09 => self.cp(Register::C),
                0x0A => self.cp(Register::D),
                0x0B => self.cp(Register::E),
                0x0C => self.cp(Register::H),
                0x0D => self.cp(Register::L),
                0x0E => self.cp(Register::HL),
                0x0F => self.cp(Register::A),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },

            0xC0 => match opcode & 0x0F {
                0x00 => panic!("Opcode not implemented"),
                0x01 => panic!("Opcode not implemented"),
                0x02 => panic!("Opcode not implemented"),
                0x03 => self.jpnz(),
                0x04 => panic!("Opcode not implemented"),
                0x05 => panic!("Opcode not implemented"),
                0x06 => self.addi(),
                0x07 => panic!("Opcode not implemented"),
                0x08 => panic!("Opcode not implemented"),
                0x09 => panic!("Opcode not implemented"),
                0x0A => self.jpz(),
                0x0B => self.nop(),
                0x0C => self.adci(),
                0x0D => panic!("Opcode not implemented"),
                0x0E => self.adci(),
                0x0F => panic!("Opcode not implemented"),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },
            0xD0 => match opcode & 0x0F {
                0x00 => panic!("Opcode not implemented"),
                0x01 => panic!("Opcode not implemented"),
                0x02 => panic!("Opcode not implemented"),
                0x03 => self.nop(),
                0x04 => panic!("Opcode not implemented"),
                0x05 => panic!("Opcode not implemented"),
                0x06 => self.subi(),
                0x07 => panic!("Opcode not implemented"),
                0x08 => panic!("Opcode not implemented"),
                0x09 => panic!("Opcode not implemented"),
                0x0A => self.jpc(),
                0x0B => self.nop(),
                0x0C => panic!("Opcode not implemented"),
                0x0D => self.nop(),
                0x0E => self.sbci(),
                0x0F => panic!("Opcode not implemented"),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },
            0xE0 => match opcode & 0x0F {
                0x00 => panic!("Opcode not implemented"),
                0x01 => panic!("Opcode not implemented"),
                0x02 => panic!("Opcode not implemented"),
                0x03 => self.nop(),
                0x04 => self.nop(),
                0x05 => panic!("Opcode not implemented"),
                0x06 => self.andi(),
                0x07 => panic!("Opcode not implemented"),
                0x08 => panic!("Opcode not implemented"),
                0x09 => panic!("Opcode not implemented"),
                0x0A => panic!("Opcode not implemented"),
                0x0B => self.nop(),
                0x0C => self.nop(),
                0x0D => self.nop(),
                0x0E => self.xori(),
                0x0F => panic!("Opcode not implemented"),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },
            0xF0 => match opcode & 0x0F {
                0x00 => panic!("Opcode not implemented"),
                0x01 => panic!("Opcode not implemented"),
                0x02 => panic!("Opcode not implemented"),
                0x03 => {println!("Opcode not implemented"); self.nop()},
                0x04 => self.nop(),
                0x05 => panic!("Opcode not implemented"),
                0x06 => self.ori(),
                0x07 => panic!("Opcode not implemented"),
                0x08 => panic!("Opcode not implemented"),
                0x09 => panic!("Opcode not implemented"),
                0x0A => panic!("Opcode not implemented"),
                0x0B => panic!("Opcode not implemented"),
                0x0C => self.nop(),
                0x0D => self.nop(),
                0x0E => self.cpi(),
                0x0F => panic!("Opcode not implemented"),
                _ => println!("Opcode not implemented: {:02X}", opcode),
            },
            _ => println!("Opcode not implemented: {:02X}", opcode),
        }
    }

    // OPCODE Helper Functions
    // Function to get the address of an operand from an opcode
    fn get_jump_address(&self) -> u16 {
        let low_byte = self.read_memory(self.pc + 1);
        let high_byte = self.read_memory(self.pc + 2);
        let address = ((high_byte as u16) << 8) | low_byte as u16;
        return address;
    }

    // Get 16-bit address from two registers
    fn get_double_register(&self, reg1: Register, reg2: Register) -> u16 {
        (self.read_register(reg1) as u16) << 8 | self.read_register(reg2) as u16
    }

    // Set 16-bit value to two registers
    fn set_double_register(&mut self, reg1: Register, reg2: Register, value: u16) {
        self.write_register(reg1, (value >> 8) as u8);
        self.write_register(reg2, value as u8);
    }

    // OPCODES
    // Arithmetic Instructions
    // Add reg and carry flag to reg_a, store in reg_a
    fn adc(&mut self, reg: Register) {
        let value = self.read_register(reg);
        let carry = self.read_flag(Register::CARRY);
        let (result, carry1) = self.read_register(Register::A).overflowing_add(value);
        let (result, carry2) = result.overflowing_add(carry as u8);
        
        // Set flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, ((self.read_register(Register::A) & 0x0F) + (value & 0x0F) + carry as u8) > 0x0F);
        self.write_flag(Register::CARRY, carry1 || carry2);
        
        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // Add an immediate 8-bit value and carry flag to the A register
    fn adci(&mut self) {
        let value = self.read_memory(self.pc + 1);
        let carry = self.read_flag(Register::CARRY);
        let (result, carry1) = self.read_register(Register::A).overflowing_add(value);
        let (result, carry2) = result.overflowing_add(carry as u8);
        
        // Set flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, ((self.read_register(Register::A) & 0x0F) + (value & 0x0F) + carry as u8) > 0x0F);
        self.write_flag(Register::CARRY, carry1 || carry2);
        
        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }
    
    // Add reg to reg_a, store in reg_a
    fn add(&mut self, reg: Register) {
        let value = self.read_register(reg);
        let (result, carry) = self.read_register(Register::A).overflowing_add(value);
        
        // Set flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, self.read_register(Register::A) & 0x0F < value & 0x0F);
        self.write_flag(Register::CARRY, carry);
        
        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // Add an immediate 8-bit value to the A register
    fn addi(&mut self) {
        let value = self.read_memory(self.pc + 1);
        let (result, carry) = self.read_register(Register::A).overflowing_add(value);
        
        // Set flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, self.read_register(Register::A) & 0x0F < value & 0x0F);
        self.write_flag(Register::CARRY, carry);
        
        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // AND reg with reg_a, store in reg_a
    fn and(&mut self, reg: Register) {
        // Calulate result
        let value = self.read_register(reg);
        let result = self.read_register(Register::A) & value;

        // Set Flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, true);
        self.write_flag(Register::CARRY, false);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // AND immediate 8-bit value with reg_a, store in reg_a
    fn andi(&mut self) {
        // Calulate result
        let value = self.read_memory(self.pc + 1);
        let result = self.read_register(Register::A) & value;

        // Set Flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, true);
        self.write_flag(Register::CARRY, false);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // Compare reg with reg_a via reg_a - reg, set Z flag if equal, reg_a is unaffected
    fn cp(&mut self, reg: Register) {
        // Calulate result
        let value = self.read_register(reg);
        let (result, borrow) = self.read_register(Register::A).overflowing_sub(value);

        // Set Flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, true);
        self.write_flag(Register::HC, (self.reg_a & 0x0F) < ((value & 0x0F)));
        self.write_flag(Register::CARRY, borrow);

        // Continue
        self.pc += 1;
    }

    // Compare immediate 8-bit value with reg_a via reg_a - value, set Z flag if equal, reg_a is unaffected
    fn cpi(&mut self) {
        // Calulate result
        let value = self.read_memory(self.pc + 1);
        let (result, borrow) = self.read_register(Register::A).overflowing_sub(value);

        // Set Flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, true);
        self.write_flag(Register::HC, (self.reg_a & 0x0F) < ((value & 0x0F)));
        self.write_flag(Register::CARRY, borrow);

        // Continue
        self.pc += 1;
    }

    // Decrement reg1, if reg2 is Some, decrement reg2
    fn dec(&mut self, reg1: Register, reg2: Option<Register>) {
        match reg2 {
            Some(r2) => {
                let value = self.get_double_register(reg1, r2).wrapping_sub(1);
                self.set_double_register(reg1, r2, value);
            },
            None => {
                let value = self.read_register(reg1).wrapping_sub(1);
                self.write_register(reg1, value);
            }
        }

        // Set Flags
        self.write_flag(Register::ZERO, self.read_register(reg1) == 0);
        self.write_flag(Register::SUB, true);
        self.write_flag(Register::HC, (self.read_register(reg1) & 0x0F) == 0x0F);

        self.pc += 1;
    }

    // Decrement SP
    fn decsp(&mut self) {
        self.sp = self.sp.wrapping_sub(1);
        self.pc += 1;
    }

    // Halt CPU
    fn halt(&self) {
        println!("HALT");
        panic!("Implement Halt");
    }

    // Increment reg1, if reg2 is Some, increment reg2
    fn inc(&mut self, reg1: Register, reg2: Option<Register>) {
        match reg2 {
            Some(r2) => {
                let value = self.get_double_register(reg1, r2).wrapping_add(1);
                self.set_double_register(reg1, r2, value);
            },
            None => {
                let value = self.read_register(reg1).wrapping_add(1);
                self.write_register(reg1, value);
            }
        }

        // Set Flags
        self.write_flag(Register::ZERO, self.read_register(reg1) == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, (self.read_register(reg1) & 0x0F) == 0x00);

        self.pc += 1;
    }

    // Increment SP
    fn incsp(&mut self) {
        self.sp = self.sp.wrapping_add(1);
        self.pc += 1;
    }

    // JUMP INSTRUCTIONS
    // Jump to address in next 16 bits
    fn jp(&mut self) {
        let address = self.get_jump_address();
        self.pc = address;

    }

    // Jump if carry is set
    fn jpc(&mut self) {
        let address = self.get_jump_address();

        if self.read_flag(Register::CARRY) {
            self.pc = address;
        }
        else {
            self.pc += 3;
        }
    }

    // Jump to address at HL
    fn jphl(&mut self) {
        let address = self.get_double_register(Register::H, Register::L);
        self.pc = address;
    }

    // Jump if carry is not set
    fn jpnc(&mut self) {
        let address = self.get_jump_address();

        if !(self.read_flag(Register::CARRY)) {
            self.pc = address;
        }
        else {
            self.pc += 3;
        }
    }

    // Jump is zero is not set
    fn jpnz(&mut self) {
        // Read the 16 bit immediate operand
        let address = self.get_jump_address();

        // Check Zero Flag
        if !(self.read_flag(Register::ZERO)) {
            self.pc = address;
        }
        else {
            // Increment to next instruction, skip operand
            self.pc += 3;
        }
    }

    // Jump if zero is set
    fn jpz(&mut self) {
        let address = self.get_jump_address();

        if !(self.read_flag(Register::ZERO)) {
            self.pc = address;
        }
        else {
            self.pc += 3;
        }
    }

    // Jump relative number of steps in next 16 bits
    fn jr(&mut self) {
        let offset = self.read_memory(self.pc + 1) as i8;
        self.pc = self.pc.wrapping_add(2).wrapping_add(offset as u16);
    }

    // Jump relative if carry is set
    fn jrc(&mut self) {
        let offset = self.read_memory(self.pc + 1) as i8;
        if self.read_flag(Register::CARRY) {
            self.pc = self.pc.wrapping_add(2).wrapping_add(offset as u16);
        }
        else {
            self.pc += 2;
        }
    }

    // Jump relative if carry is not set
    fn jrnc(&mut self) {
        let offset = self.read_memory(self.pc + 1) as i8;
        if !self.read_flag(Register::CARRY) {
            self.pc = self.pc.wrapping_add(2).wrapping_add(offset as u16);
        }
        else {
            self.pc += 2;
        }
    }

    // Jump relative is zero is not set
    fn jrnz(&mut self) {
        let offset = self.read_memory(self.pc + 1) as i8;
        if !self.read_flag(Register::ZERO) {
            self.pc = self.pc.wrapping_add(2).wrapping_add(offset as u16);
        }
        else {
            self.pc += 2;
        }
    }

    // Jump relative if zero is set
    fn jrz(&mut self) {
        let offset = self.read_memory(self.pc + 1) as i8;
        if self.read_flag(Register::ZERO) {
            self.pc = self.pc.wrapping_add(2).wrapping_add(offset as u16);
        }
        else {
            self.pc += 2;
        }
    }

    // Load reg2 into reg1
    fn ld(&mut self, reg1: Register, reg2: Register) {
        let value = self.read_register(reg2);
        self.write_register(reg1, value);
        self.pc += 1;
    }

    // Load immediate 8 bit value into reg
    fn ldi1(&mut self, reg: Register) {
        // Get immediate 8 bits
        let value = self.read_memory(self.pc + 1);

        // Load to proper register
        self.write_register(reg, value);
        self.pc += 2;
    }

    // Load immediate 16 bit value into regs
    fn ldi2(&mut self, reg1: Register, reg2: Register) {
        // Get immediate 16 bits
        let low = self.read_memory(self.pc + 1);
        let high = self.read_memory(self.pc + 2);

        // Load to proper registers
        self.write_register(reg1, low);
        self.write_register(reg2, high);
        self.pc += 3;

    }

    // Load address from two registers into reg_a, if HL, increment or decrement
    fn lda(&mut self, reg1: Register, reg2: Register, increment: bool, decrement: bool) {
        let address = self.get_double_register(reg1, reg2);
        let value = self.read_memory(address);
        self.write_register(Register::A, value);

        // Increment or Decrement HL
        if increment {
            self.inc(reg1, Some(reg2));
        }
        else if decrement {
            self.dec(reg1, Some(reg2));
        }

        self.pc += 1;
    }

    // Load immediate pair from memory into the stack pointer
    fn ldspi(&mut self) {
        let low = self.read_memory(self.pc + 1);
        let high = self.read_memory(self.pc + 2);
        let address = ((high as u16) << 8) | low as u16;
        self.sp = address;
        self.pc += 3;
    }

    // No opperation, continue
    fn nop(&mut self) {
        self.pc += 1;
    }

    // OR reg with reg_a, store in reg_a
    fn or(&mut self, reg: Register) {
        // Calulate result
        let value = self.read_register(reg);
        let result = self.read_register(Register::A) | value;

        // Set Flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, false);
        self.write_flag(Register::CARRY, false);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // OR immediate value with reg_a, store in reg_a
    fn ori(&mut self) {
        let value = self.read_memory(self.pc + 1);
        let result = self.read_register(Register::A) | value;

        // Set Flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, false);
        self.write_flag(Register::CARRY, false);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // Rotate reg_a Right through carry
    fn rra(&mut self) {
        let carry = self.read_flag(Register::CARRY);
        let value = self.read_register(Register::A);
        let new_carry = value & 0x01;
        let result = (value >> 1) | ((carry as u8) << 7);

        // Set Flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, false);
        self.write_flag(Register::CARRY, new_carry == 1);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // Rotate reg_a Right
    fn rrca(&mut self) {
        let value = self.read_register(Register::A);
        let new_carry = value & 0x01;
        let result = (value >> 1) | (new_carry << 7);

        // Set Flags
        self.write_flag(Register::ZERO, false);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, false);
        self.write_flag(Register::CARRY, new_carry == 1);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // Sub reg and carry flag from reg_a, store in reg_a
    fn sbc(&mut self, reg: Register) {
        // Compute Result
        let value = self.read_register(reg);        
        let carry = self.read_flag(Register::CARRY);

        let (result, borrow1) = self.read_register(Register::A).overflowing_sub(value);
        let (result, borrow2) = result.overflowing_sub(carry as u8);

        // Set flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, true);
        self.write_flag(Register::HC, (self.reg_a & 0x0F) < ((value & 0x0F) + carry as u8));
        self.write_flag(Register::CARRY, borrow1 || borrow2);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // Subtract immediate value and carry flag from reg_a
    fn sbci(&mut self) {
        // Compute Result
        let value = self.read_memory(self.pc + 1);        
        let carry = self.read_flag(Register::CARRY);

        let (result, borrow1) = self.read_register(Register::A).overflowing_sub(value);
        let (result, borrow2) = result.overflowing_sub(carry as u8);

        // Set flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, true);
        self.write_flag(Register::HC, (self.reg_a & 0x0F) < ((value & 0x0F) + carry as u8));
        self.write_flag(Register::CARRY, borrow1 || borrow2);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // Sub reg from reg_a, store in reg_a
    fn sub(&mut self, reg: Register) {
        // Compute Result
        let value = self.read_register(reg);
        let (result, borrow) = self.read_register(Register::A).overflowing_sub(value);

        // Set flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, true);
        self.write_flag(Register::HC, self.read_register(Register::A) & 0x0F < value & 0x0F);
        self.write_flag(Register::CARRY, borrow);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // Subtract and immediate value from reg_a
    fn subi(&mut self) {
        let value = self.read_memory(self.pc + 1);
        let (result, borrow) = self.read_register(Register::A).overflowing_sub(value);

        // Set flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, true);
        self.write_flag(Register::HC, self.read_register(Register::A) & 0x0F < value & 0x0F);
        self.write_flag(Register::CARRY, borrow);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // XOR reg with reg_a, store in reg_a
    fn xor(&mut self, reg: Register) {
        // Calulate result
        let value = self.read_register(reg);
        let result = self.read_register(Register::A) ^ value;

        // Set Flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, false);
        self.write_flag(Register::CARRY, false);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }

    // XOR with immediate 8-bit value
    fn xori(&mut self) {
        // Calulate result
        let value = self.read_memory(self.pc + 1);
        let result = self.read_register(Register::A) ^ value;

        // Set Flags
        self.write_flag(Register::ZERO, result == 0);
        self.write_flag(Register::SUB, false);
        self.write_flag(Register::HC, false);
        self.write_flag(Register::CARRY, false);

        // Store result and continue
        self.write_register(Register::A, result);
        self.pc += 1;
    }
}