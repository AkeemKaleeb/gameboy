use crate::mmu::{self, RomMetadata, MMU};
use crate::registers::{Flag, Reg, Register};
use core::panic;
use std::cell::RefCell;
use std::rc::Rc;

pub struct CPU {
    // Registers
    reg: Reg,
    // Memory Management System
    mmu: Rc<RefCell<MMU>>,

    // Interrupt Flags
    ime: bool,
    halted: bool,
    setdi: u8,
    setei: u8,
}

impl CPU {
    pub fn new(mmu: Rc<RefCell<MMU>>) -> CPU {
        CPU {
            reg: Reg::new(),
            mmu,
            ime: true,
            halted: false,
            setdi: 0,
            setei: 0,
        }
    }

    /// Public instruction to send ROM to MMU
    pub fn load_rom(&mut self, rom: &Vec<u8>) -> RomMetadata {
        return self.mmu.borrow_mut().load_rom(rom);
    }

    /// Function to Run ROM by instruction
    pub fn do_step(&mut self) -> u8 {        
        let ticks = self.step() * 4;
        return self.mmu.borrow_mut().do_step(ticks);
    }

    /// Function to Run ROM by instruction
    fn step(&mut self) -> u8{
        // Update Time
        self.update_time();

        // Handle Interrupts
        match self.handle_interrupt() {
            0 => {},
            n => return n,
        };

        // Check if Stopped
        if self.halted {
            return 1;
        }
        else {
            // Fetch and Execute Opcode
            return self.execute();
        }
    }

    /// Update the CPU time
    fn update_time(&mut self) {
        // TODO: Implement Timer
    }

    /// Test if an interrupt has been called
    fn handle_interrupt(&mut self) -> u8{
        // TODO: Implement
        0
    }

    /// Get OPCODE and execute the appropriate function
    pub fn execute(&mut self) -> u8 {
        let opcode = self.mmu.borrow().read_byte(self.reg.pc);
        match opcode {
            0x00 => self.nop(),
            0x01 => self.ldi_rr(Register::B, Register::C),
            0x02 => self.ld_aa_ra(Register::B, Register::C),
            0x03 => self.inc_rr(Register::B, Register::C),
            0x04 => self.inc(Register::B),
            0x05 => self.dec(Register::B),
            0x06 => self.ld_r_i(Register::B),
            0x07 => self.rlc_a(),
            0x08 => self.ldi_ram_sp(),
            0x09 => self.add_hl_rr(Register::B, Register::C),
            0x0A => self.ld_ra_aa(Register::B, Register::C),
            0x0B => self.dec_rr(Register::B, Register::C),
            0x0C => self.inc(Register::C),
            0x0D => self.dec(Register::C),
            0x0E => self.ld_r_i(Register::C),
            0x0F => self.rrc_a(),
            0x10 => self.stop(),
            0x11 => self.ldi_rr(Register::D, Register::E),
            0x12 => self.ld_aa_ra(Register::D, Register::E),
            0x13 => self.inc_rr(Register::D, Register::E),
            0x14 => self.inc(Register::D),
            0x15 => self.dec(Register::D),
            0x16 => self.ld_r_i(Register::D),
            0x17 => self.rl_a(),
            0x18 => self.jr(),
            0x19 => self.add_hl_rr(Register::D, Register::E),
            0x1A => self.ld_ra_aa(Register::D, Register::E),
            0x1B => self.dec_rr(Register::D, Register::E),
            0x1C => self.inc(Register::E),
            0x1D => self.dec(Register::E),
            0x1E => self.ld_r_i(Register::E),
            0x1F => self.rr_a(),
            0x20 => self.jrcc(Flag::ZERO, false),
            0x21 => self.ldi_rr(Register::H, Register::L),
            0x22 => self.ldinc_hl_a(),
            0x23 => self.inc_rr(Register::H, Register::L),
            0x24 => self.inc(Register::H),
            0x25 => self.dec(Register::H),
            0x26 => self.ld_r_i(Register::H),
            0x27 => self.daa(),
            0x28 => self.jrcc(Flag::ZERO, true),
            0x29 => self.add_hl_rr(Register::H, Register::L),
            0x2A => self.ldinc_a_hl(),
            0x2B => self.dec_rr(Register::H, Register::L),
            0x2C => self.inc(Register::L),
            0x2D => self.dec(Register::L),
            0x2E => self.ld_r_i(Register::L),
            0x2F => self.cpl(),
            0x30 => self.jrcc(Flag::CARRY, false),
            0x31 => self.ldi_rr(Register::SP, Register::SP),
            0x32 => self.lddec_hl_a(),
            0x33 => self.inc_rr(Register::SP, Register::SP),
            0x34 => self.inc_hl(),
            0x35 => self.dec_hl(),
            0x36 => self.ld_hl_i(),
            0x37 => self.scf(),
            0x38 => self.jrcc(Flag::CARRY, true),
            0x39 => self.add_hl_rr(Register::SP, Register::SP),
            0x3A => self.lddec_a_hl(),
            0x3B => self.dec_rr(Register::SP, Register::SP),
            0x3C => self.inc(Register::A),
            0x3D => self.dec(Register::A),
            0x3E => self.ld_r_i(Register::A),
            0x3F => self.ccf(),
            0x40 => self.ld_r_r(Register::B, Register::B),
            0x41 => self.ld_r_r(Register::B, Register::C),
            0x42 => self.ld_r_r(Register::B, Register::D),
            0x43 => self.ld_r_r(Register::B, Register::E),
            0x44 => self.ld_r_r(Register::B, Register::H),
            0x45 => self.ld_r_r(Register::B, Register::L),
            0x46 => self.ld_r_hl(Register::B),
            0x47 => self.ld_r_r(Register::B, Register::A),
            0x48 => self.ld_r_r(Register::C, Register::B),
            0x49 => self.ld_r_r(Register::C, Register::C),
            0x4A => self.ld_r_r(Register::C, Register::D),
            0x4B => self.ld_r_r(Register::C, Register::E),
            0x4C => self.ld_r_r(Register::C, Register::H),
            0x4D => self.ld_r_r(Register::C, Register::L),
            0x4E => self.ld_r_hl(Register::C),
            0x4F => self.ld_r_r(Register::C, Register::A),
            0x50 => self.ld_r_r(Register::D, Register::B),
            0x51 => self.ld_r_r(Register::D, Register::C),
            0x52 => self.ld_r_r(Register::D, Register::D),
            0x53 => self.ld_r_r(Register::D, Register::E),
            0x54 => self.ld_r_r(Register::D, Register::H),
            0x55 => self.ld_r_r(Register::D, Register::L),
            0x56 => self.ld_r_hl(Register::D),
            0x57 => self.ld_r_r(Register::D, Register::A),
            0x58 => self.ld_r_r(Register::E, Register::B),
            0x59 => self.ld_r_r(Register::E, Register::C),
            0x5A => self.ld_r_r(Register::E, Register::D),
            0x5B => self.ld_r_r(Register::E, Register::E),
            0x5C => self.ld_r_r(Register::E, Register::H),
            0x5D => self.ld_r_r(Register::E, Register::L),
            0x5E => self.ld_r_hl(Register::E),
            0x5F => self.ld_r_r(Register::E, Register::A),
            0x60 => self.ld_r_r(Register::H, Register::B),
            0x61 => self.ld_r_r(Register::H, Register::C),
            0x62 => self.ld_r_r(Register::H, Register::D),
            0x63 => self.ld_r_r(Register::H, Register::E),
            0x64 => self.ld_r_r(Register::H, Register::H),
            0x65 => self.ld_r_r(Register::H, Register::L),
            0x66 => self.ld_r_hl(Register::H),
            0x67 => self.ld_r_r(Register::H, Register::A),
            0x68 => self.ld_r_r(Register::L, Register::B),
            0x69 => self.ld_r_r(Register::L, Register::C),
            0x6A => self.ld_r_r(Register::L, Register::D),
            0x6B => self.ld_r_r(Register::L, Register::E),
            0x6C => self.ld_r_r(Register::L, Register::H),
            0x6D => self.ld_r_r(Register::L, Register::L),
            0x6E => self.ld_r_hl(Register::L),
            0x6F => self.ld_r_r(Register::L, Register::A),
            0x70 => self.ld_hl_r(Register::B),
            0x71 => self.ld_hl_r(Register::C),
            0x72 => self.ld_hl_r(Register::D),
            0x73 => self.ld_hl_r(Register::E),
            0x74 => self.ld_hl_r(Register::H),
            0x75 => self.ld_hl_r(Register::L),
            0x76 => self.halt(),
            0x77 => self.ld_hl_r(Register::A),
            0x78 => self.ld_r_r(Register::A, Register::B),
            0x79 => self.ld_r_r(Register::A, Register::C),
            0x7A => self.ld_r_r(Register::A, Register::D),
            0x7B => self.ld_r_r(Register::A, Register::E),
            0x7C => self.ld_r_r(Register::A, Register::H),
            0x7D => self.ld_r_r(Register::A, Register::L),
            0x7E => self.ld_r_hl(Register::A),
            0x7F => self.ld_r_r(Register::A, Register::A),
            0x80 => self.add_r(Register::B),
            0x81 => self.add_r(Register::C),
            0x82 => self.add_r(Register::D),
            0x83 => self.add_r(Register::E),
            0x84 => self.add_r(Register::H),
            0x85 => self.add_r(Register::L),
            0x86 => self.add_hl(),
            0x87 => self.add_r(Register::A),
            0x88 => self.adc_r(Register::B),
            0x89 => self.adc_r(Register::C),
            0x8A => self.adc_r(Register::D),
            0x8B => self.adc_r(Register::E),
            0x8C => self.adc_r(Register::H),
            0x8D => self.adc_r(Register::L),
            0x8E => self.adc_hl(),
            0x8F => self.adc_r(Register::A),
            0x90 => self.sub_r(Register::B),
            0x91 => self.sub_r(Register::C),
            0x92 => self.sub_r(Register::D),
            0x93 => self.sub_r(Register::E),
            0x94 => self.sub_r(Register::H),
            0x95 => self.sub_r(Register::L),
            0x96 => self.sub_hl(),
            0x97 => self.sub_r(Register::A),
            0x98 => self.sbc_r(Register::B),
            0x99 => self.sbc_r(Register::C),
            0x9A => self.sbc_r(Register::D),
            0x9B => self.sbc_r(Register::E),
            0x9C => self.sbc_r(Register::H),
            0x9D => self.sbc_r(Register::L),
            0x9E => self.sbc_hl(),
            0x9F => self.sbc_r(Register::A),
            0xA0 => self.and_r(Register::B),
            0xA1 => self.and_r(Register::C),
            0xA2 => self.and_r(Register::D),
            0xA3 => self.and_r(Register::E),
            0xA4 => self.and_r(Register::H),
            0xA5 => self.and_r(Register::L),
            0xA6 => self.and_hl(),
            0xA7 => self.and_r(Register::A),
            0xA8 => self.xor_r(Register::B),
            0xA9 => self.xor_r(Register::C),
            0xAA => self.xor_r(Register::D),
            0xAB => self.xor_r(Register::E),
            0xAC => self.xor_r(Register::H),
            0xAD => self.xor_r(Register::L),
            0xAE => self.xor_hl(),
            0xAF => self.xor_r(Register::A),
            0xB0 => self.or_r(Register::B),
            0xB1 => self.or_r(Register::C),
            0xB2 => self.or_r(Register::D),
            0xB3 => self.or_r(Register::E),
            0xB4 => self.or_r(Register::H),
            0xB5 => self.or_r(Register::L),
            0xB6 => self.or_hl(),
            0xB7 => self.or_r(Register::A),
            0xB8 => self.cp_r(Register::B),
            0xB9 => self.cp_r(Register::C),
            0xBA => self.cp_r(Register::D),
            0xBB => self.cp_r(Register::E),
            0xBC => self.cp_r(Register::H),
            0xBD => self.cp_r(Register::L),
            0xBE => self.cp_hl(),
            0xBF => self.cp_r(Register::A),
            0xC0 => self.retcc(Flag::ZERO, false),
            0xC1 => self.pop(Register::B, Register::C),
            0xC2 => self.jpcc(Flag::ZERO, false),
            0xC3 => self.jp(),
            0xC4 => self.callcc(Flag::ZERO, false),
            0xC5 => self.push(Register::B, Register::C),
            0xC6 => self.add_i(),
            0xC7 => self.rst(0x00),
            0xC8 => self.retcc(Flag::ZERO, true),
            0xC9 => self.ret(),
            0xCA => self.jpcc(Flag::ZERO, true),
            0xCB => {
                let cb_opcode = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1));
                self.execute_cb(cb_opcode)
            },
            0xCC => self.callcc(Flag::ZERO, true),
            0xCD => self.call(),
            0xCE => self.adc_i(),
            0xCF => self.rst(0x08),
            0xD0 => self.retcc(Flag::CARRY, false),
            0xD1 => self.pop(Register::D, Register::E),
            0xD2 => self.jpcc(Flag::CARRY, false),
            0xD3 => self.nop(), // Unused
            0xD4 => self.callcc(Flag::CARRY, false),
            0xD5 => self.push(Register::D, Register::E),
            0xD6 => self.sub_i(),
            0xD7 => self.rst(0x10),
            0xD8 => self.retcc(Flag::CARRY, true),
            0xD9 => self.reti(),
            0xDA => self.jpcc(Flag::CARRY, true),
            0xDB => self.nop(), // Unused
            0xDC => self.callcc(Flag::CARRY, true),
            0xDD => self.nop(), // Unused
            0xDE => self.sbc_i(),
            0xDF => self.rst(0x18),
            0xE0 => self.ldi_ram_a(),
            0xE1 => self.pop(Register::H, Register::L),
            0xE2 => self.ld_ram_ra(),
            0xE3 => self.nop(), // Unused
            0xE4 => self.nop(), // Unused
            0xE5 => self.push(Register::H, Register::L),
            0xE6 => self.and_i(),
            0xE7 => self.rst(0x20),
            0xE8 => self.add_sp_i(),
            0xE9 => self.jphl(),
            0xEA => self.ld_ii_ra(),
            0xEB => self.nop(), // Unused
            0xEC => self.nop(), // Unused
            0xED => self.nop(), // Unused
            0xEE => self.xor_i(),
            0xEF => self.rst(0x28),
            0xF0 => self.ldi_a_ram(),
            0xF1 => self.pop(Register::A, Register::F),
            0xF2 => self.ld_ra_ram(),
            0xF3 => self.di(),
            0xF4 => self.nop(), // Unused
            0xF5 => self.push(Register::A, Register::F),
            0xF6 => self.or_i(),
            0xF7 => self.rst(0x30),
            0xF8 => self.ld_hl_sp(),
            0xF9 => self.ld_sp_hl(),
            0xFA => self.ld_ra_ii(),
            0xFB => self.ei(),
            0xFC => self.nop(), // Unused
            0xFD => self.nop(), // Unused
            0xFE => self.cp_i(),
            0xFF => self.rst(0x38),
        }
    }

    /// Handle Prefixed CB OPCODES
    fn execute_cb(&mut self, opcode: u8) -> u8 {
        match opcode {
            0x00 => self.rlc_r(Register::B),
            0x01 => self.rlc_r(Register::C),
            0x02 => self.rlc_r(Register::D),
            0x03 => self.rlc_r(Register::E),
            0x04 => self.rlc_r(Register::H),
            0x05 => self.rlc_r(Register::L),
            0x06 => self.rlc_hl(),
            0x07 => self.rlc_r(Register::A),
            0x08 => self.rrc_r(Register::B),
            0x09 => self.rrc_r(Register::C),
            0x0A => self.rrc_r(Register::D),
            0x0B => self.rrc_r(Register::E),
            0x0C => self.rrc_r(Register::H),
            0x0D => self.rrc_r(Register::L),
            0x0E => self.rrc_hl(),
            0x0F => self.rrc_r(Register::A),
            0x10 => self.rl_r(Register::B),
            0x11 => self.rl_r(Register::C),
            0x12 => self.rl_r(Register::D),
            0x13 => self.rl_r(Register::E),
            0x14 => self.rl_r(Register::H),
            0x15 => self.rl_r(Register::L),
            0x16 => self.rl_hl(),
            0x17 => self.rl_r(Register::A),
            0x18 => self.rr_r(Register::B),
            0x19 => self.rr_r(Register::C),
            0x1A => self.rr_r(Register::D),
            0x1B => self.rr_r(Register::E),
            0x1C => self.rr_r(Register::H),
            0x1D => self.rr_r(Register::L),
            0x1E => self.rr_hl(),
            0x1F => self.rr_r(Register::A),
            0x20 => self.sla_r(Register::B),
            0x21 => self.sla_r(Register::C),
            0x22 => self.sla_r(Register::D),
            0x23 => self.sla_r(Register::E),
            0x24 => self.sla_r(Register::H),
            0x25 => self.sla_r(Register::L),
            0x26 => self.sla_hl(),
            0x27 => self.sla_r(Register::A),
            0x28 => self.sra_r(Register::B),
            0x29 => self.sra_r(Register::C),
            0x2A => self.sra_r(Register::D),
            0x2B => self.sra_r(Register::E),
            0x2C => self.sra_r(Register::H),
            0x2D => self.sra_r(Register::L),
            0x2E => self.sra_hl(),
            0x2F => self.sra_r(Register::A),
            0x30 => self.swap_r(Register::B),
            0x31 => self.swap_r(Register::C),
            0x32 => self.swap_r(Register::D),
            0x33 => self.swap_r(Register::E),
            0x34 => self.swap_r(Register::H),
            0x35 => self.swap_r(Register::L),
            0x36 => self.swap_hl(),
            0x37 => self.swap_r(Register::A),
            0x38 => self.srl_r(Register::B),
            0x39 => self.srl_r(Register::C),
            0x3A => self.srl_r(Register::D),
            0x3B => self.srl_r(Register::E),
            0x3C => self.srl_r(Register::H),
            0x3D => self.srl_r(Register::L),
            0x3E => self.srl_hl(),
            0x3F => self.srl_r(Register::A),
            0x40 => self.bit_r(0, Register::B),
            0x41 => self.bit_r(0, Register::C),
            0x42 => self.bit_r(0, Register::D),
            0x43 => self.bit_r(0, Register::E),
            0x44 => self.bit_r(0, Register::H),
            0x45 => self.bit_r(0, Register::L),
            0x46 => self.bit_hl(0),
            0x47 => self.bit_r(0, Register::A),
            0x48 => self.bit_r(1, Register::B),
            0x49 => self.bit_r(1, Register::C),
            0x4A => self.bit_r(1, Register::D),
            0x4B => self.bit_r(1, Register::E),
            0x4C => self.bit_r(1, Register::H),
            0x4D => self.bit_r(1, Register::L),
            0x4E => self.bit_hl(1),
            0x4F => self.bit_r(1, Register::A),
            0x50 => self.bit_r(2, Register::B),
            0x51 => self.bit_r(2, Register::C),
            0x52 => self.bit_r(2, Register::D),
            0x53 => self.bit_r(2, Register::E),
            0x54 => self.bit_r(2, Register::H),
            0x55 => self.bit_r(2, Register::L),
            0x56 => self.bit_hl(2),
            0x57 => self.bit_r(2, Register::A),
            0x58 => self.bit_r(3, Register::B),
            0x59 => self.bit_r(3, Register::C),
            0x5A => self.bit_r(3, Register::D),
            0x5B => self.bit_r(3, Register::E),
            0x5C => self.bit_r(3, Register::H),
            0x5D => self.bit_r(3, Register::L),
            0x5E => self.bit_hl(3),
            0x5F => self.bit_r(3, Register::A),
            0x60 => self.bit_r(4, Register::B),
            0x61 => self.bit_r(4, Register::C),
            0x62 => self.bit_r(4, Register::D),
            0x63 => self.bit_r(4, Register::E),
            0x64 => self.bit_r(4, Register::H),
            0x65 => self.bit_r(4, Register::L),
            0x66 => self.bit_hl(4),
            0x67 => self.bit_r(4, Register::A),
            0x68 => self.bit_r(5, Register::B),
            0x69 => self.bit_r(5, Register::C),
            0x6A => self.bit_r(5, Register::D),
            0x6B => self.bit_r(5, Register::E),
            0x6C => self.bit_r(5, Register::H),
            0x6D => self.bit_r(5, Register::L),
            0x6E => self.bit_hl(5),
            0x6F => self.bit_r(5, Register::A),
            0x70 => self.bit_r(6, Register::B),
            0x71 => self.bit_r(6, Register::C),
            0x72 => self.bit_r(6, Register::D),
            0x73 => self.bit_r(6, Register::E),
            0x74 => self.bit_r(6, Register::H),
            0x75 => self.bit_r(6, Register::L),
            0x76 => self.bit_hl(6),
            0x77 => self.bit_r(6, Register::A),
            0x78 => self.bit_r(7, Register::B),
            0x79 => self.bit_r(7, Register::C),
            0x7A => self.bit_r(7, Register::D),
            0x7B => self.bit_r(7, Register::E),
            0x7C => self.bit_r(7, Register::H),
            0x7D => self.bit_r(7, Register::L),
            0x7E => self.bit_hl(7),
            0x7F => self.bit_r(7, Register::A),
            0x80 => self.res_bit_r(0, Register::B),
            0x81 => self.res_bit_r(0, Register::C),
            0x82 => self.res_bit_r(0, Register::D),
            0x83 => self.res_bit_r(0, Register::E),
            0x84 => self.res_bit_r(0, Register::H),
            0x85 => self.res_bit_r(0, Register::L),
            0x86 => self.res_bit_hl(0),
            0x87 => self.res_bit_r(0, Register::A),
            0x88 => self.res_bit_r(1, Register::B),
            0x89 => self.res_bit_r(1, Register::C),
            0x8A => self.res_bit_r(1, Register::D),
            0x8B => self.res_bit_r(1, Register::E),
            0x8C => self.res_bit_r(1, Register::H),
            0x8D => self.res_bit_r(1, Register::L),
            0x8E => self.res_bit_hl(1),
            0x8F => self.res_bit_r(1, Register::A),
            0x90 => self.res_bit_r(2, Register::B),
            0x91 => self.res_bit_r(2, Register::C),
            0x92 => self.res_bit_r(2, Register::D),
            0x93 => self.res_bit_r(2, Register::E),
            0x94 => self.res_bit_r(2, Register::H),
            0x95 => self.res_bit_r(2, Register::L),
            0x96 => self.res_bit_hl(2),
            0x97 => self.res_bit_r(2, Register::A),
            0x98 => self.res_bit_r(3, Register::B),
            0x99 => self.res_bit_r(3, Register::C),
            0x9A => self.res_bit_r(3, Register::D),
            0x9B => self.res_bit_r(3, Register::E),
            0x9C => self.res_bit_r(3, Register::H),
            0x9D => self.res_bit_r(3, Register::L),
            0x9E => self.res_bit_hl(3),
            0x9F => self.res_bit_r(3, Register::A),
            0xA0 => self.res_bit_r(4, Register::B),
            0xA1 => self.res_bit_r(4, Register::C),
            0xA2 => self.res_bit_r(4, Register::D),
            0xA3 => self.res_bit_r(4, Register::E),
            0xA4 => self.res_bit_r(4, Register::H),
            0xA5 => self.res_bit_r(4, Register::L),
            0xA6 => self.res_bit_hl(4),
            0xA7 => self.res_bit_r(4, Register::A),
            0xA8 => self.res_bit_r(5, Register::B),
            0xA9 => self.res_bit_r(5, Register::C),
            0xAA => self.res_bit_r(5, Register::D),
            0xAB => self.res_bit_r(5, Register::E),
            0xAC => self.res_bit_r(5, Register::H),
            0xAD => self.res_bit_r(5, Register::L),
            0xAE => self.res_bit_hl(5),
            0xAF => self.res_bit_r(5, Register::A),
            0xB0 => self.res_bit_r(6, Register::B),
            0xB1 => self.res_bit_r(6, Register::C),
            0xB2 => self.res_bit_r(6, Register::D),
            0xB3 => self.res_bit_r(6, Register::E),
            0xB4 => self.res_bit_r(6, Register::H),
            0xB5 => self.res_bit_r(6, Register::L),
            0xB6 => self.res_bit_hl(6),
            0xB7 => self.res_bit_r(6, Register::A),
            0xB8 => self.res_bit_r(7, Register::B),
            0xB9 => self.res_bit_r(7, Register::C),
            0xBA => self.res_bit_r(7, Register::D),
            0xBB => self.res_bit_r(7, Register::E),
            0xBC => self.res_bit_r(7, Register::H),
            0xBD => self.res_bit_r(7, Register::L),
            0xBE => self.res_bit_hl(7),
            0xBF => self.res_bit_r(7, Register::A),
            0xC0 => self.set_bit_r(0, Register::B),
            0xC1 => self.set_bit_r(0, Register::C),
            0xC2 => self.set_bit_r(0, Register::D),
            0xC3 => self.set_bit_r(0, Register::E),
            0xC4 => self.set_bit_r(0, Register::H),
            0xC5 => self.set_bit_r(0, Register::L),
            0xC6 => self.set_bit_hl(0),
            0xC7 => self.set_bit_r(0, Register::A),
            0xC8 => self.set_bit_r(1, Register::B),
            0xC9 => self.set_bit_r(1, Register::C),
            0xCA => self.set_bit_r(1, Register::D),
            0xCB => self.set_bit_r(1, Register::E),
            0xCC => self.set_bit_r(1, Register::H),
            0xCD => self.set_bit_r(1, Register::L),
            0xCE => self.set_bit_hl(1),
            0xCF => self.set_bit_r(1, Register::A),
            0xD0 => self.set_bit_r(2, Register::B),
            0xD1 => self.set_bit_r(2, Register::C),
            0xD2 => self.set_bit_r(2, Register::D),
            0xD3 => self.set_bit_r(2, Register::E),
            0xD4 => self.set_bit_r(2, Register::H),
            0xD5 => self.set_bit_r(2, Register::L),
            0xD6 => self.set_bit_hl(2),
            0xD7 => self.set_bit_r(2, Register::A),
            0xD8 => self.set_bit_r(3, Register::B),
            0xD9 => self.set_bit_r(3, Register::C),
            0xDA => self.set_bit_r(3, Register::D),
            0xDB => self.set_bit_r(3, Register::E),
            0xDC => self.set_bit_r(3, Register::H),
            0xDD => self.set_bit_r(3, Register::L),
            0xDE => self.set_bit_hl(3),
            0xDF => self.set_bit_r(3, Register::A),
            0xE0 => self.set_bit_r(4, Register::B),
            0xE1 => self.set_bit_r(4, Register::C),
            0xE2 => self.set_bit_r(4, Register::D),
            0xE3 => self.set_bit_r(4, Register::E),
            0xE4 => self.set_bit_r(4, Register::H),
            0xE5 => self.set_bit_r(4, Register::L),
            0xE6 => self.set_bit_hl(4),
            0xE7 => self.set_bit_r(4, Register::A),
            0xE8 => self.set_bit_r(5, Register::B),
            0xE9 => self.set_bit_r(5, Register::C),
            0xEA => self.set_bit_r(5, Register::D),
            0xEB => self.set_bit_r(5, Register::E),
            0xEC => self.set_bit_r(5, Register::H),
            0xED => self.set_bit_r(5, Register::L),
            0xEE => self.set_bit_hl(5),
            0xEF => self.set_bit_r(5, Register::A),
            0xF0 => self.set_bit_r(6, Register::B),
            0xF1 => self.set_bit_r(6, Register::C),
            0xF2 => self.set_bit_r(6, Register::D),
            0xF3 => self.set_bit_r(6, Register::E),
            0xF4 => self.set_bit_r(6, Register::H),
            0xF5 => self.set_bit_r(6, Register::L),
            0xF6 => self.set_bit_hl(6),
            0xF7 => self.set_bit_r(6, Register::A),
            0xF8 => self.set_bit_r(7, Register::B),
            0xF9 => self.set_bit_r(7, Register::C),
            0xFA => self.set_bit_r(7, Register::D),
            0xFB => self.set_bit_r(7, Register::E),
            0xFC => self.set_bit_r(7, Register::H),
            0xFD => self.set_bit_r(7, Register::L),
            0xFE => self.set_bit_hl(7),
            0xFF => self.set_bit_r(7, Register::A),
        }
    }


// OPCODE Functions
// region: Load Operations
    // 8-Bit Loads
    // ld r, s; ld d, r
    /// Load register with value of register
    fn ld_r_r(&mut self, reg1: Register, reg2: Register) -> u8{
        self.reg.set_register(reg1, self.reg.get_register(reg2));
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    /// Load register with value of immediate
    fn ld_r_i(&mut self, reg: Register) -> u8 {
        // Get immediate 8 bits
        let value = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1));

        // Load to proper register
        self.reg.set_register(reg, value);
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Load register with value of address (HL)
    fn ld_r_hl(&mut self, reg: Register) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        
        self.reg.set_register(reg, value);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    /// Load address (HL) with value of register
    fn ld_hl_r(&mut self, reg: Register) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.reg.get_register(reg);
        
        self.mmu.borrow_mut().write_byte(adr, value);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    /// Load 8-bit immediate value into address (HL)
    fn ld_hl_i(&mut self) -> u8 {
        let value = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1));
        let address = self.reg.get_double_register(Register::H, Register::L);

        self.mmu.borrow_mut().write_byte(address, value);
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 3;
    }

    // ld A, (ss)
    /// Load A with value at address in two registers
    fn ld_ra_aa(&mut self, reg1: Register, reg2: Register) -> u8 {
        let adr = self.reg.get_double_register(reg1, reg2);
        let value = self.mmu.borrow().read_byte(adr);

        self.reg.set_register(Register::A, value);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    /// Load A with value at address in immediate value
    fn ld_ra_ii(&mut self) -> u8 {
        let adr = self.mmu.borrow().read_word(self.reg.pc.wrapping_add(1));
        let value = self.mmu.borrow().read_byte(adr);

        self.reg.set_register(Register::A, value);
        self.reg.pc = self.reg.pc.wrapping_add(3);
        return 4;
    }

    // ld (dd), A
    /// Store contents of A in memory location specified by two registers
    fn ld_aa_ra(&mut self, reg1: Register, reg2: Register) -> u8 {
        self.mmu.borrow_mut().write_byte(self.reg.get_double_register(reg1, reg2), self.reg.get_register(Register::A));
        
        self.reg.pc.wrapping_add(1);
        return 2;
    }

    /// Store contents of A in memory location specified by immediate value
    fn ld_ii_ra(&mut self) -> u8 {
        let adr = self.mmu.borrow().read_word(self.reg.pc.wrapping_add(1));
        self.mmu.borrow_mut().write_byte(adr, self.reg.get_register(Register::A));
        
        self.reg.pc.wrapping_add(3);
        return 4;
    }

    // ld A, (C)
    /// Load A with value at address in C + 0xFF00
    fn ld_ra_ram(&mut self) -> u8 {
        let adr = 0xFF00 + self.reg.get_register(Register::C) as u16;
        let value = self.mmu.borrow().read_byte(adr);

        self.reg.set_register(Register::A, value);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // ld (C), A
    /// Store contents of A in memory location specified by register C + 0xFF00
    fn ld_ram_ra(&mut self) -> u8 {
        let adr = 0xFF00 + self.reg.get_register(Register::C) as u16;
        let value = self.reg.get_register(Register::A);

        self.mmu.borrow_mut().write_byte(adr, value);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // ldd A, (HL)
    /// Load A with value at address in HL, decrement HL
    fn lddec_a_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);

        self.reg.set_register(Register::A, value);
        self.reg.set_double_register(Register::H, Register::L, adr.wrapping_sub(1));
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // ldd (HL), A
    /// Store contents of A in memory location specified by HL, decrement HL
    fn lddec_hl_a(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.reg.get_register(Register::A);

        self.mmu.borrow_mut().write_byte(adr, value);
        self.reg.set_double_register(Register::H, Register::L, adr.wrapping_sub(1));
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // ldi A, (HL)
    /// Load A with value at address in HL, increment HL
    fn ldinc_a_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);

        self.reg.set_register(Register::A, value);
        self.reg.set_double_register(Register::H, Register::L, adr.wrapping_add(1));
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // ldi (HL), A
    /// Store contents of A in memory location specified by HL, increment HL
    fn ldinc_hl_a(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.reg.get_register(Register::A);

        self.mmu.borrow_mut().write_byte(adr, value);
        self.reg.set_double_register(Register::H, Register::L, adr.wrapping_add(1));
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // ldh (n), A
    /// Store contents of A in memory location specified by immediate value + 0xFF00
    fn ldi_ram_a(&mut self) -> u8 {
        let address = 0xFF00 + self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1)) as u16;
        let value = self.reg.get_register(Register::A);
        self.mmu.borrow_mut().write_byte(address, value);
        
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 3;
    }

    // ldh A, (n)
    /// Load A with value at address specified by immediate value + 0xFF00
    fn ldi_a_ram(&mut self) -> u8 {
        let address = 0xFF00 + self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1)) as u16;
        let value = self.mmu.borrow_mut().read_byte(address);
        self.reg.set_register(Register::A, value);
        
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 3
    }

    // 16-Bit Loads
    // ld dd, nn
    /// Load 16-bit immediate value into double register
    fn ldi_rr(&mut self, reg1: Register, reg2: Register) -> u8 {
        let value = self.mmu.borrow().read_word(self.reg.pc);
        self.reg.set_double_register(reg1, reg2, value);
        
        self.reg.pc = self.reg.pc.wrapping_add(3);
        return 3;
    }

    // ld (nn), SP
    /// Store contents of SP in memory location specified by immediate value
    fn ldi_ram_sp(&mut self) -> u8 {
        let address = self.mmu.borrow().read_word(self.reg.pc);
        let value = self.reg.get_double_register(Register::SP, Register::SP);
        self.mmu.borrow_mut().write_word(address, value);
        
        self.reg.pc = self.reg.pc.wrapping_add(3);
        return 5;
    }

    // ld SP, HL
    /// Load SP with HL
    fn ld_sp_hl(&mut self) -> u8 {
        let value = self.reg.get_double_register(Register::H, Register::L);
        self.reg.set_double_register(Register::SP, Register::SP, value);
        
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // ld HL, (SP+e)
    /// Load HL with value at address specified by SP + immediate signed value
    fn ld_hl_sp(&mut self) -> u8 {
        let e = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1)) as i8;
        let sp = self.reg.get_double_register(Register::SP, Register::SP);
        let result = sp.wrapping_add(e as u16);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, false);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, ((sp & 0x0F) + (e & 0x0F) as u16) > 0x0F);
        self.reg.set_flag(Flag::CARRY, ((sp & 0xFF) + (e & 0xFF) as u16) > 0xFF);

        self.reg.set_double_register(Register::H, Register::L, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 3;
    }

    // push ss
    /// Push double register onto stack
    fn push(&mut self, reg1: Register, reg2: Register) -> u8 {
        self.reg.sp = self.reg.sp.wrapping_sub(1);
        self.mmu.borrow_mut().write_byte(self.reg.sp, self.reg.get_register(reg1));
        self.reg.sp = self.reg.sp.wrapping_sub(1);
        self.mmu.borrow_mut().write_byte(self.reg.sp, self.reg.get_register(reg2));

        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 4;
    }

    // pop dd
    /// Pop double register from stack
    fn pop(&mut self, reg1: Register, reg2: Register) -> u8 {
        let value1 = self.mmu.borrow().read_byte(self.reg.sp);
        self.reg.sp = self.reg.sp.wrapping_add(1);
        let value2 = self.mmu.borrow().read_byte(self.reg.sp);
        self.reg.sp = self.reg.sp.wrapping_add(1);

        self.reg.set_register(reg1, value1);
        self.reg.set_register(reg2, value2);

        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 3;
    }

    /// Push PC to stack
    fn push_pc(&mut self) {
        self.reg.sp = self.reg.sp.wrapping_sub(1);
        self.mmu.borrow_mut().write_byte(self.reg.sp, (self.reg.pc >> 8) as u8);
        self.reg.sp = self.reg.sp.wrapping_sub(1);
        self.mmu.borrow_mut().write_byte(self.reg.sp, self.reg.pc as u8);
    }

    /// Pop PC from stack
    fn pop_pc(&mut self) -> u16 {
        let value1 = self.mmu.borrow().read_byte(self.reg.sp);
        self.reg.sp = self.reg.sp.wrapping_add(1);
        let value2 = self.mmu.borrow().read_byte(self.reg.sp);
        self.reg.sp = self.reg.sp.wrapping_add(1);

        return ((value1 as u16) << 8) | value2 as u16;
    }

    // endregion

// region: ALU Operations
    // 8-Bit Operations
    // add A, s
    /// Add reg to register A
    fn add_r(&mut self, reg: Register) -> u8 {
        let value = self.reg.get_register(reg);
        let a = self.reg.get_register(Register::A);
        let (result, carry) = a.overflowing_add(value);
        
        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, ((result & 0x0F) + (value & 0x0F) as u8) > 0x0F);
        self.reg.set_flag(Flag::CARRY, carry);
        
        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    /// Add immediate to register A
    fn add_i(&mut self) -> u8 {
        let value = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1));
        let a = self.reg.get_register(Register::A);
        let (result, carry) = a.overflowing_add(value);
        
        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, ((result & 0x0F) + (value & 0x0F) as u8) > 0x0F);
        self.reg.set_flag(Flag::CARRY, carry);
        
        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Add address (HL) to register A
    fn add_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let a = self.reg.get_register(Register::A);
        let (result, carry) = a.overflowing_add(value);
        
        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, ((result & 0x0F) + (value & 0x0F) as u8) > 0x0F);
        self.reg.set_flag(Flag::CARRY, carry);
        
        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // adc A, s
    /// Add reg with carry to reg A
    fn adc_r(&mut self, reg: Register) -> u8{
        let value = self.reg.get_register(reg);
        let carry = self.reg.get_flag(Flag::CARRY);
        let (result, carry1) = self.reg.get_register(Register::A).overflowing_add(value);
        let (result, carry2) = result.overflowing_add(carry as u8);
        
        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, ((result & 0x0F) + (value & 0x0F) + carry as u8) > 0x0F);
        self.reg.set_flag(Flag::CARRY, carry1 || carry2);
        
        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    /// Add immediate with carry to reg A
    fn adc_i(&mut self) -> u8 {
        let value = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1));
        let carry = self.reg.get_flag(Flag::CARRY);
        let (result, carry1) = self.reg.get_register(Register::A).overflowing_add(value);
        let (result, carry2) = result.overflowing_add(carry as u8);
        
        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, ((result & 0x0F) + (value & 0x0F) + carry as u8) > 0x0F);
        self.reg.set_flag(Flag::CARRY, carry1 || carry2);
        
        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    /// Add address (HL) with carry to reg A
    fn adc_hl(&mut self) -> u8{
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let carry = self.reg.get_flag(Flag::CARRY);
        let (result, carry1) = self.reg.get_register(Register::A).overflowing_add(value);
        let (result, carry2) = result.overflowing_add(carry as u8);
        
        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, ((result & 0x0F) + (value & 0x0F) + carry as u8) > 0x0F);
        self.reg.set_flag(Flag::CARRY, carry1 || carry2);
        
        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // sub s
    /// Subtract reg from register A
    fn sub_r(&mut self, reg: Register) -> u8 {
        // Compute Result
        let value = self.reg.get_register(reg);
        let (result, borrow) = self.reg.get_register(Register::A).overflowing_sub(value);

        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, (result & 0x0F) < (value & 0x0F));
        self.reg.set_flag(Flag::CARRY, borrow);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    /// Subtract immediate from register A
    fn sub_i(&mut self) -> u8 {
        let value = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1));
        let (result, borrow) = self.reg.get_register(Register::A).overflowing_sub(value);

        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, (result & 0x0F) < (value & 0x0F));
        self.reg.set_flag(Flag::CARRY, borrow);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Subtract address (HL) from register A
    fn sub_hl(&mut self) -> u8 {
        // Compute Result
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let (result, borrow) = self.reg.get_register(Register::A).overflowing_sub(value);

        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, (result & 0x0F) < (value & 0x0F));
        self.reg.set_flag(Flag::CARRY, borrow);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // sbc A, s
    /// Subtract reg with carry from reg A
    fn sbc_r(&mut self, reg: Register) -> u8 {
        // Compute Result
        let value = self.reg.get_register(reg);        
        let carry = self.reg.get_flag(Flag::CARRY);

        let (result, borrow1) = self.reg.get_register(Register::A).overflowing_sub(value);
        let (result, borrow2) = result.overflowing_sub(carry as u8);

        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, (result & 0x0F) < ((value & 0x0F) + carry as u8));
        self.reg.set_flag(Flag::CARRY, borrow1 || borrow2);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    /// Subtract immediate with carry from reg A
    fn sbc_i(&mut self) -> u8 {
        // Compute Result
        let value = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1));        
        let carry = self.reg.get_flag(Flag::CARRY);

        let (result, borrow1) = self.reg.get_register(Register::A).overflowing_sub(value);
        let (result, borrow2) = result.overflowing_sub(carry as u8);

        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, (result & 0x0F) < ((value & 0x0F) + carry as u8));
        self.reg.set_flag(Flag::CARRY, borrow1 || borrow2);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Subtract address (HL) with carry from reg A
    fn sbc_hl(&mut self) -> u8 {
        // Compute Result
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);        
        let carry = self.reg.get_flag(Flag::CARRY);

        let (result, borrow1) = self.reg.get_register(Register::A).overflowing_sub(value);
        let (result, borrow2) = result.overflowing_sub(carry as u8);

        // Set flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, (result & 0x0F) < ((value & 0x0F) + carry as u8));
        self.reg.set_flag(Flag::CARRY, borrow1 || borrow2);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // and s
    /// Logical AND reg with register A
    fn and_r(&mut self, reg: Register) -> u8 {
        // Calulate result
        let value = self.reg.get_register(reg);
        let result = self.reg.get_register(Register::A) & value;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, true);
        self.reg.set_flag(Flag::CARRY, false);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    /// Logical AND immediate with register A
    fn and_i(&mut self) -> u8 {
        // Calulate result
        let value = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1));
        let result = self.reg.get_register(Register::A) & value;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, true);
        self.reg.set_flag(Flag::CARRY, false);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Logical AND address (HL) with register A
    fn and_hl(&mut self) -> u8 {
        // Calulate result
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let result = self.reg.get_register(Register::A) & value;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, true);
        self.reg.set_flag(Flag::CARRY, false);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // or s
    /// Logical OR reg with register A
    fn or_r(&mut self, reg: Register) -> u8 {
        // Calulate result
        let value = self.reg.get_register(reg);
        let result = self.reg.get_register(Register::A) | value;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, false);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    /// Logical OR immediate with register A
    fn or_i(&mut self) -> u8 {
        // Calulate result
        let value = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1));
        let result = self.reg.get_register(Register::A) | value;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, false);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Logical OR address (HL) with register A
    fn or_hl(&mut self) -> u8 {
        // Calulate result
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let result = self.reg.get_register(Register::A) | value;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, false);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // xor s
    /// Logical XOR reg with register A
    fn xor_r(&mut self, reg: Register) -> u8 {
        // Calulate result
        let value = self.reg.get_register(reg);
        let result = self.reg.get_register(Register::A) ^ value;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, false);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    /// Logical XOR immediate with register A
    fn xor_i(&mut self) -> u8 {
        // Calulate result
        let value = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1));
        let result = self.reg.get_register(Register::A) ^ value;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, false);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Logical XOR address (HL) with register A
    fn xor_hl(&mut self) -> u8 {
        // Calulate result
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let result = self.reg.get_register(Register::A) ^ value;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, false);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // cp s
    /// Compare reg with register A
    fn cp_r(&mut self, reg: Register) -> u8 {
        // Calulate result
        let value = self.reg.get_register(reg);
        let (result, borrow) = self.reg.get_register(Register::A).overflowing_sub(value);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, (result & 0x0F) < ((value & 0x0F)));
        self.reg.set_flag(Flag::CARRY, borrow);

        // Continue
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    /// Compare immediate with register A
    fn cp_i(&mut self) -> u8 {
        // Calulate result
        let value = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1));
        let (result, borrow) = self.reg.get_register(Register::A).overflowing_sub(value);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, (result & 0x0F) < ((value & 0x0F)));
        self.reg.set_flag(Flag::CARRY, borrow);

        // Continue
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Compare address (HL) with register A
    fn cp_hl(&mut self) -> u8 {
        // Calulate result
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let (result, borrow) = self.reg.get_register(Register::A).overflowing_sub(value);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, (result & 0x0F) < ((value & 0x0F)));
        self.reg.set_flag(Flag::CARRY, borrow);

        // Continue
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // inc s
    /// Increment register
    fn inc(&mut self, reg: Register) -> u8 {
        let value = self.reg.get_register(reg).wrapping_add(1);
        self.reg.set_register(reg, value);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, value == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, (value & 0x0F) == 0);

        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    /// Increment address (HL)
    fn inc_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr).wrapping_add(1);
        self.mmu.borrow_mut().write_byte(adr, value);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, value == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, (value & 0x0F) == 0);

        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 3;
    }

    // dec s
    /// Decrement register
    fn dec(&mut self, reg: Register) -> u8 {
        let value = self.reg.get_register(reg).wrapping_sub(1);
        self.reg.set_register(reg, value);


        // Set Flags
        self.reg.set_flag(Flag::ZERO, value == 0);
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, (value & 0x0F) == 0x0F);

        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    /// Decrement address (HL)
    fn dec_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr).wrapping_sub(1);
        self.mmu.borrow_mut().write_byte(adr, value);


        // Set Flags
        self.reg.set_flag(Flag::ZERO, value == 0);
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, (value & 0x0F) == 0x0F);

        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 3;
    }

    // 16-Bit Operations
    // add HL, ss
    /// Add double register to HL
    fn add_hl_rr(&mut self, reg1: Register, reg2: Register) -> u8 {
        let value = self.reg.get_double_register(reg1, reg2);
        let result = self.reg.get_double_register(Register::H, Register::L).wrapping_add(value);
        self.reg.set_double_register(Register::H, Register::L, result);

        // Set Flags
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, (result & 0x0FFF) < (value & 0x0FFF));
        self.reg.set_flag(Flag::CARRY, result > 0xFFFF - value);

        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 2;
    }

    // add SP, e
    /// Add immediate signed value to SP
    fn add_sp_i(&mut self) -> u8 {
        let value = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1)) as i8;
        let result = self.reg.sp.wrapping_add(value as u16);
        self.reg.sp = result;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, false);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, (result & 0x0F) + ((value as u16) & 0x0F) > 0x0F);
        self.reg.set_flag(Flag::CARRY, (result & 0xFF) + ((value as u16) & 0xFF) > 0xFF);

        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 4;
    }

    // inc ss
    /// Increment Double Register
    fn inc_rr(&mut self, reg1: Register, reg2: Register) -> u8 {
        self.reg.set_double_register(reg1, reg2, self.reg.get_double_register(reg1, reg2).wrapping_add(1));
        self.reg.pc = self.reg.pc.wrapping_add(1);

        return 2;
    }

    // dec ss
    /// Decrement Double Register
    fn dec_rr(&mut self, reg1: Register, reg2: Register) -> u8 {
        self.reg.set_double_register(reg1, reg2, self.reg.get_double_register(reg1, reg2).wrapping_sub(1));
        self.reg.pc = self.reg.pc.wrapping_add(1);

        return 2;
    }
// endregion

// region: Rotates and Shifts Operations
    // rlc s
    /// Rotate A left
    fn rlc_a(&mut self) -> u8 {
        let value = self.reg.get_register(Register::A);
        let new_carry = (value & 0x80) >> 7;
        let result = (value << 1) | new_carry;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, false);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);

        return 1;
    }

    /// Rotate register left
    fn rlc_r(&mut self, reg: Register) -> u8 {
        let value = self.reg.get_register(reg);
        let new_carry = (value & 0x80) >> 7;
        let result = (value << 1) | new_carry;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        self.reg.set_register(reg, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Rotate address (HL) left
    fn rlc_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let new_carry = (value & 0x80) >> 7;
        let result = (value << 1) | new_carry;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        self.mmu.borrow_mut().write_byte(adr, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 4;
    }

    // rla
    /// Rotate A left through carry
    fn rl_a(&mut self) -> u8 {
        let value = self.reg.get_register(Register::A);
        let new_carry = (value & 0x80) >> 7;
        let result = (value << 1) | self.reg.get_flag(Flag::CARRY) as u8;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, false);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);

        return 1;
    }

    /// Rotate register left through carry
    fn rl_r(&mut self, reg: Register) -> u8 {
        let value = self.reg.get_register(reg);
        let new_carry = (value & 0x80) >> 7;
        let result = (value << 1) | self.reg.get_flag(Flag::CARRY) as u8;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);

        return 2;
    }

    /// Rotate address (HL) left through carry
    fn rl_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let new_carry = (value & 0x80) >> 7;
        let result = (value << 1) | self.reg.get_flag(Flag::CARRY) as u8;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.mmu.borrow_mut().write_byte(adr, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);

        return 4;
    }

    // rrca
    /// Rotate A right
    fn rrc_a(&mut self) -> u8 {
        let value = self.reg.get_register(Register::A);
        let new_carry = value & 0x01;
        let result = (value >> 1) | (new_carry << 7);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, false);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);

        return 1;
    }

    /// Rotate register right
    fn rrc_r(&mut self, reg: Register) -> u8 {
        let value = self.reg.get_register(reg);
        let new_carry = value & 0x01;
        let result = (value >> 1) | (new_carry << 7);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.reg.set_register(reg, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);

        return 2;
    }

    /// Rotate address (HL) right
    fn rrc_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let new_carry = value & 0x01;
        let result = (value >> 1) | (new_carry << 7);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.mmu.borrow_mut().write_byte(adr, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);

        return 4;
    }

    // rra
    /// Rotate A right through carry
    fn rr_a(&mut self) -> u8 {
        let carry = self.reg.get_flag(Flag::CARRY);
        let value = self.reg.get_register(Register::A);
        let new_carry = value & 0x01;
        let result = (value >> 1) | ((carry as u8) << 7);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, false);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.reg.set_register(Register::A, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);

        return 1;
    }

    // rr s
    /// Rotate register right through carry
    fn rr_r(&mut self, reg: Register) -> u8 {
        let carry = self.reg.get_flag(Flag::CARRY);
        let value = self.reg.get_register(reg);
        let new_carry = value & 0x01;
        let result = (value >> 1) | ((carry as u8) << 7);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.reg.set_register(reg, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);

        return 2;
    }

    /// Rotate address (HL) right through carry
    fn rr_hl(&mut self) -> u8 {
        let carry = self.reg.get_flag(Flag::CARRY);
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let new_carry = value & 0x01;
        let result = (value >> 1) | ((carry as u8) << 7);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.mmu.borrow_mut().write_byte(adr, result);
        self.reg.pc = self.reg.pc.wrapping_add(2);

        return 4;
    }

    // sla s
    /// Shift register left
    fn sla_r(&mut self, reg: Register) -> u8 {
        let value = self.reg.get_register(reg);
        let new_carry = (value & 0x80) != 0; // Bit 7 to CY flag
        let result = value << 1;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry);

        // Store result and continue
        self.reg.set_register(reg, result);

        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Shift address (HL) left
    fn sla_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let new_carry = (value & 0x80) != 0; // Bit 7 to CY flag
        let result = value << 1;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry);

        // Store result and continue
        self.mmu.borrow_mut().write_byte(adr, result);

        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 4;
    }

    // sra s
    /// Shift register right
    fn sra_r(&mut self, reg: Register) -> u8 {
        let value = self.reg.get_register(reg);
        let new_carry = value & 0x01; // Bit 0 to CY flag
        let result = (value & 0x80) | (value >> 1);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.reg.set_register(reg, result);

        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Shift address (HL) right
    fn sra_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let new_carry = value & 0x01; // Bit 0 to CY flag
        let result = (value & 0x80) | (value >> 1);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.mmu.borrow_mut().write_byte(adr, result);

        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 4;
    }

    // srl s
    /// Shift regitser right
    fn srl_r(&mut self, reg: Register) -> u8 {
        let value = self.reg.get_register(reg);
        let new_carry = value & 0x01; // Bit 0 to CY flag
        let result = value >> 1;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.reg.set_register(reg, result);

        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Shift address (HL) right
    fn srl_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let new_carry = value & 0x01; // Bit 0 to CY flag
        let result = value >> 1;

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, new_carry == 1);

        // Store result and continue
        self.mmu.borrow_mut().write_byte(adr, result);

        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 4;
    }
// endregion

// region: Bit Operations
    // bit b, s
    /// Copy Complements of Given Bit to Zero Flag
    fn bit_r(&mut self, bit: u8, reg: Register) -> u8 {
        let value = self.reg.get_register(reg);
        let result = value & (1 << bit);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, true);

        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 2;
    }

    /// Copy complement of given bit in address (HL) to Zero Flag
    fn bit_hl(&mut self, bit: u8) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let result = value & (1 << bit);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, true);

        self.reg.pc = self.reg.pc.wrapping_add(2);
        return 3;
    }

    // set b, s
    /// Set Given Bit in Register
    fn set_bit_r(&mut self, bit: u8, reg: Register) -> u8 {
        let mut value = self.reg.get_register(reg);
        value |= 1 << bit;

        self.reg.set_register(reg, value);
        self.reg.pc = self.reg.pc.wrapping_add(2);

        return 2;
    }

    /// Set Given Bit in Address (HL)
    fn set_bit_hl(&mut self, bit: u8) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let mut value = self.mmu.borrow().read_byte(adr);
        value |= 1 << bit;

        self.mmu.borrow_mut().write_byte(adr, value);
        self.reg.pc = self.reg.pc.wrapping_add(2);

        return 4;
    }

    // res b, s
    /// Reset Given Bit in register
    fn res_bit_r(&mut self, bit: u8, reg: Register) -> u8 {
        let mut value = self.reg.get_register(reg);
        value &= 0 << bit;

        self.reg.set_register(reg, value);
        self.reg.pc = self.reg.pc.wrapping_add(2);

        return 2;
    }

    /// Reset Given Bit in Address (HL)
    fn res_bit_hl(&mut self, bit: u8) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let mut value = self.mmu.borrow().read_byte(adr);
        value &= 0 << bit;

        self.mmu.borrow_mut().write_byte(adr, value);
        self.reg.pc = self.reg.pc.wrapping_add(2);

        return 4;
    }
// endregion

// region: Jump Operations
    // jp nn
    /// Jump to immediate 16-bit value
    fn jp(&mut self) -> u8 {
        let address = self.mmu.borrow().read_word(self.reg.pc.wrapping_add(1));
        self.reg.pc = address;

        return 4;
    }

    // jp cc, nn
    /// Conditional Jump to immediate 16-bit value
    fn jpcc(&mut self, flag: Flag, condition: bool) -> u8 {
        if self.reg.get_flag(flag) == condition {
            let address = self.mmu.borrow().read_word(self.reg.pc.wrapping_add(1));
            self.reg.pc = address;
            return 4;
        }
        else {
            self.reg.pc = self.reg.pc.wrapping_add(3);
            return 3;
        }
    }

    // jp (HL)
    /// Jump to address in HL
    fn jphl(&mut self) -> u8 {
        let address = self.reg.get_double_register(Register::H, Register::L);
        self.reg.pc = address;

        return 1;
    }

    // jr e
    /// Jump to immediate signed value
    fn jr(&mut self) -> u8 {
        let offset = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1)) as i8;
        self.reg.pc = self.reg.pc.wrapping_add(2).wrapping_add(offset as u16);

        return 3;
    }

    // jr cc, e
    /// Conditional Jump to immediate signed value
    fn jrcc(&mut self, flag: Flag, condition: bool) -> u8 {
        if self.reg.get_flag(flag) == condition {
            let offset = self.mmu.borrow().read_byte(self.reg.pc.wrapping_add(1)) as i8;
            self.reg.pc = self.reg.pc.wrapping_add(2).wrapping_add(offset as u16);

            return 3;
        }
        else {
            self.reg.pc = self.reg.pc.wrapping_add(2);
            return 2;
        }
    }
// endregion

// region: Call Operations
    // call nn
    /// Call immediate 16-bit value
    fn call(&mut self) -> u8 {
        let address = self.mmu.borrow().read_word(self.reg.pc.wrapping_add(1));
        self.reg.pc = self.reg.pc.wrapping_add(3);
        self.push_pc();
        self.reg.pc = address;

        return 6;
    }

    // call cc, nn
    /// Conditional Call to immediate 16-bit value
    fn callcc(&mut self, flag: Flag, condition: bool) -> u8 {
        if self.reg.get_flag(flag) == condition {
            let address = self.mmu.borrow().read_word(self.reg.pc.wrapping_add(1));
            self.reg.pc = self.reg.pc.wrapping_add(3);
            self.push(Register::PC, Register::PC);
            self.reg.pc = address;

            return 6;
        }
        else {
            self.reg.pc = self.reg.pc.wrapping_add(3);
            return 3;
        }

    }

    // RST f
    /// Push present address onto stack and jump to address n
    fn rst(&mut self, n: u8) -> u8 {
        self.push_pc();
        self.reg.pc = (n as u16) * 0x08;

        return 4;
    }
// endregion

// region: Return Operations
    // ret
    /// Return from subroutine
    fn ret(&mut self) -> u8{
        self.reg.pc = self.pop_pc();
        return 4;
    }

    // ret cc
    /// Conditional Return from subroutine
    fn retcc(&mut self, flag: Flag, condition: bool) -> u8 {
        if self.reg.get_flag(flag) == condition {
            self.reg.pc = self.pop_pc();
            return 5;
        }
        else {
            self.reg.pc = self.reg.pc.wrapping_add(1);
            return 2;
        }
    }

    // reti
    /// Return from interrupt
    fn reti(&mut self) -> u8 {
        self.pop_pc();
        self.ime = true;

        return 4;
    }
// endregion

// region: Miscellaneuous
    // swap s
    /// Swap Nibles in register
    fn swap_r(&mut self, reg: Register) -> u8 {
        let value = self.reg.get_register(reg);
        let result = (value << 4) | (value >> 4);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, false);

        // Store result and continue
        self.reg.set_register(reg, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);

        return 2;
    }

    /// Swap Nibles in address (HL)
    fn swap_hl(&mut self) -> u8 {
        let adr = self.reg.get_double_register(Register::H, Register::L);
        let value = self.mmu.borrow().read_byte(adr);
        let result = (value << 4) | (value >> 4);

        // Set Flags
        self.reg.set_flag(Flag::ZERO, result == 0);
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, false);

        // Store result and continue
        self.mmu.borrow_mut().write_byte(adr, result);
        self.reg.pc = self.reg.pc.wrapping_add(1);

        return 4;
    }

    // daa
    /// Decimal Adjust A
    fn daa(&mut self) -> u8 {
        let mut a = self.reg.get_register(Register::A);
        let mut adjust = 0;
        let carry = self.reg.get_flag(Flag::CARRY);
        let half_carry = self.reg.get_flag(Flag::HC);

        if self.reg.get_flag(Flag::SUB) {
            if half_carry {
                adjust |= 0x06;
            }
            if carry {
                adjust |= 0x60;
            }
            a = a.wrapping_sub(adjust);
        } else {
            if half_carry || (a & 0x0F) > 0x09 {
                adjust |= 0x06;
            }
            if carry || a > 0x99 {
                adjust |= 0x60;
            }
            a = a.wrapping_add(adjust);
        }

        self.reg.set_register(Register::A, a);

        // Set flags
        self.reg.set_flag(Flag::ZERO, a == 0);
        self.reg.set_flag(Flag::HC, false);
        if adjust >= 0x60 {
            self.reg.set_flag(Flag::CARRY, true);
        }

        return 1;
    }

    // cpl
    /// Complement A
    fn cpl(&mut self) -> u8 {
        self.reg.set_register(Register::A, !self.reg.get_register(Register::A));
        self.reg.set_flag(Flag::SUB, true);
        self.reg.set_flag(Flag::HC, true);
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    // ccf
    /// Complement Carry Flag
    fn ccf(&mut self) -> u8 {
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, !self.reg.get_flag(Flag::CARRY));
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    // scf
    /// Set Carry Flag
    fn scf(&mut self) -> u8 {
        self.reg.set_flag(Flag::SUB, false);
        self.reg.set_flag(Flag::HC, false);
        self.reg.set_flag(Flag::CARRY, true);
        self.reg.pc = self.reg.pc.wrapping_add(1);

        return 1;
    }

    // nop
    /// No Operation
    fn nop(&mut self) -> u8 {
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    // halt
    /// Halt CPU
    fn halt(&mut self) -> u8 {
        panic!("Halt not implemented");
        //return 1;
    }

    // stop
    /// Stop CPU
    fn stop(&mut self) -> u8 {
        panic!("Stop not implemented");
        //return 1;
    }

    // di
    /// Disable Interrupts
    fn di(&mut self) -> u8 {
        self.ime = false;
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }

    // ei
    /// Enable Interrupts
    fn ei(&mut self) -> u8 {
        self.ime = true;
        self.reg.pc = self.reg.pc.wrapping_add(1);
        return 1;
    }
// endregion

    /// Print out the current state of the CPU
    pub fn log_state(&self, opcode: u8, pc: u16) {
        println!(
            "PC: {:04X}, Opcode: {:02X}, op1: {:02X}, op2: {:02X}, op3: {:02X}\n
            AF: {:02X}{:02X}, BC: {:02X}{:02X}, DE: {:02X}{:02X}, HL: {:02X}{:02X},\n
            (AF): {:02X}, (BC): {:02X}, (DE): {:02X} (HL): {:02X} SP: {:04X},\n
            Z:{} N:{} H:{} C:{}",
            pc,
            opcode,
            self.mmu.borrow().read_word(self.reg.pc),
            self.mmu.borrow().read_byte(self.reg.pc + 2),
            self.mmu.borrow().read_byte(self.reg.pc + 3),
            self.reg.get_register(Register::A),
            self.reg.get_register(Register::F),
            self.reg.get_register(Register::B),
            self.reg.get_register(Register::C),
            self.reg.get_register(Register::D),
            self.reg.get_register(Register::E),
            self.reg.get_register(Register::H),
            self.reg.get_register(Register::L),
            self.mmu.borrow().read_byte(self.reg.get_double_register(Register::A, Register::F)),
            self.mmu.borrow().read_byte(self.reg.get_double_register(Register::B, Register::C)),
            self.mmu.borrow().read_byte(self.reg.get_double_register(Register::D, Register::E)),
            self.mmu.borrow().read_byte(self.reg.get_double_register(Register::H, Register::L)),
            self.reg.sp,
            self.reg.get_flag(Flag::ZERO),
            self.reg.get_flag(Flag::SUB),
            self.reg.get_flag(Flag::HC),
            self.reg.get_flag(Flag::CARRY),
        );
    }

}