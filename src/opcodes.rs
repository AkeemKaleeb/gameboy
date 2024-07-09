/**************************************************************************************************
* File Name         : opcodes.rs
* Description       : Defines the implementation of opcodes and their associated operations
**************************************************************************************************/
#![allow(unused_variables)]
#![allow(dead_code)]

use crate::{cpu::CPU, registers::Registers};

pub struct OPReturn {
    ran: bool,
    inc: u8,
}

fn get_target(register: &Registers, index: u8) -> u8 {
    match index {
        0 => register.reg_b,
        1 => register.reg_c,
        2 => register.reg_d,
        3 => register.reg_e,
        4 => register.reg_h,
        5 => register.reg_l,
        //6 => register.get_hl(), 
        7 => register.reg_a,
        _ => panic!("Index out of bounds"),
    }
}

pub fn decode_execute(cpu: &mut CPU, opcode: u8) -> u8 {
    let prefixed = opcode == 0xCB;
    let mut opreturn: OPReturn = if opcode == 0xCB {
        execute_prefixed(cpu, cpu.memory.read_byte(cpu.pc + 1))
    }
    else {
        execute_horizontal(cpu, opcode)
    };

    if !opreturn.ran {
        opreturn = execute_vertical(cpu, opcode);
    }

    if opreturn.ran {
        return opreturn.inc;
    }
    panic!("Unknown opcode: 0x{:02X}", opcode);
}

// Implement opcode stored horizontally execution
pub fn execute_horizontal(cpu: &mut CPU, opcode: u8) -> OPReturn {
    // Horizontal opcodes 0x40-0xBF
    match opcode & 0xF0 {
        // Load Instructions
        0x40 => match opcode & 0x0F{
            0x0..=0x7 => {}
            0x8..=0xF => {}
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        // Load Instructions
        0x50 => match opcode & 0x0F{
            0x0..=0x7 => {}
            0x8..=0xF => {}
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        // Load Instructions
        0x60 => match opcode & 0x0F{
            0x0..=0x7 => {}
            0x8..=0xF => {}
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        // Load Instructions
        0x70 => match opcode & 0x0F{
            0x0..=0x7 => {}
            0x8..=0xF => {}
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        // ADD and ADC Instructions
        0x80 => match opcode & 0x0F{
            0x0..=0x7 => { add(cpu, opcode & 0x07); return OPReturn{ran: true, inc: 1}; }
            0x8..=0xF => { adc(cpu, opcode & 0x07); return OPReturn{ran: true, inc: 1}; }
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        // SUB and SBC Instructions
        0x90 => match opcode & 0x0F{
            0x0..=0x7 => { sub(cpu, opcode & 0x07); return OPReturn{ran: true, inc: 1}; }
            0x8..=0xF => { sbc(cpu, opcode & 0x07); return OPReturn{ran: true, inc: 1}; }
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        // AND and XOR Instructions
        0xA0 => match opcode & 0x0F{
            0x0..=0x7 => { and(cpu, opcode & 0x07); return OPReturn{ran: true, inc: 1}; }
            0x8..=0xF => { xor(cpu, opcode & 0x07); return OPReturn{ran: true, inc: 1}; }
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        // OR and Compare Instructions
        0xB0 => match opcode & 0x0F{
            0x0..=0x7 => { or(cpu, opcode & 0x07); return OPReturn{ran: true, inc: 1}; }
            0x8..=0xF => { cp(cpu, opcode & 0x07); return OPReturn{ran: true, inc: 1}; }
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        _ => panic!("Unknown opcode: 0x{:02X}", opcode),        // Skip unknown code
    }
    OPReturn{ran: false, inc: 0}
}

fn execute_vertical(cpu: &mut CPU, opcode: u8) -> OPReturn {
    OPReturn{ran: false, inc: 0}
}

fn execute_prefixed(cpu: &mut CPU, opcode: u8) -> OPReturn {
    OPReturn{ran: false, inc: 0}
}

fn set_flags(cpu: &mut CPU, zero: bool, sub: bool, hc: bool, carry: bool) {
    cpu.registers.reg_f.zero = zero;
    cpu.registers.reg_f.subtract = sub;
    cpu.registers.reg_f.half_carry = hc;
    cpu.registers.reg_f.carry = carry;
}

// No operation
fn nop(_cpu: &mut CPU) {
}

// Load bc register with 2 byte value
fn ld_bc_nn(cpu: &mut CPU, value: u16) {
    cpu.registers.set_bc(value);
}

// Add value with reg_a and store in reg_a
fn add(cpu: &mut CPU, reg_code: u8){
    let value = get_target(&cpu.registers, reg_code);
    let(result, did_overflow) = cpu.registers.reg_a.overflowing_add(value);
    
    set_flags(cpu, result == 0, false, (cpu.registers.reg_a & 0xF) + (value & 0xF) > 0xF, did_overflow);
    cpu.registers.reg_a = result;
}

// Add value with reg_a and store in reg_a with the carry bit
fn adc(cpu: &mut CPU, reg_code: u8) {
    let value = get_target(&cpu.registers, reg_code);
    let carry = if cpu.registers.reg_f.carry { 1 } else { 0 };
    let (result, did_overflow) = cpu.registers.reg_a.overflowing_add(value.wrapping_add(carry));

    set_flags(cpu, result == 0, false, (cpu.registers.reg_a & 0xF) + (carry) > 0xF, did_overflow || (value as u16 + carry as u16) > 0xFF);
    cpu.registers.reg_a = result;
}

// Subtract value from reg_a and store in reg_a
fn sub(cpu: &mut CPU, reg_code: u8){
    let value = get_target(&cpu.registers, reg_code);
    let(result, did_borrow) = cpu.registers.reg_a.overflowing_sub(value);
    
    set_flags(cpu, result == 0, true, (cpu.registers.reg_a & 0xF) < (value & 0xF), did_borrow);
    cpu.registers.reg_a = result;
}

// Subtract value with reg_a and store in reg_a with the carry bit
fn sbc(cpu: &mut CPU, reg_code: u8) {
    let value = get_target(&cpu.registers, reg_code);
    let carry = if cpu.registers.reg_f.carry { 1 } else { 0 };
    let result = cpu.registers.reg_a.wrapping_sub(value).wrapping_sub(carry);
    
    set_flags(cpu, result == 0, true, (cpu.registers.reg_a & 0xF) < ((value & 0xF) + carry), (cpu.registers.reg_a as u16) < (value as u16) + (carry as u16));
    cpu.registers.reg_a = result;
}

// AND the value with reg_a and store in reg_a
fn and(cpu: &mut CPU, reg_code: u8) {
    let value = get_target(&cpu.registers, reg_code);
    let result = cpu.registers.reg_a & value;

    set_flags(cpu, result == 0, false, true, false);
    cpu.registers.reg_a = result;
}

// XOR the value with reg_a and store in reg_a
fn xor(cpu: &mut CPU, reg_code: u8) {
    let value = get_target(&cpu.registers, reg_code);
    let result = cpu.registers.reg_a ^ value;

    set_flags(cpu, result == 0, false, false, false);
    cpu.registers.reg_a = result;
}

// OR the value with reg_a and store in reg_a
fn or(cpu: &mut CPU, reg_code: u8) {
    let value = get_target(&cpu.registers, reg_code);
    let result = cpu.registers.reg_a | value;

    set_flags(cpu, result == 0, false, false, false);
    cpu.registers.reg_a = result;
}

// Compare the value with reg_a, do not store it in reg_a
fn cp(cpu: &mut CPU, reg_code: u8) {
    let value = get_target(&cpu.registers, reg_code);
    let(result, did_borrow) = cpu.registers.reg_a.overflowing_sub(value);
    
    set_flags(cpu, result == 0, true, (cpu.registers.reg_a & 0xF) < (value & 0xF), did_borrow);
}
