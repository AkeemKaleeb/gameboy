/**************************************************************************************************
* File Name         : opcodes.rs
* Description       : Defines the implementation of opcodes and their associated operations
**************************************************************************************************/
#![allow(unused_variables)]
#![allow(dead_code)]

use crate::cpu::CPU;

// Implement opcode execution
pub fn decode_execute(cpu: &mut CPU, opcode: u8) {
    match opcode & 0xF0 {
        0x00 => match opcode & 0x0F {
            0x0 => nop(cpu),
            0x1 => {
                let value = cpu.fetch_word();
                ld_bc_nn(cpu, value);
            },
            0x2 => ld_bc_a(cpu),
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        0x10 => match opcode & 0x0F{
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        0x20 => match opcode & 0x0F{
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        0x30 => match opcode & 0x0F{
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
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
            0x0..=0x7 => { add(cpu, opcode & 0x07) }
            0x8..=0xF => { adc(cpu, opcode & 0x07) }
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        // SUB and SBC Instructions
        0x90 => match opcode & 0x0F{
            0x0..=0x7 => {}
            0x8..=0xF => {}
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        // AND and XOR Instructions
        0xA0 => match opcode & 0x0F{
            0x0..=0x7 => {}
            0x8..=0xF => {}
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        // OR and CP Instructions
        0xB0 => match opcode & 0x0F{
            0x0..=0x7 => {}
            0x8..=0xF => {}
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        0xC0 => match opcode & 0x0F{
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        0xD0 => match opcode & 0x0F{
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        0xE0 => match opcode & 0x0F{
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        0xF0 => match opcode & 0x0F{
            _ => panic!("Unknown opcode: 0x{:02X}", opcode),    // Skip unknown code
        }
        _ => panic!("Unknown opcode: 0x{:02X}", opcode),        // Skip unknown code
    }
}

fn nop(_cpu: &mut CPU) {
    // No operation
}

fn ld_bc_nn(cpu: &mut CPU, value: u16) {
    cpu.registers.set_bc(value);
}

fn ld_bc_a(cpu: &mut CPU) {
    let value = cpu.registers.reg_a;
    let bc = cpu.registers.get_bc();
    cpu.memory.write_byte(bc, value);
}

// Add value and store in reg_a
fn add(cpu: &mut CPU, reg_code: u8){
    let value = match reg_code {
        0x0 => cpu.registers.reg_b,
        0x1 => cpu.registers.reg_c,
        0x2 => cpu.registers.reg_d,
        0x3 => cpu.registers.reg_e,
        0x4 => cpu.registers.reg_h,
        0x5 => cpu.registers.reg_l,
        0x6 => cpu.memory.read_byte(cpu.registers.get_hl()),
        0x7 => cpu.registers.reg_a,
        _ => panic!("Unknown opcode: 0x{:02X}", reg_code),        // Skip unknown code
    };

    let(result, did_overflow) = cpu.registers.reg_a.overflowing_add(value);
    cpu.registers.reg_a = result;

    // Set Flags
    cpu.registers.reg_f.zero = result == 0;
    cpu.registers.reg_f.subtract = false;
    cpu.registers.reg_f.half_carry = (cpu.registers.reg_a & 0xF) + (value & 0xF) > 0xF;
    cpu.registers.reg_f.carry = (cpu.registers.reg_a as u16) + (value as u16) > 0xFF;
}

fn adc(cpu: &mut CPU, reg_code: u8) {
    let value = match reg_code {
        0x0 => cpu.registers.reg_b,
        0x1 => cpu.registers.reg_c,
        0x2 => cpu.registers.reg_d,
        0x3 => cpu.registers.reg_e,
        0x4 => cpu.registers.reg_h,
        0x5 => cpu.registers.reg_l,
        0x6 => cpu.memory.read_byte(cpu.registers.get_hl()),
        0x7 => cpu.registers.reg_a,
        _ => panic!("Unknown opcode: 0x{:02X}", reg_code),        // Skip unknown code
    };

    let carry = if cpu.registers.reg_f.carry { 1 } else { 0 };
    let result = cpu.registers.reg_a.wrapping_add(value).wrapping_add(carry);
    cpu.registers.reg_a = result;

    cpu.registers.reg_f.zero = result == 0;
    cpu.registers.reg_f.subtract = false;
    cpu.registers.reg_f.half_carry = (cpu.registers.reg_a & 0xF) + (carry) > 0xF;
    cpu.registers.reg_f.carry = (cpu.registers.reg_a as u16) + (value as u16) + (carry as u16) > 0xFF;
}