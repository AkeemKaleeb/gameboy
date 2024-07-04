/**************************************************************************************************
* File Name         : debugger.rs
* Description       : Contains tools for debugging such as logging and breakpoint management
**************************************************************************************************/

use crate::cpu::CPU;
use crate::memory::Memory;
use crate::ppu::PPU;
use crate::timers::Timers;
use crate::interrupts::Interrupts;
use crate::io::IO;

pub fn debug(cpu: &CPU, memory: &Memory, ppu: &PPU, timers: &Timers, interrupts: &Interrupts, io: &IO) {
    // Debugging code
}