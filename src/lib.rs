mod bus;
mod cpu;
mod dram;
mod instruction_format;

pub use cpu::Xlen;

use crate::cpu::Cpu;

pub struct Emulator {
    cpu: Cpu,
}

impl Emulator {
    pub fn new(xlen: Xlen, binary: Vec<u8>) -> Self {
        Self {
            cpu: Cpu::new(xlen, binary),
        }
    }

    pub fn run(&mut self) {
        while true {
            self.cpu.step();
        }
    }
}
