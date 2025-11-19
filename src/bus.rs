use crate::dram::Dram;

pub struct Bus {
    memory: Dram,
}

impl Bus {
    pub fn new(memory: Dram) -> Self {
        Self { memory }
    }

    pub fn load(&self, address: u64, size: usize) -> u64 {
        self.memory.load(address, size)
    }

    pub fn store(&mut self, address: u64, size: usize, value: u64) {
        self.memory.store(address, size, value);
    }
}
