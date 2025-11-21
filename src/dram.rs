pub struct Dram {
    base: u64,
    cells: Vec<u8>,
}

impl Dram {
    pub fn new(base: u64, size: usize, mut code: Vec<u8>) -> Self {
        code.resize(size, 0);
        Self { base, cells: code }
    }

    pub fn load(&self, address: u64, size: usize) -> u64 {
        let address = address as usize;
        let mut data = self.cells[address] as u64;
        if size > 8 {
            data |= (self.cells[address + 1] as u64) << 8;
        }
        if size > 16 {
            data |= (self.cells[address + 2] as u64) << 16 | (self.cells[address + 3] as u64) << 24;
        }
        if size > 32 {
            data |= (self.cells[address + 4] as u64) << 32
                | (self.cells[address + 5] as u64) << 40
                | (self.cells[address + 6] as u64) << 48
                | (self.cells[address + 7] as u64) << 56;
        }
        data
    }

    pub fn store(&mut self, address: u64, size: usize, value: u64) {
        let address = address as usize;
        self.cells[address] = (value & 0xff) as u8;
        if size > 8 {
            self.cells[address + 1] = ((value >> 8) & 0xff) as u8;
        }
        if size > 16 {
            self.cells[address + 2] = ((value >> 16) & 0xff) as u8;
        }
        if size > 32 {
            self.cells[address + 3] = ((value >> 24) & 0xff) as u8;
            self.cells[address + 4] = ((value >> 32) & 0xff) as u8;
            self.cells[address + 5] = ((value >> 40) & 0xff) as u8;
            self.cells[address + 6] = ((value >> 48) & 0xff) as u8;
            self.cells[address + 7] = ((value >> 56) & 0xff) as u8;
        }
    }
}
