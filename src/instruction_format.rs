pub struct R {
    pub opcode: u32,
    pub rd: usize,
    pub funct3: u32,
    pub rs1: usize,
    pub rs2: usize,
    pub funct7: u32,
}

impl R {
    pub fn new(instruction: u32) -> Self {
        Self {
            opcode: instruction & 0x7f,
            rd: ((instruction >> 7) & 0x1f) as usize,
            funct3: (instruction >> 12) & 0x7,
            rs1: ((instruction >> 15) & 0x1f) as usize,
            rs2: ((instruction >> 20) & 0x1f) as usize,
            funct7: (instruction >> 25) & 0x7f,
        }
    }
}

pub struct I {
    pub opcode: u32,
    pub rd: usize,
    pub funct3: u32,
    pub rs1: usize,
    pub imm: u32,
}

impl I {
    pub fn new(instruction: u32) -> Self {
        Self {
            opcode: instruction & 0x7f,
            rd: ((instruction >> 7) & 0x1f) as usize,
            funct3: (instruction >> 12) & 0x7,
            rs1: ((instruction >> 15) & 0x1f) as usize,
            imm: (instruction >> 20) & 0xfff,
        }
    }
}

pub struct S {
    pub opcode: u32,
    pub funct3: u32,
    pub rs1: usize,
    pub rs2: usize,
    pub imm: u32,
}

impl S {
    pub fn new(instruction: u32) -> Self {
        let imm1 = (instruction >> 7) & 0x1f;
        let imm2 = (instruction >> 25) & 0x3f;
        Self {
            opcode: (instruction >> 0) & 0x7f,
            funct3: (instruction >> 12) & 0x7,
            rs1: ((instruction >> 15) & 0x1f) as usize,
            rs2: ((instruction >> 20) & 0x1f) as usize,
            imm: imm1 | (imm2 << 5),
        }
    }
}

pub struct B {
    pub opcode: u32,
    pub funct3: u32,
    pub rs1: usize,
    pub rs2: usize,
    pub imm: u32,
}

impl B {
    pub fn new(instruction: u32) -> Self {
        let imm1 = (instruction >> 7) & 0x1;
        let imm2 = (instruction >> 8) & 0xf;
        let imm3 = (instruction >> 25) & 0x3f;
        let imm4 = (instruction >> 31) & 0x1;
        Self {
            opcode: (instruction >> 0) & 0x7f,
            funct3: (instruction >> 12) & 0x7,
            rs1: ((instruction >> 15) & 0x1f) as usize,
            rs2: ((instruction >> 20) & 0x1f) as usize,
            imm: 0 | (imm2 << 1) | (imm3 << 5) | (imm1 << 11) | (imm4 << 12),
        }
    }
}

pub struct U {
    pub opcode: u32,
    pub rd: usize,
    pub imm: u32,
}

impl U {
    pub fn new(instruction: u32) -> Self {
        Self {
            opcode: instruction & 0x7f,
            rd: ((instruction >> 7) & 0x1f) as usize,
            imm: ((instruction >> 12) & 0xf_ffff),
        }
    }
}

pub struct J {
    pub opcode: u32,
    pub rd: usize,
    pub imm: u32,
}

impl J {
    pub fn new(instruction: u32) -> Self {
        let imm1 = (instruction >> 12) & 0xff;
        let imm2 = (instruction >> 20) & 0x1;
        let imm3 = (instruction >> 21) & 0x3ff;
        let imm4 = (instruction >> 31) & 0x1;
        Self {
            opcode: instruction & 0x7f,
            rd: ((instruction >> 7) & 0x1f) as usize,
            imm: 0 | (imm3 << 1) | (imm2 << 11) | (imm1 << 12) | (imm4 << 20),
        }
    }
}
