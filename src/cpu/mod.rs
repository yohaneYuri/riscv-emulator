use crate::{bus::Bus, dram::Dram};

mod alias;
mod instruction_format;
mod trap;

use alias::*;
use instruction_format::*;
use trap::*;

fn sign_extend(x: u32, bits: usize, xlen: Xlen) -> u64 {
    if (x >> bits) & 0x1 == 1 {
        ((1 << (xlen.bits() - bits + 1) - 1) << bits) | x as u64
    } else {
        x as u64
    }
}

type Result<T> = std::result::Result<T, Trap>;

pub struct Cpu {
    x: [u64; 32],
    pc: u64,
    bus: Bus,
    xlen: Xlen,
    csr: [u64; 4096],
    privilege_mode: PrivilegeMode,
}

impl Cpu {
    pub fn new(xlen: Xlen, code: Vec<u8>) -> Self {
        Self {
            x: [0; 32],
            pc: 0,
            bus: Bus::new(Dram::new(0x8000_0000, 1 * 1024 * 1024, code)),
            xlen,
            csr: [0; 4096],
            privilege_mode: PrivilegeMode::Machine,
        }
    }

    pub fn dump(&self) {
        println!(
            "CPU state:\n- GP: {:?}\n- PC: {}\n- mstatus: {}\n- mepc: {}\n- mtvec: {}\n- Privilege: {:?}",
            self.x,
            self.pc,
            self.csr[MSTATUS],
            self.csr[MEPC],
            self.csr[MTVEC],
            self.privilege_mode
        );
    }

    pub fn step(&mut self) {
        match self.fetch() {
            Ok(instruction) => {
                if let Err(Trap { kind, information }) = self.execute(instruction) {
                    self.trap(kind, information);
                }
            }
            Err(Trap { kind, information }) => self.trap(kind, information),
        }
    }

    fn fetch(&mut self) -> Result<u32> {
        let instruction = self
            .load(self.pc, MemoryOperationSize::Bit32)
            .map_err(|_| Trap::with_information(TrapKind::InstructionAccessFault, self.pc))?
            as u32;
        self.pc += 4;
        Ok(instruction)
    }

    fn read_csr(&self, number: usize) -> u64 {
        let min_privilege_mode = (number >> 8) & 0x3;
        if min_privilege_mode > self.privilege_mode.value() {
            panic!("Read a CSR in lower privilege mode");
        }
        self.csr[number]
    }

    fn write_csr(&mut self, number: usize, value: u64) {
        let access_mode = (number >> 10) & 0x3;
        let min_privilege_mode = (number >> 8) & 0x3;
        if min_privilege_mode > self.privilege_mode.value() {
            panic!("Write a CSR in lower privilege mode");
        }
        if access_mode == 0b10 {
            panic!("Write to a read-only CSR");
        }
        self.csr[number] = value;
    }

    fn get_pmp_configuration(&self, address: u64) -> PmpConfiguration {
        for i in PMPADDR0..=PMPADDR15 {
            if address < self.read_csr(i) {
                let byte = match self.xlen {
                    Xlen::Bit32 => {
                        let (index, offset) = (i / 4, i % 4);
                        let pmpcfg_value = self.read_csr(PMPCFG0 + index);
                        ((pmpcfg_value >> (offset * 4)) & 0xff) as u8
                    }
                    Xlen::Bit64 => {
                        let (index, offset) = (i / 8, i % 8);
                        let pmpcfg_value = self.read_csr(PMPCFG0 + index * 2);
                        ((pmpcfg_value >> (offset * 8)) & 0xff) as u8
                    }
                };
                return PmpConfiguration::new(byte);
            }
        }
        PmpConfiguration::forbid_all()
    }

    fn load(&self, address: u64, size: MemoryOperationSize) -> Result<u64> {
        let pmpcfg = self.get_pmp_configuration(address);
        if !pmpcfg.r {
            return Err(Trap::with_information(TrapKind::LoadAccessFault, address));
        }
        Ok(self.bus.load(address, size.bits()))
    }

    fn store(&mut self, address: u64, size: MemoryOperationSize, value: u64) -> Result<()> {
        let pmpcfg = self.get_pmp_configuration(address);
        if !pmpcfg.w {
            return Err(Trap::with_information(
                TrapKind::StoreOrAmoAccessFault,
                address,
            ));
        }
        Ok(self.bus.store(address, size.bits(), value))
    }

    fn execute(&mut self, instruction: u32) -> Result<()> {
        use TrapKind::*;

        // 0x00000000 is halt
        if instruction == 0x0 {
            self.dump();
            panic!("Halt");
        }

        let opcode = instruction & 0x7f;
        match opcode {
            // lui
            0b011_0111 => {
                let U { rd, imm, .. } = U::new(instruction);
                self.x[rd] = sign_extend(imm << 12, 20, self.xlen);
            }
            // auipc
            0b001_0111 => {
                let U { rd, imm, .. } = U::new(instruction);
                self.x[rd] = self.pc + sign_extend(imm << 12, 20, self.xlen);
            }
            0b001_0011 => {
                let I {
                    rd,
                    rs1,
                    imm,
                    funct3,
                    ..
                } = I::new(instruction);

                match funct3 {
                    // addi
                    0b000 => self.x[rd] = self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                    // slli
                    0b001 => {
                        let shamt = imm & 0x1f;
                        if self.xlen == Xlen::Bit32 && (shamt >> 5) & 0x1 == 1 {
                            return Err(Trap::new(IllegalInstruction));
                        }
                        self.x[rd] = self.x[rs1] << shamt;
                    }
                    // slti
                    0b010 => {
                        self.x[rd] =
                            match (self.x[rs1] as i64) < (sign_extend(imm, 12, self.xlen) as i64) {
                                true => 1,
                                false => 0,
                            };
                    }
                    // sltu
                    0b011 => {
                        self.x[rd] = match self.x[rs1] < sign_extend(imm, 12, self.xlen) {
                            true => 1,
                            false => 0,
                        };
                    }
                    // xori
                    0b100 => self.x[rd] = self.x[rs1] ^ sign_extend(imm, 12, self.xlen),
                    0b101 => {
                        let shamt = instruction & 0x1f;
                        if self.xlen == Xlen::Bit32 && (shamt >> 5) & 0x1 == 1 {
                            return Err(Trap::new(IllegalInstruction));
                        }
                        match (instruction >> 25) & 0x7f {
                            // srli
                            0b000_0000 => self.x[rd] = self.x[rs1] << shamt,
                            // srai
                            0b010_0000 => self.x[rd] = ((self.x[rs1] as i64) << shamt) as u64,
                            _ => return Err(Trap::new(IllegalInstruction)),
                        }
                    }
                    // ori
                    0b110 => self.x[rd] = self.x[rs1] | sign_extend(imm, 12, self.xlen),
                    // andi
                    0b111 => self.x[rd] = self.x[rs1] & sign_extend(imm, 12, self.xlen),
                    _ => return Err(Trap::new(IllegalInstruction)),
                }
            }
            0b011_0011 => {
                let R {
                    rd,
                    rs1,
                    rs2,
                    funct3,
                    funct7,
                    ..
                } = R::new(instruction);

                match funct3 {
                    0b000 => {
                        match funct7 {
                            // add
                            0b000_0000 => self.x[rd] = self.x[rs1].wrapping_add(self.x[rs2]),
                            // sub
                            0b011_0000 => self.x[rd] = self.x[rs1].wrapping_sub(self.x[rs2]),
                            _ => return Err(Trap::new(IllegalInstruction)),
                        }
                    }
                    // sll
                    0b001 => self.x[rd] = self.x[rs1] << self.x[rs2],
                    // slt
                    0b010 => {
                        self.x[rd] = match (self.x[rs1] as i64) < (self.x[rs2] as i64) {
                            true => 1,
                            false => 0,
                        }
                    }
                    // sltu
                    0b011 => {
                        self.x[rd] = match self.x[rs1] < self.x[rs2] {
                            true => 1,
                            false => 0,
                        }
                    }
                    // xor
                    0b100 => self.x[rd] = self.x[rs1] ^ self.x[rs2],
                    0b101 => {
                        match funct7 {
                            // srl
                            0b000_0000 => self.x[rd] = self.x[rs1] >> self.x[rs2],
                            // sra
                            0b010_0000 => {
                                self.x[rd] = ((self.x[rs1] as i64) >> self.x[rs2]) as u64;
                            }
                            _ => return Err(Trap::new(IllegalInstruction)),
                        }
                    }
                    // and
                    0b111 => self.x[rd] = self.x[rs1] ^ self.x[rs2],
                    _ => return Err(Trap::new(IllegalInstruction)),
                }
            }
            0b000_1111 => {
                let funct3 = (instruction >> 12) & 0x7;
                let succ = (instruction >> 20) & 0xf;
                let pred = (instruction >> 24) & 0xf;

                match funct3 {
                    // fence
                    0b000 => unimplemented!("Fence"),
                    // fence.i
                    0b001 => unimplemented!("Fence.i"),
                    _ => return Err(Trap::new(IllegalInstruction)),
                }
            }
            0b111_0011 => {
                let I {
                    rd,
                    funct3,
                    rs1,
                    imm,
                    ..
                } = I::new(instruction);
                let csr = imm as usize;

                match funct3 {
                    // csrrw
                    0b001 => {
                        if rd == ZERO {
                            self.write_csr(csr, self.x[rs1]);
                        } else {
                            let t = self.read_csr(csr);
                            self.write_csr(csr, self.x[rs1]);
                            self.x[rd] = t;
                        }
                    }
                    // csrrs
                    0b010 => {
                        let t = self.read_csr(csr);
                        self.write_csr(csr, t | self.x[rs1]);
                        self.x[rd] = t;
                    }
                    // csrrc
                    0b011 => {
                        let t = self.read_csr(csr);
                        self.write_csr(csr, t & !self.x[rs1]);
                        self.x[rd] = t;
                    }
                    // csrrwi
                    0b101 => {
                        self.x[rd] = self.read_csr(csr);
                        self.write_csr(csr, (rs1 as u64) & 0x1f);
                    }
                    // csrrsi
                    0b110 => {
                        let t = self.read_csr(csr);
                        self.write_csr(csr, t | ((rs1 as u64) & 0x1f));
                        self.x[rd] = t;
                    }
                    // csrrci
                    0b111 => {
                        let t = self.read_csr(csr);
                        self.write_csr(csr, t & !((rs1 as u64) & 0x1f));
                        self.x[rd] = t;
                    }
                    0b000 => {
                        match imm {
                            // ecall
                            0b0000_0000_0000 => {
                                use PrivilegeMode::*;

                                let kind = match self.privilege_mode {
                                    Machine => MModeEnvironemntCall,
                                    Supervisor => SModeEnvironmentCall,
                                    User => UModeEnvironmentCall,
                                };
                                return Err(Trap::new(kind));
                            }
                            // ebreak
                            0b0000_0000_0001 => {
                                return Err(Trap::with_information(Breakpoint, self.pc - 4));
                            }
                            _ => {
                                let R {
                                    funct7, rs2, rs1, ..
                                } = R::new(instruction);

                                match rs2 {
                                    0b0_0010 => match funct7 {
                                        // sret
                                        0b000_0100 => unimplemented!("Sret"),
                                        // mret
                                        0b001_1000 => {
                                            if self.privilege_mode != PrivilegeMode::Machine {
                                                return Err(Trap::new(IllegalInstruction));
                                            }
                                            // M-mode now, direct access for optimization
                                            self.pc = self.csr[MEPC];
                                            self.privilege_mode = PrivilegeMode::from(
                                                (self.csr[MSTATUS] >> 11) & 0x3,
                                            );
                                            let mpie = (self.csr[MSTATUS] >> 7) & 0x1;
                                            let mask = 0b1_1000_1000_1000;
                                            self.csr[MSTATUS] =
                                                (self.csr[MSTATUS] & !mask) | (mpie << 7);
                                        }
                                        _ => return Err(Trap::new(IllegalInstruction)),
                                    },
                                    // wfi
                                    0b0_0101 => unimplemented!("Wfi"),
                                    // sfence.vma
                                    _ if funct7 == 0b000_1001 => unimplemented!("Sfence.vma"),
                                    _ => return Err(Trap::new(IllegalInstruction)),
                                }
                            }
                        }
                    }
                    _ => return Err(Trap::new(IllegalInstruction)),
                }
            }
            0b000_0011 => {
                let I {
                    rd,
                    funct3,
                    rs1,
                    imm,
                    ..
                } = I::new(instruction);

                match funct3 {
                    // lb
                    0b000 => {
                        self.x[rd] = sign_extend(
                            self.load(
                                self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                                MemoryOperationSize::Bit8,
                            )? as u32,
                            8,
                            self.xlen,
                        )
                    }
                    // lh
                    0b001 => {
                        self.x[rd] = sign_extend(
                            self.load(
                                self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                                MemoryOperationSize::Bit16,
                            )? as u32,
                            8,
                            self.xlen,
                        )
                    }
                    // lw
                    0b010 => {
                        self.x[rd] = sign_extend(
                            self.load(
                                self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                                MemoryOperationSize::Bit32,
                            )? as u32,
                            8,
                            self.xlen,
                        )
                    }
                    // lbu
                    0b100 => {
                        self.x[rd] = self.load(
                            self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                            MemoryOperationSize::Bit8,
                        )?;
                    }
                    // lhu
                    0b101 => {
                        self.x[rd] = self.load(
                            self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                            MemoryOperationSize::Bit16,
                        )?;
                    }
                    _ => return Err(Trap::new(IllegalInstruction)),
                }
            }
            0b010_0011 => {
                let S {
                    imm,
                    funct3,
                    rs1,
                    rs2,
                    ..
                } = S::new(instruction);

                match funct3 {
                    // sb
                    0b000 => {
                        self.store(
                            self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                            MemoryOperationSize::Bit8,
                            self.x[rs2] & 0xff,
                        )?;
                    }
                    // sh
                    0b001 => {
                        self.store(
                            self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                            MemoryOperationSize::Bit16,
                            self.x[rs2] & 0xff,
                        )?;
                    }
                    // sw
                    0b010 => {
                        self.store(
                            self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                            MemoryOperationSize::Bit16,
                            self.x[rs2] & 0xff,
                        )?;
                    }
                    _ => return Err(Trap::new(IllegalInstruction)),
                }
            }
            // jal
            0b110_1111 => {
                let J { rd, imm, .. } = J::new(instruction);
                self.x[rd] = self.pc + 4;
                self.pc = self.pc.wrapping_add(sign_extend(imm, 20, self.xlen));
            }
            // jalr
            0b110_0111 => {
                let I { rd, rs1, imm, .. } = I::new(instruction);

                let t = self.pc + 4;
                self.pc = (self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen))) & !1;
                self.x[rd] = t;
            }
            0b110_0011 => {
                let B {
                    imm,
                    funct3,
                    rs1,
                    rs2,
                    ..
                } = B::new(instruction);

                match funct3 {
                    // beq
                    0b000 => {
                        if self.x[rs1] == self.x[rs2] {
                            self.pc = self.pc.wrapping_add(sign_extend(imm, 13, self.xlen));
                        }
                    }
                    // bne
                    0b001 => {
                        if self.x[rs1] != self.x[rs2] {
                            self.pc = self.pc.wrapping_add(sign_extend(imm, 13, self.xlen));
                        }
                    }
                    // blt
                    0b100 => {
                        if (self.x[rs1] as i64) < (self.x[rs2] as i64) {
                            self.pc = self.pc.wrapping_add(sign_extend(imm, 13, self.xlen));
                        }
                    }
                    // bge
                    0b101 => {
                        if self.x[rs1] as i64 >= self.x[rs2] as i64 {
                            self.pc = self.pc.wrapping_add(sign_extend(imm, 13, self.xlen));
                        }
                    }
                    // bltu
                    0b110 => {
                        if self.x[rs1] < self.x[rs2] {
                            self.pc = self.pc.wrapping_add(sign_extend(imm, 13, self.xlen));
                        }
                    }
                    // bgeu
                    0b111 => {
                        if self.x[rs1] >= self.x[rs2] {
                            self.pc = self.pc.wrapping_add(sign_extend(imm, 13, self.xlen));
                        }
                    }
                    _ => return Err(Trap::new(IllegalInstruction)),
                }
            }
            _ => return Err(Trap::new(IllegalInstruction)),
        }
        // Harewired to zero
        self.x[ZERO] = 0;
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum Xlen {
    Bit32,
    Bit64,
}

impl Xlen {
    fn bits(&self) -> usize {
        match self {
            Self::Bit32 => 32,
            Self::Bit64 => 64,
        }
    }
}

#[derive(PartialEq, PartialOrd)]
enum MemoryOperationSize {
    Bit8,
    Bit16,
    Bit32,
    Bit64,
}

impl MemoryOperationSize {
    fn bits(&self) -> usize {
        match self {
            Self::Bit8 => 8,
            Self::Bit16 => 16,
            Self::Bit32 => 32,
            Self::Bit64 => 64,
        }
    }
}

#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
enum PrivilegeMode {
    User,
    Supervisor,
    Machine,
}

impl PrivilegeMode {
    fn value(&self) -> usize {
        match self {
            Self::Machine => 0b11,
            Self::Supervisor => 0b01,
            Self::User => 0b00,
        }
    }

    fn from(x: u64) -> Self {
        match x {
            0b11 => Self::Machine,
            0b01 => Self::Supervisor,
            0b00 => Self::User,
            _ => unreachable!(),
        }
    }
}

struct PmpConfiguration {
    r: bool,
    w: bool,
    x: bool,
    a: u8,
    l: bool,
}

impl PmpConfiguration {
    fn new(byte: u8) -> Self {
        Self {
            r: (byte & 0x1) != 0,
            w: ((byte >> 1) & 0x1) != 0,
            x: ((byte >> 2) & 0x1) != 0,
            a: (byte >> 3) & 0x3,
            l: ((byte >> 7) & 0x1) != 0,
        }
    }

    fn forbid_all() -> Self {
        Self {
            r: false,
            w: false,
            x: false,
            a: 0,
            l: false,
        }
    }
}
