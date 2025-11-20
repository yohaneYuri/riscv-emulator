use crate::{bus::Bus, dram::Dram, instruction_format::*};

// General purpose register alias
pub const ZERO: usize = 0;
pub const RA: usize = 1;
pub const SP: usize = 2;
pub const GP: usize = 3;
pub const TP: usize = 4;
pub const T0: usize = 5;
pub const T1: usize = 6;
pub const T2: usize = 7;
pub const S0: usize = 8;
pub const FP: usize = 8;
pub const S1: usize = 9;
pub const A0: usize = 10;
pub const A1: usize = 11;
pub const A2: usize = 12;
pub const A3: usize = 13;
pub const A4: usize = 14;
pub const A5: usize = 15;
pub const A6: usize = 16;
pub const A7: usize = 17;
pub const S2: usize = 18;
pub const S3: usize = 19;
pub const S4: usize = 20;
pub const S5: usize = 21;
pub const S6: usize = 22;
pub const S7: usize = 23;
pub const S8: usize = 24;
pub const S9: usize = 25;
pub const S10: usize = 26;
pub const S11: usize = 27;
pub const T3: usize = 28;
pub const T4: usize = 29;
pub const T5: usize = 30;
pub const T6: usize = 31;

// CSRs

// Machine trap setup
const MSTATUS: usize = 0x300;
const MISA: usize = 0x301;
const MEDELEG: usize = 0x302;
const MIDELEG: usize = 0x303;
const MIE: usize = 0x304;
const MTVEC: usize = 0x305;
const MCOUNTEREN: usize = 0x306;
const MSTATUSH: usize = 0x310;
const MEDELEGH: usize = 0x312;

// Machine trap handling
const MSCRATCH: usize = 0x340;
const MEPC: usize = 0x341;
const MCAUSE: usize = 0x342;
const MTVAL: usize = 0x343;
const MIP: usize = 0x344;
const MTINST: usize = 0x34a;
const MTVAL2: usize = 0x34b;

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
        let instruction = self.fetch();
        if let Err((kind, information)) = self.execute(instruction) {
            self.trap(kind, information);
        }
    }

    fn fetch(&mut self) -> u32 {
        let instruction = self.bus.load(self.pc, 32) as u32;
        self.pc += 4;
        instruction
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

    fn trap(&mut self, kind: Trap, trap_information: u64) {
        // Cpu behavior, so use direct access to CSRs
        // instead of write_csr and read_csr
        self.csr[MEPC] = self.pc - 4;
        self.pc = self.csr[MTVEC];
        self.csr[MCAUSE] = kind.value(self.xlen);
        self.csr[MTVAL] = trap_information;
        let mut mstatus = self.csr[MSTATUS];
        let mie = (mstatus >> 3) & 0x1;
        // Clear mstatus.MIE
        mstatus &= !(1 << 3);
        // mstatus.MPIE = mstatus.MIE
        mstatus |= mie << 7;
        // mstatus.MPP = privilege mode
        let mpp_mask = 0b0001_1000_0000_0000;
        mstatus = (mstatus & !mpp_mask) | ((self.privilege_mode.value() << 11) as u64);
        self.csr[MSTATUS] = mstatus;
        // Switch privilege mode
        self.privilege_mode = PrivilegeMode::Machine;

        self.handle_trap(kind, trap_information);
    }

    fn handle_trap(&mut self, kind: Trap, information: u64) {
        use Trap::*;

        match kind {
            SupervisorSoftwareInterrupt => todo!(),
            MachineSoftwareInterrupt => todo!(),
            SuperviserTimerInterrupt => todo!(),
            MachineTimerInterrupt => todo!(),
            SupervisorExternalInterrupt => todo!(),
            MachineExternalInterrupt => todo!(),
            CounterOverflowInterrupt => todo!(),
            InstructionAddressMisaligned => todo!(),
            InstructionAccessFault => todo!(),
            IllegalInstruction => todo!(),
            Breakpoint => todo!(),
            LoadAddressMisaligned => todo!(),
            LoadAccessFault => todo!(),
            StoreOrAmoAddressMisaligned => todo!(),
            StoreOrAmoAccessFault => todo!(),
            UModeEnvironmentCall | SModeEnvironmentCall | MModeEnvironemntCall => {}
            LoadPageFault => todo!(),
            InstructionPageFault => todo!(),
            StoreOrAmoPageFault => todo!(),
            DoubleTrap => todo!(),
            SoftwareCheck => todo!(),
            HardwareError => todo!(),
        }
    }

    fn execute(&mut self, instruction: u32) -> Result<(), (Trap, u64)> {
        use Trap::*;

        // 0x00000000 is halt
        if instruction == 0x0 {
            println!("Illegal opcode");
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
                            return Err((IllegalInstruction, 0));
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
                            return Err((IllegalInstruction, 0));
                        }
                        match (instruction >> 25) & 0x7f {
                            // srli
                            0b000_0000 => self.x[rd] = self.x[rs1] << shamt,
                            // srai
                            0b010_0000 => self.x[rd] = ((self.x[rs1] as i64) << shamt) as u64,
                            _ => return Err((IllegalInstruction, 0)),
                        }
                    }
                    // ori
                    0b110 => self.x[rd] = self.x[rs1] | sign_extend(imm, 12, self.xlen),
                    // andi
                    0b111 => self.x[rd] = self.x[rs1] & sign_extend(imm, 12, self.xlen),
                    _ => return Err((IllegalInstruction, 0)),
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
                            _ => return Err((IllegalInstruction, 0)),
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
                            _ => return Err((IllegalInstruction, 0)),
                        }
                    }
                    // and
                    0b111 => self.x[rd] = self.x[rs1] ^ self.x[rs2],
                    _ => return Err((IllegalInstruction, 0)),
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
                    _ => return Err((IllegalInstruction, 0)),
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
                                return Err((kind, 0));
                            }
                            // ebreak
                            0b0000_0000_0001 => return Err((Breakpoint, self.pc - 4)),
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
                                                return Err((IllegalInstruction, 0));
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
                                        _ => return Err((IllegalInstruction, 0)),
                                    },
                                    // wfi
                                    0b0_0101 => unimplemented!("Wfi"),
                                    // sfence.vma
                                    _ if funct7 == 0b000_1001 => unimplemented!("Sfence.vma"),
                                    _ => return Err((IllegalInstruction, 0)),
                                }
                            }
                        }
                    }
                    _ => return Err((IllegalInstruction, 0)),
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
                            self.bus
                                .load(self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)), 8)
                                as u32,
                            8,
                            self.xlen,
                        )
                    }
                    // lh
                    0b001 => {
                        self.x[rd] = sign_extend(
                            self.bus.load(
                                self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                                16,
                            ) as u32,
                            8,
                            self.xlen,
                        )
                    }
                    // lw
                    0b010 => {
                        self.x[rd] = sign_extend(
                            self.bus.load(
                                self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                                32,
                            ) as u32,
                            8,
                            self.xlen,
                        )
                    }
                    // lbu
                    0b100 => {
                        self.x[rd] = self
                            .bus
                            .load(self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)), 8);
                    }
                    // lhu
                    0b101 => {
                        self.x[rd] = self.bus.load(
                            self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                            16,
                        );
                    }
                    _ => return Err((IllegalInstruction, 0)),
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
                        self.bus.store(
                            self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                            8,
                            self.x[rs2] & 0xff,
                        );
                    }
                    // sh
                    0b001 => {
                        self.bus.store(
                            self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                            16,
                            self.x[rs2] & 0xff,
                        );
                    }
                    // sw
                    0b010 => {
                        self.bus.store(
                            self.x[rs1].wrapping_add(sign_extend(imm, 12, self.xlen)),
                            32,
                            self.x[rs2] & 0xff,
                        );
                    }
                    _ => return Err((IllegalInstruction, 0)),
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
                    _ => return Err((IllegalInstruction, 0)),
                }
            }
            _ => return Err((IllegalInstruction, 0)),
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

enum Trap {
    SupervisorSoftwareInterrupt,
    MachineSoftwareInterrupt,
    SuperviserTimerInterrupt,
    MachineTimerInterrupt,
    SupervisorExternalInterrupt,
    MachineExternalInterrupt,
    CounterOverflowInterrupt,
    InstructionAddressMisaligned,
    InstructionAccessFault,
    IllegalInstruction,
    Breakpoint,
    LoadAddressMisaligned,
    LoadAccessFault,
    StoreOrAmoAddressMisaligned,
    StoreOrAmoAccessFault,
    UModeEnvironmentCall,
    SModeEnvironmentCall,
    MModeEnvironemntCall,
    LoadPageFault,
    InstructionPageFault,
    StoreOrAmoPageFault,
    DoubleTrap,
    SoftwareCheck,
    HardwareError,
}

impl Trap {
    fn value(&self, xlen: Xlen) -> u64 {
        use Trap::*;

        let bits = xlen.bits();
        let interrupt = match self {
            SupervisorSoftwareInterrupt
            | MachineSoftwareInterrupt
            | SuperviserTimerInterrupt
            | MachineTimerInterrupt
            | SupervisorExternalInterrupt
            | MachineExternalInterrupt
            | CounterOverflowInterrupt => 1,
            _ => 0,
        };
        let exception_code = match self {
            SupervisorSoftwareInterrupt => 1,
            MachineSoftwareInterrupt => 3,
            SuperviserTimerInterrupt => 5,
            MachineTimerInterrupt => 7,
            SupervisorExternalInterrupt => 9,
            MachineExternalInterrupt => 11,
            CounterOverflowInterrupt => 13,
            InstructionAddressMisaligned => 0,
            InstructionAccessFault => 1,
            IllegalInstruction => 2,
            Breakpoint => 3,
            LoadAddressMisaligned => 4,
            LoadAccessFault => 5,
            StoreOrAmoAddressMisaligned => 6,
            StoreOrAmoAccessFault => 7,
            UModeEnvironmentCall => 8,
            SModeEnvironmentCall => 9,
            MModeEnvironemntCall => 11,
            InstructionPageFault => 12,
            LoadPageFault => 13,
            StoreOrAmoPageFault => 15,
            DoubleTrap => 16,
            SoftwareCheck => 18,
            HardwareError => 19,
        };
        (interrupt << (bits - 1)) | exception_code
    }
}

fn sign_extend(x: u32, bits: usize, xlen: Xlen) -> u64 {
    if (x >> bits) & 0x1 == 1 {
        ((1 << (xlen.bits() - bits + 1) - 1) << bits) | x as u64
    } else {
        x as u64
    }
}
