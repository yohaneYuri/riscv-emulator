use super::{Cpu, PrivilegeMode, Xlen, alias::*};

impl Cpu {
    pub(super) fn trap(&mut self, kind: TrapKind, trap_information: u64) {
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
    }
}

pub enum TrapKind {
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

impl TrapKind {
    pub fn value(&self, xlen: Xlen) -> u64 {
        use TrapKind::*;

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

pub struct Trap {
    pub kind: TrapKind,
    pub information: u64,
}

impl Trap {
    pub fn new(kind: TrapKind) -> Self {
        Self {
            kind,
            information: 0,
        }
    }

    pub fn with_information(kind: TrapKind, information: u64) -> Self {
        Self { kind, information }
    }
}
