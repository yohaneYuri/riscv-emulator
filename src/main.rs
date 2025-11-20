use riscv_emulator::{Emulator, Xlen};

fn main() {
    let instructions: [u32; _] = [
        0x305c5073, // csrrwi x0, mtvec, 24
        0x00000093, // addi, x1, 0
        0x00000113, // addi x2, x0, 0
        0x00000073, // ecall
        0x02a00113, // add x2, x0, 42
        0x00000000, // HALT
        // Handler
        0x06300093, // addi x1, x0, 99
        0x3411a073, // csrrs x3, mepc, x0
        0x00418193, // addi x3, x3, 4
        0x34119073, // csrrw x0, mepc, x3
        0x30200073, // mret
    ];
    let binary = instructions
        .into_iter()
        .map(|i| i.to_le_bytes())
        .flatten()
        .collect::<Vec<_>>();
    let mut emulator = Emulator::new(Xlen::Bit64, Vec::from(binary));
    emulator.run();
}
