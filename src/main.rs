use riscv_emulator::{Emulator, Xlen};

fn main() {
    let instructions: [u32; 4] = [
        0x02a00093, // addi x1, x0, 42
        0x108133,   // add x2, x1, x1
        0x0040006f, // jal x0, 4
        0x00000000, // HALT
    ];
    let binary = instructions
        .into_iter()
        .map(|i| i.to_le_bytes())
        .flatten()
        .collect::<Vec<_>>();
    let mut emulator = Emulator::new(Xlen::Bit64, Vec::from(binary));
    emulator.run();
}
