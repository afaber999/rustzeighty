cargo build --release
cargo +nightly size -Ax --release
cargo +nightly objdump --release -- --disassemble
