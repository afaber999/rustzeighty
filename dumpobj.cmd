cargo build --release
cargo +nightly size --release
cargo +nightly objdump --release -- --disassemble
