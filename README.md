# RapidScale

Scale firmware for the Raspberry Pi Pico 2 (RP2350), written in Rust with Embassy.

## Build

```sh
cargo build --release
```

The Cargo configuration targets `thumbv8m.main-none-eabihf` and uses `picotool` to flash and run the firmware with `cargo run --release`.
