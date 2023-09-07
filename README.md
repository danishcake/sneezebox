# Sneezebox firmware
Firmware for the sneezebox

The target hardware is a Raspberry Pi Pico.

## Building and deploying
First install dependencies.
```bash
# Update rust
rustup self update
rustup update stable
rustup target add thumbv6m-none-eabi
cargo install flip-link

# Required to flash device
cargo install elf2uf2-rs --locked
```

Then run the project. This will flash it onto the attached Raspberry Pi Pico. This must be placed in bootloader mode by holiding bootsel first

```bash
cargo run --release
```