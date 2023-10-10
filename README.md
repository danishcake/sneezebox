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

# If you're using a pico-probe, run these two lines
cargo install probe-run --locked
cargo install probe-rs --features=cli --locked

# If you're not using a debugger, install this
# You'll also need to edit .cargo/config to use this
cargo install elf2uf2-rs --locked
```

Then run the project. This will flash it onto the attached Raspberry Pi Pico. This must be placed in bootloader mode by holiding bootsel first

```bash
cargo run --release
```

## Wiring

We're going to use GPIO 10 as the keyboard input, and GPIO 9 as the big LED output
