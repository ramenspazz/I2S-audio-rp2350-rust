# Pico Audio Pack timing test

This change is based on upstream commit 62cfaa6. It replaces the earlier patch
against i2s_module.rs. Apply it to the latest unmodified main branch.

## Build

Install Rust and the ARM target, then build from the repository directory:

```sh
rustup target add thumbv8m.main-none-eabihf
cargo build --release --locked
python3 tools/check_i2s.py
```

The ELF is `target/thumbv8m.main-none-eabihf/release/rp-2350-i2s-output`.
Flash it using your existing RP2350 ELF workflow (or convert to RP2350 UF2 using
your existing tools). An ELF cannot simply be copied to the BOOTSEL drive.

## Expected behavior

The LCD displays the configured 144000000 Hz system clock, 48000 Hz sample rate,
and a 300 Hz tone at 5% amplitude. These values describe configuration; they do
not measure the physical pins. LCD initialization uses the existing PCF8574T
20x4 setup on GP26/GP27 and its default I2C address. LCD errors halt startup.

Pico Audio Pack connections are unchanged:

| Signal | GPIO | Expected |
| --- | --- | --- |
| DIN | 9 | Signed stereo PCM, matching channels |
| BCLK | 10 | 3.072 MHz average |
| LRCLK | 11 | 48 kHz, 64 BCLK periods/frame |
| MUTE | 22 | Low during playback |

Both analog channels should produce a continuous 300 Hz sine wave. If you have
a scope or logic analyzer, measure BCLK and LRCLK first. The fractional divider
produces small variations in individual BCLK periods; complete stereo frames
are exactly 3000 system cycles with these settings.

## Changes and rationale

- The old divider of 1 at 193.5 MHz requested approximately 987245 frames/s with
  the old 196-cycle PIO loop, before FIFO stalls. Raising CPU speed cannot fix
  that clock-ratio error.
- The replacement PIO uses two instructions/bit, data changes on falling BCLK
  edges, and LRCLK switches one bit before the next channel's MSB. Autopull is
  the only mechanism consuming FIFO words; explicit nonblocking pulls are gone.
- A 144 MHz system clock and 16.8 divider 23 + 112/256 provide 6.144 MHz PIO
  instruction rate and 48 kHz stereo. The custom voltage writes are removed.
- The broken Taylor series and unsigned float conversion are replaced with
  startup-only sine generation and signed 24-bit samples in 32-bit slots.
- Each DMA buffer holds 960 stereo frames (six full 300 Hz periods, 20 ms).
  Identical buffers can repeat without phase resets or runtime synthesis.
- LCD writes occur before playback. Keep lengthy blocking operations out of the
  DMA loop: this design still requires software to requeue each completed buffer.
- Adds the missing LCD dependency and ARM target/linker configuration.

## Validation and limits

`cargo build --release --locked` succeeds for thumbv8m.main-none-eabihf.
The Python PIO model checks serial bits, clock cadence, and channel alignment
under continuous FIFO supply. It does not model DMA starvation or analog output.
This firmware has not been flashed or measured on physical hardware here.

This is a fixed-tone diagnostic baseline, not a general variable-frequency
synthesizer. Once hardware timing is confirmed, dynamic synthesis can refill
buffers with a continuous phase accumulator while preserving the DMA deadline.
