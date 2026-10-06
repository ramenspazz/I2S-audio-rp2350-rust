<<<<<<< HEAD
# RP2350 USB audio playback prototype

Default firmware: UAC 1.0, stereo 48 kHz S32_LE USB playback through Pico Audio Pack.
See [USB_AUDIO.md](USB_AUDIO.md) for flashing, first playback, architecture, and validation limits.
Use `cargo run --release --features test-tone` to flash the standalone 300 Hz test.

The new USB path builds and passes host-side tests, but awaits physical enumeration
and playback testing. USB VID/PID 1209:0001 is for private testing only.
