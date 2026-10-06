# RP2350 USB audio playback prototype

Default firmware: UAC 1.0, stereo 48 kHz S32_LE USB playback through Pico Audio Pack.
See [USB_AUDIO.md](USB_AUDIO.md) for flashing, first playback, architecture, and validation limits.
Use `cargo run --release --features test-tone` to flash the standalone 300 Hz test.

USB VID/PID 1209:0001 is for private testing only.

# Usage Notes
The program overclocks the rp2350 to 144MHz, and in my tests accross multiple rp2350 devices this was stable and did not require voltage increases to sustain.
If you want to build for the rp2040, this is possible in my testing, but overclocking to 144MHZ was required due to the
125MHZ core speed not being sufficient to pass enough data at 32bits-48kHz. Overclock at your own risk! I also just wanted to use the the shiny rp2350, so I havent
tested on the rp2040 much but don't see a reason that it wouldn't work! Feel free to fork the repo and modify it to do this, or maybe I will get bored one day and
do this myself. O:

# Running
The easiest way to get this project running on hardware is to first clone the repository with `git clone https://github.com/ramenspazz/I2S-audio-rp2350-rust/`
and then use [picotool](https://github.com/raspberrypi/picotool#building--installing) as a runner, so that uploading the UF2 file to the pico is automated. Next, make sure that the target listed in .cargo/config.toml is correct for your device and make sure that you have added the `thumbv8m.main-none-eabihf` target with rustup by running 
`cargo run --release --target=thumbv8m.main-none-eabihf` to build for the rp2350. I build, upload and run the project on my pimoroni pico lipo 2xl w using `cargo run --release --locked`.

Cheers!
