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

## What the computer sees

- Product name: **RP2350 USB Audio**.
- USB Audio Class **1.0**, playback only (no microphone).
- Full-speed USB, fixed **48,000 Hz**, two channels, signed **32-bit little-endian** PCM.
- An asynchronous isochronous OUT endpoint, plus an explicit feedback IN endpoint.
- No hardware USB volume/mute control; use the computer's software mixer.

UAC version, USB bus speed, and PCM word size are separate choices. UAC 1.0
supports this PCM format; UAC 2.0 is not required just to send 32-bit samples.
The 32-bit transport does not imply 32 bits of effective analog DAC precision.
The 48 kHz sample rate is a selected operating point, not the maximum waveform
frequency and not a claim about the hardware's maximum supported sample rate.

At 48 kHz, each millisecond normally contains 48 stereo frames:
48 * 2 channels * 4 bytes = **384 bytes**. Endpoint capacity is 392 bytes,
allowing a 49-frame packet when feedback asks the host to catch up. Smaller
whole-frame packets are accepted too. USB OUT means computer-to-device; USB IN
means device-to-computer, so the feedback endpoint is IN.

## How the firmware works

1. `usb_audio.rs` supplies the descriptors that identify AudioControl and
   AudioStreaming interfaces, stereo PCM format, 48 kHz rate, and endpoints.
2. The host selects alternate setting 1 to start streaming (setting 0 is idle).
3. `main.rs` polls USB continuously and receives packets without blocking.
4. `audio_buffer.rs` converts little-endian channel words into a stereo ring.
5. Two 48-frame DMA buffers feed PIO. One plays while software prepares the next.
6. PIO keeps the existing 48 kHz/3.072 MHz LRCLK/BCLK and signed 32-bit I2S slots.

USB packets arrive in bursts, whereas the DAC requires evenly spaced samples.
The ring prebuffers four milliseconds; two DMA buffers add about two milliseconds.
It outputs zeros while idle or rebuffering and resets the queue on stream changes,
USB reset, and suspend/resume state changes. On stop, up to two milliseconds of
already queued audio can still play. Error recovery can cause audible gaps.

### Clock feedback

The USB host and Pico have independent clocks. Always sending exactly 48 frames
per USB millisecond would eventually empty or overfill the queue if those clocks
differ, even though both sides call the rate 48 kHz.

The feedback endpoint sends a **3-byte 10.14 fixed-point** value in stereo frames
per USB millisecond. Nominal 48 is `0x0c0000`, transmitted `00 00 0c`.
A smoothed queue-fill controller asks for slightly more samples when the queue
is low and slightly fewer when it is high. It targets 192 frames remaining after
each refill, with correction bounded to +/-0.125 frame/ms. It does not resample
or alter the DAC clock. This is an occupancy-based controller, not direct SOF
clock measurement; tuning and real-host feedback behavior need hardware testing.

The main loop must run promptly: USB is polled, and each DMA completion must be
serviced within roughly one millisecond. Do not insert blocking LCD writes,
delays, or lengthy synthesis into it. Larger UI/DSP tasks will need a different
scheduling design (for example, interrupts or a second core).

No TinyUSB C integration is required. This uses `rp235x-hal` for the USB hardware,
`usb-device` for enumeration/control transfers, and a focused Rust UAC1 class.
The inspected `usbd-audio` implementation did not offer the required S32 format
and explicit feedback path, so it is not added as a dependency.

## Validation and limitations

- Both default USB and `test-tone` ARM release builds pass.
- Host tests exercise the actual `usb-device` control-transfer stack with a mock
  bus: descriptor lengths/endpoint association, alternate settings, fixed-rate
  requests, reset, and feedback byte encoding.
- Ring tests cover stereo order, negative samples, wraparound, 47/49-frame packets,
  malformed packets, overflow/underflow, and 60-second drift simulations at
  +/-100 and +/-1000 ppm with 16 ms of host feedback delay.
- These tests do not replace enumeration, timing, and sustained playback tests on
  a physical Pico. USB scheduling, electrical behavior, hotplug, suspend power,
  and host-specific driver behavior are not certified by the mock tests.

- Audio data formats 1.0: https://www.usb.org/sites/default/files/frmts10.pdf
- USB stack: https://docs.rs/usb-device/0.3.2/usb_device/
- TinyUSB UAC1 descriptor reference: https://github.com/hathach/tinyusb/blob/master/src/device/usbd.h
- Private-test USB ID: https://pid.codes/1209/0001/

