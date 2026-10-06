#![no_std]
#![no_main]

use core::fmt::Write;
use embedded_hal::digital::OutputPin;
use fugit::RateExtU32;
use heapless::String;
use i2c_character_display::{CharacterDisplayPCF8574T, LcdDisplayType};
// LCD is connected to vBUS-red, ground-brown, gpio27-yellow, gpio28-orange
use rp235x_hal::{
    self as hal,
    clocks::{self, Clock},
    dma::{double_buffer, DMAExt},
    gpio::{FunctionI2C, Pin},
    pac,
    pio::PIOExt,
    singleton,
};

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

#[cfg(not(feature = "test-tone"))]
mod audio_buffer;
mod i2s_lib;
#[cfg(not(feature = "test-tone"))]
mod usb_audio;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32; // 12.0 Mhz

// 12 MHz crystal -> 1152 MHz VCO / 4 / 2 = 144 MHz.
// Below the RP2350's 150 MHz rating; no voltage override is needed.
// PIO divider 23 + 112/256 gives exactly 6.144 MHz on average.
const REFDIV: u8 = 1;
const POST_DIVIDER_1: u8 = 4;
const POST_DIVIDER_2: u8 = 2;
const VCO_FREQ: u32 = 1152;

// This is output for the usb clock pll settings
// $ ./vcocalc.py 48
// Requested: 48.0 MHz
// Achieved:  48.0 MHz
// REFDIV:    1
// FBDIV:     120 (VCO = 1440.0 MHz)
// PD1:       6
// PD2:       5
const USB_REFDIV: u8 = 1;
const USB_POST_DIVIDER_1: u8 = 6;
const USB_POST_DIVIDER_2: u8 = 5;
const USB_VCO_FREQ: u32 = 1440;

#[rp235x_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let mut clocks = clocks::ClocksManager::new(pac.CLOCKS);

    watchdog.enable_tick_generation((XTAL_FREQ_HZ / 1_000_000) as u16);

    let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ.Hz())
        .map_err(clocks::InitError::XoscErr)
        .unwrap();

    pub const PLL_SYS_144MHZ: hal::pll::PLLConfig = hal::pll::PLLConfig {
        vco_freq: fugit::HertzU32::MHz(VCO_FREQ),
        refdiv: REFDIV,
        post_div1: POST_DIVIDER_1,
        post_div2: POST_DIVIDER_2,
    };

    pub const PLL_USB_48MHZ: hal::pll::PLLConfig = hal::pll::PLLConfig {
        vco_freq: fugit::HertzU32::MHz(USB_VCO_FREQ),
        refdiv: USB_REFDIV,
        post_div1: USB_POST_DIVIDER_1,
        post_div2: USB_POST_DIVIDER_2,
    };

    let pll_sys = hal::pll::setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency().into(),
        PLL_SYS_144MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();

    let pll_usb = hal::pll::setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency().into(),
        PLL_USB_48MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();

    // Initialize the system clock to 144 MHz and USB to 48 MHz.
    clocks.init_default(&xosc, &pll_sys, &pll_usb).unwrap();

    let sio = rp235x_hal::Sio::new(pac.SIO);

    let pins = rp235x_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // set up and configure the HD44780 based LCD display over i2c
    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio26.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio27.reconfigure();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let delay = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks); // init LCD1602

    // PCF8574T adapter for a single HD44780 controller using a 20x4 character LCD.
    let mut lcd = CharacterDisplayPCF8574T::new(i2c, LcdDisplayType::Lcd20x4, delay);
    if let Err(e) = lcd.init() {
        panic!("Error initializing LCD: {}", e);
    }

    // Display configured values before audio starts. These are not measurements
    // of the physical BCLK/LRCLK pins. Do not block the DMA loop on LCD writes.
    let sys_freq_hz = clocks.system_clock.freq().to_Hz();
    assert_eq!(sys_freq_hz, 144_000_000);
    let mut output_string: String<20> = String::new();
    write!(&mut output_string, "SYS {} Hz", sys_freq_hz).unwrap();
    lcd.backlight(true).unwrap();
    lcd.clear().unwrap();
    lcd.home().unwrap();
    lcd.print(&output_string).unwrap();
    lcd.set_cursor(0, 1).unwrap();
    lcd.print("I2S 48000 Hz (cfg)").unwrap();
    lcd.set_cursor(0, 2).unwrap();
    #[cfg(feature = "test-tone")]
    lcd.print("Test tone 300 Hz, 5%").unwrap();
    #[cfg(not(feature = "test-tone"))]
    lcd.print("USB audio S32 stereo").unwrap();

    // PIO Globals
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let exact_divider = i2s_lib::PioClockDivider::FromSystemClock(clocks.system_clock.freq());

    let dac_output = i2s_lib::I2SOutput::new(
        &mut pio0,
        exact_divider,
        sm0,
        pins.gpio9,
        pins.gpio10,
        pins.gpio11,
    )
    .unwrap();

    let (dac_sm, _, dac_fifo_tx) = dac_output.split();

    let dma = pac.DMA.split(&mut pac.RESETS);

    // GP22 directly drives DAC XSMT and headphone amp EN: HIGH enables audio.
    let mut audio_enable = pins.gpio22.into_push_pull_output();
    audio_enable.set_low().unwrap();

    #[cfg(feature = "test-tone")]
    {
        let tx_buf1 = singleton!(: [u32; i2s_lib::TABLE_SIZE] = [0; i2s_lib::TABLE_SIZE]).unwrap();
        let tx_buf2 = singleton!(: [u32; i2s_lib::TABLE_SIZE] = [0; i2s_lib::TABLE_SIZE]).unwrap();
        i2s_lib::fill_test_tone(tx_buf1, 0.05);
        i2s_lib::fill_test_tone(tx_buf2, 0.05);
        let mut transfer = double_buffer::Config::new((dma.ch0, dma.ch1), tx_buf1, dac_fifo_tx)
            .start()
            .read_next(tx_buf2);
        let _running_sm = dac_sm.start();
        audio_enable.set_high().unwrap();
        loop {
            let (completed, next) = transfer.wait();
            transfer = next.read_next(completed);
        }
    }

    #[cfg(not(feature = "test-tone"))]
    {
        use audio_buffer::{AudioBuffer, BLOCK_WORDS};
        use usb_device::{class_prelude::UsbBusAllocator, prelude::*, UsbError};
        let bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USB,
            pac.USB_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        ));
        let mut audio = usb_audio::UsbAudio::new(&bus);
        // pid.codes shared test VID/PID: local prototypes only, not a product ID.
        let mut device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x1209, 0x0001))
            .strings(&[StringDescriptors::default()
                .manufacturer("Dalton Tinoco")
                .product("RP2350 USB Audio")
                .serial_number("RP2350-AUDIO-001")])
            .unwrap()
            .max_packet_size_0(64)
            .unwrap()
            .max_power(100)
            .unwrap()
            .build();

        let ring = singleton!(: AudioBuffer = AudioBuffer::new()).unwrap();
        let tx_buf1 = singleton!(: [u32; BLOCK_WORDS] = [0; BLOCK_WORDS]).unwrap();
        let tx_buf2 = singleton!(: [u32; BLOCK_WORDS] = [0; BLOCK_WORDS]).unwrap();
        let mut transfer = double_buffer::Config::new((dma.ch0, dma.ch1), tx_buf1, dac_fifo_tx)
            .start()
            .read_next(tx_buf2);
        let _running_sm = dac_sm.start();
        audio_enable.set_high().unwrap(); // continuous silence until host starts
        let mut packet = [0u8; usb_audio::MAX_PACKET];
        let mut epoch = audio.epoch();
        let mut was_streaming = false;

        loop {
            // USB must be polled continuously; never block waiting for DMA or LCD.
            device.poll(&mut [&mut audio]);
            let streaming = device.state() == UsbDeviceState::Configured && audio.streaming();
            if audio.epoch() != epoch || streaming != was_streaming {
                ring.reset();
                epoch = audio.epoch();
                was_streaming = streaming;
            }
            // Drain OUT packets even while idle, so stale packets cannot block reception.
            for _ in 0..4 {
                match audio.read(&mut packet) {
                    Ok(n) if streaming => ring.push_packet(&packet[..n]),
                    Ok(_) => {}
                    Err(UsbError::WouldBlock) => break,
                    Err(_) => {
                        ring.reset();
                        break;
                    }
                }
            }
            if transfer.is_done() {
                // The other channel is running, so wait() returns immediately here.
                let (completed, next) = transfer.wait();
                if streaming {
                    ring.fill_dma(completed);
                } else {
                    completed.fill(0);
                }
                transfer = next.read_next(completed);
            }
            if streaming {
                // WouldBlock means the previous feedback packet is still queued.
                // Host polls the feedback endpoint independently of audio OUT packets.
                let _ = audio.send_feedback(ring.feedback());
            }
        }
    }
}
