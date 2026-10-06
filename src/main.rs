#![no_std]
#![no_main]

#![allow(unused_imports)]
use cortex_m_rt::entry; use heapless::String;
// used, but compiler will complain if unused_inports are not allowed
use rp235x_hal::{
    self as hal,
    clocks,
    // clocks::ClockSource,
    dma::{double_buffer, single_buffer, DMAExt},
    gpio::{FunctionI2C, Pin},
    pac,
    pio::{PIOExt},
    singleton,
    Sio,
    // usb::UsbBus,
};
use rp235x_hal::clocks::ClockSource;
use core::fmt::Write;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;
use fugit::{self, RateExtU32};

// https://crates.io/crates/i2c-character-display
use i2c_character_display::{AdafruitLCDBackpack, CharacterDisplayPCF8574T, LcdDisplayType};

// usb stuff
// use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
// use usbd_serial::SerialPort;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

mod i2s_lib;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32; // 12.0 Mhz

// This is output for the system clock pll settings
// $ ./vcocalc.py 193.608
// Requested: 193.608 MHz
// Achieved:  193.5 MHz
// REFDIV:    1
// FBDIV:     129 (VCO = 1548.0 MHz)
// PD1:       4
// PD2:       2
const REFDIV: u8 = 1;
const POST_DIVIDER_1: u8 = 4;
const POST_DIVIDER_2: u8 = 2;
const VCO_FREQ: u32 = 1548; 

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

fn output_u32_num_as_string(number: &u32) -> String<16> {
    // Create an empty and growable `String`
    let mut string = String::new();
    // collect numbers from the string first.
    // The first step is to determine how many digits the number consists of.
    let mut digit_count = 0;
    let mut temp_number = *number;
    while temp_number > 0 {
        temp_number /= 10;
        digit_count += 1;
    }
    // Now we can extract each digit and convert it to a char, starting from the most significant digit.
    for i in (0..digit_count).rev() {
        let divisor = 10u32.pow(i);
        let digit = (number / divisor) % 10;
        string.push((digit as u8 + b'0') as char);
    }
    // return string
    string
}

fn fill_buffer(buf: &mut [u32], phase: &mut u16) {
    for i in 0..(buf.len() / 2) {
        let sample = i2s_lib::generate_sine_wave_single_sample_angular(
            (i as u16).wrapping_add(*phase)
        );
        let idx = i * 2;
        buf[idx]     = sample;  // Left channel
        buf[idx + 1] = sample;  // Right channel (mono test)
    }
    *phase = phase.wrapping_add(1);
}

#[rp235x_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    // SAFETY: We are the only ones accessing the power management registers, and we are following the procedure
    //outlined in the datasheet, so this should be safe.
    unsafe {
        let powman = &*rp235x_pac::POWMAN::ptr();

        // 1. Unlock the VREG control
        // This is usually done by writing password + unlock bit (often bit 13 or bit 12)
        powman.vreg_ctrl().write(|w| {
            w.bits(
                (0x5AFEu32 << 16) | (1u32 << 13)   // password + unlock bit
                // You may need to OR in other bits (e.g. temperature threshold) — check datasheet
            )
        });

        // 2. Set the voltage to 01100 binary (1.15 V)
        // Assuming the voltage select field is in bits 8:4 (very common on RP series)
        powman.vreg().write(|w| {
            w.bits(
                (0x5AFEu32 << 16)               // password
                | (0b01100u32 << 4)             // voltage encoding 01100 → 1.15 V
                // | other bits if needed (e.g. enable bits)
            )
        });
    }

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let mut clocks = clocks::ClocksManager::new(pac.CLOCKS);
    
    watchdog.enable_tick_generation((XTAL_FREQ_HZ / 1_000_000) as u16);

    let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ.Hz())
        .map_err(clocks::InitError::XoscErr)
        .unwrap();

    pub const PLL_SYS_193P5MHZ: hal::pll::PLLConfig = hal::pll::PLLConfig {
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
        PLL_SYS_193P5MHZ,
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

    // initialize the system clock to 196.608 MHz and the usb clock to 48 MHz
    clocks.init_default(&xosc, &pll_sys, &pll_usb).unwrap();

    let sio = rp235x_hal::Sio::new(pac.SIO);

    let pins = rp235x_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS
    );

    // set up and configure the HD44780 based LCD display over i2c
    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio26.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio27.reconfigure();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut delay = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);    // init LCD1602

    // PCF8574T adapter for a single HD44780 controller using a 20x4 character LCD.
    let mut lcd = CharacterDisplayPCF8574T::new(i2c, LcdDisplayType::Lcd20x4, delay);
    if let Err(e) = lcd.init() {
        panic!("Error initializing LCD: {}", e);
    }

    // set up the display
    lcd.backlight(true);
    lcd.clear();
    lcd.home();
    lcd.print("I2S AUDIO OUTPUT");
    lcd.blink_cursor(true);
    delay.delay_ms(500u32); // wait for 0.5 seconds


    // Get the actual system clock frequency (this reflects your overclock)
    let sys_freq_hz = clocks.system_clock.get_freq().to_Hz();
    let mut output_string = output_u32_num_as_string(&sys_freq_hz);
    lcd.clear();
    // append unit to the numeric string
    lcd.print("SYS CLOCK in Hz");
    lcd.set_cursor(0, 1);
    lcd.print(&output_string);

    // PIO Globals
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let exact_divider = i2s_lib::PioClockDivider::Exact { integer: 1, fraction: 0 };

    let dac_output = i2s_lib::I2SOutput::new(
        &mut pio0,
        exact_divider,
        sm0,
        pins.gpio9,
        pins.gpio10,
        pins.gpio11
    ).unwrap();
    
    let (dac_sm, _, dac_fifo_tx) = dac_output.split();
    
    let dma = pac.DMA.split(&mut pac.RESETS);

    // Static buffers
    let tx_buf1 = singleton!(: [u32; i2s_lib::TABLE_SIZE] = [0; i2s_lib::TABLE_SIZE]).unwrap();
    let tx_buf2 = singleton!(: [u32; i2s_lib::TABLE_SIZE] = [0; i2s_lib::TABLE_SIZE]).unwrap();

    let mut phase: u16 = 0;

    // Fill both buffers with stereo data (left + right)
    fill_buffer(tx_buf1, &mut phase);
    fill_buffer(tx_buf2, &mut phase);

    // === Double-buffered DMA setup ===
    // Create and queue the initial double-buffered transfer (start with tx_buf1, queue tx_buf2)
    let mut tx_transfer = double_buffer::Config::new(
        (dma.ch0, dma.ch1),
        tx_buf1,
        dac_fifo_tx,
    ).start().read_next(tx_buf2);

    // === Start audio ===
    let mut mute = pins.gpio22.into_push_pull_output();
    mute.set_low();   // unmute DAC
    dac_sm.start();

    // Main loop
    loop {
        if tx_transfer.is_done() {
            let (completed_buf, next_transfer) = tx_transfer.wait();

            // Refill the buffer that just finished
            fill_buffer(completed_buf, &mut phase);

            // Hand it back as the next buffer
            tx_transfer = next_transfer.read_next(completed_buf);
        }
    }
}

