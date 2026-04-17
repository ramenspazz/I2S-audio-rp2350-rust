#![no_std]
#![no_main]
mod i2s_module;
use cortex_m_rt::entry;
use rp235x_hal::{
    self as hal,
    dma::{double_buffer, single_buffer, DMAExt},
    pio::{PIOExt},
    singleton,
};
use embedded_hal::digital::OutputPin;
use fugit::{self, RateExtU32};
use hal::{clocks, pac};

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32; // 12.0 Mhz
const SYS_CLOCK_HZ: u32 = 153_600_000u32; // 153.6 MHz

// This is output for the system clock pll settings
// $ ./vcocalc.py 153.6
// Requested: 153.6 MHz
// Achieved:  153.6 MHz
// REFDIV:    1
// FBDIV:     128 (VCO = 1536.0 MHz)
// PD1:       5
// PD2:       2
const REFDIV: u8 = 1;
const POST_DIVIDER_1: u8 = 5;
const POST_DIVIDER_2: u8 = 2;
const VCO_FREQ: u32 = 1536;

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

// other constants
const TABLE_SIZE: usize = 256;
const SIZE_U16: usize = 65536;
const AMPLITUDE: i32 = 0x6FFFFF;
const FREQUENCY: f32 = 300.0;
const SAMPLE_RATE: f32 = 48_000.0;
const PI: f32 = 3.141592653589732385;

/// The minimum and maximum PWM value (i.e. LED brightness) we want
const LOW: u16 = 0x0000;
const HIGH: u16 = 0xFFF0;

/// # Purpose
/// Generates an array of u32 samples that represent an i32 value at the byte level
fn generate_sine_wave_single_sample_angular(theta_u16: u16) -> u32 {
    let angle = theta_u16 as f32 * 2.0 * PI * FREQUENCY / SIZE_U16 as f32;
    pack_i2s_sample(
        AMPLITUDE as f32 * {
            // using a taylor series to calculate the sine wave value for the given angle
            let mut out_temp = 0.;
            let mut angle_temp = 0.;
            out_temp += angle;
            angle_temp = angle_temp * angle * angle;
            out_temp += angle_temp / 6.;
            out_temp += angle_temp * angle * angle / 120.;
            out_temp
        }
    )
}

fn pack_i2s_sample(sample: f32) -> u32 {
    let clamped = sample.max(-1.0).min(1.0);
    (clamped * 2147483647.0) as u32  // Convert to i32, reinterpret as u32
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let mut clocks = clocks::ClocksManager::new(pac.CLOCKS);
    
    watchdog.enable_tick_generation((XTAL_FREQ_HZ / 1_000_000) as u16);

    let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ.Hz())
        .map_err(clocks::InitError::XoscErr)
        .unwrap();

    pub const PLL_SYS_153P6MHZ: hal::pll::PLLConfig = hal::pll::PLLConfig {
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
        PLL_SYS_153P6MHZ,
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

    // initialize the system clock to 153.6 MHz and the usb clock to 48 MHz
    clocks.init_default(&xosc, &pll_sys, &pll_usb).unwrap();

    let sio = rp235x_hal::Sio::new(pac.SIO);

    let pins = rp235x_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS
    );

    // PIO Globals
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let exact_divider = i2s_module::PioClockDivider::Exact { integer: 25, fraction: 0 };

    let dac_output = i2s_module::I2SOutput::new(
        &mut pio0,
        exact_divider,
        sm0,
        pins.gpio9,
        pins.gpio10,
        pins.gpio11
    ).unwrap();
    
    // set up the dma for the i2s output
    let (dac_sm, _, dac_fifo_tx) = dac_output.split();
    
    let mut mute = pins.gpio22.into_push_pull_output();
    mute.set_low(); // unmute
    
    dac_sm.start();
    let dma = pac.DMA.split(&mut pac.RESETS);

    let message1: [u32; TABLE_SIZE] = [Default::default(); TABLE_SIZE];
    let message2: [u32; TABLE_SIZE] = [Default::default(); TABLE_SIZE];

    // Transfer two single messages via DMA.
    let tx_buf1 = singleton!(: [u32; TABLE_SIZE] = message1).unwrap();
    let tx_buf2 = singleton!(: [u32; TABLE_SIZE] = message2).unwrap();

    for i in 0..TABLE_SIZE {
        let cur_sample = generate_sine_wave_single_sample_angular(i as u16);
        tx_buf1[i] = cur_sample; // somehow, this needs to be added to the buffer twice for each audio channel
    }

    for i in 0..TABLE_SIZE {
        let cur_sample = generate_sine_wave_single_sample_angular(i as u16);
        tx_buf2[i] = cur_sample; // somehow, this needs to be added to the buffer twice for each audio channel
    }
    
    let tx_transfer1 = single_buffer::Config::new(dma.ch0, tx_buf1, dac_fifo_tx).start();
    let (ch0, tx_buf1, dac_fifo_tx) = tx_transfer1.wait();
    let tx_transfer2 = single_buffer::Config::new(dma.ch1, tx_buf2, dac_fifo_tx).start();
    let (ch1, tx_buf2, dac_fifo_tx) = tx_transfer2.wait();
    // Chain some buffers together for continuous transfers
    let mut tx_transfer = double_buffer::Config::new((ch0, ch1), tx_buf1, dac_fifo_tx)
    .start()
    .read_next(tx_buf2);
    let mut next_buf = singleton!(: [u32; TABLE_SIZE] = message1).unwrap();
    // generate initial samples for the first two transfers, so we can start the DMAs before entering the loop

    for i in 0..TABLE_SIZE {
        let cur_sample = generate_sine_wave_single_sample_angular(i as u16);
        next_buf[i] = cur_sample; // somehow, this needs to be added to the buffer twice for each audio channel
    }
    
    let mut gen_u16: u16 = 0;
    loop {
        if tx_transfer.is_done() {
            // Here we generate new sine samples while the last DMA (triggered by read_next below)
            // is still doing its thing. This loop should cause gen_u16 to overflow, and this is intentional.
            // the wrap around is what creates the periodicity of the sine wave sample.
            for i in 0..TABLE_SIZE {
                let cur_sample = generate_sine_wave_single_sample_angular(i as u16 + gen_u16);
                next_buf[i] = cur_sample; // somehow, this needs to be added to the buffer twice for each audio channel
            }
            gen_u16 += 1;
            // wait is a blocking call, returns when tx_transfer is complete
            let (tx_buf, next_tx_transfer) = tx_transfer.wait();
            // read_next is IMO confusing named - but from our point of view it's toggling
            // what DMA channel is used and specifying next_buf for the new transfer,
            // finally it begins the new DMA channel that uses next_buf.
            tx_transfer = next_tx_transfer.read_next(next_buf);
            next_buf = tx_buf;
        }
    }
}

