//! I2S Library

// always good to have pi and e, cuz why not
pub const PI: f32 = 3.141592653589793238;
pub const E_MATH: f32 = 2.71828182845904523536;

// clock constants
pub const SAMPLE_RATE: u32 = 48_000;
// The bit clock pulses once for each discrete bit of data on the data lines. The bit clock
// frequency is the product of the sample rate, the number of bits per channel and the number
// of channels. So, for example, CD Audio with a sample frequency of 44.1 kHz, with 16 bits of
// precision and two channels (stereo) has a bit clock frequency of:
//     44.1 kHz × 16 × 2 = 1.4112 MHz
// With out 48 kHz sample rate and 32 bits per channel, the bit clock frequency is:
pub const BIT_CLOCK_HZ: u32 = SAMPLE_RATE * 32 * 2; // 6.144 MHz
pub const CLOCK_MULTIPLIER: u32 = 64;
pub const SYS_CLOCK_HZ: u32 = 193_500_000; // 193.5 MHz.
// The USB clock needs to be 48 MHz. This is sufficent to transmit 48 kHz stereo audio at 32 bits per channel.
pub const USB_CLOCK_HZ: u32 = 48_000_000u32; // 48 MHz

// other constants
const SIZE_U16: usize = 65536;
const AMPLITUDE: i32 = 0x6FFFFF;
const FREQUENCY: f32 = 300.0;
pub const TABLE_SIZE: usize = 254;

// The minimum and maximum PWM value (i.e. LED brightness) we want
pub const LOW: u16 = 0x0000;
pub const HIGH: u16 = 0xFFF0;

/// # Purpose
/// Generates an array of u32 samples that represent an i32 value at the byte level
pub fn generate_sine_wave_single_sample_angular(theta_u16: u16) -> u32 {
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

pub fn pack_i2s_sample(sample: f32) -> u32 {
    let clamped = sample.max(-1.0).min(1.0);
    (clamped * 2147483647.0) as u32  // Convert to i32, reinterpret as u32
}

use pio::pio_asm;  // For PIO assembly macro
use rp235x_hal::{
    gpio::{FunctionNull, Pin, PinId, PullDown, ValidFunction}, pio::{
        InstallError, PIO, PIOExt, Rx, StateMachine, StateMachineIndex, Stopped, Tx, UninitStateMachine, ValidStateMachine
    }
};
use fugit::{self, HertzU32};

#[derive(Debug)]
pub enum I2SError {
    PioInstallationError(InstallError),
}

pub enum PioClockDivider {
    Exact { integer: u16, fraction: u8 },
    FromSystemClock(HertzU32),
}

impl PioClockDivider {
    fn pio_divider(&self) -> (u16, u8) {
        match self {
            Self::Exact { integer, fraction } => (*integer, *fraction),
            Self::FromSystemClock(system_clock_hz) => {
                let hertz = system_clock_hz.to_Hz();
                let (fraction, integer) = libm::modf(hertz as f64 / SYS_CLOCK_HZ as f64);

                (integer as u16, (fraction * 256.0) as u8)
            },
        }
    }
}

pub trait SampleReader {
    fn read(&mut self) -> Option<u32>;
}

impl<SM: ValidStateMachine> SampleReader for Rx<SM> {
    fn read(&mut self) -> Option<u32> {
        self.read()
    }
}

pub struct I2SOutput<P: rp235x_hal::pio::PIOExt, SM: rp235x_hal::pio::StateMachineIndex> {
    state_machine: StateMachine<(P, SM), Stopped>,
    fifo_rx: Rx<(P, SM)>,
    fifo_tx: Tx<(P, SM)>,
}

impl<P: PIOExt, SM: StateMachineIndex> I2SOutput<P, SM> {
    /// Create an I2S output with a data line, a bit clock, and a left/right word clock.
    /// The left/right word clock pin MUST consecutively follow the bit clock pin. So if
    /// the bit clock pin is 7, the word clock pin MUST be 8.
    pub fn new<DataPin, BitClockPin, LeftRightClockPin>(
        pio: &mut PIO<P>,
        clock_divider: PioClockDivider,
        state_machine: UninitStateMachine<(P, SM)>,
        data_out_pin: Pin<DataPin, FunctionNull, PullDown>,
        bit_clock_pin: Pin<BitClockPin, FunctionNull, PullDown>,
        left_right_clock_pin: Pin<LeftRightClockPin, FunctionNull, PullDown>,
    ) -> Result<Self, I2SError>
    where
        DataPin: PinId + ValidFunction<P::PinFunction>,
        BitClockPin: PinId + ValidFunction<P::PinFunction>,
        LeftRightClockPin: PinId + ValidFunction<P::PinFunction>,
    {
        let data_out_pin: Pin<_, P::PinFunction, _> = data_out_pin.into_function();
        let bit_clock_pin: Pin<_, P::PinFunction, _> = bit_clock_pin.into_function();
        let left_right_clock_pin: Pin<_, P::PinFunction, _> = left_right_clock_pin.into_function();

        let data_pin_id = data_out_pin.id().num;
        let bit_clock_pin_id = bit_clock_pin.id().num;
        let left_right_clock_pin_id = left_right_clock_pin.id().num;

        assert_eq!(
            left_right_clock_pin_id - bit_clock_pin_id,
            1,
            "The word clock pin must consecutively follow the bit clock pin"
        );

        #[rustfmt::skip]
        let dac_pio_program = pio_asm!(
            ".side_set 2",
            ".wrap_target",

            // Left channel
            "    pull noblock     side 0b00",
            "    set x, 31        side 0b01",   // BCLK high, LR low
            "left_loop:",
            "    out pins, 1      side 0b01",   // data bit + BCLK high
            "    nop              side 0b00",   // BCLK low (data held)
            "    jmp x-- left_loop side 0b01",

            // Right channel
            "    pull noblock     side 0b10",
            "    set x, 31        side 0b11",   // BCLK high, LR high
            "right_loop:",
            "    out pins, 1      side 0b11",
            "    nop              side 0b10",
            "    jmp x-- right_loop side 0b11",

            ".wrap",
        );

        let installed =
            pio.install(&dac_pio_program.program).map_err(I2SError::PioInstallationError)?;

        let (divider_int, divider_fraction) = clock_divider.pio_divider();

        let (mut dac_sm, fifo_rx, fifo_tx) =
            rp235x_hal::pio::PIOBuilder::from_installed_program(installed)
                .out_pins(data_pin_id, 1)
                .side_set_pin_base(bit_clock_pin_id)
                .out_shift_direction(rp235x_hal::pio::ShiftDirection::Left)
                .clock_divisor_fixed_point(divider_int, divider_fraction)
                .autopull(true)
                .pull_threshold(32)
                .buffers(rp235x_hal::pio::Buffers::OnlyTx)
                .build(state_machine);

        dac_sm.set_pindirs([
            (data_pin_id, rp235x_hal::pio::PinDir::Output),
            (bit_clock_pin_id, rp235x_hal::pio::PinDir::Output),
            (left_right_clock_pin_id, rp235x_hal::pio::PinDir::Output),
        ]);

        Ok(Self { state_machine: dac_sm, fifo_rx, fifo_tx })
    }

    #[allow(clippy::type_complexity)]
    pub fn split(self) -> (StateMachine<(P, SM), Stopped>, Rx<(P, SM)>, Tx<(P, SM)>) {
        (self.state_machine, self.fifo_rx, self.fifo_tx)
    }
}
