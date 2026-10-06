//! I2S Library

/// Stereo frames per second, with two 32-bit slots and two PIO cycles/bit.
pub const SAMPLE_RATE: u32 = 48_000;
pub const BIT_CLOCK_HZ: u32 = SAMPLE_RATE * 32 * 2; // 3.072 MHz
pub const PIO_CLOCK_HZ: u32 = BIT_CLOCK_HZ * 2; // 6.144 MHz

// 300 Hz diagnostic tone: 160 frames per period, six periods per buffer.
#[cfg(feature = "test-tone")]
pub const FRAMES_PER_PERIOD: usize = 160;
#[cfg(feature = "test-tone")]
pub const TABLE_SIZE: usize = FRAMES_PER_PERIOD * 6 * 2;

/// Identical buffers of whole periods can be replayed without discontinuities.
/// Compute once at startup, so sample generation cannot starve DMA.
#[cfg(feature = "test-tone")]
pub fn fill_test_tone(buf: &mut [u32; TABLE_SIZE], volume_level: f32) {
    for (frame, stereo) in buf.chunks_exact_mut(2).enumerate() {
        let angle = (frame % FRAMES_PER_PERIOD) as f32 * 2.0 * core::f32::consts::PI
            / FRAMES_PER_PERIOD as f32;
        // Signed 24-bit PCM at 5% amplitude, left-aligned in a 32-bit slot.
        let sample = (libm::sinf(angle) * (0x7fffff as f32 * volume_level / 100.0)) as i32;
        let word = (sample as u32) << 8;
        stereo[0] = word;
        stereo[1] = word;
    }
}

use fugit::{self, HertzU32};
use pio::pio_asm; // For PIO assembly macro
use rp235x_hal::{
    gpio::{FunctionNull, Pin, PinId, PullDown, ValidFunction},
    pio::{
        InstallError, PIOExt, Rx, StateMachine, StateMachineIndex, Stopped, Tx, UninitStateMachine,
        ValidStateMachine, PIO,
    },
};

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
                // Round to the nearest representable 16.8 divider.
                let fixed = ((system_clock_hz.to_Hz() as u64 * 256 + PIO_CLOCK_HZ as u64 / 2)
                    / PIO_CLOCK_HZ as u64) as u32;
                assert!((256..=0x00ff_ffff).contains(&fixed));
                ((fixed >> 8) as u16, (fixed & 255) as u8)
            }
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
            ".side_set 2", // bit 0 BCLK, bit 1 LRCLK
            // One-time setup. LRCLK changes one bit before each next MSB.
            "set x, 30 side 0b01",
            ".wrap_target",
            "left_loop:",
            "out pins, 1 side 0b00",
            "jmp x-- left_loop side 0b01",
            "out pins, 1 side 0b10", // left LSB; announce right channel
            "set x, 30 side 0b11",
            "right_loop:",
            "out pins, 1 side 0b10",
            "jmp x-- right_loop side 0b11",
            "out pins, 1 side 0b00", // right LSB; announce left channel
            "set x, 30 side 0b01",
            ".wrap",
        );

        let installed = pio
            .install(&dac_pio_program.program)
            .map_err(I2SError::PioInstallationError)?;

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

        Ok(Self {
            state_machine: dac_sm,
            fifo_rx,
            fifo_tx,
        })
    }

    #[allow(clippy::type_complexity)]
    pub fn split(self) -> (StateMachine<(P, SM), Stopped>, Rx<(P, SM)>, Tx<(P, SM)>) {
        (self.state_machine, self.fifo_rx, self.fifo_tx)
    }
}
