#![no_std]
#![no_main]
use pio::pio_asm;  // For PIO assembly macro
use rp235x_hal::{
    gpio::{FunctionNull, Pin, PinId, PullDown, ValidFunction}, pio::{
        InstallError, PIO, PIOExt, Rx, StateMachine, StateMachineIndex, Stopped, Tx, UninitStateMachine, ValidStateMachine
    }
};
use fugit::{self, HertzU32};

const SYS_CLOCK_HZ: u32 = 153_600_000u32; // 153.6 MHz

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
            ".side_set 2",          // bit 0 = BCLK, bit 1 = LRCLK (adjust mapping in I2SOutput::new if needed)
            ".wrap_target",
            "    pull noblock side 0b00",     // get next 32-bit sample (left or right)
            "    set x, 31 side 0b01",        // prepare for 32 bits, start BCLK high + LRCLK low (left channel)
            "left_loop:",
            "    out pins, 1 side 0b01",      // output data bit, BCLK high
            "    nop     side 0b00",          // BCLK low (data held stable)
            "    jmp x-- left_loop side 0b01",// back to high
            "    pull noblock side 0b10",     // next sample (right channel)
            "    set x, 31 side 0b11",        // LRCLK high for right
            "right_loop:",
            "    out pins, 1 side 0b11",
            "    nop     side 0b10",
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
