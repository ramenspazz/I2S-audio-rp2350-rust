//! Minimal UAC 1.0 playback: 48 kHz, stereo S32_LE, explicit 10.14 feedback.
//! No feature unit: volume is controlled in host software, not by USB hardware controls.
use usb_device::{
    class_prelude::*,
    control::{Recipient, RequestType},
    UsbError,
};

pub const RATE: u32 = 48_000;
pub const MAX_PACKET: usize = 49 * 2 * 4;

pub struct UsbAudio<'a, B: UsbBus> {
    control: InterfaceNumber,
    stream: InterfaceNumber,
    output: EndpointOut<'a, B>,
    feedback: EndpointIn<'a, B>,
    alternate: u8,
    epoch: u32,
}

impl<'a, B: UsbBus> UsbAudio<'a, B> {
    pub fn new(bus: &'a UsbBusAllocator<B>) -> Self {
        Self {
            control: bus.interface(),
            stream: bus.interface(),
            output: bus
                .alloc(
                    None,
                    EndpointType::Isochronous {
                        synchronization: IsochronousSynchronizationType::Asynchronous,
                        usage: IsochronousUsageType::Data,
                    },
                    MAX_PACKET as u16,
                    1,
                )
                .unwrap(),
            // UAC1 sync endpoint descriptor uses bmAttributes=0x01 (reserved bits zero).
            feedback: bus
                .alloc(
                    None,
                    EndpointType::Isochronous {
                        synchronization: IsochronousSynchronizationType::NoSynchronization,
                        usage: IsochronousUsageType::Data,
                    },
                    3,
                    1,
                )
                .unwrap(),
            alternate: 0,
            epoch: 0,
        }
    }

    pub fn streaming(&self) -> bool {
        self.alternate == 1
    }
    pub fn epoch(&self) -> u32 {
        self.epoch
    }
    pub fn read(&self, packet: &mut [u8]) -> Result<usize, UsbError> {
        self.output.read(packet)
    }
    pub fn send_feedback(&self, value: u32) -> Result<usize, UsbError> {
        self.feedback.write(&value.to_le_bytes()[..3])
    }
}

impl<B: UsbBus> UsbClass<B> for UsbAudio<'_, B> {
    fn get_configuration_descriptors(&self, w: &mut DescriptorWriter) -> usb_device::Result<()> {
        // AudioControl interface; 30-byte class-specific AC collection.
        w.interface(self.control, 1, 1, 0)?;
        w.write(0x24, &[1, 0x00, 0x01, 30, 0, 1, self.stream.into()])?;
        // USB streaming input terminal 1, stereo left/right.
        w.write(0x24, &[2, 1, 0x01, 0x01, 0, 2, 3, 0, 0, 0])?;
        // Speaker output terminal 2, sourced directly from terminal 1.
        w.write(0x24, &[3, 2, 0x01, 0x03, 0, 1, 0])?;
        // Alternate 0 has no endpoints; alternate 1 is the active stream.
        w.interface(self.stream, 1, 2, 0)?;
        w.interface_alt(self.stream, 1, 1, 2, 0, None)?;
        w.write(0x24, &[1, 1, 1, 1, 0])?; // AS general, PCM format
        w.write(0x24, &[2, 1, 2, 4, 32, 1, 0x80, 0xbb, 0])?; // 48000 Hz
        w.endpoint_ex(&self.output, |extra| {
            extra[0] = 0; // bRefresh: data endpoint
            extra[1] = self.feedback.address().into(); // bSynchAddress
            Ok(2)
        })?;
        // Sampling-frequency control; no pitch control, no lock delay.
        w.write(0x25, &[1, 1, 0, 0, 0])?;
        w.endpoint_ex(&self.feedback, |extra| {
            extra[0] = 4; // Feedback refresh bound: 2^4 USB frames
            extra[1] = 0;
            Ok(2)
        })
    }

    fn reset(&mut self) {
        self.alternate = 0;
        self.epoch = self.epoch.wrapping_add(1);
    }
    fn get_alt_setting(&mut self, interface: InterfaceNumber) -> Option<u8> {
        if interface == self.stream {
            Some(self.alternate)
        } else if interface == self.control {
            Some(0)
        } else {
            None
        }
    }
    fn set_alt_setting(&mut self, interface: InterfaceNumber, alternative: u8) -> bool {
        if interface == self.stream && alternative <= 1 {
            self.alternate = alternative;
            self.epoch = self.epoch.wrapping_add(1);
            true
        } else {
            interface == self.control && alternative == 0
        }
    }
    fn control_in(&mut self, xfer: ControlIn<B>) {
        let r = xfer.request();
        if r.request_type == RequestType::Class
            && r.recipient == Recipient::Endpoint
            && r.index == u8::from(self.output.address()) as u16
        {
            if r.value == 0x0100 && r.length == 3 {
                let value = match r.request {
                    0x81..=0x83 => RATE, // GET_CUR/MIN/MAX: one fixed rate
                    0x84 => 0,           // GET_RES: no adjustable range
                    _ => {
                        let _ = xfer.reject();
                        return;
                    }
                };
                let _ = xfer.accept_with(&value.to_le_bytes()[..3]);
            } else {
                let _ = xfer.reject();
            }
        }
    }
    fn control_out(&mut self, xfer: ControlOut<B>) {
        let r = xfer.request();
        if r.request_type == RequestType::Class
            && r.recipient == Recipient::Endpoint
            && r.index == u8::from(self.output.address()) as u16
        {
            if r.request == 1
                && r.value == 0x0100
                && r.length == 3
                && xfer.data() == &RATE.to_le_bytes()[..3]
            {
                let _ = xfer.accept();
            } else {
                let _ = xfer.reject();
            }
        }
    }
}
