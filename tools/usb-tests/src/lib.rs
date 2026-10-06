#[path = "../../../src/audio_buffer.rs"]
pub mod audio_buffer;
#[path = "../../../src/usb_audio.rs"]
pub mod usb_audio;

#[cfg(test)]
mod tests {
    use super::usb_audio::*;
    use std::{
        collections::VecDeque,
        sync::{Arc, Mutex},
    };
    use usb_device::{bus::PollResult, class_prelude::*, prelude::*, UsbDirection, UsbError};

    #[derive(Default)]
    struct Wire {
        events: VecDeque<PollResult>,
        reads: VecDeque<(u8, Vec<u8>)>,
        writes: Vec<(u8, Vec<u8>)>,
        stalled: bool,
    }
    struct MockBus {
        wire: Arc<Mutex<Wire>>,
        next_in: usize,
        next_out: usize,
    }
    impl UsbBus for MockBus {
        fn alloc_ep(
            &mut self,
            dir: UsbDirection,
            addr: Option<EndpointAddress>,
            _: EndpointType,
            _: u16,
            _: u8,
        ) -> usb_device::Result<EndpointAddress> {
            if let Some(addr) = addr {
                return Ok(addr);
            }
            let next = if dir == UsbDirection::In {
                &mut self.next_in
            } else {
                &mut self.next_out
            };
            let addr = EndpointAddress::from_parts(*next, dir);
            *next += 1;
            Ok(addr)
        }
        fn enable(&mut self) {}
        fn reset(&self) {}
        fn set_device_address(&self, _: u8) {}
        fn write(&self, addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
            let mut w = self.wire.lock().unwrap();
            w.writes.push((addr.into(), buf.to_vec()));
            w.events.push_back(PollResult::Data {
                ep_out: 0,
                ep_setup: 0,
                ep_in_complete: 1 << addr.index(),
            });
            Ok(buf.len())
        }
        fn read(&self, addr: EndpointAddress, buf: &mut [u8]) -> usb_device::Result<usize> {
            let mut w = self.wire.lock().unwrap();
            let Some(i) = w.reads.iter().position(|(a, _)| *a == u8::from(addr)) else {
                return Err(UsbError::WouldBlock);
            };
            let (_, bytes) = w.reads.remove(i).unwrap();
            if bytes.len() > buf.len() {
                return Err(UsbError::BufferOverflow);
            }
            buf[..bytes.len()].copy_from_slice(&bytes);
            Ok(bytes.len())
        }
        fn set_stalled(&self, _: EndpointAddress, stalled: bool) {
            self.wire.lock().unwrap().stalled = stalled;
        }
        fn is_stalled(&self, _: EndpointAddress) -> bool {
            self.wire.lock().unwrap().stalled
        }
        fn suspend(&self) {}
        fn resume(&self) {}
        fn poll(&self) -> PollResult {
            self.wire
                .lock()
                .unwrap()
                .events
                .pop_front()
                .unwrap_or(PollResult::None)
        }
    }
    fn bus() -> (UsbBusAllocator<MockBus>, Arc<Mutex<Wire>>) {
        let wire = Arc::new(Mutex::new(Wire::default()));
        (
            UsbBusAllocator::new(MockBus {
                wire: wire.clone(),
                next_in: 1,
                next_out: 1,
            }),
            wire,
        )
    }
    fn transfer(
        device: &mut UsbDevice<MockBus>,
        audio: &mut UsbAudio<MockBus>,
        wire: &Arc<Mutex<Wire>>,
        setup: [u8; 8],
        data: &[u8],
    ) -> (Vec<u8>, bool) {
        {
            let mut w = wire.lock().unwrap();
            w.writes.clear();
            w.stalled = false;
            w.reads.push_back((0, setup.to_vec()));
            w.events.push_back(PollResult::Data {
                ep_setup: 1,
                ep_out: 0,
                ep_in_complete: 0,
            });
        }
        device.poll(&mut [audio]);
        if setup[0] & 0x80 == 0 && !data.is_empty() {
            let mut w = wire.lock().unwrap();
            w.reads.push_back((0, data.to_vec()));
            w.events.push_back(PollResult::Data {
                ep_setup: 0,
                ep_out: 1,
                ep_in_complete: 0,
            });
        }
        for _ in 0..32 {
            device.poll(&mut [audio]);
        }
        let answer = {
            let w = wire.lock().unwrap();
            (
                w.writes
                    .iter()
                    .filter(|(a, _)| *a == 0x80)
                    .flat_map(|(_, b)| b.clone())
                    .collect(),
                w.stalled,
            )
        };
        if setup[0] & 0x80 != 0 && !answer.1 {
            {
                let mut w = wire.lock().unwrap();
                w.reads.push_back((0, vec![]));
                w.events.push_back(PollResult::Data {
                    ep_setup: 0,
                    ep_out: 1,
                    ep_in_complete: 0,
                });
            }
            device.poll(&mut [audio]);
        }
        answer
    }
    #[test]
    fn real_stack_emits_consistent_uac1_descriptors() {
        let (bus, wire) = bus();
        let mut audio = UsbAudio::new(&bus);
        let mut device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x1209, 1))
            .max_packet_size_0(64)
            .unwrap()
            .build();
        let (bytes, stalled) = transfer(
            &mut device,
            &mut audio,
            &wire,
            [0x80, 6, 0, 2, 0, 0, 255, 0],
            &[],
        );
        assert!(!stalled);
        assert_eq!(bytes.len(), 109);
        assert_eq!(bytes[4], 2);
        assert_eq!(
            u16::from_le_bytes([bytes[2], bytes[3]]) as usize,
            bytes.len()
        );
        let mut descriptors = Vec::new();
        let mut remaining = bytes.as_slice();
        while !remaining.is_empty() {
            let n = remaining[0] as usize;
            assert!(n >= 2);
            descriptors.push(&remaining[..n]);
            remaining = &remaining[n..];
        }
        let interfaces: Vec<_> = descriptors.iter().filter(|d| d[1] == 4).collect();
        assert_eq!(
            interfaces
                .iter()
                .map(|d| (d[2], d[3], d[4]))
                .collect::<Vec<_>>(),
            vec![(0, 0, 0), (1, 0, 0), (1, 1, 2)]
        );
        assert!(descriptors.contains(&&[11, 0x24, 2, 1, 2, 4, 32, 1, 0x80, 0xbb, 0][..]));
        let endpoints: Vec<_> = descriptors.iter().filter(|d| d[1] == 5).collect();
        assert_eq!(*endpoints[0], &[9, 5, 1, 5, 0x88, 1, 1, 0, 0x81]); // 392 bytes, async, sync=81
        assert_eq!(*endpoints[1], &[9, 5, 0x81, 1, 3, 0, 1, 4, 0]);
    }
    #[test]
    fn rate_controls_alt_settings_and_reset() {
        let (bus, wire) = bus();
        let mut audio = UsbAudio::new(&bus);
        let mut device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x1209, 1))
            .max_packet_size_0(64)
            .unwrap()
            .build();
        let (data, stall) = transfer(
            &mut device,
            &mut audio,
            &wire,
            [0xa2, 0x81, 0, 1, 1, 0, 3, 0],
            &[],
        );
        assert!(!stall);
        assert_eq!(data, [0x80, 0xbb, 0]);
        assert!(
            !transfer(
                &mut device,
                &mut audio,
                &wire,
                [0x22, 1, 0, 1, 1, 0, 3, 0],
                &[0x80, 0xbb, 0]
            )
            .1
        );
        assert!(
            transfer(
                &mut device,
                &mut audio,
                &wire,
                [0x22, 1, 0, 1, 1, 0, 3, 0],
                &[0x44, 0xac, 0]
            )
            .1
        );
        assert!(!audio.streaming());
        assert!(
            !transfer(
                &mut device,
                &mut audio,
                &wire,
                [1, 11, 1, 0, 1, 0, 0, 0],
                &[]
            )
            .1
        );
        assert!(audio.streaming());
        let epoch = audio.epoch();
        assert!(
            transfer(
                &mut device,
                &mut audio,
                &wire,
                [1, 11, 2, 0, 1, 0, 0, 0],
                &[]
            )
            .1
        );
        assert!(audio.streaming());
        wire.lock().unwrap().events.push_back(PollResult::Reset);
        device.poll(&mut [&mut audio]);
        assert!(!audio.streaming());
        assert_ne!(epoch, audio.epoch());
    }
    #[test]
    fn feedback_is_three_byte_10_14() {
        let (bus, wire) = bus();
        let audio = UsbAudio::new(&bus);
        let _device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x1209, 1)).build();
        assert_eq!(audio.send_feedback(48 << 14).unwrap(), 3);
        assert_eq!(
            wire.lock().unwrap().writes.last().unwrap(),
            &(0x81, vec![0, 0, 12])
        );
    }
}
