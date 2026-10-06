//! Single-owner PCM ring and buffer-fill feedback controller. No interrupts/locks.
pub const BLOCK_FRAMES: usize = 48; // 1 ms at 48 kHz
pub const BLOCK_WORDS: usize = BLOCK_FRAMES * 2;
pub const CAPACITY: usize = 1024; // stereo frames, not individual samples
pub const TARGET: usize = 192; // four milliseconds in the software queue
const NOMINAL: i32 = 48 << 14;

pub struct AudioBuffer {
    frames: [[u32; 2]; CAPACITY],
    read: usize,
    len: usize,
    primed: bool,
    filtered_fill_q8: i32,
    pub underruns: u32,
    pub overruns: u32,
    pub malformed: u32,
}
impl AudioBuffer {
    pub const fn new() -> Self {
        Self {
            frames: [[0; 2]; CAPACITY],
            read: 0,
            len: 0,
            primed: false,
            filtered_fill_q8: (TARGET * 256) as i32,
            underruns: 0,
            overruns: 0,
            malformed: 0,
        }
    }
    pub fn reset(&mut self) {
        self.read = 0;
        self.len = 0;
        self.primed = false;
        self.filtered_fill_q8 = (TARGET * 256) as i32;
    }
    pub fn push_packet(&mut self, packet: &[u8]) {
        if packet.len() % 8 != 0 {
            self.malformed = self.malformed.wrapping_add(1);
            return;
        }
        if packet.len() / 8 > CAPACITY - self.len {
            self.overruns = self.overruns.wrapping_add(1);
            self.reset(); // recover by rebuffering instead of replaying old audio
        }
        if packet.len() / 8 > CAPACITY {
            return;
        }
        for frame in packet.chunks_exact(8) {
            let index = (self.read + self.len) % CAPACITY;
            self.frames[index] = [
                u32::from_le_bytes(frame[..4].try_into().unwrap()),
                u32::from_le_bytes(frame[4..].try_into().unwrap()),
            ];
            self.len += 1;
        }
    }
    /// Called once per completed 1-ms DMA buffer. Output zero until prebuffered.
    pub fn fill_dma(&mut self, output: &mut [u32; BLOCK_WORDS]) {
        output.fill(0);
        if !self.primed {
            if self.len < TARGET {
                return;
            }
            self.primed = true;
        }
        if self.len < BLOCK_FRAMES {
            self.underruns = self.underruns.wrapping_add(1);
            self.reset();
            return;
        }
        for stereo in output.chunks_exact_mut(2) {
            stereo.copy_from_slice(&self.frames[self.read]);
            self.read = (self.read + 1) % CAPACITY;
            self.len -= 1;
        }
        // Low-pass the after-refill occupancy to suppress USB-packet jitter.
        self.filtered_fill_q8 += ((self.len * 256) as i32 - self.filtered_fill_q8) / 32;
    }
    /// Full-speed UAC1 feedback: stereo frames per USB millisecond, in 10.14.
    /// Low queue => ask host for more; high queue => ask for fewer. The fixed
    /// DAC clock never changes. Gain: 1/2048 frame/ms per frame of fill error.
    pub fn feedback(&self) -> u32 {
        let error_q8 = (TARGET * 256) as i32 - self.filtered_fill_q8;
        let correction = (error_q8 / 32).clamp(-2048, 2048); // +/- 0.125 frame/ms
        (NOMINAL + correction) as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn packet(n: usize, left: i32, right: i32) -> std::vec::Vec<u8> {
        let frame = [left.to_le_bytes(), right.to_le_bytes()].concat();
        frame.repeat(n)
    }
    #[test]
    fn signed_stereo_and_prefill() {
        let mut q = AudioBuffer::new();
        let mut out = [99; BLOCK_WORDS];
        q.push_packet(&packet(48, i32::MIN, i32::MAX));
        q.fill_dma(&mut out);
        assert_eq!(out, [0; BLOCK_WORDS]);
        q.push_packet(&packet(144, i32::MIN, i32::MAX));
        q.fill_dma(&mut out);
        for pair in out.chunks_exact(2) {
            assert_eq!(pair, &[0x80000000, 0x7fffffff]);
        }
    }
    #[test]
    fn ring_wrap_and_variable_packets_preserve_order() {
        let mut q = AudioBuffer::new();
        let mut expected = std::collections::VecDeque::new();
        let mut next = 0i32;
        let mut out = [0; BLOCK_WORDS];
        for size in std::iter::once(192).chain((0..400).map(|i| if i % 2 == 0 { 47 } else { 49 })) {
            for _ in 0..size {
                q.push_packet(&packet(1, next, -next));
                expected.push_back(next);
                next += 1;
            }
            q.fill_dma(&mut out);
            for pair in out.chunks_exact(2) {
                let value = expected.pop_front().unwrap();
                assert_eq!(pair, &[value as u32, (-value) as u32]);
            }
        }
        assert_eq!(q.underruns, 0);
        assert_eq!(q.overruns, 0);
    }
    #[test]
    fn malformed_overflow_underflow_and_reset() {
        let mut q = AudioBuffer::new();
        let mut out = [0; BLOCK_WORDS];
        q.push_packet(&[1, 2, 3]);
        assert_eq!(q.malformed, 1);
        assert_eq!(q.len, 0);
        q.push_packet(&packet(CAPACITY, 1, 2));
        q.push_packet(&packet(48, 3, 4));
        assert_eq!(q.overruns, 1);
        assert_eq!(q.len, 48);
        q.push_packet(&packet(144, 3, 4));
        for _ in 0..5 {
            q.fill_dma(&mut out);
        }
        assert_eq!(q.underruns, 1);
        assert_eq!(out, [0; BLOCK_WORDS]);
        q.reset();
        assert_eq!(q.feedback(), NOMINAL as u32);
    }
    #[test]
    fn feedback_handles_clock_drift_and_delayed_host_response() {
        for ppm in [-1000.0, -100.0, 0.0, 100.0, 1000.0] {
            let mut q = AudioBuffer::new();
            let mut out = [0; BLOCK_WORDS];
            let mut fractions = 0.0;
            let mut delayed = std::collections::VecDeque::from([48.0; 16]);
            // Each iteration represents one DAC millisecond; the host clock differs.
            for _ in 0..60_000 {
                fractions += delayed.pop_front().unwrap() / (1.0 + ppm / 1e6);
                let count = fractions as usize;
                fractions -= count as f64;
                q.push_packet(&packet(count, 100, -100));
                q.fill_dma(&mut out);
                delayed.push_back(q.feedback() as f64 / 16384.0);
            }
            assert_eq!(q.underruns, 0, "ppm={ppm}");
            assert_eq!(q.overruns, 0, "ppm={ppm}");
            assert!(q.len > 48 && q.len < 384, "ppm={ppm} fill={}", q.len);
        }
    }
}
