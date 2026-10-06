"""Generate a quiet 48 kHz signed-32-bit stereo WAV for USB playback testing."""
import math
import struct
import sys
import wave

if len(sys.argv) != 2:
    raise SystemExit('Usage: python3 tools/make_usb_test.py OUTPUT.wav')
rate = 48_000
samples = bytearray()
for channel in range(2):
    for i in range(2 * rate):
        envelope = min(1.0, i / (0.05 * rate), (2 * rate - 1 - i) / (0.05 * rate))
        value = round(0.05 * (2**31 - 1) * envelope * math.sin(2 * math.pi * 300 * i / rate))
        pair = (value, 0) if channel == 0 else (0, value)
        samples.extend(struct.pack('<ii', *pair))
with wave.open(sys.argv[1], 'wb') as output:
    output.setnchannels(2)
    output.setsampwidth(4)
    output.setframerate(rate)
    output.writeframes(samples)
print(f'Wrote {sys.argv[1]}: 48 kHz S32_LE, 5% peak, left then right.')
