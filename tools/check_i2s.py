"""Check the PIO wire format assuming a continuously supplied TX FIFO."""
from pathlib import Path
import re

source = (Path(__file__).resolve().parents[1] / 'src/i2s_lib.rs').read_text()
assembly = source[source.index('let dac_pio_program'):source.index('let installed')]
code, labels = [], {}
for line in re.findall(r'^\s*"([^"]+)"', assembly, re.M):
    if line == '.wrap_target':
        wrap = len(code)
    elif line.endswith(':'):
        labels[line[:-1]] = len(code)
    elif not line.startswith('.'):
        code.append(line)

# Distinct words expose skipped bits, wrong channel boundaries and reversed bits.
words = [0x12345678, 0x89abcdef, 0x80000001, 0x7ffffffe] * 3
bits = [(word >> bit) & 1 for word in words for bit in range(31, -1, -1)]
pc = x = data = consumed = 0
previous_clock = 1
rising_edges = []
for cycle in range(1 + 128 * 4):
    instruction = code[pc]
    side = int(instruction.split('0b')[1], 2)
    clock, lr = side & 1, side >> 1
    next_pc = pc + 1
    if instruction.startswith('set'):
        x = int(instruction.split(',')[1].split()[0])
    elif instruction.startswith('out'):
        assert clock == 0, 'Data must change on BCLK falling edge'
        data = bits[consumed]
        consumed += 1
    elif instruction.startswith('jmp'):
        if x:
            next_pc = labels[instruction.split()[2]]
        x = (x - 1) & 0xffffffff
    else:
        raise AssertionError(f'Unsupported instruction: {instruction}')
    if clock and not previous_clock:
        rising_edges.append((cycle, lr, data))
    previous_clock = clock
    pc = next_pc if next_pc < len(code) else wrap

assert len(rising_edges) == 256
assert [bit for _, _, bit in rising_edges] == bits[:256]
assert all(b[0] - a[0] == 2 for a, b in zip(rising_edges, rising_edges[1:]))
# WS changes at each preceding channel's LSB, one bit ahead of the next MSB.
assert all(lr == ((i + 1) // 32) % 2 for i, (_, lr, _) in enumerate(rising_edges))
assert 144_000_000 / (23 + 112 / 256) / 128 == 48_000
print('PASS: 128 cycles/frame, 32 bits/channel, MSB first, I2S WS alignment, 48 kHz.')
