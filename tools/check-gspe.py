#!/usr/bin/env python3
"""Validate a HakiOS gs201 GSPE payload without executing it."""
import pathlib
import struct
import sys

def validate(data):
    if len(data) < 48 or data[:8] != b'GSPE01\0\0':
        raise ValueError('invalid GSPE header')
    hdr, count, kvb, entry, size, reserved, stub = struct.unpack_from('<IIQQIIQ', data, 8)
    if hdr != 48 or not 1 <= count <= 32 or reserved != 0:
        raise ValueError('invalid header size, segment count or reserved field')
    start = hdr + count * 32
    if start > len(data) or size != len(data) - start:
        raise ValueError('kernel payload length mismatch')
    segments = []
    for i in range(count):
        vm, vmsize, offset, filesz = struct.unpack_from('<QQQQ', data, hdr + i * 32)
        if filesz > vmsize or offset > size or filesz > size - offset:
            raise ValueError(f'segment {i}: invalid file or virtual bounds')
        if vmsize and (vm < kvb or vm + vmsize > 0xffffffffffffffff):
            raise ValueError(f'segment {i}: invalid virtual address')
        if vmsize:
            segments.append((vm, vm + vmsize))
    if not any(lo <= entry < hi for lo, hi in segments):
        raise ValueError('entry outside segments')
    if stub and not any(lo <= stub and stub + 32 <= hi for lo, hi in segments):
        raise ValueError('return slot outside segments')
    return count, size, kvb, entry, stub

def main():
    try:
        count, size, kvb, entry, stub = validate(pathlib.Path(sys.argv[1]).read_bytes())
    except (IndexError, OSError, ValueError, struct.error) as e:
        sys.exit(f'check-gspe: {e}')
    print(f'PASS: {count} segments, {size} bytes, base={kvb:#x}, entry={entry:#x}, return={stub:#x}')

if __name__ == '__main__':
    main()
