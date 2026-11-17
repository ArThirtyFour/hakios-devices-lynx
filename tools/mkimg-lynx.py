#!/usr/bin/env python3
"""Build a lynx fastboot-boot image: loader at Image+0x10000, code1 -> loader,
optional LYNXPAY1(GSPE) at Image+0x200000. Keeps the stock 64 MiB AVB container tail.
Host-only; never touches a device."""
import argparse, pathlib, struct, subprocess, sys, tempfile, zlib
LOADER_OFF, PAYLOAD_OFF, TEXT_END = 0x10000, 0x200000, 0x21b0000
a = argparse.ArgumentParser(); a.add_argument('stock_boot'); a.add_argument('raw_kernel'); a.add_argument('loader'); a.add_argument('out'); a.add_argument('--gspe'); a.add_argument('--rd'); a.add_argument('--cmd', default='')
a = a.parse_args()
stock = pathlib.Path(a.stock_boot).read_bytes(); raw = pathlib.Path(a.raw_kernel).read_bytes(); ld = pathlib.Path(a.loader).read_bytes()
if raw[:2] != b'MZ' or raw[56:60] != b'ARMd': sys.exit('not an arm64 Image')
if LOADER_OFF + len(ld) > PAYLOAD_OFF: sys.exit('loader too big')
k = bytearray(raw); k[LOADER_OFF:LOADER_OFF + len(ld)] = ld
def al16(b): return b + bytes((-len(b)) % 16)
if a.gspe and a.rd:
    g = pathlib.Path(a.gspe).read_bytes(); r = pathlib.Path(a.rd).read_bytes(); c = a.cmd.encode() + b'\0'
    if len(c) > 400: sys.exit('cmdline too long')
    blob = b'LYNXPAY2' + struct.pack('<IIIIII', len(g), zlib.crc32(g), len(r), zlib.crc32(r), len(c), 0) + al16(g) + al16(r) + c
elif a.gspe:
    g = pathlib.Path(a.gspe).read_bytes()
    blob = b'LYNXPAY1' + struct.pack('<II', len(g), zlib.crc32(g)) + g
if a.gspe:
    if PAYLOAD_OFF + len(blob) > TEXT_END: sys.exit('payload past .text')
    k[PAYLOAD_OFF:PAYLOAD_OFF + len(blob)] = blob
ins = struct.unpack_from('<I', k, 4)[0]
if ins >> 26 != 0b000101: sys.exit('code1 is not a branch')
struct.pack_into('<I', k, 4, (0b000101 << 26) | ((LOADER_OFF - 4) // 4))
with tempfile.TemporaryDirectory() as t:
    rp, lz, ck = (pathlib.Path(t, x) for x in ('k', 'k.lz4', 'ck'))
    rp.write_bytes(k)
    subprocess.run(['lz4', '-l', '-12', '-q', '-f', str(rp), str(lz)], check=True)
    subprocess.run(['lz4', '-d', '-q', '-f', str(lz), str(ck)], check=True)
    if ck.read_bytes() != bytes(k): sys.exit('lz4 roundtrip mismatch')
    c = lz.read_bytes()
vb = (4096 + struct.unpack_from('<I', stock, 8)[0] + 4095) // 4096 * 4096
if stock[vb:vb + 4] != b'AVB0' or stock[-64:-60] != b'AVBf': sys.exit('stock AVB layout not found')
if 4096 + len(c) > vb: sys.exit(f'kernel overruns vbmeta by {4096 + len(c) - vb} bytes')
img = bytearray(stock); struct.pack_into('<I', img, 8, len(c)); img[4096:vb] = bytes(vb - 4096); img[4096:4096 + len(c)] = c
assert img[vb:] == stock[vb:]
with open(a.out, 'xb') as f: f.write(img)
print(f'OK {a.out}: loader {len(ld)}B @0x10000, payload {"none" if not a.gspe else hex(len(g))} @0x200000, lz4 {len(c)}B, free {vb-4096-len(c)}B')
