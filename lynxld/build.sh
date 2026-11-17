#!/bin/sh
# Build lynxld, the Pixel 7a loader, into lynxld/out/lynxld.bin.
# Needs clang, ld.lld and llvm-objcopy.
#
#   HAKIOS=<the HakiOS tree>   default ../../.. (this repository cloned as <HakiOS>/devices/lynx);
#                              its xnuloader/identity.h (made from your own IPSW) is compiled in
#   FW=<directory>             the phone's own firmware, default ../fw-lynx (see README.md). With hpbtfw21.tlv and
#                              hpnv21.bin in it the loader also starts Bluetooth; without, it is built without it
set -e
cd "$(dirname "$0")"
HAKIOS=${HAKIOS:-../../..}
FW=${FW:-../fw-lynx}
OUT=out
mkdir -p "$OUT"
if [ ! -e "$HAKIOS/xnuloader/identity.h" ]; then
	echo "lynxld: no $HAKIOS/xnuloader/identity.h: set HAKIOS=<the HakiOS tree> and run its xnuloader/mkidentity.sh" >&2
	exit 1
fi
F="--target=aarch64-none-elf -mcmodel=tiny -ffreestanding -fno-builtin -fno-stack-protector -mgeneral-regs-only -mstrict-align -Os -Wall -Wextra -DUSB_GADGET_HS_ONLY -I .. -I $HAKIOS/xnuloader"
ACPM="-include acpm/acpm_lynx_cfg.h"

BT=
if [ -s "$FW/hpbtfw21.tlv" ] && [ -s "$FW/hpnv21.bin" ]; then
	F="$F -DLYNX_BT"
	clang $F -c bt_up.c -o "$OUT/bt_up.o"
	clang --target=aarch64-none-elf -I "$FW" -c bt_fw.S -o "$OUT/bt_fw.o"
	BT="$OUT/bt_up.o $OUT/bt_fw.o"
else
	echo "lynxld: no hpbtfw21.tlv + hpnv21.bin in $FW: built without Bluetooth"
fi

clang --target=aarch64-none-elf -c start.S -o "$OUT/start.o"
clang $F -c log.c -o "$OUT/log.o"
clang $F -Wno-unused-const-variable -c lynxld.c -o "$OUT/lynxld.o"
clang $F $ACPM -c touch_up.c -o "$OUT/touch_up.o"
clang $F $ACPM -c acpm/acpm.c -o "$OUT/acpm.o"
clang $F -c ../gspe-plan.c -o "$OUT/gspe.o"
clang $F -c usb/usbgadget.c -o "$OUT/usbgadget.o"
clang $F -c usb/usbproto.c -o "$OUT/usbproto.o"

ld.lld -T lynxld.ld "$OUT/start.o" "$OUT/log.o" "$OUT/lynxld.o" "$OUT/touch_up.o" "$OUT/acpm.o" "$OUT/gspe.o" \
	"$OUT/usbgadget.o" "$OUT/usbproto.o" $BT -o "$OUT/lynxld.elf"
llvm-objcopy -O binary "$OUT/lynxld.elf" "$OUT/lynxld.bin"
if llvm-nm "$OUT/lynxld.elf" | grep -q " U "; then
	echo "lynxld: undefined symbols" >&2
	exit 1
fi
echo "lynxld: $(wc -c < "$OUT/lynxld.bin") bytes -> lynxld/$OUT/lynxld.bin"
