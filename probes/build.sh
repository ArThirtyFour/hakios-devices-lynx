#!/bin/sh
# Build one hardware probe into probes/out/<name>.bin: a small program put where lynxld goes (tools/mkimg-lynx.py),
# started by the bootloader, that writes what it finds into ramoops and waits for the bootloader's watchdog to
# bring Android back (README.md, "Probes").
#
#   probes/build.sh acpmprobe | tpower | tspi | btprobe | btfw | pcieprobe
#
#   FW=<directory>   the phone's own firmware, default ../fw-lynx: btfw needs hpbtfw21.tlv and hpnv21.bin,
#                    pcieprobe needs amss20.bin
set -e
cd "$(dirname "$0")"
FW=${FW:-../fw-lynx}
L=../lynxld
OUT=out
mkdir -p "$OUT"
F="--target=aarch64-none-elf -mcmodel=tiny -ffreestanding -fno-builtin -fno-stack-protector -mgeneral-regs-only -mstrict-align -Os -Wall -Wextra -I $L -I $L/acpm"
A="--target=aarch64-none-elf"

clang $A -c $L/start.S -o "$OUT/start.o"
clang $F -c $L/log.c -o "$OUT/log.o"
OBJS="$OUT/start.o $OUT/log.o"
acpm() {
	clang $F -include $L/acpm/acpm_lynx_cfg.h -c $L/acpm/acpm.c -o "$OUT/acpm.o"
	OBJS="$OBJS $OUT/acpm.o"
	F="$F -include $L/acpm/acpm_lynx_cfg.h"
}
need() {
	for f in "$@"; do
		[ -s "$FW/$f" ] || { echo "probes: no $f in $FW (pull it from the phone's /vendor/firmware)" >&2; exit 1; }
	done
}

case "$1" in
acpmprobe|tpower|tspi)
	acpm ;;
btprobe)
	;;
btfw)
	need hpbtfw21.tlv hpnv21.bin
	clang $A -I "$FW" -c bt_fw.S -o "$OUT/bt_fw.o"
	OBJS="$OBJS $OUT/bt_fw.o" ;;
pcieprobe)
	need amss20.bin
	acpm
	clang $A -c vec.S -o "$OUT/vec.o"
	clang $A -I "$FW" -c wifi_fw.S -o "$OUT/wifi_fw.o"
	OBJS="$OBJS $OUT/vec.o $OUT/wifi_fw.o" ;;
*)
	echo "usage: probes/build.sh acpmprobe|tpower|tspi|btprobe|btfw|pcieprobe" >&2
	exit 2 ;;
esac

clang $F -c "$1.c" -o "$OUT/$1.o"
ld.lld -T $L/lynxld.ld $OBJS "$OUT/$1.o" -o "$OUT/$1.elf"
llvm-objcopy -O binary "$OUT/$1.elf" "$OUT/$1.bin"
if llvm-nm "$OUT/$1.elf" | grep -q " U "; then
	echo "probes: $1: undefined symbols" >&2
	exit 1
fi
echo "probes: $(wc -c < "$OUT/$1.bin") bytes -> probes/$OUT/$1.bin"
