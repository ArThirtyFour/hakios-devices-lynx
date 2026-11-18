# hakios-devices-lynx

HakiOS for the **Google Pixel 7a (lynx)**: Google Tensor G2 (**gs201**), Mali-G710, UFS. The user space of
iOS 16.5 (launchd → backboardd → SpringBoard) on Apple's open XNU kernel (xnu-8796.121.2, Darwin 22.5), with
this phone's own loader and drivers.

This is a [HakiOS](https://github.com/oosexgang/hakios-xnu-kernel) device repository: clone it as
`devices/lynx` of the HakiOS tree. Beside the device layer it carries what the common repository has no
equivalent of for this phone: its loader (`lynxld/`), the tools that pack and feed it, and the hardware
probes the drivers were worked out with.

**Nothing on the phone is written.** Every run is `fastboot boot` into RAM: no partition is flashed, no slot is
changed, and the next reboot is Android again. The system image is not on the phone either: the loader takes
it from the computer over USB at every boot.

## What works

| | | |
| --- | --- | --- |
| boot | works | `lynxld/`: entered by the bootloader as a Linux Image (EL2, MMU off), USB gadget for the system image, Apple device tree and boot arguments, kernel started on a Cortex-X1 |
| display | works | 1080×2400, the bootloader's own scanout buffer kept running (DECON0); everything is drawn in software |
| touch | works | Goodix GT9895 on SPI (PERIC1 USI0): power through ACPM, pins and clocks in `lynxld/touch_up.c`, the protocol in `drivers/input/hakios.lynx_touch.c` (`lynx_touch=1`) |
| buttons | works | power and volume down, plain GPIOs polled with the touch screen. Volume up is in the driver the same way but **untested**: that button is physically broken on the phone this was done with |
| CPU | partial | 7 of 8 cores: 2 X1 at 2048 MHz, 2 A78 at 1663 MHz, 3 A55 at 1197 MHz, set once through ACPM DVFS. The eighth core is the loader's. **No thermal control** |
| battery | works | MAX77759 fuel gauge and charger read over I2C, bit-banged on the bus's pads (`drivers/power/hakios.lynx_battery.c`, `lynx_battery=1`): charge, voltage, current, temperature, charger state |
| GPU / Metal | partial | the Mali-G710 driver and its IOKit service are in `drivers/gpu/` (kept out of this repository, see there): the GPU comes up step by step (`mali=N`), its MCU runs Arm's CSF firmware (the phone's own `mali_csffw-r54p3.bin`) and answers a ping; the command stream, the shaders and `hakios-metal`'s back end were moved from the Pixel 8a's GPU architecture (G715, arch 11) to this one (G710, arch 10): scordboard, wait masks, GPU_ID. Not yet a working Metal device in iOS: the precompiled blit shaders and a few drawing registers still have to be brought over. |
| Bluetooth | partial | the loader loads the chip's firmware (WCN6740, UART), the kernel proves it (HCI 12.0000, its own address, LE scan finds devices) and gives it to iOS: the kernel control, the Skywalk pipe and the nodes `/dev/btwake|btreset|btpoweroff` (`drivers/bluetooth/hakios.lynx_bt.c`, the chip-independent half in the main tree). iOS's `bluetoothd` attaches and its logs are answered by an inserted library; it still will not power the controller (the toggle springs back), which is what is being chased. |
| Wi-Fi | missing | probes only (`probes/pcieprobe.c`): the PCIe link comes up, the chip (17cb:1103) and its MHI registers are read; the firmware download through BHI does not complete, reason unknown |
| storage | missing | none of the phone's: the system image is in RAM |
| audio, cellular, camera, sensors | missing | audio started: `drivers/aoc/` (gs201's AoC: `aoc_req` 0x181d0000, S2MPU 0x1a0b0000, SysMMU 0x1a090000, GSA 0x17c90000) and `drivers/audio/` (the two CS35L41 on PERIC0 USI7). The AoC needs its firmware (`/vendor/firmware/aoc.bin`, not pulled yet) and a Trusty client to cold-start it; the amplifiers need their SPI clock confirmed. Cellular, camera and sensors: not started. |

What was seen on the phone and what was only built is said in each row; `device.conf` has the same in the
form HakiOS reads.

### Things to know before running it

- **Heat.** Nothing in this port watches the temperature. With all clusters at their top rates the SoC's own
  hardware trip reset the phone 40 seconds into the boot; the rates above are a guess at "safe", tried for
  4.5 minutes and no longer.
- **It runs until you stop it.** The loader gives the kernel no time limit (`RETURN_AFTER "0"` in
  `lynxld/lynxld.c`). Hold the power key for 10–15 s to reset the phone into Android.
- **Android will say "watchdog" or "longkey" as the boot reason** after a run: the bootloader's watchdogs and the
  long key are how these runs end.

## What is here

- `device.conf`, `configs/`, `soc/` — the device layer HakiOS reads (`make lynx_defconfig`).
- `drivers/` — this phone's kernel drivers: touch and buttons, battery, Bluetooth step one.
- `xnu/start-el2-secondary.patch` — a change to XNU's `start.s`: on this SoC a core started with PSCI CPU_ON
  arrives in EL2, and is brought down to EL1 before the kernel's own reset code. Without it only one core runs.
- `gspe-plan.c`, `gspe-plan.h` — the reader of the kernel container the loader is given (see "Build").
- `lynxld/` — the loader. `acpm/` and `usb/` in it are the ACPM client and the DWC3 gadget of
  [PixOS](https://github.com/PixOS-Project/pixelios-kernel) (MIT, their licence file beside them), with this
  SoC's addresses.
- `probes/` — small programs put in the loader's place, one question each (ACPM, the touch rails, the touch
  SPI, the Bluetooth UART, the Bluetooth firmware, PCIe and the Wi-Fi chip). `probes/SOURCES.md` says which
  vendor sources each register came from.
- `tools/` — `mkimg-lynx.py` (the `fastboot boot` image), `check-gspe.py`, `lynxrun.c` (the computer's side of
  the USB transfer).

## Not here, by design

No file of iOS, no firmware of the phone, no boot image or kernel of Android, no device tree pulled from a
phone, no `identity.h` (made from your own IPSW), no log with a device's data. `LEGAL.md` says it in full.
Build outputs are ignored by `.gitignore`.

## Build

You need an **unlocked bootloader**, the HakiOS tree with its toolchain (`clang`, `ld.lld`, `llvm-objcopy`,
`lz4`, `libusb-1.0`), and an IPSW of your own (iPhone10,3/10,6, iOS 16.5, 20F66 is what this was done with).

1. HakiOS, this repository in it, and the Darwin user space from your IPSW (HakiOS's own documentation has the
   details of the last two commands):
   ```
   git clone https://github.com/oosexgang/hakios-xnu-kernel hakios && cd hakios
   git clone https://github.com/ArThirtyFour/hakios-devices-lynx devices/lynx
   make unpack IPSW=/path/to/your.ipsw
   PREBUILT=1 REST=1 userland/build.sh          # userland/system.apfs and userland/root.apfs
   xnuloader/mkidentity.sh                      # xnuloader/identity.h
   ```
2. The kernel. `make kernel` stops with an error at its last step (`check_all_exports`); the kernel is built
   by then:
   ```
   make lynx_defconfig
   xnu/apply.sh
   patch -p1 -d xnu-src/xnu-8796.121.2 < devices/lynx/xnu/start-el2-secondary.patch
   make kernel JOBS=8
   K=xnu-src/xnu-8796.121.2/BUILD/obj/DEVELOPMENT_ARM64_BCM2837/mach.development.bcm2837
   ```
3. The kernel as the loader takes it (GSPE), with `mkpe-manifest.py` of the Pixel 7 work this port's loader
   format comes from ([hakios-gs201](https://github.com/jiracipika/hakios-gs201), `gsloader/`):
   ```
   python3 mkpe-manifest.py < $K > kernel.gspe
   python3 devices/lynx/tools/check-gspe.py kernel.gspe
   ```
4. From **your own phone**, in Android with root: its `boot_a` (the container the image is made in; it is only
   read) and, for Bluetooth, two firmware files:
   ```
   adb shell su -c 'dd if=/dev/block/by-name/boot_a' > boot_a.img
   python3 - <<'EOF'
   import struct, subprocess
   d = open('boot_a.img', 'rb').read()
   open('kernel.lz4', 'wb').write(d[4096:4096 + struct.unpack_from('<I', d, 8)[0]])
   subprocess.run(['lz4', '-d', '-f', 'kernel.lz4', 'Image'], check=True)
   EOF
   mkdir -p devices/lynx/fw-lynx
   adb pull /vendor/firmware/hpbtfw21.tlv devices/lynx/fw-lynx/
   adb pull /vendor/firmware/hpnv21.bin devices/lynx/fw-lynx/
   ```
5. The loader and the image:
   ```
   devices/lynx/lynxld/build.sh
   python3 devices/lynx/tools/mkimg-lynx.py boot_a.img Image devices/lynx/lynxld/out/lynxld.bin lynx.img \
       --gspe kernel.gspe --rd userland/root.apfs \
       --cmd "hakios_kver=2.3.4-hakios-darwin22.5-gs201-lynx epoch=$(date +%s)"
   cc -O2 -o lynxrun devices/lynx/tools/lynxrun.c -lusb-1.0
   ```

## Run

```
adb reboot bootloader
fastboot boot lynx.img
./lynxrun userland/system.apfs out/        # sends 2.4 GB over USB, about a minute
```

`lynxrun` needs access to the USB device `18d1:d00d` (a udev rule, or root). The home screen is there one and a
half to two minutes after `JUMP`; with Bluetooth in the loader about 14 seconds more.

After the run, back in Android, what the loader and the drivers said is in
`/sys/fs/pstore/console-ramoops-0` (root): the loader's stages, then lines beginning `lynx_touch:`,
`lynx_battery:`, `lynx_bt:`.

## Probes

A probe is built with `probes/build.sh <name>`, packed with `tools/mkimg-lynx.py` in the loader's place (with
any `--gspe`), and started with `fastboot boot`. It writes into ramoops, and the bootloader's watchdog brings
Android back about 100 seconds later. Each one switches off what it switched on.

| Probe | What it asks |
| --- | --- |
| `acpmprobe` | is the ACPM mailbox where the device tree says, and can the PMIC be read through it |
| `tpower` | do the touch rails (LDO25M, LDO26M) come up, and does the chip then raise its interrupt line |
| `tspi` | does the Goodix chip answer on SPI |
| `btprobe` | does the Bluetooth half answer on its UART |
| `btfw` | does it take its firmware and run it |
| `pcieprobe` | does PCIe RC 1 link up, is the Wi-Fi chip there, does it take the first part of its firmware (it does not) |
| `gpuprobe` | is the Mali-G710 powered and does it obey: identity, a soft reset, the L2 cache on and off |
| `gpumcu` | does the GPU's own processor boot Arm's CSF firmware (needs `mali_csffw-r54p3.bin` in `fw-lynx/`) and answer a ping |

## Licences

- `drivers/` and `xnu/` are a derivative work of Apple's XNU and of HakiOS's drivers: Apple Public Source
  License 2.0, as HakiOS's `LICENSE.APSL`.
- `lynxld/acpm/` and `lynxld/usb/`: MIT, PixOS-Project contributors (`LICENSE.PixOS-MIT` in each).
- `probes/gs201_cal.h` is Google's and Samsung's PCIe PHY calibration, statement for statement: GPL-2.0-only,
  and so is `probes/pcieprobe.c`, which includes it.
- Everything else here: MIT, as HakiOS's own device layer is.

`LICENSE` has the same with the texts.
