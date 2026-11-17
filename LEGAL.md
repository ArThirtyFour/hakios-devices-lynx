# Legal

This is an independent hobby port. It is **not affiliated with, endorsed by, or connected to Apple Inc. or
Google LLC.** "iOS", "Darwin", "XNU", "Pixel", "Tensor" and other marks belong to their owners and are used
here only to say what the code works with. This file describes what the repository holds; it is not legal
advice. HakiOS's own `LEGAL.md` applies to everything this repository is used with.

## What is in this repository

Source code only: a boot loader, kernel drivers, a patch to Apple's open-source XNU, build scripts and tools.

## What is not, and has to come from you

- **Apple's software.** No file of iOS, no IPSW, nothing extracted from one, and no `identity.h`. HakiOS's
  tools lay out an IPSW you have obtained yourself; whether you may run that software on this hardware is your
  decision and your responsibility.
- **The phone's firmware.** The Bluetooth and Wi-Fi firmware of the WCN6740 (`hpbtfw21.tlv`, `hpnv21.bin`,
  `amss20.bin` and the like) are Qualcomm's, shipped by Google on the phone. The build takes them from a
  directory you fill from your own phone (`fw-lynx/`, ignored by git). They are not redistributed here.
- **Android's boot image and kernel.** The `fastboot boot` image is made inside a copy of your own phone's
  `boot_a`, which holds Google's kernel. Neither the copy nor any image built from it is in this repository,
  and such an image should not be passed on.
- **The phone's device tree** and any log with a device's data (serial numbers, addresses).

## Risk

Running this needs an unlocked bootloader and starts code on the phone that drives its power controller,
clocks and radios directly, with no thermal management. The runs here were from RAM and the phone came back
to Android each time, but nothing guarantees that for your phone. You run it at your own risk.
