# Where the registers came from

The probes and the drivers cite vendor sources by short names. None of those files is in this repository; this
is where each one is.

Google's kernel for this phone, `https://android.googlesource.com/kernel/gs`, branch
`android-gs-lynx-5.10-android15`:

| Name in the comments | Path |
| --- | --- |
| `rc.c`, `rc.h`, `common.h` (`ref/dwc-whi/…`) | `drivers/pci/controller/dwc/pcie-exynos-rc.c`, `pcie-exynos-rc.h`, `pcie-exynos-common.h` |
| the gs201 PHY calibration (`gs201_cal.h` is it) | `drivers/pci/controller/dwc/pcie-exynos-gs201-rc-cal.c` |
| `ref/dwc-core/…` | `drivers/pci/controller/dwc/pcie-designware.c`, `pcie-designware-host.c`, `pcie-designware.h` |
| `sfr`, cmucal-sfr.c, cmucal-node.c, cmucal-vclklut.c | `drivers/soc/google/cal-if/gs201/cmucal-sfr.c`, `cmucal-node.c`, `cmucal-vclklut.c` |
| pinctrl-gs201.c | `drivers/pinctrl/samsung/pinctrl-gs201.c` |
| clk-gs201.c | `drivers/clk/samsung/clk-gs201.c` |
| exynos_tty.c, `serial_s3c.h.source` | `drivers/tty/serial/exynos_tty.c`, `include/linux/serial_s3c.h` |
| spi-s3c64xx.c | `drivers/spi/spi-s3c64xx.c` |
| s2mpg12 / s2mpg13 registers and regulators | `include/linux/mfd/samsung/s2mpg12-register.h`, `s2mpg13-register.h`, `drivers/regulator/s2mpg12-regulator.c`, `s2mpg13-regulator.c`, `drivers/mfd/s2mpg12-core.c`, `s2mpg13-core.c` |
| `ref/s2mpu/s2mpu.c`, `kvm_s2mpu.h` | `arch/arm64/kvm/hyp/nvhe/iommu/s2mpu.c`, `arch/arm64/include/asm/kvm_s2mpu.h` |
| `ref/s2mpu/exynos-pcie-iommu.c`, `.h` | `drivers/iommu/exynos-pcie-iommu.c`, `exynos-pcie-iommu.h` |
| `ref/misc/exynos-pd_el3.c`, `exynos-smc.h` | `drivers/soc/google/exynos-pd_el3.c`, `include/linux/soc/samsung/exynos-smc.h` |
| acpm_dvfs.c | `drivers/soc/google/cal-if/acpm_dvfs.c` |

Google's modules, same branch name:

| Name in the comments | Repository and path |
| --- | --- |
| goodix_brl_hw.c, goodix_brl_spi.c | `kernel/google-modules/touch/goodix` |
| touch_bus_negotiator.c | `kernel/google-modules/touch/common` |
| max77759 registers, max1720x_battery.h, max_m5.h | `kernel/google-modules/bms` |
| `ref/cnss2/pci.c`, `reg.h`, `mhi_*.c` | `kernel/google-modules/wlan/qcom/wcn6740/cnss2`: `cnss2/pci.c`, `cnss2/reg.h`, `mhi/core/` |
| `btpower.c.source` | Google's Bluetooth power driver for the chip (a google-modules Bluetooth repository; the dmesg strings `bt_power …` are its) |

Mainline Linux, `https://github.com/torvalds/linux`:

| Name in the comments | Path |
| --- | --- |
| `btqca.c.source`, `btqca.h.source`, `hci_qca.c.source`, `hci_ldisc.c.source` | `drivers/bluetooth/btqca.c`, `btqca.h`, `hci_qca.c`, `hci_ldisc.c` |
| `hci.h.source` | `include/net/bluetooth/hci.h` |
| `ref/mhi/common.h`, `host_boot.c` | `drivers/bus/mhi/common.h`, `drivers/bus/mhi/host/boot.c` (v6.6) |

Line numbers in the comments are those of the files as fetched in November 2026 and will drift.

Addresses, pins and phandles come from the phone's own flattened device tree (`/sys/firmware/fdt` in Android),
which is not in this repository either.
