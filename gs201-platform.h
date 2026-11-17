/* Live Android FDT observations for lynx; NOT verified at bootloader handoff.
 * Do not use this header for hardware access without a separate boot-time audit.
 * Source: /sys/firmware/fdt, captured locally on Pixel 7a.
 */
#ifndef HAKIOS_LYNX_GS201_PLATFORM_H
#define HAKIOS_LYNX_GS201_PLATFORM_H

#define LYNX_GICD_BASE 0x10400000ULL
#define LYNX_GICD_SIZE 0x00010000ULL
#define LYNX_GICR_BASE 0x10440000ULL
#define LYNX_GICR_SIZE 0x00100000ULL
#define LYNX_PSCI_USES_SMC 1

#endif
