/* ACPM config for gs201 / lynx. Addresses from the lynx FDT (/acpm_ipc reg, initdata-base; s2mpg12mfd acpm-ipc-channel = 2);
 * command framing as Google's s2mpg12-core.c (channel 0, i2c addr I2C_ADDR_PMIC = 0x01); mailbox register offsets and the
 * descriptor layout as verified by PixOS on gs101 (same ACPM framework), re-checked on lynx by acpm_describe before any write. */
#define SOC_GS101 1                     /* PixOS's guard for "this SoC has the ACPM layout below" */
#define ACPM_MBOX_BASE    0x18210000u
#define ACPM_SRAM_BASE    0x18500000u
#define ACPM_SRAM_SIZE    0x48000u
#define ACPM_INITDATA_OFF 0xA000u
#define ACPM_CH_ID        2u
#define ACPM_SPEEDY       0u
#define ACPM_PMIC_ADDR    1u
#define ACPM_INTGR1       0x40u
#define ACPM_INTCR0       0x24u
#define ACPM_INTSR0       0x2cu
#define ACPM_DESC_STRIDE  0x48u
#define ACPM_LAYOUT_VERIFIED 1u
#define ACPM_ELEM_UNIT    1u
#define ACPM_ELEM_UNIT_VERIFIED 1u
#define ACPM_IPC_PROTOCOL_SEQ_NUM 16u
/* rails: LDO25M (TSP DVDD, 1.6-1.95 V, group 4: 0.7 V + 25 mV) and LDO26M (TS AVDD, 3.0 V, group 5: 1.8 V + 25 mV) */
#define ACPM_RAIL0_REG 0x44u
#define ACPM_RAIL0_VSEL_KNOWN 1u
#define ACPM_RAIL0_VSEL_MIN 36u
#define ACPM_RAIL0_VSEL_MAX 50u
#define ACPM_RAIL1_REG 0x45u
#define ACPM_RAIL1_VSEL_KNOWN 1u
#define ACPM_RAIL1_VSEL_MIN 48u
#define ACPM_RAIL1_VSEL_MAX 48u
/* DVFS (exynos-dm acpm-ipc-channel 1; exynos-acme domain@2 = CPUCL2 cal-id idx 4, X1 cores 6-7): FDT min/resume/max only */
#define ACPM_DVFS_CH 1u
#define ACPM_DVFS_N 3
#define ACPM_DVFS_IDXS 4u, 3u, 2u   /* CPUCL2 (X1, cores 6-7), CPUCL1 (A78, cores 4-5) */
#define ACPM_DVFS_WRITER 1
#define ACPM_DVFS_MAXLV 2
#define ACPM_DVFS_CEILS 2850000u, 2348000u, 1803000u
#define ACPM_DVFS_NRS 2u, 2u, 2u
#define ACPM_DVFS_ROWS { 2850000u, 2048000u }, { 2348000u, 1663000u }, { 1803000u, 1197000u }   /* CPUCL0 (A55): FDT max / resume */
#define TOUCH_DVFS_WRITE 1
#define ACPM_DVFS_MIF_IDX 0u
#define ACPM_DVFS_MIF_KHZ 0u
