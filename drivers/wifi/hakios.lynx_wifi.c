/*
 * The Wi-Fi of the Pixel 7a (lynx): the Qualcomm WCN6740 on PCIe root-complex channel 1 (pcie@14520000,
 * 17cb:1103, link 5.0 GT/s x1), brought up step by step, exactly as the loader probe that found it
 * (probes/pcieprobe.c) does, but in the kernel and with the one piece the probe left out: the chip's DMA
 * to our memory is made to pass the two gates in front of it -- the S2MPU of HSI2 (0x145e0000) and the
 * PCIe System MMU (0x145c0000) -- before the firmware is offered. Nothing here is touched unless a
 * boot-arg asks for it, and a step that does not find what it needs ends the sequence instead of going on
 * blind; what was switched on is switched back off.
 *
 *   wifi=N    perform steps 1 to N and stop (0: nothing). Each step says what it is about to touch before
 *             it touches it, and what it found after, through lynx_tlog (the ramoops console).
 *      1      the rails, the clocks, the PHY, the pins: the link trained to L0, the chip's configuration
 *             space read, its bar0 assigned and the outbound ATU viewport programmed
 *      2      the MHI registers read and said (MHIREGLEN .. MHISTATUS, the BHI and BHIE offsets)
 *      3      the S2MPU opened (all gigabytes, read-write) and the PCIe SysMMU given a 1:1 first-level
 *             table, the WLAN domain released (WLAON_QFPROM_PWR_CTRL's shutdown / low-rail bits cleared,
 *             which is what the v7 probe showed was missing), then the SBL (the first 512 KiB of
 *             fw/amss20.bin) handed to the chip over BHI and BHI_EXECENV awaited (0 PBL -> 1 SBL)
 *      4      the MHI context arrays built and the MMIO registers programmed, then the RESET -> READY
 *             transition awaited (MHISTATUS bit 0)
 *
 * More boot-args: wifi_delay=SECONDS (10) before step 1 (the small image has to be there for the
 * firmware); wifi_dl_s=SECONDS (4) how long step 3 awaits the SBL; wifi_iatu=N read (1) or write (2) the
 * inbound iATU region 0 for diagnosis; wifi_keep=1 the chip and the link stay up while the system runs
 * (they are stopped when the run ends in order).
 *
 * What the hardware is, and where each value comes from: Google's kernel for this phone (SOURCES.md in
 * probes/, read for how the hardware behaves -- that code is GPL and none of it is here; the register
 * offsets, bit values and order are facts and are written again in this file's own words). The one source
 * that is ours is probes/pcieprobe.c: every sequence below was already run on the phone there (PMIC rails
 * through ACPM, the PHY-isolation SMC, the link training, the ATU, the MHI reads, the BHI doorbell), so
 * this driver is that probe moved into the kernel, plus the S2MPU / SysMMU stage that Android's pKVM and
 * PCIe-iommu drivers do for every buffer before the chip may touch it.
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t va);
extern void clean_dcache(vm_offset_t addr, unsigned count, int phys);
extern void flush_dcache(vm_offset_t addr, unsigned count, int phys);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);
/* Every lynx driver logs through the touch driver's lynx_tlog (printf and the ramoops console). */
extern void lynx_tlog(const char *fmt, ...) __printflike(1, 2);

/* ---- the PCIe root complex channel 1, as the FDT /pcie@14520000 lays it out ---- */
#define ELBI            0x14520000ull
#define PHY             0x14550000ull
#define DBI             0x14800000ull
#define PCS             0x14540000ull
#define CFG             0x60ffe000ull   /* the "config" window (cfg0_base / cfg0_size) */
#define CFG_SIZE        0x2000u
#define CMU             0x14400000ull   /* CMU_HSI2 */
#define CMUTOP          0x1e080000ull   /* CMU_TOP */
#define PMU             0x18060000ull   /* /system-controller@18060000 */
#define GPH2            0x14440000ull   /* PERST 0, CLKREQ 1, WLAN_EN 4, buck 5 */
#define MAP_SIZE        0x10000u

/* ---- the two DMA gates in front of the chip's reads of our memory ---- */
#define S2MPU           0x145e0000ull
#define SYSMMU          0x145c0000ull

/* the chip's bar0, where Android puts it, and the outbound window the probe programs for it */
#define BAR0            0x60200000ull
#define BAR0_SIZE       0x200000u       /* 2 MiB */

/* the chip's remap window (cnss2's cnss_pci_select_window): on-chip registers at 0x80000 and above are
 * reached by writing the window number (offset >> 19) | enable into bar+0x310C, then reading
 * bar+0x80000 + (offset & 0x7ffff). The probe's winrd() does the same; this adds the write side. */
#define REMAP_BAR_CTRL  0x310C
#define WINDOW_ENABLE    0x40000000u
#define WINDOW_START     0x80000u
#define WINDOW_SHIFT     19u
#define WINDOW_MASK      0x7ffffu

/* WLAON_QFPROM_PWR_CTRL: the chip's own WLAN power control. Before the firmware, the two "keep the WLAN
 * domain down" bits must be cleared -- cnss2's cnss_pci_set_wlaon_pwr_ctrl(false, false, false) right
 * before cnss_pci_start_mhi (ref/cnss2/pci.c:2566). VDD4BLOW holds the WLAN/BT rail low, SHUTDOWN holds
 * the WLAN core in shutdown; with them set the PBL takes the BHI doorbell (it latches IMGADDR/IMGSIZE
 * into ERRDGB, which is what v7 saw) but never issues a DMA read of the image. */
#define WLAON_QFPROM_PWR_CTRL_REG           0x1F8031C
#define QFPROM_PWR_CTRL_VDD4BLOW_SW_EN_MASK 0x4
#define QFPROM_PWR_CTRL_SHUTDOWN_EN_MASK    0x1

/* the DesignWare inbound iATU, region 0, in the same unrolled space the probe's ATU0 (0x300000) uses for
 * the outbound viewports: direction = bit 8 of the region base (pcie-designware.h PCIE_ATU_UNROLL_BASE). */
#define ATU_INBOUND0    0x300100u       /* DBI + this: inbound region 0 (CTRL1 +0, CTRL2 +4, ... TARGET +0x14) */

/* ---- rc.h offsets (the probe used them all) ---- */
#define PCIE_APP_LTSSM_ENABLE           0x054
#define PCIE_ELBI_RDLH_LINKUP           0x2C8
#define PCIE_PM_DSTATE                  0x2E8
#define PCIE_LINKDOWN_RST_CTRL_SEL      0x3A0
#define PCIE_SOFT_RESET                 0x3A4
#define SOFT_PWR_RESET                  (0x1 << 1)
#define SOFT_NON_STICKY_RESET           (0x1 << 3)
#define PCIE_QCH_SEL                    0x3A8
#define PCIE_APP_REQ_EXIT_L1_MODE       0x3BC
#define PCIE_MSTR_PEND_SEL_NAK          0x474
#define PCIE_DBI_L1_EXIT_DISABLE        0x1078
#define PCIE_MISC_CONTROL               0x8BC
#define PCIE_AUX_CLK_FREQ_OFF           0xB40
#define PCIE_L1_SUBSTATES_OFF           0xB44
#define PCIE_CAP_OFFSET                 0x70
#define PM_CAP_OFFSET                   0x40
#define ATU0                            0x300000        /* viewport 0 at +0, viewport 1 at +0x200 */
#define S_L0                            0x11            /* the link state we wait for */

/* ---- MHI and BHI registers at bar0 (drivers/bus/mhi/common.h, kept as ref/mhi/common.h) ---- */
#define MHIREGLEN       0x00
#define MHIVER          0x08
#define MHICFG          0x10
#define CHDBOFF         0x18
#define ERDBOFF         0x20
#define BHIOFF          0x28
#define BHIEOFF         0x2c
#define MHICTRL         0x38
#define MHISTATUS       0x48
#define CCABAP_LOWER    0x58
#define CCABAP_HIGHER   0x5c
#define ECABAP_LOWER    0x60
#define ECABAP_HIGHER   0x64
#define CRCBAP_LOWER    0x68
#define CRCBAP_HIGHER   0x6c
#define CRDB_LOWER      0x70
#define MHICTRLBASE_LOWER 0x80
#define MHICTRLBASE_HIGHER 0x84
#define MHICTRLLIMIT_LOWER 0x88
#define MHICTRLLIMIT_HIGHER 0x8c
#define MHIDATABASE_LOWER 0x98
#define MHIDATABASE_HIGHER 0x9c
#define MHIDATALIMIT_LOWER 0xa0
#define MHIDATALIMIT_HIGHER 0xa4

#define BHI_BHIVERSION_MINOR    0x00
#define BHI_IMGADDR_LOW         0x08
#define BHI_IMGADDR_HIGH        0x0c
#define BHI_IMGSIZE             0x10
#define BHI_IMGTXDB             0x18
#define BHI_INTVEC              0x20
#define BHI_EXECENV             0x28
#define BHI_STATUS              0x2c
#define BHI_ERRCODE             0x30
#define BHI_ERRDBG1             0x34
#define BHI_ERRDBG2             0x38
#define BHI_ERRDBG3             0x3c
#define BHI_SERIALNU            0x40

#define BHI_STATUS_MASK         0xc0000000u     /* bits 31:30: 0 reset, 2 taken, 3 error */
#define BHI_STATUS_SUCCESS      0x2u
#define BHI_STATUS_ERROR        0x3u
#define MHICTRL_RESET           (1u << 1)
#define MHICTRL_MHISTATE_MASK   0x0000ff00u
#define MHISTATUS_READY         0x1u

/* ---- the S2MPU (arch/arm64/kvm/s2mpu.c + kvm_s2mpu.h: same offsets, v8 = 0x11000000) ---- */
#define S2MPU_CTRL0             0x0
#define S2MPU_CTRL1             0x4
#define S2MPU_CFG               0x10
#define S2MPU_VERSION           0x60
#define S2MPU_STATUS            0x68
#define S2MPU_NUM_CONTEXT       0x100
#define S2MPU_CONTEXT_CFG       0x104
#define S2MPU_ALL_INVALIDATION  0x1000
#define S2MPU_L1ENTRY_ATTR(vid, gb)     (0x4004 + (vid) * 0x200 + (gb) * 8)
#define S2MPU_FAULT_STATUS      0x2000
#define S2MPU_CTRL0_ENABLE      (1u << 0)
#define S2MPU_CTRL0_FAULT_SLVERR (1u << 2)     /* v8's fault response (v9 uses the same bit for DECERR) */
#define S2MPU_STATUS_BUSY       (1u << 0)
#define S2MPU_STATUS_INVALIDATING (1u << 1)
#define S2MPU_VER_MASK          0xffff0000u
#define S2MPU_VER_8             0x11000000u
#define S2MPU_VER_9             0x20000000u
#define S2MPU_L1ENTRY_1G_RW     (3u << 1)      /* L1ENTRY_ATTR_1G(MPT_PROT_RW): bits 2:1 = 3 */

/* ---- the PCIe System MMU (exynos-pcie-iommu.h: SysMMU v8, VID 0) ---- */
#define SYSMMU_CTRL             0x000
#define SYSMMU_CFG              0x004
#define SYSMMU_CTRL_ENABLE      (0x5)          /* the driver's CTRL_ENABLE (enable, the mode the kernel uses) */
#define SYSMMU_CTRL_BLOCK       (0x7)
#define SYSMMU_CTRL_DISABLE     0x0
#define SYSMMU_VM_CTRL          0x8000          /* VID 0 (REG_MMU_CTRL_VID(0)) */
#define SYSMMU_VM_FLPT_BASE     0x800c          /* the first-level page-table base (PPN) */
#define SYSMMU_VM_ALL_INV       0x8010          /* invalidate all, VID 0 */
#define SYSMMU_VID_ENABLE       (1u << 0)
#define SYSMMU_VID_STALL        (1u << 3)
#define SYSMMU_SECT             2               /* a first-level 1 MiB section descriptor */
#define SYSMMU_SHAREABLE        (1u << 6)
#define SYSMMU_LV1_ENTRIES      65536           /* 64 GiB of 1 MiB sections */
#define SYSMMU_LV1_SIZE         (SYSMMU_LV1_ENTRIES * 4)

/* the first part of the firmware handed to the chip's first loader (sbl_size = SZ_512K in cnss2) */
#define FW_NAME                 "amss20.bin"
#define SBL_SIZE                (0x80000u)

/* CMU gates and QCHs of the block, as tspi.c / the probe switch them: bit 20 MANUAL, bit 21 CG_VAL */
static const uint16_t gates[] = {
	0x2008,         /* PCIE_003 PHY_REFCLK_IN */
	0x2050, 0x2054, 0x2058, 0x205c,     /* G4X2 DBI / MSTR / SLV ACLK, SUB_CTRL APB */
	0x2070, 0x2074, 0x2078,     /* PCS_PMA UDBG / PIPE_PAL / PHY APB */
	0x202c,         /* AS_APB_PCIEPHY */
	0x2094, 0x2098,         /* QE_PCIE_GEN4A */
	0x20c8, 0x20cc, 0x20d0, 0x20d4,     /* UASC_PCIE_GEN4A DBI / SLV */
};
static const uint16_t qchs[] = { 0x3000, 0x3040, 0x3048, 0x3050, 0x3058, 0x305c, 0x3060, 0x3074, 0x309c, 0x30a0 };

/* the DMA gates' own clocks (the probe gates these before touching S2MPU/SysMMU) */
static const uint16_t dma_gates[] = { 0x20c0, 0x2028, 0x20b8, 0x20bc };
static const uint16_t dma_qchs[] = { 0x3090, 0x3094, 0x3084 };

static uint32_t steps, step;
static vm_offset_t elbi, phy, dbi, pcs, cmu, cmutop, pmu, gph2, s2mpu, sysmmu;
static bool link_up;            /* set by step 1; steps 2..4 touch the chip's bar only if it is */
/* what step 1 found and changed, to be put back by teardown (the probe restores all of it) */
static uint32_t saved_con, saved_dat, saved_pud, saved_opt;
static bool saved_bypassed;

static inline uint32_t rd(vm_offset_t a) { return *(volatile uint32_t *)a; }
static inline void wr(vm_offset_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile ("dsb sy" ::: "memory"); }
static inline void wr16(vm_offset_t a, uint16_t v) { *(volatile uint16_t *)a = v; __asm__ volatile ("dsb sy" ::: "memory"); }
static inline void rmw(vm_offset_t a, uint32_t clr, uint32_t set) { wr(a, (rd(a) & ~clr) | set); }
static inline uint32_t r32(vm_offset_t b, uint32_t o) { return *(volatile uint32_t *)(b + o); }
static inline void w32(vm_offset_t b, uint32_t o, uint32_t v) { *(volatile uint32_t *)(b + o) = v; __asm__ volatile ("dsb sy" ::: "memory"); }

/* the chip's remap window, read and write (cnss_pci_reg_read/write in cnss2, via bar + 0x310C) */
static uint32_t
winrd(vm_offset_t bar, uint32_t offset)
{
	wr(bar + REMAP_BAR_CTRL, WINDOW_ENABLE | ((offset >> WINDOW_SHIFT) & 0x3fu));
	(void)rd(bar + REMAP_BAR_CTRL);         /* make sure the window switch took effect */
	return rd(bar + WINDOW_START + (offset & WINDOW_MASK));
}

static void
winwr(vm_offset_t bar, uint32_t offset, uint32_t val)
{
	wr(bar + REMAP_BAR_CTRL, WINDOW_ENABLE | ((offset >> WINDOW_SHIFT) & 0x3fu));
	(void)rd(bar + REMAP_BAR_CTRL);
	wr(bar + WINDOW_START + (offset & WINDOW_MASK), val);
}

static uint64_t
now_ms(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000000;
}

/* waits up to max_ms for (reg & mask) == want; true if it came */
static bool
wait_for(vm_offset_t a, uint32_t mask, uint32_t want, uint32_t max_ms)
{
	uint64_t until = now_ms() + max_ms;

	do {
		if ((rd(a) & mask) == want) {
			return true;
		}
		delay(1000);
	} while (now_ms() < until);
	return (rd(a) & mask) == want;
}

static bool
announce(unsigned int n, const char *what)
{
	if (n > steps) {
		return false;
	}
	step = n;
	lynx_tlog("lynx_wifi: %u: %s\n", n, what);
	return true;
}

/*
 * The PHY-isolation bypass: rc.c's rmw_priv_reg() is an SMC into EL3 (SMC_CMD_PRIV_REG 0x82000504,
 * exynos-pd_el3.c / exynos-smc.h): x1 the register, x2 PRIV_REG_OPTION_RMW (2), x3 the mask, x4 the
 * value. The probe calls it at EL2; the kernel calls it at EL1 and EL3 answers the same way. Returns the
 * SMC's x0 (0 or an EXYNOS_ERROR_*), and the register is re-read by the caller.
 */
static int
smc_priv_reg(uint64_t reg, uint32_t mask, uint32_t val)
{
	register uint64_t x0 __asm__("x0") = 0x82000504ull;
	register uint64_t x1 __asm__("x1") = reg;
	register uint64_t x2 __asm__("x2") = 2;        /* PRIV_REG_OPTION_RMW */
	register uint64_t x3 __asm__("x3") = mask;
	register uint64_t x4 __asm__("x4") = val;
	register uint64_t x5 __asm__("x5") = 0;
	register uint64_t x6 __asm__("x6") = 0;
	register uint64_t x7 __asm__("x7") = 0;

	__asm__ volatile ("smc #0"
	    : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3), "+r" (x4), "+r" (x5), "+r" (x6), "+r" (x7)
	    :: "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return (int)x0;
}

/* ---- a minimal ACPM client: read / update one S2MPG13 register (the probe's acpm.c, MIT, condensed) ---- */

#define ACPM_MBOX       0x18210000u
#define ACPM_SRAM       0x18500000u
#define ACPM_SRAM_SIZE  0x48000u
#define ACPM_INITDATA   0xa000u
#define ACPM_CH_ID      2u
#define ACPM_PMIC_ADDR  1u
#define ACPM_SPEEDY     1u              /* the sub-PMIC (S2MPG13) is on speedy channel 1 */
#define ACPM_INTGR1     0x40u
#define ACPM_INTCR0     0x24u
#define ACPM_INTSR0     0x2cu
#define ACPM_STRIDE     0x48u
#define INIT_CH_TAB     0x08u
#define INIT_AP_MAX     0x18u
#define D_ID            0u
#define D_TYPE          3u
#define D_RX_REAR       4u
#define D_RX_FRONT      5u
#define D_RX_BASE       6u
#define D_TX_REAR       10u
#define D_TX_FRONT      11u
#define D_TX_BASE       12u
#define D_Q_LEN         13u
#define D_Q_ELEM        14u
#define FUNC_READ       0u
#define FUNC_UPDATE     2u
#define SEQ_SHIFT       16u
#define RET_SHIFT       24u

static vm_offset_t acpm_mbox, acpm_sram;

/* the channel's descriptor and queue pointers, read and validated once per transaction */
struct acpm_chan {
	uint32_t rr, rf, sr, sf, qlen, esz_b;
	vm_offset_t req_rear, req_front, req_base, rsp_rear, rsp_front, rsp_base;
};

static int
acpm_describe(struct acpm_chan *h)
{
	uintptr_t init = acpm_sram + ACPM_INITDATA;
	uint32_t n, tab, id = 0, off, bytes, idx;
	vm_offset_t desc;

	h->req_rear = h->req_front = h->req_base = h->rsp_rear = h->rsp_front = h->rsp_base = 0;
	h->rr = h->rf = h->sr = h->sf = h->qlen = h->esz_b = 0;
	n = r32(init, INIT_AP_MAX);
	tab = r32(init, INIT_CH_TAB);
	if (n < 1 || n > 32 || (tab & 3u) || (uint64_t)tab + (uint64_t)n * ACPM_STRIDE > ACPM_SRAM_SIZE) {
		return -1;
	}
	for (idx = 0; idx < n; idx++) {
		id = r32(acpm_sram + tab, idx * ACPM_STRIDE + 4 * D_ID);
		if (id == ACPM_CH_ID) {
			break;
		}
	}
	if (idx == n) {
		return -2;
	}
	desc = acpm_sram + tab + idx * ACPM_STRIDE;
	if (r32(desc, 4 * D_TYPE) != 1 || idx != ACPM_CH_ID) {
		return -3;
	}
	h->qlen = r32(desc, 4 * D_Q_LEN);
	h->esz_b = r32(desc, 4 * D_Q_ELEM);      /* q_elem_size, bytes (elem_unit 1) */
	if (h->qlen < 3 || h->qlen > 256 || h->esz_b != 16) {
		return -4;
	}
	/* the four pointer words and the two queue buffers, all SRAM offsets, bounds-checked */
	bytes = h->qlen * h->esz_b;
	off = r32(desc, 4 * D_RX_REAR);
	if ((off & 3u) || off > ACPM_SRAM_SIZE - 4) {
		return -5;
	}
	h->req_rear = acpm_sram + off;
	off = r32(desc, 4 * D_RX_FRONT);
	if ((off & 3u) || off > ACPM_SRAM_SIZE - 4) {
		return -5;
	}
	h->req_front = acpm_sram + off;
	off = r32(desc, 4 * D_TX_REAR);
	if ((off & 3u) || off > ACPM_SRAM_SIZE - 4) {
		return -5;
	}
	h->rsp_rear = acpm_sram + off;
	off = r32(desc, 4 * D_TX_FRONT);
	if ((off & 3u) || off > ACPM_SRAM_SIZE - 4) {
		return -5;
	}
	h->rsp_front = acpm_sram + off;
	off = r32(desc, 4 * D_RX_BASE);
	if ((off & 3u) || (uint64_t)off + bytes > ACPM_SRAM_SIZE) {
		return -6;
	}
	h->req_base = acpm_sram + off;
	off = r32(desc, 4 * D_TX_BASE);
	if ((off & 3u) || (uint64_t)off + bytes > ACPM_SRAM_SIZE) {
		return -6;
	}
	h->rsp_base = acpm_sram + off;
	h->rr = r32(h->req_rear, 0);
	h->rf = r32(h->req_front, 0);
	h->sr = r32(h->rsp_rear, 0);
	h->sf = r32(h->rsp_front, 0);
	if (h->rr >= h->qlen || h->rf >= h->qlen || h->sr >= h->qlen || h->sf >= h->qlen) {
		return -7;
	}
	return 0;
}

/*
 * One 4-word request and its reply. w0 carries REG | (pmic_addr << 8) | (speedy << 12); w1 is FUNC_READ
 * (0) or FUNC_UPDATE | (val << 8) | (mask << 16). The reply's value is (rep[1] >> 8) & 0xff. No retry, no
 * rollback past the publish point, as acpm.c: a failure after the front write leaves the channel as it is.
 */
static int
acpm_xfer(uint32_t w0, uint32_t w1, uint32_t *value)
{
	static uint32_t seq = 1;
	struct acpm_chan h;
	uint32_t req[4], rep[4], nf, sf, sr, cnt;
	vm_offset_t slot, ent;
	int e;

	seq = seq % 63u + 1u;
	req[0] = w0 | (seq << SEQ_SHIFT);
	req[1] = w1;
	req[2] = req[3] = 0;
	if ((e = acpm_describe(&h)) != 0) {
		return e;
	}
	if (h.rf != h.rr || h.sf != h.sr) {             /* a queue is not empty: refuse, never touch it */
		return -8;
	}
	nf = (h.rf + 1) % h.qlen;
	if ((h.rf + 2) % h.qlen == h.sf || nf == h.rr) { /* the full conditions of acpm_ipc.c:712/:723 */
		return -9;
	}
	slot = h.req_base + 16u * h.rf;
	for (int i = 0; i < 4; i++) {
		w32(slot, 4 * i, req[i]);
	}
	w32(h.req_front, 0, nf);                        /* published: the APM may run it from here on */
	w32(acpm_mbox + ACPM_INTGR1, 0, 1u << ACPM_CH_ID);
	for (uint64_t until = now_ms() + 50;;) {        /* 50 ms, the probe's bound */
		sf = r32(h.rsp_front, 0);
		sr = r32(h.rsp_rear, 0);
		if (sf != sr) {
			break;
		}
		if (now_ms() >= until) {
			return -10;
		}
		delay(1000);
	}
	if (sf >= h.qlen || sr >= h.qlen) {
		return -11;
	}
	ent = h.rsp_base + 16u * sr;
	for (int i = 0; i < 4; i++) {
		rep[i] = r32(ent, 4 * i);
	}
	cnt = (sf + h.qlen - sr) % h.qlen;
	w32(h.rsp_rear, 0, sf);                         /* drain: rear := front */
	w32(acpm_mbox + ACPM_INTCR0, 0, 1u << ACPM_CH_ID);
	if (((rep[0] >> SEQ_SHIFT) & 0x3fu) != seq || cnt != 1) {
		return -12;
	}
	if ((rep[1] >> RET_SHIFT) != 0) {
		return -13;
	}
	*value = (rep[1] >> 8) & 0xffu;
	return 0;
}

/* the S2MPG13's LDO3S (0x2e, code 10 = 0.85 V) and LDO18S (0x3d, code 44 = 1.8 V): enable bits 7:6 */
static int
acpm_rail_on(uint32_t reg, uint32_t vsel, const char *name)
{
	uint32_t v = 0;
	int e;

	e = acpm_xfer((reg & 0xffu) | (ACPM_PMIC_ADDR << 8) | (ACPM_SPEEDY << 12), FUNC_READ, &v);
	lynx_tlog("lynx_wifi: 1: %s: read err %d ctrl 0x%02x\n", name, e, v);
	if (e != 0) {
		return 0;
	}
	if (v & 0xc0u) {
		return 1;
	}
	if ((v & 0x3fu) != vsel) {
		lynx_tlog("lynx_wifi: 1: %s: off, and its voltage code is not Android's (0x%02x)\n", name, v & 0x3fu);
		return 0;
	}
	e = acpm_xfer((reg & 0xffu) | (ACPM_PMIC_ADDR << 8) | (ACPM_SPEEDY << 12), FUNC_UPDATE | (0xc0u << 8) | (0xc0u << 16), &v);
	(void)acpm_xfer((reg & 0xffu) | (ACPM_PMIC_ADDR << 8) | (ACPM_SPEEDY << 12), FUNC_READ, &v);
	lynx_tlog("lynx_wifi: 1: %s: switched on: err %d ctrl now 0x%02x\n", name, e, v);
	return e == 0 && (v & 0xc0u) != 0;
}

/* ---- the PHY calibration (gs201's pcie-exynos-gs201-rc-cal.c, the probe's gs201_cal.h, rewritten) ---- */

static void
phy_all_pwrdn(void)
{
	const struct { uint32_t o, v; } w[] = {
		{ 0x404, 0xA8 }, { 0x408, 0x20 }, { 0x40C, 0x0A },
		{ 0x800, 0x0A }, { 0x804, 0xBF }, { 0xA40, 0x02 }, { 0xA44, 0x2A }, { 0xA48, 0xAA }, { 0xA4C, 0xA8 }, { 0xA50, 0x80 },
		{ 0x1000, 0x0A }, { 0x1004, 0xBF }, { 0x1240, 0x02 }, { 0x1244, 0x2A }, { 0x1248, 0xAA }, { 0x124C, 0xA8 }, { 0x1250, 0x80 },
	};

	for (unsigned i = 0; i < sizeof(w) / sizeof(w[0]); i++) {
		wr(phy + w[i].o, w[i].v);
	}
	rmw(phy + 0x400, 0x1 << 7, 0);         /* disable PHY PMA */
	rmw(pcs + 0x184, 0xB0, 0);             /* disable rate switching */
}

static void
phy_all_pwrdn_clear(void)
{
	const struct { uint32_t o, v; } w[] = {
		{ 0x404, 0x00 }, { 0x408, 0x00 }, { 0x40C, 0x00 },
		{ 0x800, 0x00 }, { 0x804, 0x00 }, { 0xA40, 0x00 }, { 0xA44, 0x00 }, { 0xA48, 0x00 }, { 0xA4C, 0x00 }, { 0xA50, 0x00 },
		{ 0x1000, 0x00 }, { 0x1004, 0x00 }, { 0x1240, 0x00 }, { 0x1244, 0x00 }, { 0x1248, 0x00 }, { 0x124C, 0x00 }, { 0x1250, 0x00 },
	};

	wr(phy + 0xD8, 0x28);
	delay(1000);                            /* mdelay(1) */
	for (unsigned i = 0; i < sizeof(w) / sizeof(w[0]); i++) {
		wr(phy + w[i].o, w[i].v);
	}
	wr(pcs + 0x184, 0x0);                   /* reset rate change and powerdown value */
}

/* one lane's PMA and RX tuning words; the second set of RX-tune writes is the same array replayed */
static void
phy_lane(uint64_t lane)
{
	static const struct { uint32_t o, v; } pma[] = {
		{ 0x82C, 0x08 }, { 0x830, 0x24 }, { 0x878, 0x80 }, { 0x894, 0x40 }, { 0x8C0, 0x00 }, { 0x8F4, 0x30 },
		{ 0x908, 0x05 }, { 0x90C, 0xE0 }, { 0x914, 0xD4 }, { 0x91C, 0xD3 }, { 0x920, 0xCE }, { 0x924, 0x01 },
		{ 0x928, 0x35 }, { 0x92C, 0xBA }, { 0x930, 0x41 }, { 0x934, 0x15 }, { 0x938, 0x13 }, { 0x93C, 0x4E },
		{ 0x948, 0x43 }, { 0x94C, 0xFC }, { 0x954, 0x10 }, { 0x958, 0x69 }, { 0x964, 0x40 }, { 0x9B4, 0xF6 },
		{ 0x9C0, 0x2D }, { 0x9C4, 0xB7 }, { 0x9CC, 0x3C }, { 0x9DC, 0x7E }, { 0xA40, 0x02 }, { 0xA70, 0x26 },
		{ 0xA74, 0x00 }, { 0xB40, 0x06 }, { 0xB44, 0x06 }, { 0xB48, 0x04 }, { 0xB4C, 0x03 }, { 0xB50, 0x03 },
		{ 0xB54, 0x03 }, { 0xB58, 0x03 }, { 0xB5C, 0x03 }, { 0xC10, 0x1B }, { 0xC44, 0x10 }, { 0xC48, 0x10 },
		{ 0xC4C, 0x10 }, { 0xC50, 0x10 }, { 0xC54, 0x02 }, { 0xC58, 0x02 }, { 0xC5C, 0x02 }, { 0xC60, 0x02 },
		{ 0xC6C, 0x02 }, { 0xC70, 0x02 }, { 0xCA8, 0xE7 }, { 0xCAC, 0x00 }, { 0xCB0, 0x0E }, { 0xCCC, 0x1C },
		{ 0xCD4, 0x05 }, { 0xCD8, 0x77 }, { 0xCDC, 0x7A }, { 0xDB4, 0x2F },
		{ 0x8FC, 0x80 }, { 0x914, 0xF4 }, { 0x91C, 0xD3 }, { 0x920, 0xCA }, { 0x928, 0x3D }, { 0x92C, 0xB8 },
		{ 0x930, 0x41 }, { 0x934, 0x17 }, { 0x93C, 0x4C }, { 0x948, 0x73 }, { 0x94C, 0xFC }, { 0x96C, 0x55 },
		{ 0x988, 0x78 }, { 0x994, 0x3B }, { 0x9B4, 0xF6 }, { 0x9C4, 0xFF }, { 0x9C8, 0x20 }, { 0xA08, 0x2F },
		{ 0xB9C, 0x3F }, { 0x9CC, 0x00 }, { 0xCD8, 0xFF }, { 0xCDC, 0x6E },
		{ 0x82C, 0x0F }, { 0x830, 0x60 }, { 0x834, 0x7E },
		{ 0xC08, 0x00 }, { 0xC10, 0x09 }, { 0xC40, 0x04 }, { 0xC70, 0x00 }, { 0x9B4, 0x76 },
	};

	for (unsigned i = 0; i < sizeof(pma) / sizeof(pma[0]); i++) {
		wr(lane + pma[i].o, pma[i].v);
	}
}

static void
phy_config(void)
{
	static const struct { uint32_t o, v; } common[] = {
		{ 0x018, 0x50 }, { 0x048, 0x33 }, { 0x068, 0x01 }, { 0x070, 0x12 }, { 0x08C, 0x00 }, { 0x090, 0x21 },
		{ 0x0B0, 0x14 }, { 0x0B8, 0x50 }, { 0x0E0, 0x51 }, { 0x100, 0x00 }, { 0x104, 0x80 }, { 0x140, 0x38 },
		{ 0x180, 0xA4 }, { 0x188, 0x03 }, { 0x2A8, 0x38 }, { 0x2E4, 0x12 },
		{ 0x400, 0x80 }, { 0x408, 0x20 }, { 0x550, 0x00 }, { 0x5A8, 0x00 }, { 0x5EC, 0xFF },
		{ 0x458, 0x02 }, { 0x5B0, 0x34 }, { 0x450, 0x20 },
		{ 0xAC, 0x34 }, { 0xB4, 0x34 }, { 0xE0, 0x50 }, { 0xF4, 0x00 }, { 0xF8, 0x08 }, { 0x104, 0xA0 },
		{ 0x11C, 0x19 }, { 0x124, 0x17 }, { 0x220, 0x41 },
	};
	static const struct { uint32_t o, v; } clk38[] = {      /* both lanes: PHY INPUT CLK 38.4 */
		{ 0xBCC, 0x41 }, { 0xBD4, 0x41 }, { 0xBDC, 0x68 }, { 0xBE0, 0x00 }, { 0xBE4, 0xD0 },
	};
	uint32_t val;

	wr(phy + 0xD8, 0x28);                   /* init input clk path */

	val = rd(pcs + 0x008);
	val |= (1 << 7);
	wr(pcs + 0x008, val);                   /* PCS MUX glitch W/A (num-lanes 1) */

	/* PHY CMN_RST, INIT_RST, PORT_RST asserted then deasserted */
	wr(elbi + 0x1404, 0x1);
	wr(elbi + 0x1408, 0x1);
	wr(elbi + 0x1400, 0x1);
	wr(elbi + 0x1404, 0x0);
	wr(elbi + 0x1408, 0x0);
	wr(elbi + 0x1400, 0x0);
	delay(10);
	wr(elbi + 0x1404, 0x1);
	delay(10);

	for (unsigned i = 0; i < sizeof(common) / sizeof(common[0]); i++) {
		wr(phy + common[i].o, common[i].v);
	}
	phy_lane(phy);                          /* lane 0 (num-lanes 1) */
	for (unsigned i = 0; i < sizeof(clk38) / sizeof(clk38[0]); i++) {
		wr(phy + clk38[i].o, clk38[i].v);
		wr(phy + 0x800 + clk38[i].o, clk38[i].v);
	}

	/* PCS setting (num-lanes 1): aggregation off, the RC / L2-entry / L1.2 / PLL-BIAS words */
	wr(pcs + 0x004, 0x00);
	wr(pcs + 0x154, 0x700D5);
	wr(pcs + 0x150, 0x300FF);
	wr(pcs + 0x170, 0x40);
	val = rd(pcs + 0x008);
	val |= (1 << 4) | (1 << 5);
	wr(pcs + 0x008, val);
	wr(pcs + 0x190, 0x100B0808);

	/* PHY CMN_RST, PORT_RST released */
	wr(elbi + 0x1400, 0x1);
	wr(elbi + 0x1408, 0x1);

	/* additional PMA configuration: the XO clock, then AFC cal from an initial value (a CDR workaround) */
	val = rd(phy + 0x5D0);
	val |= (0x1 << 4);
	val &= ~(0x1 << 3);
	wr(phy + 0x5D0, val);
	wr(phy + 0xBF4, 0x4);
}

/* ---- the DBI (root complex) setup, dw_pcie_setup_rc + exynos_pcie_setup_rc ---- */
static void
dbi_setup(void)
{
	uint32_t val;

	rmw(dbi + PCIE_MISC_CONTROL, 0, 1u);    /* dw_pcie_dbi_ro_wr_en */
	val = rd(dbi + 0x710);
	val &= ~(1u << 7);
	val |= 1u << 5;
	wr(dbi + 0x710, val);
	val &= ~(0x3fu << 16);
	val |= 0x1u << 16;
	wr(dbi + 0x710, val);
	val = rd(dbi + 0x80C);
	val &= ~(0x1fu << 8);
	val |= 0x1u << 8;
	wr(dbi + 0x80C, val);
	wr(dbi + 0x10, 0x00000004);
	wr(dbi + 0x14, 0);
	val = rd(dbi + 0x3C);
	val &= 0xffff00ff;
	val |= 0x00000100;
	wr(dbi + 0x3C, val);
	val = rd(dbi + 0x18);
	val &= 0xff000000;
	val |= 0x00ff0100;
	wr(dbi + 0x18, val);
	val = rd(dbi + 0x04);
	val &= 0xffff0000;
	val |= 0x1 | 0x2 | 0x4 | 0x100;
	wr(dbi + 0x04, val);
	wr(dbi + 0x10, 0);
	wr16(dbi + 0x0A, 0x0604);              /* PCI_CLASS_BRIDGE_PCI */
	val = rd(dbi + 0x80C);
	val |= 1u << 17;
	wr(dbi + 0x80C, val);
	rmw(dbi + PCIE_MISC_CONTROL, 1u, 0);

	/* exynos_pcie_setup_rc() */
	wr(dbi + PCIE_MISC_CONTROL, 1u);
	wr16(dbi + 0x00, 0x144d);              /* PCI_VENDOR_ID_SAMSUNG */
	wr16(dbi + 0x02, 0xecec + 1);          /* PCI_DEVICE_ID_EXYNOS + ch_num */
	val = rd(dbi + PCIE_CAP_OFFSET + 0x0C);
	val &= ~(0x00038000u | 0x0000000fu);
	val |= 0x7u << 15;
	val |= 0x3;
	wr(dbi + PCIE_CAP_OFFSET + 0x0C, val);
	wr(dbi + PCIE_AUX_CLK_FREQ_OFF, 0x1A);
	wr(dbi + PCIE_L1_SUBSTATES_OFF, 0xEA);
	wr(dbi + PM_CAP_OFFSET + 0x04, 0);
	val = rd(dbi + PCIE_CAP_OFFSET + 0x30);
	val &= ~0xfu;
	val |= 2;                              /* FDT max-link-speed */
	wr(dbi + PCIE_CAP_OFFSET + 0x30, val);
	val = rd(dbi + PCIE_CAP_OFFSET + 0x28);
	val &= ~0xfu;
	wr(dbi + PCIE_CAP_OFFSET + 0x28, val);
}

/* up to 10 ms looking a microsecond apart; true when (reg & mask) == want */
static bool
wait_bits(vm_offset_t a, uint32_t mask, uint32_t want)
{
	uint32_t v = 0;
	int i;

	for (i = 0; i < 10000; i++) {
		delay(1);
		v = rd(a);
		if ((v & mask) == want) {
			break;
		}
	}
	lynx_tlog("lynx_wifi: 1: waiting %s: %d looks, value 0x%x\n", i < 10000 ? "done" : "TIMED OUT", i, v);
	return i < 10000;
}

static bool
establish_link(void)
{
	uint32_t val, last = 0xff;
	uint64_t t0;

	phy_config();
	lynx_tlog("lynx_wifi: 1: phy config written\n");
	wait_bits(phy + 0x03F0, 1u << 3, 1u << 3);      /* pll lock */
	wait_bits(phy + 0x0E0C, 1u << 2, 1u << 2);      /* cdr lock */
	wait_bits(phy + 0x0DE8, 0xf, 0xf);              /* offset cal */

	val = rd(elbi + PCIE_SOFT_RESET);
	val &= ~SOFT_PWR_RESET;
	wr(elbi + PCIE_SOFT_RESET, val);
	delay(1000);
	val |= SOFT_PWR_RESET;
	wr(elbi + PCIE_SOFT_RESET, val);
	wr(elbi + 0x80, 0x04);                          /* device type */
	val = rd(elbi + PCIE_SOFT_RESET);
	val |= SOFT_NON_STICKY_RESET;
	wr(elbi + PCIE_SOFT_RESET, val);
	delay(11);
	val &= ~SOFT_NON_STICKY_RESET;
	wr(elbi + PCIE_SOFT_RESET, val);
	delay(1000);
	val |= SOFT_NON_STICKY_RESET;
	wr(elbi + PCIE_SOFT_RESET, val);

	wr(dbi + 0x890, 0x12000);                       /* EQ off */
	rmw(gph2 + 4, 0, 1u << 0);                      /* PERST high */
	delay(19000);
	val = rd(elbi + PCIE_APP_REQ_EXIT_L1_MODE);
	val |= 0x1;
	val |= 0x1 << 4;
	wr(elbi + PCIE_APP_REQ_EXIT_L1_MODE, val);
	wr(elbi + PCIE_LINKDOWN_RST_CTRL_SEL, 0x1 << 1);
	val = rd(elbi + PCIE_QCH_SEL);
	val &= ~((0xFu << 8) | (0xFu << 4) | 0xFu);
	wr(elbi + PCIE_QCH_SEL, val);
	wr(elbi + PCIE_MSTR_PEND_SEL_NAK, 0x1);
	wr(elbi + PCIE_DBI_L1_EXIT_DISABLE, 0x1);
	dbi_setup();
	lynx_tlog("lynx_wifi: 1: D state 0x%x, LTSSM before 0x%x\n", rd(elbi + PCIE_PM_DSTATE) & 0x3,
	    rd(elbi + PCIE_ELBI_RDLH_LINKUP) & 0x3f);

	wr(elbi + PCIE_APP_LTSSM_ENABLE, 0x1);
	t0 = now_ms();
	for (;;) {                                      /* 100 ms, 10 us apart */
		val = rd(elbi + PCIE_ELBI_RDLH_LINKUP) & 0x3f;
		if (val != last) {
			lynx_tlog("lynx_wifi: 1: ltssm %u\n", val);
			last = val;
		}
		if (val == S_L0) {
			break;
		}
		if (now_ms() - t0 > 100) {
			break;
		}
		delay(10);
	}
	lynx_tlog("lynx_wifi: 1: ltssm final 0x%x\n", val);
	return val == S_L0;
}

/* ---- step 1: rails, clocks, SMC bypass, pins, link, bar0 and the outbound viewport ---- */
static void
lynx_wifi_step1(void)
{
	uint32_t opt, con, dat, pud;

	if (!announce(1, "the rails, clocks, PHY and link")) {
		return;
	}
	/* as ABL left it, reads only */
	lynx_tlog("lynx_wifi: 1: pmu+0x3ec4 (phy isolation) 0x%x, pmu+0x3c10 (TCXO) 0x%x\n", rd(pmu + 0x3ec4), rd(pmu + 0x3C10));
	con = rd(gph2);
	dat = rd(gph2 + 4);
	pud = rd(gph2 + 8);
	opt = rd(cmu + 0x800);
	saved_con = con;
	saved_dat = dat;
	saved_pud = pud;
	saved_opt = opt;
	saved_bypassed = false;
	lynx_tlog("lynx_wifi: 1: gph2 con 0x%x dat 0x%x pud 0x%x\n", con, dat, pud);

	/* the PHY's rails: LDO3S 0.85 V (code 10) and LDO18S 1.8 V (code 44) */
	if (!acpm_rail_on(0x2e, 10, "LDO3S") || !acpm_rail_on(0x3d, 44, "LDO18S")) {
		lynx_tlog("lynx_wifi: 1: a PHY rail is off and was not switched on: stopping\n");
		return;
	}
	delay(5000);

	/* the TCXO always-on bits, then the automatic gating of OSCCLK off while the link comes up */
	if ((rd(pmu + 0x3C10) & 0x40cu) != 0x40cu) {
		lynx_tlog("lynx_wifi: 1: TCXO always-on bits missing: writing them as the driver does\n");
		wr(pmu + 0x3C10, rd(pmu + 0x3C10) | 0x40cu);
	}
	wr(cmu + 0x800, opt & ~(1u << 28));
	if (!(rd(cmutop + 0x20c8) & (1u << 21))) {
		rmw(cmutop + 0x20c8, 0, 3u << 20);
	}
	if (!(rd(cmu + 0x620) & (1u << 4))) {           /* user mux to CLKCMU_HSI2_PCIE (99.84 MHz) */
		rmw(cmu + 0x620, 0, 1u << 4);
		for (int i = 0; i < 1000 && (rd(cmu + 0x620) & (1u << 16)); i++) {
			delay(1);
		}
	}
	for (unsigned i = 0; i < sizeof(gates) / sizeof(gates[0]); i++) {
		rmw(cmu + gates[i], 0, 3u << 20);
	}
	for (unsigned i = 1; i < sizeof(qchs) / sizeof(qchs[0]); i++) {
		rmw(cmu + qchs[i], 1u, 2u);
	}
	delay(200);
	lynx_tlog("lynx_wifi: 1: clocks forced on\n");

	/* the PHY isolation: if bit 0 is clear, EL3 must set it (the driver's write is an SMC) */
	if (!(rd(pmu + 0x3ec4) & 1u)) {
		lynx_tlog("lynx_wifi: 1: PHY isolation is on: asking EL3 for bypass\n");
		lynx_tlog("lynx_wifi: 1: smc returns %d\n", smc_priv_reg(pmu + 0x3ec4, 0x1, 0x1));
		delay(1000);
		saved_bypassed = (rd(pmu + 0x3ec4) & 1u) != 0;
		lynx_tlog("lynx_wifi: 1: pmu+0x3ec4 now 0x%x\n", rd(pmu + 0x3ec4));
	}

	/* the pins (FDT pinctrl "active"): PERST out, CLKREQ function 2 pull-up, buck then WLAN_EN high */
	rmw(gph2 + 4, 1u << 0, 0);
	rmw(gph2, 0xffu, 0x21u);
	rmw(gph2 + 8, 0xfu << 4, 0x3u << 4);
	rmw(gph2 + 4, 0, 1u << 5);
	rmw(gph2, 0xfu << 20, 0x1u << 20);             /* buck high */
	delay(100000);
	rmw(gph2 + 4, 0, 1u << 4);
	rmw(gph2, 0xfu << 16, 0x1u << 16);             /* WLAN_EN high */
	delay(10000);
	lynx_tlog("lynx_wifi: 1: gph2 con 0x%x dat 0x%x\n", rd(gph2), rd(gph2 + 4));

	/* exynos_pcie_rc_poweron: the PHY's power-down cleared, then up to three link tries */
	phy_all_pwrdn_clear();
	lynx_tlog("lynx_wifi: 1: phy power-down cleared\n");
	for (int tryn = 0; tryn < 3 && !link_up; tryn++) {
		lynx_tlog("lynx_wifi: 1: link try %d\n", tryn);
		if (establish_link()) {
			link_up = true;
			break;
		}
		rmw(gph2 + 4, 1u << 0, 0);
		wr(elbi + PCIE_APP_LTSSM_ENABLE, 0);
		delay(2000);
	}
	if (!link_up) {
		lynx_tlog("lynx_wifi: 1: THE LINK DID NOT COME UP\n");
		return;
	}
	lynx_tlog("lynx_wifi: 1: LINK UP (L0); rc ids 0x%x, lnkctl/lnksta 0x%x\n", rd(dbi + 0x0), rd(dbi + PCIE_CAP_OFFSET + 0x10));
	delay(100000);                                  /* the wait before the first configuration request */

	/* the config viewport (bus 1, device 0, function 0) and the chip's configuration space */
	wr(dbi + ATU0 + 0x08, (uint32_t)CFG);
	wr(dbi + ATU0 + 0x0C, 0);
	wr(dbi + ATU0 + 0x10, (uint32_t)CFG + CFG_SIZE - 1);
	wr(dbi + ATU0 + 0x14, 1u << 24);
	wr(dbi + ATU0 + 0x18, 0);
	wr(dbi + ATU0 + 0x00, 0x4);                     /* PCIE_ATU_TYPE_CFG0 */
	wr(dbi + ATU0 + 0x04, 1u << 31);
	delay(1000);
	lynx_tlog("lynx_wifi: 1: wifi vendor/device 0x%x (want 0x110317cb), command/status 0x%x, bar0 0x%x\n", rd(CFG + 0x00),
	    rd(CFG + 0x04), rd(CFG + 0x10));

	/* bar0 where Android puts it, the bridge's memory window, then the outbound viewport for it */
	wr(CFG + 0x10, 0x60200000u);
	wr(CFG + 0x14, 0);
	wr(dbi + 0x20, 0x6020u | (0x6030u << 16));
	wr(dbi + ATU0 + 0x200 + 0x00, 0x0);
	wr(dbi + ATU0 + 0x200 + 0x08, 0x60000000u);
	wr(dbi + ATU0 + 0x200 + 0x0C, 0);
	wr(dbi + ATU0 + 0x200 + 0x10, 0x60000000u + 0x200000u - 1);
	wr(dbi + ATU0 + 0x200 + 0x14, 0x60000000u);
	wr(dbi + ATU0 + 0x200 + 0x18, 0);
	wr(dbi + ATU0 + 0x200 + 0x04, 1u << 31);
	rmw(CFG + 0x04, 0, 0x2u | 0x4u);                /* memory space, bus master */
	delay(1000);
	lynx_tlog("lynx_wifi: 1: wifi bar0 now 0x%x, command now 0x%x, rc mem window 0x%x\n", rd(CFG + 0x10), rd(CFG + 0x04),
	    rd(dbi + 0x20));
}

/* ---- step 2: the MHI registers ---- */
static void
lynx_wifi_step2(void)
{
	vm_offset_t bar = BAR0;
	uint32_t bhi, bhie;

	if (!announce(2, "the MHI registers")) {
		return;
	}
	if (!link_up) {
		lynx_tlog("lynx_wifi: 2: no link: nothing is read\n");
		return;
	}
	lynx_tlog("lynx_wifi: 2: MHIREGLEN 0x%x MHIVER 0x%x MHICFG 0x%x CHDBOFF 0x%x ERDBOFF 0x%x\n", rd(bar + MHIREGLEN),
	    rd(bar + MHIVER), rd(bar + MHICFG), rd(bar + CHDBOFF), rd(bar + ERDBOFF));
	lynx_tlog("lynx_wifi: 2: BHIOFF 0x%x BHIEOFF 0x%x MHICTRL 0x%x MHISTATUS 0x%x\n", rd(bar + BHIOFF), rd(bar + BHIEOFF),
	    rd(bar + MHICTRL), rd(bar + MHISTATUS));
	bhi = rd(bar + BHIOFF);
	bhie = rd(bar + BHIEOFF);
	if (bhi && bhi < 0x100000 && !(bhi & 3)) {
		lynx_tlog("lynx_wifi: 2: BHI version %u.%u EXECENV 0x%x STATUS 0x%x ERRCODE 0x%x SERIALNU 0x%x\n",
		    rd(bar + bhi + 0x04), rd(bar + bhi + 0x00), rd(bar + bhi + BHI_EXECENV), rd(bar + bhi + BHI_STATUS),
		    rd(bar + bhi + BHI_ERRCODE), rd(bar + bhi + BHI_SERIALNU));
	}
	if (bhie && bhie < 0x100000 && !(bhie & 3)) {
		lynx_tlog("lynx_wifi: 2: BHIE_MSMSOCID 0x%x\n", rd(bar + bhie + 0x00));
	}
}

/* ---- step 3: open the DMA gates, then the BHI download of the SBL ---- */

static void
s2mpu_open(void)
{
	uint32_t ver = rd(s2mpu + S2MPU_VERSION), n = 0;
	uint64_t until;

	if ((ver & S2MPU_VER_MASK) == S2MPU_VER_9) {    /* v9: contexts first, before the L1ENTRY writes mean anything */
		uint32_t num = rd(s2mpu + S2MPU_NUM_CONTEXT) & 0xf, cfg = 0;

		for (uint32_t ctx = 0; ctx < 8 && ctx < num; ctx++) {
			cfg |= (ctx << (4 * ctx)) | (1u << (4 * ctx + 3));     /* context n = VID n */
		}
		if (!cfg) {
			lynx_tlog("lynx_wifi: 3: s2mpu: no contexts\n");
			return;
		}
		wr(s2mpu + S2MPU_CONTEXT_CFG, cfg);
	} else if ((ver & S2MPU_VER_MASK) != S2MPU_VER_8) {
		lynx_tlog("lynx_wifi: 3: s2mpu: a version the driver does not know (0x%x)\n", ver);
		return;
	}
	/* every gigabyte, every VID, read-write; 4..33 are read-only in hardware, leave them */
	for (uint32_t vid = 0; vid < 8; vid++) {
		for (uint32_t gb = 0; gb < 64; gb++) {
			if (gb >= 4 && gb <= 33) {
				continue;
			}
			wr(s2mpu + S2MPU_L1ENTRY_ATTR(vid, gb), S2MPU_L1ENTRY_1G_RW);
			n++;
		}
	}
	wr(s2mpu + S2MPU_ALL_INVALIDATION, 1u);         /* then wait for the invalidation to finish */
	until = now_ms() + 100;
	while ((rd(s2mpu + S2MPU_STATUS) & (S2MPU_STATUS_BUSY | S2MPU_STATUS_INVALIDATING)) ==
	    (S2MPU_STATUS_BUSY | S2MPU_STATUS_INVALIDATING) && now_ms() < until) {
		delay(1000);
	}
	wr(s2mpu + S2MPU_CFG, 0);
	wr(s2mpu + S2MPU_CTRL1, 0);
	wr(s2mpu + S2MPU_CTRL0, S2MPU_CTRL0_ENABLE | S2MPU_CTRL0_FAULT_SLVERR);
	lynx_tlog("lynx_wifi: 3: s2mpu: %u L1 entries opened, CTRL0 now 0x%x\n", n, rd(s2mpu + S2MPU_CTRL0));
}

/*
 * The PCIe System MMU: a 256 KiB first-level table of 1 MiB sections, every one identity-mapped (the chip
 * is handed physical addresses in BHI_IMGADDR, and the MMU translates them 1:1), VID 0 enabled. This is what
 * Android's pcie_iommu_map(dma, dma, ...) ends up as, and the AoC driver's sysmmu_install() on the same
 * SysMMU-v8 hardware does for the AoC. The table is cleaned to memory: the MMU walks it through its own bus
 * master, not through the processors' caches.
 */
static void
sysmmu_identity(void)
{
	vm_offset_t table = 0;

	if (kmem_alloc_contig(kernel_map, &table, SYSMMU_LV1_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		lynx_tlog("lynx_wifi: 3: no 256 KiB for the System MMU's first-level table\n");
		return;
	}
	for (uint32_t i = 0; i < SYSMMU_LV1_ENTRIES; i++) {
		*(volatile uint32_t *)(table + 4 * i) = (i << 16) | SYSMMU_SECT | SYSMMU_SHAREABLE;
	}
	clean_dcache(table, SYSMMU_LV1_SIZE, FALSE);

	wr(sysmmu + SYSMMU_CTRL, SYSMMU_CTRL_BLOCK);    /* block while the table goes in */
	wr(sysmmu + SYSMMU_VM_FLPT_BASE, (uint32_t)(kvtophys(table) >> 12));
	wr(sysmmu + SYSMMU_VM_ALL_INV, 1);
	wr(sysmmu + SYSMMU_VM_CTRL, SYSMMU_VID_STALL | SYSMMU_VID_ENABLE);       /* stall on fault, VID 0 on */
	wr(sysmmu + SYSMMU_CTRL, SYSMMU_CTRL_ENABLE);
	__asm__ volatile ("dsb sy" ::: "memory");
	lynx_tlog("lynx_wifi: 3: the System MMU: 1:1 table at 0x%llx, CTRL 0x%x\n", kvtophys(table), rd(sysmmu + SYSMMU_CTRL));
}

/* the DMA gates' own clocks, then the gates themselves (the probe gates these before touching S2MPU/SysMMU) */
static void
dma_gates_on(void)
{
	for (unsigned i = 0; i < sizeof(dma_gates) / sizeof(dma_gates[0]); i++) {
		rmw(cmu + dma_gates[i], 0, 3u << 20);
	}
	for (unsigned i = 0; i < sizeof(dma_qchs) / sizeof(dma_qchs[0]); i++) {
		rmw(cmu + dma_qchs[i], 1u, 2u);
	}
	delay(200);
}

/* the SBL (the first 512 KiB of fw/amss20.bin) into a contiguous buffer of ours, cleaned to memory */
static vm_offset_t
sbl_buffer(uint64_t *phys)
{
	uint64_t size = 0;
	const uint8_t *fw = hakios_image_file("fw", FW_NAME, &size, NULL);
	vm_offset_t buf = 0;

	if (fw == NULL || size < SBL_SIZE) {
		lynx_tlog("lynx_wifi: 3: no fw/%s in the small image (or it is short): put the phone's /vendor/firmware/%s into "
		    "fw-lynx and build the user space again\n", FW_NAME, FW_NAME);
		return 0;
	}
	if (kmem_alloc_contig(kernel_map, &buf, SBL_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) !=
	    KERN_SUCCESS) {
		lynx_tlog("lynx_wifi: 3: no 512 KiB for the SBL buffer\n");
		return 0;
	}
	memcpy((void *)buf, fw, SBL_SIZE);
	clean_dcache(buf, SBL_SIZE, FALSE);
	*phys = kvtophys(buf);
	lynx_tlog("lynx_wifi: 3: fw/%s: %llu bytes, the first %u handed to the chip at 0x%llx\n", FW_NAME, size, SBL_SIZE, *phys);
	return buf;
}

/*
 * cnss2's cnss_pci_set_wlaon_pwr_ctrl(false, false, false), right before cnss_pci_start_mhi: the WLAN
 * domain is released from its shutdown / low-rail state. Without it the PBL takes the BHI doorbell (the
 * address and size are latched into ERRDGB) but never issues a DMA read of the image -- exactly what the
 * v7 probe saw (BHI_ERRDBG1/2/3 held the address and size, BHI_STATUS/EXECENV stayed 0, AER stayed clean).
 */
static void
wlaon_pwr_ctrl(void)
{
	vm_offset_t bar = BAR0;
	uint32_t before = winrd(bar, WLAON_QFPROM_PWR_CTRL_REG);
	uint32_t after;

	lynx_tlog("lynx_wifi: 3: WLAON_QFPROM_PWR_CTRL before 0x%x\n", before);
	winwr(bar, WLAON_QFPROM_PWR_CTRL_REG,
	    before & ~(QFPROM_PWR_CTRL_VDD4BLOW_SW_EN_MASK | QFPROM_PWR_CTRL_SHUTDOWN_EN_MASK));
	after = winrd(bar, WLAON_QFPROM_PWR_CTRL_REG);
	lynx_tlog("lynx_wifi: 3: WLAON_QFPROM_PWR_CTRL after 0x%x (the 0x1|0x4 bits cleared)\n", after);
}

/*
 * The DesignWare inbound iATU, region 0, read for diagnosis (wifi_iatu=1) and, only when asked for
 * (wifi_iatu=2), programmed as an identity window (dw_pcie_prog_inbound_atu semantics: LOWER/UPPER
 * TARGET = the host address, CTRL1 = type, CTRL2 = ENABLE | BAR_MODE | FUNC_MATCH | bar). Exynos does
 * not program it (it relies on the reset default), and v7's clean AER says the default already covers
 * our addresses, so by default nothing is written: a blind write is riskier than the missing piece.
 */
static void
inbound_atu(unsigned int level)
{
	vm_offset_t a = dbi + ATU_INBOUND0;
	uint32_t ctrl1, ctrl2, lower, limit, target;

	if (level < 1) {
		return;
	}
	ctrl1 = rd(a + 0x00);
	ctrl2 = rd(a + 0x04);
	lower = rd(a + 0x08);
	limit = rd(a + 0x10);
	target = rd(a + 0x14);
	lynx_tlog("lynx_wifi: 3: inbound ATU r0: CTRL1 0x%x CTRL2 0x%x BASE 0x%x LIMIT 0x%x TARGET 0x%x\n",
	    ctrl1, ctrl2, lower, limit, target);
	if (level < 2) {
		return;
	}
	wr(a + 0x14, 0x00000000u);      /* LOWER_TARGET = 0 (identity) */
	wr(a + 0x18, 0);                /* UPPER_TARGET */
	wr(a + 0x00, 0x0);              /* CTRL1: MEM, func 0 */
	wr(a + 0x04, (1u << 31) | (1u << 30) | (1u << 19));  /* CTRL2: ENABLE | BAR_MODE | FUNC_MATCH, bar 0 */
	__asm__ volatile ("dsb sy" ::: "memory");
	lynx_tlog("lynx_wifi: 3: inbound ATU r0 written: CTRL1 0x%x CTRL2 0x%x TARGET 0x%x\n", rd(a + 0x00),
	    rd(a + 0x04), rd(a + 0x14));
}

/* mhi_fw_load_bhi(): STATUS 0, the image's address and size, the doorbell, then EXECENV awaited */
static void
bhi_download(uint64_t img)
{
	vm_offset_t bar = BAR0, bhi = rd(bar + BHIOFF);
	uint32_t st = 0, ee = 0, lastst = 0xffffffffu, lastee = 0xffffffffu;
	uint32_t dl_s = 4;
	uint64_t until;

	if (bhi == 0 || bhi >= 0x100000 || (bhi & 3)) {
		lynx_tlog("lynx_wifi: 3: no usable BHI (BHIOFF 0x%x): the download is not attempted\n", bhi);
		return;
	}
	PE_parse_boot_argn("wifi_dl_s", &dl_s, sizeof(dl_s));
	if (dl_s > 120) {
		dl_s = 120;
	}
	until = now_ms() + dl_s * 1000ull;
	wr(bar + bhi + BHI_INTVEC, 0);
	wr(bar + bhi + BHI_STATUS, 0);
	wr(bar + bhi + BHI_IMGADDR_HIGH, (uint32_t)(img >> 32));
	wr(bar + bhi + BHI_IMGADDR_LOW, (uint32_t)img);
	wr(bar + bhi + BHI_IMGSIZE, SBL_SIZE);
	wr(bar + bhi + BHI_IMGTXDB, 0x1234567u);        /* a session id, any non-zero 30 bits */
	lynx_tlog("lynx_wifi: 3: BHI doorbell rung for 0x%llx\n", img);
	for (;;) {
		st = rd(bar + bhi + BHI_STATUS);
		ee = rd(bar + bhi + BHI_EXECENV);
		if (st != lastst || ee != lastee) {
			lynx_tlog("lynx_wifi: 3: BHI_STATUS 0x%x BHI_EXECENV 0x%x\n", st, ee);
			lastst = st;
			lastee = ee;
		}
		if (((st >> 30) & 3u) == BHI_STATUS_ERROR || ee == 1) {
			break;
		}
		if (now_ms() >= until) {
			break;
		}
		delay(200);
	}
	lynx_tlog("lynx_wifi: 3: ERRCODE 0x%x ERRDGB1..3 0x%x 0x%x 0x%x; MHISTATUS 0x%x MHICTRL 0x%x MHIVER 0x%x\n",
	    rd(bar + bhi + BHI_ERRCODE), rd(bar + bhi + BHI_ERRDBG1), rd(bar + bhi + BHI_ERRDBG2), rd(bar + bhi + BHI_ERRDBG3),
	    rd(bar + MHISTATUS), rd(bar + MHICTRL), rd(bar + MHIVER));
	if (ee == 1) {
		lynx_tlog("lynx_wifi: 3: WIFI FIRST LOADER (SBL) RUNNING: the chip read our memory\n");
	} else if (((st >> 30) & 3u) == BHI_STATUS_SUCCESS) {
		lynx_tlog("lynx_wifi: 3: image taken, but no SBL yet\n");
	} else if (((st >> 30) & 3u) == BHI_STATUS_ERROR) {
		lynx_tlog("lynx_wifi: 3: THE CHIP REPORTED AN ERROR over BHI\n");
	} else {
		lynx_tlog("lynx_wifi: 3: BHI DOWNLOAD DID NOT COMPLETE (no SBL within the bound)\n");
	}
}

static void
lynx_wifi_step3(void)
{
	vm_offset_t buf;
	uint64_t phys = 0;

	if (!announce(3, "the S2MPU and the System MMU opened, the WLAN domain released, then the SBL over BHI")) {
		return;
	}
	if (!link_up) {
		lynx_tlog("lynx_wifi: 3: no link: nothing is done\n");
		return;
	}
	buf = sbl_buffer(&phys);
	if (buf == 0) {
		return;
	}
	dma_gates_on();
	s2mpu_open();
	sysmmu_identity();
	wlaon_pwr_ctrl();               /* release the WLAN domain before the doorbell (the v7 finding) */
	{
		uint32_t iatu = 0;

		PE_parse_boot_argn("wifi_iatu", &iatu, sizeof(iatu));
		inbound_atu(iatu);
	}
	bhi_download(phys);
}

/* ---- step 4: the MHI context arrays and the RESET -> READY transition ---- */

/*
 * Every MHI context -- channel, event, command -- is the same shape (struct mhi_chan_ctxt /
 * mhi_event_ctxt / mhi_cmd_ctxt in common.h): three 32-bit words, then four 64-bit ring words
 * (base, len, read ptr, write ptr), all packed at 4-byte alignment: 44 bytes.
 */
struct mhi_ctxt { uint32_t w[11]; };
#define MHI_CTX_SIZE    sizeof(struct mhi_ctxt)

static void
lynx_wifi_step4(void)
{
	vm_offset_t bar = BAR0;
	uint32_t cfg, nch, ner;
	vm_offset_t chan = 0, ev = 0, cmd = 0;
	uint64_t chan_pa, ev_pa, cmd_pa;
	struct mhi_ctxt *e, *c;

	if (!announce(4, "the MHI contexts and the RESET -> READY transition")) {
		return;
	}
	if (!link_up) {
		lynx_tlog("lynx_wifi: 4: no link: nothing is done\n");
		return;
	}
	cfg = rd(bar + MHICFG);
	nch = cfg & 0xff;
	ner = (cfg >> 16) & 0xff;
	lynx_tlog("lynx_wifi: 4: MHICFG 0x%x: %u channels, %u event rings\n", cfg, nch, ner);
	if (nch == 0 || ner == 0) {
		lynx_tlog("lynx_wifi: 4: the chip has no channels or event rings yet: not READY\n");
		return;
	}
	if (kmem_alloc_contig(kernel_map, &chan, nch * MHI_CTX_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS ||
	    kmem_alloc_contig(kernel_map, &ev, ner * MHI_CTX_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS ||
	    kmem_alloc_contig(kernel_map, &cmd, MHI_CTX_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		lynx_tlog("lynx_wifi: 4: no memory for the MHI context arrays\n");
		return;
	}
	/* event ring 0: valid, its own (empty) ring, no msivec; the channels stay disabled (zero) */
	ev_pa = kvtophys(ev);
	e = (struct mhi_ctxt *)ev;
	e[0].w[1] = 1;                          /* ertype = MHI_ER_TYPE_VALID */
	e[0].w[3] = (uint32_t)ev_pa;            /* rbase */
	e[0].w[4] = (uint32_t)(ev_pa >> 32);
	e[0].w[5] = 1024;                       /* rlen (64 ring elements x 16 bytes) */
	e[0].w[6] = 0;
	e[0].w[7] = (uint32_t)ev_pa;            /* rp == wp == base: the ring is empty */
	e[0].w[8] = (uint32_t)(ev_pa >> 32);
	e[0].w[9] = (uint32_t)ev_pa;
	e[0].w[10] = (uint32_t)(ev_pa >> 32);
	cmd_pa = kvtophys(cmd);
	c = (struct mhi_ctxt *)cmd;
	c->w[3] = (uint32_t)cmd_pa;             /* the command ring, same shape, empty */
	c->w[4] = (uint32_t)(cmd_pa >> 32);
	c->w[5] = 1024;
	c->w[6] = 0;
	c->w[7] = (uint32_t)cmd_pa;
	c->w[8] = (uint32_t)(cmd_pa >> 32);
	c->w[9] = (uint32_t)cmd_pa;
	c->w[10] = (uint32_t)(cmd_pa >> 32);
	clean_dcache(ev, ner * MHI_CTX_SIZE, FALSE);
	clean_dcache(cmd, MHI_CTX_SIZE, FALSE);
	chan_pa = kvtophys(chan);

	/* the MMIO registers: the context array bases, the event-ring count, the control/data windows */
	wr(bar + CCABAP_LOWER, (uint32_t)chan_pa);
	wr(bar + CCABAP_HIGHER, (uint32_t)(chan_pa >> 32));
	wr(bar + ECABAP_LOWER, (uint32_t)ev_pa);
	wr(bar + ECABAP_HIGHER, (uint32_t)(ev_pa >> 32));
	wr(bar + CRCBAP_LOWER, (uint32_t)cmd_pa);
	wr(bar + CRCBAP_HIGHER, (uint32_t)(cmd_pa >> 32));
	wr(bar + MHICFG, (cfg & 0x0000ffffu) | (1u << 16));     /* keep NCH/NHWCH, tell it one event ring */
	wr(bar + MHICTRLBASE_LOWER, 0);
	wr(bar + MHICTRLBASE_HIGHER, 0);
	wr(bar + MHICTRLLIMIT_LOWER, 0xffffffffu);
	wr(bar + MHICTRLLIMIT_HIGHER, 0xf);
	wr(bar + MHIDATABASE_LOWER, 0);
	wr(bar + MHIDATABASE_HIGHER, 0);
	wr(bar + MHIDATALIMIT_LOWER, 0xffffffffu);
	wr(bar + MHIDATALIMIT_HIGHER, 0xf);

	/* RESET is cleared by the chip when it has taken the contexts; then READY (MHISTATUS bit 0) */
	wr(bar + MHICTRL, MHICTRL_RESET);
	if (!wait_for(bar + MHICTRL, MHICTRL_RESET, 0, 1000)) {
		lynx_tlog("lynx_wifi: 4: the chip did not clear MHI Reset (MHICTRL 0x%x)\n", rd(bar + MHICTRL));
		return;
	}
	if (!wait_for(bar + MHISTATUS, MHISTATUS_READY, MHISTATUS_READY, 1000)) {
		lynx_tlog("lynx_wifi: 4: the chip did not enter MHI Ready (MHISTATUS 0x%x)\n", rd(bar + MHISTATUS));
		return;
	}
	lynx_tlog("lynx_wifi: 4: THE CHIP IS IN MHI READY (MHISTATUS 0x%x, MHICTRL 0x%x)\n", rd(bar + MHISTATUS), rd(bar + MHICTRL));
}

/* ---- off again, in the reverse order, only what was turned on ---- */
static void
lynx_wifi_teardown(void)
{
	uint32_t v;

	if (!link_up) {
		return;
	}
	lynx_tlog("lynx_wifi: the chip and the link are switched back off\n");
	rmw(CFG + 0x04, 0x6u, 0);                       /* memory space, bus master off */
	wr(dbi + ATU0 + 0x200 + 0x04, 0);               /* outbound viewport off */
	rmw(gph2 + 4, 1u << 0, 0);                      /* PERST low */
	wr(elbi + PCIE_APP_LTSSM_ENABLE, 0);
	v = rd(elbi + PCIE_SOFT_RESET);
	v &= ~SOFT_PWR_RESET;
	wr(elbi + PCIE_SOFT_RESET, v);
	delay(20);
	v |= SOFT_PWR_RESET;
	wr(elbi + PCIE_SOFT_RESET, v);
	phy_all_pwrdn();
	rmw(gph2 + 4, 1u << 4, 0);                      /* WLAN_EN low */
	delay(5000);
	rmw(gph2 + 4, 1u << 5, 0);                      /* buck low */
	/* the state step 1 found, put back */
	wr(gph2 + 8, saved_pud);
	wr(gph2, saved_con);
	rmw(gph2 + 4, 0x33u, saved_dat & 0x33u);
	if (saved_bypassed) {
		smc_priv_reg(pmu + 0x3ec4, 0x1, 0x0);   /* the PHY isolation back on, as ABL had it */
	}
	wr(cmu + 0x800, saved_opt);
	link_up = false;
}

static void
lynx_wifi_run(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t keep = 0;

	elbi = ml_io_map(ELBI, MAP_SIZE);
	phy = ml_io_map(PHY, MAP_SIZE);
	dbi = ml_io_map(DBI, MAP_SIZE);
	pcs = ml_io_map(PCS, MAP_SIZE);
	cmu = ml_io_map(CMU, MAP_SIZE);
	cmutop = ml_io_map(CMUTOP, MAP_SIZE);
	pmu = ml_io_map(PMU, MAP_SIZE);
	gph2 = ml_io_map(GPH2, MAP_SIZE);
	s2mpu = ml_io_map(S2MPU, MAP_SIZE);
	sysmmu = ml_io_map(SYSMMU, MAP_SIZE);
	acpm_mbox = ml_io_map(ACPM_MBOX, 0x1000);
	acpm_sram = ml_io_map(ACPM_SRAM, ACPM_SRAM_SIZE);
	if (!elbi || !phy || !dbi || !pcs || !cmu || !cmutop || !pmu || !gph2 || !s2mpu || !sysmmu || !acpm_mbox || !acpm_sram) {
		lynx_tlog("lynx_wifi: a block could not be mapped: nothing is done\n");
		return;
	}
	lynx_wifi_step1();
	lynx_wifi_step2();
	lynx_wifi_step3();
	lynx_wifi_step4();
	PE_parse_boot_argn("wifi_keep", &keep, sizeof(keep));
	if (!keep) {
		lynx_wifi_teardown();
	}
}

static void
lynx_wifi_call(thread_call_param_t which, __unused thread_call_param_t p1)
{
	(void)which;
	(void)p1;
	thread_t thread;

	if (kernel_thread_start(lynx_wifi_run, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

static void
lynx_wifi_init(void)
{
	static thread_call_t start;
	uint32_t delay_s = 10;
	uint64_t deadline;

	if (!PE_parse_boot_argn("wifi", &steps, sizeof(steps)) || steps == 0) {
		return;
	}
	if (steps > 4) {
		steps = 4;
	}
	PE_parse_boot_argn("wifi_delay", &delay_s, sizeof(delay_s));
	if (delay_s > 120) {
		delay_s = 120;
	}
	start = thread_call_allocate(lynx_wifi_call, NULL);
	clock_interval_to_deadline(delay_s, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}

STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, lynx_wifi_init);
