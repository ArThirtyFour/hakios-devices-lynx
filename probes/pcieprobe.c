/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * lynx Wi-Fi PCIe probe: bring up PCIe RC channel 1 (pcie@14520000, the WCN6740's) as Google's
 * pcie-exynos-rc.c does and read the chip's vendor / device ID (Android: 17cb:1103, link 5.0 GT/s x1).
 * Everything is switched back off at the end.
 *
 * Sources (SOURCES.md says where each is; the ref/ paths below are their names there): dwc-whi/pcie-exynos-rc.c = "rc.c", pcie-exynos-rc.h = "rc.h",
 * pcie-exynos-common.h = "common.h", pcie-exynos-gs201-rc-cal.c (in gs201_cal.h), dwc-core/pcie-designware*.
 * CMU: gs201 cmucal-sfr.c = "sfr". Pins: pinctrl-gs201.c:148 (gph2 at +0x0 of 0x14440000).
 * Addresses: the phone's FDT, /pcie@14520000 reg (elbi, udbg, phy, sysreg, dbi, pcs, config, ia).
 *
 * What the FDT says the driver does NOT do on this phone, so neither does this:
 *   no "use-phy-isol-en": no PMU PHY isolation writes (rc.c:1934-1949);
 *   pcie-clk-num = phy-clk-num = 0: the driver enables no clock itself (rc.c:1212-1247);
 *   use-cache-coherency = false: no sysreg sharability write (rc.c:3218).
 * Left out on purpose: MSI, SysMMU / S2MPU, L1ss, the "IA" block (rc.c:2530, for L1.2 exits), interrupts.
 */
#include <stdint.h>
#include "log.h"
#include "acpm.h"

int try_read32(uintptr_t a, uint32_t *v) { *v = *(volatile uint32_t *)a; return 1; }
int try_write32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile("dsb sy" ::: "memory"); return 1; }
uint64_t counter_freq(void) { uint64_t v; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v)); return v; }
uint64_t touch_ticks(void) { uint64_t v; __asm__ volatile("isb; mrs %0, cntpct_el0" : "=r"(v)); return v; }
static void wait_us(uint64_t us) { uint64_t t = touch_ticks(); while (touch_ticks() - t < us * counter_freq() / 1000000u) { } }
static uint32_t rd(uint64_t a) { return *(volatile uint32_t *)a; }
static void wr(uint64_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile("dsb sy" ::: "memory"); }
static void wr16(uint64_t a, uint16_t v) { *(volatile uint16_t *)a = v; __asm__ volatile("dsb sy" ::: "memory"); }
static void rmw(uint64_t a, uint32_t clr, uint32_t set) { wr(a, (rd(a) & ~clr) | set); }
static void stage(const char *s) { log_str(s); log_str("\n"); log_flush(); }
static void reg(const char *k, uint64_t a) { log_hex(k, rd(a)); }

extern const uint8_t wifi_sbl[], wifi_sbl_end[];        /* fw.S */

/* vec.S: a synchronous exception of ours comes here and the faulting instruction is skipped */
extern char probe_vectors[];
uint64_t probe_smc(uint64_t fn, uint64_t a1, uint64_t a2, uint64_t a3, uint64_t a4);
static volatile uint32_t faults;
void probe_fault(uint64_t esr, uint64_t elr, uint64_t far)
{
    if (faults++ < 6) { log_hex("EXCEPTION esr", esr); log_hex(" elr", elr); log_hex(" far", far); log_flush(); }
}
void probe_serror(uint64_t esr, uint64_t elr)
{
    log_hex("SERROR esr", esr); log_hex(" elr", elr); log_str("PCIEPROBE-END (SError)\n"); log_flush();
}
/* 1 if the word could be read */
static int tryrd(const char *k, uint64_t a)
{
    uint32_t before = faults, v;
    uint64_t isr;

    log_str(k); log_str(" ..."); log_flush();
    v = rd(a);
    __asm__ volatile("mrs %0, isr_el1" : "=r"(isr));
    if (faults != before) { log_str(" FAULT\n"); log_flush(); return 0; }
    log_hex(" ", v);
    if (isr & 0x100) { log_str(" (an SError is pending)\n"); }
    log_flush();
    return 1;
}

/* FDT /pcie@14520000 reg */
#define ELBI    0x14520000ull
#define PHY     0x14550000ull
#define DBI     0x14800000ull
#define PCS     0x14540000ull
#define CFG     0x60ffe000ull   /* "config", size 0x2000: cfg0_base / cfg0_size (pcie-designware-host.c:416-417) */
#define CFG_SIZE 0x2000u
#define CMU     0x14400000ull   /* CMU_HSI2, sfr:38 */
#define CMUTOP  0x1e080000ull   /* CMU_TOP, sfr:12 */
#define PMU     0x18060000ull   /* samsung,syscon-phandle -> /system-controller@18060000 */
#define GPH2    0x14440000ull   /* CON +0 (4 bits a pin), DAT +4, PUD +8: PERST 0, CLKREQ 1, WLAN_EN 4, buck 5 */

#define CAL_ELBI ELBI
#define CAL_PHY  PHY
#define CAL_PCS  PCS
#define CAL_LANES 1             /* FDT num-lanes */
#include "gs201_cal.h"

/* rc.h */
#define PCIE_APP_LTSSM_ENABLE       0x054
#define PCIE_ELBI_RDLH_LINKUP       0x2C8
#define PCIE_PM_DSTATE              0x2E8
#define PCIE_LINKDOWN_RST_CTRL_SEL  0x3A0
#define PCIE_SOFT_RESET             0x3A4
#define SOFT_PWR_RESET              (0x1 << 1)
#define SOFT_NON_STICKY_RESET       (0x1 << 3)
#define PCIE_QCH_SEL                0x3A8
#define PCIE_APP_REQ_EXIT_L1_MODE   0x3BC
#define PCIE_MSTR_PEND_SEL_NAK      0x474
#define PCIE_DBI_L1_EXIT_DISABLE    0x1078
#define PCIE_MISC_CONTROL           0x8BC
#define PCIE_AUX_CLK_FREQ_OFF       0xB40
#define PCIE_L1_SUBSTATES_OFF       0xB44
#define PCIE_CAP_OFFSET             0x70
#define PM_CAP_OFFSET               0x40
#define ATU0                        0x300000    /* CR1 +0, CR2 +4, LOWER_BASE +8, UPPER_BASE +C, LIMIT +10, LOWER_TARGET +14, UPPER_TARGET +18 */
#define S_L0                        0x11        /* common.h:88 */

static struct acpm_cfg c, c3, c18;
static struct acpm_chan h;
static struct acpm_trace t;

/* S2MPG13 (sub PMIC, speedy channel 1: s2mpg13-core.c:57, page I2C_ADDR_PMIC 1): L3S_CTRL 0x2E, L18S_CTRL 0x3D
 * (s2mpg13-register.h:86,101), enable bits 7:6, voltage code 5:0 (s2mpg13-regulator.c:221-222,251-252).
 * acpm.c reads and updates only a config's own two rails, and its update takes the 7:6 mask only for rail[1]:
 * so one config a rail, the rail in slot 1. */
static void rail_cfg(struct acpm_cfg *x, uint32_t own, uint32_t other, uint32_t vsel)
{
    acpm_cfg_init(x);
    x->speedy = 1;
    x->rail[0].reg = other; x->rail[0].vsel_known = 0;
    x->rail[1].reg = own; x->rail[1].vsel_known = 1; x->rail[1].vsel_min = vsel; x->rail[1].vsel_max = vsel;
}

/* 1: the rail is on (already, or switched on here: *mine set); 0: it is not and was left alone */
static int rail_on(struct acpm_cfg *x, uint32_t r, uint32_t vsel, const char *name, int *mine)
{
    uint32_t v = 0xdead;
    int e = acpm_pmic_read(x, r, &v, &t);

    log_str(name); log_hex(" read err", (uint64_t)(int64_t)e); log_hex(" ctrl", v); log_flush();
    if (e) return 0;
    if (v & 0xc0u) return 1;
    if ((v & 0x3fu) != vsel) { log_hex(" off, and its voltage code is not Android's", vsel); log_flush(); return 0; }
    e = acpm_lynx_rail_update(x, r, 0xc0u, 0xc0u, &t);
    v = 0xdead; acpm_pmic_read(x, r, &v, &t);
    log_hex(" switched on: err", (uint64_t)(int64_t)e); log_hex(" ctrl now", v); log_flush();
    if (e || !(v & 0xc0u)) return 0;
    *mine = 1;
    return 1;
}

/* ---- the block's DMA gate: S2MPU of HSI2 (FDT /s2mpu_hsi2@145e0000) and the PCIe SysMMU (FDT /sysmmu@145C0000) ----
 * v4: the chip latched our BHI request (address, size) and never finished reading. Android's pKVM driver says why
 * (ref/s2mpu/s2mpu.c:136-141, kvm_s2mpu.h): an S2MPU resets to "all DMA blocked" until EL2 writes its configuration.
 * We are EL2: the same as its initialize_with_prot(MPT_PROT_RW), s2mpu.c:295-311, without the fault interrupt. */
#define S2MPU   0x145e0000ull
#define SYSMMU  0x145c0000ull

static void s2mpu_show(const char *when)
{
    log_str(when); log_str("\n");
    if (!tryrd(" s2mpu VERSION", S2MPU + 0x60)) return;
    tryrd(" s2mpu CTRL0", S2MPU + 0x0); tryrd(" s2mpu CTRL1", S2MPU + 0x4); tryrd(" s2mpu CFG", S2MPU + 0x10);
    tryrd(" s2mpu STATUS", S2MPU + 0x68);
    if ((rd(S2MPU + 0x60) & 0xffff0000u) == 0x20000000u) {      /* v9 only: on this phone's v8 (0x11000002) these two fault (v5) */
        tryrd(" s2mpu NUM_CONTEXT", S2MPU + 0x100); tryrd(" s2mpu CONTEXT_CFG_VALID_VID", S2MPU + 0x104);
    }
    tryrd(" s2mpu L1ENTRY_ATTR(vid 0, gb 2)", S2MPU + 0x4004 + 2 * 8); tryrd(" s2mpu L1ENTRY_ATTR(vid 1, gb 2)", S2MPU + 0x4004 + 0x200 + 2 * 8);
    tryrd(" s2mpu FAULT_STATUS", S2MPU + 0x2000);
    for (unsigned vid = 0; vid < 8; vid++) {
        if (rd(S2MPU + 0x2000) & (1u << vid)) {
            log_hex(" fault vid", vid); reg("  pa low", S2MPU + 0x2004 + vid * 0x20); reg("  pa high", S2MPU + 0x2008 + vid * 0x20);
            reg("  info", S2MPU + 0x2010 + vid * 0x20);
        }
    }
    if (tryrd(" sysmmu CTRL", SYSMMU + 0x0)) {
        tryrd(" sysmmu CFG", SYSMMU + 0x4); tryrd(" sysmmu STATUS", SYSMMU + 0x8); tryrd(" sysmmu VERSION", SYSMMU + 0x34);
        tryrd(" sysmmu CTRL_VID(1)", SYSMMU + 0x8000 + 0x1000);
        tryrd(" sysmmu CTRL_VID(0)", SYSMMU + 0x8000);
        /* (v6 read INT_STATUS 0x60 and the FAULT_* registers 0x70-0x88 here: this block does not answer there, it
         * returns an external abort -- and the attempt below was then dropped over it. Left out.) */
    }
    log_flush();
}

static int s2mpu_open(void)
{
    uint32_t ver = rd(S2MPU + 0x60), n;
    uint64_t t0;

    if ((ver & 0xffff0000u) == 0x20000000u) {                   /* S2MPU_VERSION_9: contexts first, s2mpu.c:60-112 */
        uint32_t num = rd(S2MPU + 0x100) & 0xf, cfg = 0;

        for (uint32_t ctx = 0; ctx < 8 && ctx < num; ctx++) cfg |= (ctx << (4 * ctx)) | (1u << (4 * ctx + 3));     /* context n = VID n */
        if (!cfg) { stage("s2mpu: no contexts"); return 0; }
        wr(S2MPU + 0x104, cfg);
    } else if ((ver & 0xffff0000u) != 0x11000000u) {
        log_hex("s2mpu: a version the driver does not know", ver); log_flush();
        return 0;
    }
    n = 0;
    for (uint32_t vid = 0; vid < 8; vid++) {                    /* for_each_gb_and_vid: gigabytes 4..33 are read-only */
        for (uint32_t gb = 0; gb < 64; gb++) {
            if (gb >= 4 && gb <= 33) continue;
            wr(S2MPU + 0x4004 + vid * 0x200 + gb * 8, 3u << 1); /* L1ENTRY_ATTR_1G(MPT_PROT_RW) */
            n++;
        }
    }
    wr(S2MPU + 0x1000, 1u);                                     /* REG_NS_ALL_INVALIDATION */
    t0 = touch_ticks();
    while ((rd(S2MPU + 0x68) & 3u) == 3u && touch_ticks() - t0 < counter_freq() / 100) { }
    wr(S2MPU + 0x10, 0); wr(S2MPU + 0x4, 0);
    wr(S2MPU + 0x0, 0x1u | 0x4u);                               /* CTRL0: ENABLE | fault response */
    log_hex("s2mpu: L1 entries opened", n); log_flush();
    return 1;
}

/* The chip's registers above 0x80000 through its remap window, as cnss2's cnss_pci_reg_read() (ref/cnss2/pci.c:693-756,
 * reg.h:10,74-79): window number (offset >> 19) & 0x3f with the enable bit into BAR + 0x310C, then BAR + 0x80000 + low 19 bits. */
static uint32_t winrd(uint64_t bar, uint32_t offset)
{
    wr(bar + 0x310C, 0x40000000u | ((offset >> 19) & 0x3fu));
    (void)rd(bar + 0x310C);
    return rd(bar + 0x80000 + (offset & 0x7ffffu));
}

/* (v8: the same window, for a write) */
static void winwr(uint64_t bar, uint32_t offset, uint32_t value)
{
    wr(bar + 0x310C, 0x40000000u | ((offset >> 19) & 0x3fu));
    (void)rd(bar + 0x310C);
    wr(bar + 0x80000 + (offset & 0x7ffffu), value);
}

/* cnss_pci_dump_bl_sram_mem(), ref/cnss2/pci.c:1360-1436, for the QCA6490: what the chip's first loader says of itself */
static void pbl_show(uint64_t bar, const char *when)
{
    log_str(when); log_str("\n");
    log_hex(" window register reads back", (wr(bar + 0x310C, 0x40000000u | (0x01B000F8u >> 19)), rd(bar + 0x310C)));
    log_hex(" TCSR_PBL_LOGGING", winrd(bar, 0x01B000F8)); log_hex(" PBL_WLAN_BOOT_CFG", winrd(bar, 0x01E22B34));
    log_hex(" TLMM_PBL_BOOTSTRAP_STATUS", winrd(bar, 0x1910008)); log_hex(" WLAN_SECURE_BOOTN", winrd(bar, 0x1E22B24));
    log_hex(" WLAN_JTAG_ID", winrd(bar, 0x1E22B3C)); log_hex(" QFPROM_RAW_OEM_SECURE_ROW0_LSB", winrd(bar, 0x1E20010));
    log_hex(" TLMM_OD_EN", winrd(bar, 0x1910020)); log_hex(" PCIE_PARF_LTSSM", winrd(bar, 0x01E081B0));
    for (uint32_t i = 0; i < 40; i += 4) log_hex(" PBL log word", winrd(bar, 0x01403DA0 + i));
    log_flush();
}

/*
 * v6: what PCI Express itself says went wrong, of one function (the root port through DBI, the chip through CFG): its
 * status, the PCIe capability's Device Status, and Advanced Error Reporting -- the uncorrectable and correctable
 * status and the header of the first packet in error, which names the request (its address, who sent it).
 * Layout: PCI Express Base Specification 7.5.1 (status at 0x06, capability list at 0x34), 7.5.3 (PCI Express
 * capability, id 0x10: Device Status at +0x0a, Link Status at +0x12), 7.8.4 (AER, extended capability id 1 from
 * 0x100: uncorrectable status +0x04, correctable status +0x10, header log +0x1c, root error status +0x30).
 */
static void pci_errors(const char *who, uint64_t cfg)
{
    uint32_t at = rd(cfg + 0x34) & 0xfc, ext = 0x100;

    log_str(who); log_str("\n");
    log_hex(" status/command", rd(cfg + 0x04));
    for (int i = 0; i < 16 && at >= 0x40; i++) {
        uint32_t h = rd(cfg + at);

        if ((h & 0xff) == 0x10) {
            log_hex(" pcie cap at", at); log_hex("  device status/control", rd(cfg + at + 0x08));
            log_hex("  link status/control", rd(cfg + at + 0x10));
            wr(cfg + at + 0x08, rd(cfg + at + 0x08));               /* the status bits are write-one-to-clear */
        }
        at = (h >> 8) & 0xfc;
    }
    for (int i = 0; i < 24 && ext >= 0x100 && ext < 0x1000; i++) {
        uint32_t h = rd(cfg + ext);

        if (h == 0 || h == 0xffffffffu) break;
        if ((h & 0xffff) == 0x0001) {
            log_hex(" aer at", ext); log_hex("  uncorrectable status", rd(cfg + ext + 0x04));
            log_hex("  uncorrectable mask", rd(cfg + ext + 0x08)); log_hex("  correctable status", rd(cfg + ext + 0x10));
            log_hex("  capabilities/first error", rd(cfg + ext + 0x18));
            log_hex("  header log 0", rd(cfg + ext + 0x1c)); log_hex("  header log 1", rd(cfg + ext + 0x20));
            log_hex("  header log 2", rd(cfg + ext + 0x24)); log_hex("  header log 3", rd(cfg + ext + 0x28));
            log_hex("  root error status", rd(cfg + ext + 0x30)); log_hex("  error source", rd(cfg + ext + 0x34));
            wr(cfg + ext + 0x04, rd(cfg + ext + 0x04)); wr(cfg + ext + 0x10, rd(cfg + ext + 0x10));
        }
        ext = h >> 20;
    }
    log_flush();
}

/* rc.c:908-925: up to 10000 looks a microsecond apart */
static int wait_bits(uint64_t a, uint32_t mask, uint32_t want, const char *name)
{
    uint32_t v = 0; int i;

    for (i = 0; i < 10000; i++) {
        wait_us(1);
        v = rd(a);
        if ((v & mask) == want) break;
    }
    log_str(name); log_hex(" looks", (uint64_t)i); log_hex(" value", v);
    return i < 10000;
}

/* sfr gate layout as tspi.c used on this SoC: bit 20 MANUAL, bit 21 CG_VAL */
static const uint16_t gates[] = {
    0x2008,     /* PCIE_003 PHY_REFCLK_IN (sfr:1952) */
    0x2050, 0x2054, 0x2058, 0x205c,     /* PCIE_003 G4X2 DBI / MSTR / SLV ACLK, SUB_CTRL APB (sfr:1937-1945) */
    0x2070, 0x2074, 0x2078,     /* PCS_PMA: UDBG APB, PIPE_PAL APB, PHY APB (sfr:1976,1939,1984) */
    0x202c,     /* AS_APB_PCIEPHY (sfr:1983) */
    0x2094, 0x2098,     /* QE_PCIE_GEN4A (sfr:1962-1963) */
    0x20c8, 0x20cc, 0x20d0, 0x20d4,     /* UASC_PCIE_GEN4A DBI / SLV (sfr:1968-1971) */
};
static const uint16_t qchs[] = { 0x3000, 0x3040, 0x3048, 0x3050, 0x3058, 0x305c, 0x3060, 0x3074, 0x309c, 0x30a0 };   /* sfr:3520-3546 */

static void dbi_setup(void)
{
    uint32_t val;

    /* dw_pcie_setup_rc(), pcie-designware-host.c:712-787, and dw_pcie_setup(), pcie-designware.c:623-706 (num-lanes 1) */
    rmw(DBI + PCIE_MISC_CONTROL, 0, 1u);                        /* dw_pcie_dbi_ro_wr_en */
    val = rd(DBI + 0x710); val &= ~(1u << 7); val |= 1u << 5; wr(DBI + 0x710, val);
    val &= ~(1u << 7); val &= ~(0x3fu << 16); val |= 0x1u << 16; wr(DBI + 0x710, val);
    val = rd(DBI + 0x80C); val &= ~(0x1fu << 8); val |= 0x1u << 8; wr(DBI + 0x80C, val);
    wr(DBI + 0x10, 0x00000004); wr(DBI + 0x14, 0);
    val = rd(DBI + 0x3C); val &= 0xffff00ff; val |= 0x00000100; wr(DBI + 0x3C, val);
    val = rd(DBI + 0x18); val &= 0xff000000; val |= 0x00ff0100; wr(DBI + 0x18, val);
    val = rd(DBI + 0x04); val &= 0xffff0000; val |= 0x1 | 0x2 | 0x4 | 0x100; wr(DBI + 0x04, val);
    wr(DBI + 0x10, 0);
    wr16(DBI + 0x0A, 0x0604);                                   /* PCI_CLASS_BRIDGE_PCI */
    val = rd(DBI + 0x80C); val |= 1u << 17; wr(DBI + 0x80C, val);
    rmw(DBI + PCIE_MISC_CONTROL, 1u, 0);

    /* exynos_pcie_setup_rc(), rc.c:2720-2790 */
    wr(DBI + PCIE_MISC_CONTROL, 1u);
    wr16(DBI + 0x00, 0x144d);                                   /* PCI_VENDOR_ID_SAMSUNG */
    wr16(DBI + 0x02, 0xecec + 1);                               /* PCI_DEVICE_ID_EXYNOS + ch_num, common.h:16 */
    val = rd(DBI + PCIE_CAP_OFFSET + 0x0C);
    val &= ~(0x00038000u | 0x0000000fu); val |= 0x7u << 15; val |= 0x3;
    wr(DBI + PCIE_CAP_OFFSET + 0x0C, val);
    wr(DBI + PCIE_AUX_CLK_FREQ_OFF, 0x1A);
    wr(DBI + PCIE_L1_SUBSTATES_OFF, 0xEA);
    wr(DBI + PM_CAP_OFFSET + 0x04, 0);
    val = rd(DBI + PCIE_CAP_OFFSET + 0x30); val &= ~0xfu; val |= 2; /* FDT max-link-speed */
    wr(DBI + PCIE_CAP_OFFSET + 0x30, val);
    val = rd(DBI + PCIE_CAP_OFFSET + 0x28); val &= ~0xfu;
    wr(DBI + PCIE_CAP_OFFSET + 0x28, val);
}

/* rc.c:3103-3247. Returns 1 when the link reached L0. */
static int establish_link(void)
{
    uint32_t val, last = 0xff, seen = 0;
    uint64_t t0;

    reg("OC initial status phy+0x5FC", PHY + 0x5FC); log_flush();
    cal_pcie_phy_config();                                      /* exynos_pcie_rc_assert_phy_reset, rc.c:2657 */
    stage("phy config written");
    int pll = wait_bits(PHY + 0x03F0, 1u << 3, 1u << 3, "pll lock");
    int cdr = wait_bits(PHY + 0x0E0C, 1u << 2, 1u << 2, "cdr lock");
    int oc = wait_bits(PHY + 0x0DE8, 0xf, 0xf, "offset cal");
    log_hex("PLL_LOCK", (uint64_t)pll); log_hex("CDR_LOCK", (uint64_t)cdr); log_hex("OC", (uint64_t)oc); log_flush();

    val = rd(ELBI + PCIE_SOFT_RESET); val &= ~SOFT_PWR_RESET; wr(ELBI + PCIE_SOFT_RESET, val);
    wait_us(1000);
    val |= SOFT_PWR_RESET; wr(ELBI + PCIE_SOFT_RESET, val);
    reg("phy+0xE18 (bit 7: OC after soft reset)", PHY + 0x0E18);
    reg("phy+0x760", PHY + 0x760); reg("phy+0xE0C", PHY + 0xE0C); reg("phy+0x3F0", PHY + 0x3F0); reg("phy+0xFC0", PHY + 0xFC0);
    wr(ELBI + 0x80, 0x04);                                      /* device type */
    val = rd(ELBI + PCIE_SOFT_RESET); val |= SOFT_NON_STICKY_RESET; wr(ELBI + PCIE_SOFT_RESET, val);
    wait_us(11);
    val &= ~SOFT_NON_STICKY_RESET; wr(ELBI + PCIE_SOFT_RESET, val);
    wait_us(1000);
    val |= SOFT_NON_STICKY_RESET; wr(ELBI + PCIE_SOFT_RESET, val);
    stage("soft resets done; first DBI write next");
    wr(DBI + 0x890, 0x12000);                                   /* EQ off */
    rmw(GPH2 + 4, 0, 1u << 0);                                  /* PERST high */
    wait_us(19000);
    val = rd(ELBI + PCIE_APP_REQ_EXIT_L1_MODE); val |= 0x1; val |= 0x1 << 4; wr(ELBI + PCIE_APP_REQ_EXIT_L1_MODE, val);
    wr(ELBI + PCIE_LINKDOWN_RST_CTRL_SEL, 0x1 << 1);
    val = rd(ELBI + PCIE_QCH_SEL); val &= ~((0xFu << 8) | (0xFu << 4) | 0xFu); wr(ELBI + PCIE_QCH_SEL, val);    /* ip-ver 0x984500 >= 0x889500 */
    wr(ELBI + PCIE_MSTR_PEND_SEL_NAK, 0x1);
    wr(ELBI + PCIE_DBI_L1_EXIT_DISABLE, 0x1);
    dbi_setup();
    log_hex("D state", rd(ELBI + PCIE_PM_DSTATE) & 0x3); log_hex("LTSSM before", rd(ELBI + PCIE_ELBI_RDLH_LINKUP) & 0x3f); log_flush();

    wr(ELBI + PCIE_APP_LTSSM_ENABLE, 0x1);
    t0 = touch_ticks();
    for (;;) {                                                  /* the driver: 4800 looks 10-12 us apart; here 100 ms */
        val = rd(ELBI + PCIE_ELBI_RDLH_LINKUP) & 0x3f;
        if (val != last && seen < 48) { log_hex("ltssm", val); seen++; last = val; }
        if (val == S_L0) break;
        if (touch_ticks() - t0 > counter_freq() / 10) break;
        wait_us(10);
    }
    log_hex("ltssm final", val); log_hex("us to here", (touch_ticks() - t0) * 1000000u / counter_freq()); log_flush();
    return val == S_L0;
}

void lynxld_main(uint64_t fdt, uint64_t self)
{
    (void)fdt; (void)self;
    int l3_mine = 0, l18_mine = 0, up = 0, bypassed = 0;
    uint32_t con, dat, pud, opt, v;

    log_init(); stage("PCIEPROBE v6");
    __asm__ volatile("msr vbar_el2, %0; isb" :: "r"((uint64_t)(uintptr_t)probe_vectors));

    /* ---- as ABL left it: reads only ---- */
    reg("pmu+0x3ec4 (phy isolation)", PMU + 0x3ec4); reg("pmu+0x3c10 (TCXO, want bits 0x40c)", PMU + 0x3C10);
    reg("top mux hsi2_pcie +0x10a4", CMUTOP + 0x10a4); reg("top div +0x18a0", CMUTOP + 0x18a0); reg("top gate +0x20c8", CMUTOP + 0x20c8);
    reg("hsi2 option +0x800", CMU + 0x800); reg("hsi2 pcie user mux +0x620", CMU + 0x620); reg(" +0x624", CMU + 0x624);
    log_flush();
    for (unsigned i = 0; i < sizeof(gates) / sizeof(gates[0]); i++) { log_hex("gate", gates[i]); reg(" =", CMU + gates[i]); }
    for (unsigned i = 0; i < sizeof(qchs) / sizeof(qchs[0]); i++) { log_hex("qch", qchs[i]); reg(" =", CMU + qchs[i]); }
    con = rd(GPH2); dat = rd(GPH2 + 4); pud = rd(GPH2 + 8); opt = rd(CMU + 0x800);
    log_hex("gph2 con", con); log_hex("gph2 dat", dat); log_hex("gph2 pud", pud); log_flush();

    /* ---- the PHY's rails: LDO3S 0.85 V (code 10) and LDO18S 1.8 V (code 44), as Android's regulator_summary has them ---- */
    acpm_cfg_init(&c);
    if (acpm_describe(&c, &h)) { log_hex("acpm describe failed why", h.why); goto end; }
    rail_cfg(&c3, 0x2E, 0x3D, 10);
    rail_cfg(&c18, 0x3D, 0x2E, 44);
    if (!rail_on(&c3, 0x2E, 10, "LDO3S", &l3_mine) || !rail_on(&c18, 0x3D, 44, "LDO18S", &l18_mine)) {
        stage("a PHY rail is off and was not switched on: stopping");
        goto rails;
    }
    wait_us(5000);

    /* ---- clocks ---- */
    v = rd(PMU + 0x3C10);
    if ((v & 0x40cu) != 0x40cu) {                               /* rc.c:2697-2701 */
        stage("TCXO always-on bits missing: writing them as the driver does");
        wr(PMU + 0x3C10, v | 0x40cu);
        reg("pmu+0x3c10 now", PMU + 0x3C10); log_flush();
    }
    wr(CMU + 0x800, opt & ~(1u << 28));                         /* rc.c:2703-2706: no automatic gating of OSCCLK while the link comes up */
    if (!(rd(CMUTOP + 0x20c8) & (1u << 21))) { rmw(CMUTOP + 0x20c8, 0, 3u << 20); reg("top gate forced on", CMUTOP + 0x20c8); }
    if (!(rd(CMU + 0x620) & (1u << 4))) {                       /* user mux to CLKCMU_HSI2_PCIE (Android: 99.84 MHz), sfr:5546-5547 */
        rmw(CMU + 0x620, 0, 1u << 4);
        for (int i = 0; i < 1000 && (rd(CMU + 0x620) & (1u << 16)); i++) wait_us(1);
        reg("user mux switched", CMU + 0x620);
    }
    for (unsigned i = 0; i < sizeof(gates) / sizeof(gates[0]); i++) rmw(CMU + gates[i], 0, 3u << 20);
    for (unsigned i = 1; i < sizeof(qchs) / sizeof(qchs[0]); i++) rmw(CMU + qchs[i], 1u, 2u);   /* ENABLE 0, CLOCK_REQ 1, as ABL leaves the others */
    wait_us(200);
    stage("clocks forced on");
    /* v1 stopped dead at the first ELBI read with pmu+0x3ec4 = 0, which rc.h:120 / common.h:51-52 call PCIE_PHY_ISOLATION
     * (bit 0; 1 = PCIE_PHY_BYPASS). The driver's own write of it is rmw_priv_reg() (rc.c:401), an SMC:
     * SMC_CMD_PRIV_REG 0x82000504, the register, PRIV_REG_OPTION_RMW 2, mask, value (exynos-pd_el3.c, exynos-smc.h:45). */
    if (!(rd(PMU + 0x3ec4) & 1u)) {
        stage("PHY isolation is on: asking EL3 for bypass");
        log_hex("smc returns", probe_smc(0x82000504u, PMU + 0x3ec4, 2, 0x1, 0x1));
        reg("pmu+0x3ec4 now", PMU + 0x3ec4); log_flush();
        bypassed = (rd(PMU + 0x3ec4) & 1u) != 0;
        wait_us(1000);
    }
    if (!tryrd("elbi+0x2C8", ELBI + PCIE_ELBI_RDLH_LINKUP) || !tryrd("elbi+0x3A4", ELBI + PCIE_SOFT_RESET) ||
        !tryrd("phy+0x10", PHY + 0x10) || !tryrd("phy+0x400", PHY + 0x400) ||
        !tryrd("pcs+0x8", PCS + 0x8) || !tryrd("pcs+0x184", PCS + 0x184) || !tryrd("dbi+0x0", DBI + 0x0)) {
        stage("a PCIe block cannot be read: stopping");
        goto off;
    }

    /* ---- pins (FDT pinctrl "active": PERST gph2-0 output, CLKREQ gph2-1 function 2 pull-up) and the chip ---- */
    rmw(GPH2 + 4, 1u << 0, 0);
    rmw(GPH2, 0xffu, 0x21u);
    rmw(GPH2 + 8, 0xfu << 4, 0x3u << 4);
    rmw(GPH2 + 4, 0, 1u << 5); rmw(GPH2, 0xfu << 20, 0x1u << 20);      /* buck high, as cnss: "BUCK GPIO is LOW, drive it HIGH" */
    wait_us(100000);
    rmw(GPH2 + 4, 0, 1u << 4); rmw(GPH2, 0xfu << 16, 0x1u << 16);      /* WLAN_EN high */
    wait_us(10000);
    log_hex("gph2 con", rd(GPH2)); log_hex("gph2 dat", rd(GPH2 + 4)); log_flush();

    /* ---- exynos_pcie_rc_poweron(), rc.c:3363-3460 ---- */
    cal_phy_all_pwrdn_clear();
    stage("phy power-down cleared");
    for (int tryn = 0; tryn < 3 && !up; tryn++) {
        log_hex("link try", (uint64_t)tryn);
        up = establish_link();
        if (!up) {                                              /* rc.c:3260-3272 */
            rmw(GPH2 + 4, 1u << 0, 0);
            wr(ELBI + PCIE_APP_LTSSM_ENABLE, 0);
            wait_us(2000);
        }
    }
    if (up) {
        stage("LINK UP (L0)");
        reg("rc ids dbi+0x0", DBI + 0x0); reg("rc lnkctl/lnksta dbi+0x80", DBI + PCIE_CAP_OFFSET + 0x10); log_flush();
        wait_us(100000);                                        /* the specification's wait before the first configuration request */
        /* exynos_pcie_rc_prog_viewport_cfg0(), rc.c:1361-1387: bus 1, device 0, function 0 */
        wr(DBI + ATU0 + 0x08, (uint32_t)CFG); wr(DBI + ATU0 + 0x0C, 0);
        wr(DBI + ATU0 + 0x10, (uint32_t)CFG + CFG_SIZE - 1);
        wr(DBI + ATU0 + 0x14, 1u << 24); wr(DBI + ATU0 + 0x18, 0);
        wr(DBI + ATU0 + 0x00, 0x4);                             /* PCIE_ATU_TYPE_CFG0 */
        wr(DBI + ATU0 + 0x04, 1u << 31);
        wait_us(1000);
        stage("viewport programmed; reading the chip's configuration space next");
        v = rd(CFG + 0x00);
        log_hex("WIFI vendor/device (want 0x110317cb)", v); log_flush();
        reg("wifi command/status", CFG + 0x04); reg("wifi class/revision", CFG + 0x08);
        reg("wifi bar0", CFG + 0x10); reg("wifi bar1", CFG + 0x14); reg("wifi subsystem", CFG + 0x2C); log_flush();
        if (v == 0x110317cbu) {
            /* ---- v3: the chip's registers. BAR0 where Android puts it ("BAR 0: assigned [mem 0x60200000-0x603fffff 64bit]",
             * bridge window "BAR 14: [mem 0x60200000-0x603fffff]"), then the MHI registers at its start (mainline
             * drivers/bus/mhi/common.h, kept as ref/mhi/common.h: MHIREGLEN 0x00 ... MHISTATUS 0x48, BHI at BHIOFF). ---- */
            wr(CFG + 0x10, 0xffffffffu); reg("bar0 size mask", CFG + 0x10);
            wr(CFG + 0x10, 0x60200000u); wr(CFG + 0x14, 0);
            wr(DBI + 0x20, 0x6020u | (0x6030u << 16));             /* the bridge's memory base / limit */
            /* exynos_pcie_rc_prog_viewport_mem_outbound(), rc.c:1389-1414: viewport 1, the window's first 2 MB, target = base */
            wr(DBI + ATU0 + 0x200 + 0x00, 0x0);
            wr(DBI + ATU0 + 0x200 + 0x08, 0x60000000u); wr(DBI + ATU0 + 0x200 + 0x0C, 0);
            wr(DBI + ATU0 + 0x200 + 0x10, 0x60000000u + 0x200000u - 1);
            wr(DBI + ATU0 + 0x200 + 0x14, 0x60000000u); wr(DBI + ATU0 + 0x200 + 0x18, 0);
            wr(DBI + ATU0 + 0x200 + 0x04, 1u << 31);
            rmw(CFG + 0x04, 0, 0x2u | 0x4u);                        /* memory space, bus master */
            wait_us(1000);
            reg("wifi bar0 now", CFG + 0x10); reg("wifi command now", CFG + 0x04); reg("rc mem window", DBI + 0x20); log_flush();
            uint64_t bar = 0x60200000ull;
            if (tryrd("MHIREGLEN", bar + 0x00) && tryrd("MHIVER", bar + 0x08) && tryrd("MHICFG", bar + 0x10) &&
                tryrd("CHDBOFF", bar + 0x18) && tryrd("ERDBOFF", bar + 0x20) && tryrd("BHIOFF", bar + 0x28) &&
                tryrd("BHIEOFF", bar + 0x2c) && tryrd("MHICTRL", bar + 0x38) && tryrd("MHISTATUS", bar + 0x48)) {
                uint32_t bhi = rd(bar + 0x28), bhie = rd(bar + 0x2c);

                if (bhi && bhi < 0x100000 && !(bhi & 3)) {
                    /* v14: what the statuses and the segment count are BEFORE any doorbell. v13's
                     * TXVECSTATUS "complete" may have been the bootloader's leftover; without this line
                     * there is no way to tell a real answer from one. */
                    log_str("BHI/BHIe before any doorbell\n");
                    reg(" BHI_STATUS pre", bar + bhi + 0x2c); reg(" BHI_EXECENV pre", bar + bhi + 0x28);
                    reg(" BHI_NUMSEG pre", bar + bhi + 0x48); reg(" BHI_ERRCODE pre", bar + bhi + 0x30);
                    if (bhie && bhie < 0x100000 && !(bhie & 3)) {
                        reg(" TXVECSTATUS pre", bar + bhie + 0x44); reg(" TXVECSIZE pre", bar + bhie + 0x34);
                    }
                    log_flush();
                    tryrd("BHI version minor", bar + bhi + 0x00); tryrd("BHI version major", bar + bhi + 0x04);
                    tryrd("BHI_EXECENV (0 PBL, 1 SBL, 2 AMSS)", bar + bhi + 0x28); tryrd("BHI_STATUS", bar + bhi + 0x2c);
                    tryrd("BHI_ERRCODE", bar + bhi + 0x30); tryrd("BHI_SERIALNU", bar + bhi + 0x40);
                }
                if (bhie && bhie < 0x100000 && !(bhie & 3)) tryrd("BHIE_MSMSOCID", bar + bhie + 0x00);
                /* ---- v4: mhi_fw_load_bhi(), ref/mhi/host_boot.c:228-300: the first 512 KB of the firmware, read by the
                 * chip itself out of our memory (its bus address = the physical address: no SysMMU mapping is made).
                 * BHI_STATUS bits 31:30: 2 = taken, 3 = error; then BHI_EXECENV goes 0 (PBL) -> 1 (SBL). ---- */
                if (bhi && bhi < 0x100000 && !(bhi & 3) && rd(bar + bhi + 0x28) == 0) {
                    uint64_t img = (uint64_t)(uintptr_t)wifi_sbl, t0;
                    uint32_t size = (uint32_t)(wifi_sbl_end - wifi_sbl), st = 0, ee = 0, lastst = 0xffffffffu, lastee = 0xffffffffu;
                    const uint8_t *fw = wifi_sbl;

                    /* v9: the whole of the phone's amss20.bin, not the first 512 KB. Its first loader's part is
                     * 5.1 MB (17 PT_LOAD segments, the SBL ones end at 0x4fd000), and cnss2's sbl_size may well
                     * be that rather than SZ_512K: v8's chip took the doorbell, wrote "loading the SBL" into its
                     * PBL log and read nothing, which is what too short an image looks like. It does not fit in
                     * the loader's slot (0x10000-0x200000), so it travels in the image's payload, where mkimg puts
                     * a ramdisk (LYNXPAY2: 8 bytes of magic, six 32-bit lengths and crcs, then the gspe padded to
                     * 16, then the ramdisk). Nothing but this program reads a payload of a probe image. */
                    {
                        const uint8_t *p = (const uint8_t *)(uintptr_t)0x80200000ull;   /* the image is loaded at 0x80000000 and mkimg writes the payload at +0x200000 */
                        int magic = 1;

                        for (unsigned i = 0; i < 8; i++) {
                            magic = magic && p[i] == (uint8_t)"LYNXPAY2"[i];
                        }
                        if (magic) {
                            uint32_t glen = *(const uint32_t *)(const void *)(p + 8);
                            uint32_t rlen = *(const uint32_t *)(const void *)(p + 16);
                            const uint8_t *gspe = p + 32;
                            const uint8_t *rd = gspe + ((glen + 15u) & ~15u);

                            if (glen > 0x1000 && glen < 0x2000000 && gspe[0] == 0x7f) {
                                fw = gspe;
                                size = glen;
                                stage("the firmware is the image's payload (gspe part)");
                            } else if (rlen > 0x1000 && rlen < 0x2000000 && rd[0] == 0x7f) {
                                fw = rd;
                                size = rlen;
                                stage("the firmware is the image's payload (ramdisk part)");
                            }
                        }
                        /* v15: every buffer of Android's own download lives in the reserved wifi_region
                         * (0xfa800000-0xfcffffff, the FDT's reserved-memory node "wifi_region"), and the driver
                         * registers each 4 KiB of it with the S2MPU and maps it 1:1 in the PCIe SysMMU
                         * (pcie_dma_alloc_attrs). Our payload at 0x81100020 is not registered anywhere, which is
                         * a good reason for a chip whose doorbell is taken to read nothing. The image is copied
                         * into that region and loaded from there. */
                        {
                            volatile uint64_t *dst = (volatile uint64_t *)(uintptr_t)0xfa800000ull;
                            const uint64_t *src = (const uint64_t *)(const void *)fw;
                            uint32_t words = (size + 7u) / 8u;

                            for (uint32_t i = 0; i < words; i++) {
                                dst[i] = src[i];
                            }
                            __asm__ volatile("dsb sy" ::: "memory");
                            fw = (const uint8_t *)(uintptr_t)0xfa800000ull;
                            stage("the firmware is copied into the reserved wifi_region at 0xfa800000");
                        }
                        log_hex(" firmware at", (uint64_t)(uintptr_t)fw); log_hex(" size", size);
                        log_hex(" first word", rd((uint64_t)(uintptr_t)fw)); log_flush();
                    }

                    /* the S2MPU's and the SysMMU's own clocks (sfr:1943-1987, 3539-3543), then the gate itself */
                    reg("gate s2mpu 0x20c0", CMU + 0x20c0); reg("gate sysmmu s1 0x2028", CMU + 0x2028);
                    reg("gate ssmt 0x20b8", CMU + 0x20b8); reg(" 0x20bc", CMU + 0x20bc);
                    reg("qch 0x3090", CMU + 0x3090); reg("qch 0x3094", CMU + 0x3094); reg("qch 0x3084", CMU + 0x3084); log_flush();
                    rmw(CMU + 0x20c0, 0, 3u << 20); rmw(CMU + 0x2028, 0, 3u << 20); rmw(CMU + 0x20b8, 0, 3u << 20); rmw(CMU + 0x20bc, 0, 3u << 20);
                    rmw(CMU + 0x3090, 1u, 2u); rmw(CMU + 0x3094, 1u, 2u); rmw(CMU + 0x3084, 1u, 2u);
                    wait_us(200);
                    s2mpu_show("DMA gate as ABL left it");
                    /* v5: this S2MPU is v8 and CTRL0 = 0: not enabled, and "otherwise all traffic would be allowed through"
                     * (s2mpu.c:145): left alone. Only when it is enabled is it opened. */
                    if ((rd(S2MPU + 0x0) & 1u) && !s2mpu_open()) { stage("the S2MPU is on and was not opened: no download"); goto nobhi; }
                    /* the SysMMU: ABL left CTRL = 0x24, which is none of the driver's values; Linux's "full disable (by pass)"
                     * is CTRL_DISABLE 0 with CFG 0 (ref/s2mpu/exynos-pcie-iommu.c:202-221) */
                    wr(SYSMMU + 0x8000 + 0x1000, 0); wr(SYSMMU + 0x4, 0); wr(SYSMMU + 0x0, 0);
                    s2mpu_show("DMA gate before the download");
                    pbl_show(bar, "first loader before the download");
                    pci_errors("errors before the download: root port", DBI);
                    pci_errors("errors before the download: wifi", CFG);
                    /* v6: twice. Every run so far gave the chip an address below 4 GB (this program is loaded at
                     * 0x80000000) and it never read it. First a copy above 4 GB (bank 3 of the memory, free here),
                     * where Android's buffers may well have been; then, if that fails too, the address as before.
                     * After each: what PCI Express recorded, on both ends. */
                    /* v8: the WLAN domain's own power control. cnss2 clears SHUTDOWN_EN (bit 0) and
                     * VDD4BLOW_SW_EN (bit 2) in WLAON_QFPROM_PWR_CTRL_REG through the same remap window before
                     * it starts MHI (ref/cnss2/pci.c:1843-1895 and its call at 2566). v7: the chip latched the
                     * doorbell (BHI_ERRDBG1/2/3 held our address and size), never read, and PCI Express had
                     * nothing to report -- a domain still in shutdown looks exactly like that. */
                    /* v11: the inbound ATU. A TLP from the chip to our memory passes this window; if it is
                     * enabled and does not match, the root complex drops it and nothing anywhere says so --
                     * which is exactly what v7-v10 look like (doorbell taken, no read, AER clean). DWC puts
                     * region n of the inbound direction at ATU + (n << 9) | BIT(8) (pcie-designware.h:121-125,
                     * iatu_unroll). What the bootloader left is read, and an identity window (all of memory,
                     * match mode off) is written over it. */
                    {
                        static const uint32_t in0 = ATU0 + 0x100;

                        log_str("inbound ATU as left by the bootloader\n");
                        reg(" in0 CR1", DBI + in0 + 0x00); reg(" in0 CR2", DBI + in0 + 0x04);
                        reg(" in0 lower base", DBI + in0 + 0x08); reg(" in0 upper base", DBI + in0 + 0x0c);
                        reg(" in0 limit", DBI + in0 + 0x10); reg(" in0 lower target", DBI + in0 + 0x14);
                        reg(" in0 upper target", DBI + in0 + 0x18); log_flush();
                        wr(DBI + in0 + 0x14, 0); wr(DBI + in0 + 0x18, 0);        /* target: 1:1 */
                        wr(DBI + in0 + 0x08, 0); wr(DBI + in0 + 0x0c, 0);
                        wr(DBI + in0 + 0x10, 0xffffffffu);
                        wr(DBI + in0 + 0x00, 0);                                   /* memory, no function match */
                        wr(DBI + in0 + 0x04, 0x80000000u);                         /* enabled, match mode off */
                        log_str("inbound ATU after the identity window\n");
                        reg(" in0 CR1", DBI + in0 + 0x00); reg(" in0 CR2", DBI + in0 + 0x04);
                        reg(" in0 lower target", DBI + in0 + 0x14); reg(" in0 upper target", DBI + in0 + 0x18);
                        log_flush();
                    }
                    /* v12: whether the chip's own power domains are up. The PBL runs on little power and
                     * needs no DMA master; a download that never starts, with every register write landing,
                     * is what a half-powered WLAN domain looks like (v11 left power as the last suspect). */
                    log_hex("WLAON_SOC_POWER_CTRL", winrd(bar, 0x1F80000));
                    log_hex("WLAON_SW_COLD_RESET", winrd(bar, 0x1F8000C));
                    log_hex("WLAON_WL_PWR_STATUS", winrd(bar, 0x1F8002C));
                    log_hex("WLAON_APM_STATUS0", winrd(bar, 0x1F800C0));
                    log_hex("WLAON_APM_STATUS1", winrd(bar, 0x1F800C4));
                    log_hex("WLAON_APM_STATUS2", winrd(bar, 0x1F800C8)); log_flush();
                    {
                        uint32_t before = winrd(bar, 0x1F8031Cu), after;

                        log_hex("WLAON_QFPROM_PWR_CTRL before", before);
                        winwr(bar, 0x1F8031Cu, before & ~0x5u);
                        after = winrd(bar, 0x1F8031Cu);
                        log_hex("WLAON_QFPROM_PWR_CTRL after", after); log_flush();
                    }
                    {
                        uint32_t v;

                        log_str("v16: the reset before the download\n");
                        log_hex(" PCIE_TXVECDB before", rd(bar + 0x360)); log_hex(" PCIE_TXVECSTATUS before", rd(bar + 0x368));
                        log_hex(" PCIE_RXVECDB before", rd(bar + 0x394)); log_hex(" PCIE_RXVECSTATUS before", rd(bar + 0x39C));
                        wr(bar + 0x360, 0); wr(bar + 0x368, 0); wr(bar + 0x394, 0); wr(bar + 0x39C, 0);
                        log_hex(" PCIE_TXVECSTATUS after", rd(bar + 0x368)); log_flush();
                        log_hex(" PCIE_INT_ALL_CLEAR before", winrd(bar, 0x1E08228));
                        winwr(bar, 0x1E08228, 0xffffffffu);                 /* all interrupts cleared */
                        log_hex(" WLAON_SOC_RESET_CAUSE (read clears)", winrd(bar, 0x1F8060C));
                        log_hex(" WLAON_WARM_SW_ENTRY before", winrd(bar, 0x1F80504));
                        winwr(bar, 0x1F80504, 0);                           /* no warm path for the Q6 */
                        wait_us(10000);
                        log_hex(" WLAON_WARM_SW_ENTRY after", winrd(bar, 0x1F80504));
                        v = rd(bar + 0x3008);                               /* PCIE_SOC_GLOBAL_RESET */
                        log_hex(" PCIE_SOC_GLOBAL_RESET before", v);
                        wr(bar + 0x3008, v | 1u); (void)rd(bar + 0x3008);
                        wait_us(10000);
                        wr(bar + 0x3008, v & ~1u); (void)rd(bar + 0x3008);
                        wait_us(10000);
                        log_hex(" PCIE_SOC_GLOBAL_RESET after", rd(bar + 0x3008));
                        log_hex(" MHISTATUS before", rd(bar + 0x48));
                        wr(bar + 0x38, 2);              /* MHICTRL = MHICTRL_RESET_MASK: clears SYSERR */
                        wait_us(10000);
                        log_hex(" MHISTATUS after", rd(bar + 0x48)); log_hex(" MHICTRL after", rd(bar + 0x38));
                        log_flush();
                    }
                    if (ee == 1 && bhie && bhie < 0x100000 && !(bhie & 3)) {
                    
                    if (bhie && bhie < 0x100000 && !(bhie & 3)) {
                        struct { uint64_t addr, size; } *table = (void *)(uintptr_t)0xfb400000ull;   /* in wifi_region, after the image */
                        uint32_t seg = 0x80000u, at = 0, n = 0, vst = 0xffffffffu, lastv = 0xfffffffeu;

                        while (at < size && n < 23) {
                            table[n].addr = (uint64_t)(uintptr_t)fw + at;
                            table[n].size = size - at < seg ? size - at : seg;
                            at += (uint32_t)table[n].size;
                            n++;
                        }
                        __asm__ volatile("dsb sy" ::: "memory");
                        log_hex("BHIe segments", n); log_hex(" table at", (uint64_t)(uintptr_t)table);
                        log_hex(" table size", (uint64_t)n * 16u);
                        log_hex(" entry 0 addr", table[0].addr); log_hex(" entry 0 size", table[0].size); log_flush();
                        wr(bar + bhie + 0x30, 0);                                  /* TXVECADDR_HIGH */
                        wr(bar + bhie + 0x2c, (uint32_t)(uintptr_t)table);         /* TXVECADDR_LOW */
                        wr(bar + bhie + 0x34, n * 16u);                            /* TXVECSIZE */
                        wr(bar + bhie + 0x3c, 0x1234567u);                         /* TXVECDB: a sequence id */
                        stage("BHIe doorbell rung");
                        t0 = touch_ticks();
                        while (touch_ticks() - t0 < 6 * counter_freq()) {
                            vst = rd(bar + bhie + 0x44);
                            if (vst != lastv) {
                                log_hex("ms", (touch_ticks() - t0) * 1000u / counter_freq());
                                log_hex(" BHIe TXVECSTATUS", vst); log_flush();
                                lastv = vst;
                            }
                            if ((vst >> 30) == 2) break;
                            wait_us(200);
                        }
                        log_hex("BHIe TXVECSTATUS final", vst); log_hex(" BHI_EXECENV", rd(bar + bhi + 0x28));
                        log_hex(" BHIe ERRCODE", rd(bar + bhie + 0x4c)); log_flush();
                        ee = rd(bar + bhi + 0x28);
                        stage((vst >> 30) == 2 ? "BHIe: the chip read the table and the image" : "BHIe did not complete");
                        pci_errors("errors after BHIe: wifi", CFG);
                    }
                    }
                    for (int attempt = 0; attempt < 2 && ee != 1; attempt++) {
                        if (attempt == 0) {
                            img = (uint64_t)(uintptr_t)fw;   /* Android's first BHI buffer is in wifi_region */
                            size = 0x80000;                 /* Android's sbl_size; BHIe loads the rest later */
                        } else {
                            img = (uint64_t)(uintptr_t)fw;
                        }
                        lastst = lastee = 0xffffffffu;
                        log_hex("BHI attempt", (uint64_t)attempt);
                        log_hex(" image at", img); log_hex(" size", size); log_hex(" its first word", rd(img)); log_flush();
                        /* Android never writes BHI_NUMSEG; leave its bootloader value untouched. */
                        wr(bar + bhi + 0x20, 0);                    /* BHI_INTVEC: nobody listens */
                        wr(bar + bhi + 0x2c, 0);                    /* BHI_STATUS */
                        wr(bar + bhi + 0x0c, (uint32_t)(img >> 32));
                        wr(bar + bhi + 0x08, (uint32_t)img);
                        wr(bar + bhi + 0x10, size);
                        wr(bar + bhi + 0x18, 0x1234567u + (uint32_t)attempt);  /* BHI_IMGTXDB: a session id, any non-zero 30 bits */
                        stage("BHI doorbell rung");
                        t0 = touch_ticks();
                        while (touch_ticks() - t0 < 20 * counter_freq()) {
                            st = rd(bar + bhi + 0x2c); ee = rd(bar + bhi + 0x28);
                            if (st != lastst || ee != lastee) {
                                log_hex("ms", (touch_ticks() - t0) * 1000u / counter_freq());
                                log_hex(" BHI_STATUS", st); log_hex(" BHI_EXECENV", ee); log_flush();
                                lastst = st; lastee = ee;
                            }
                            if ((st >> 30) == 3 || ee == 1) break;
                            wait_us(200);
                        }
                        reg("BHI_ERRCODE", bar + bhi + 0x30); reg("BHI_ERRDBG1", bar + bhi + 0x34);
                        reg("BHI_ERRDBG2", bar + bhi + 0x38); reg("BHI_ERRDBG3", bar + bhi + 0x3c);
                        reg("BHI_IMGADDR_LOW back", bar + bhi + 0x08); reg("BHI_IMGADDR_HIGH back", bar + bhi + 0x0c);
                        reg("BHI_IMGSIZE back", bar + bhi + 0x10); reg("BHI_IMGTXDB back", bar + bhi + 0x18);
                        reg("MHISTATUS", bar + 0x48); reg("MHICTRL", bar + 0x38); reg("MHIVER", bar + 0x08);
                        reg("BHI_NUMSEG after", bar + bhi + 0x48); log_flush();
                        pci_errors("errors after the download: root port", DBI);
                        pci_errors("errors after the download: wifi", CFG);
                        if (ee == 1) stage("WIFI FIRST LOADER (SBL) RUNNING: the chip read our memory");
                        else if ((st >> 30) == 2) stage("image taken, but no SBL yet");
                        else stage("BHI DOWNLOAD DID NOT COMPLETE");
                    }
                    s2mpu_show("DMA gate after the download");
                    pbl_show(bar, "first loader after the download");
                nobhi: ;
                }
            }
            rmw(CFG + 0x04, 0x6u, 0);
            wr(DBI + ATU0 + 0x200 + 0x04, 0);
        }
    } else {
        stage("LINK DID NOT COME UP");
    }

    stage("tidy");
    rmw(GPH2 + 4, 1u << 0, 0);                                  /* PERST low */
    wr(ELBI + PCIE_APP_LTSSM_ENABLE, 0);
    v = rd(ELBI + PCIE_SOFT_RESET); v &= ~SOFT_PWR_RESET; wr(ELBI + PCIE_SOFT_RESET, v);
    wait_us(20);
    v |= SOFT_PWR_RESET; wr(ELBI + PCIE_SOFT_RESET, v);
    cal_phy_all_pwrdn();
    rmw(GPH2 + 4, 1u << 4, 0); wait_us(5000);                   /* WLAN_EN low */
    rmw(GPH2 + 4, 1u << 5, 0);                                  /* buck low */
    wr(GPH2 + 8, pud); wr(GPH2, con);
    rmw(GPH2 + 4, 0x33u, dat & 0x33u);
off:
    if (bypassed) { probe_smc(0x82000504u, PMU + 0x3ec4, 2, 0x1, 0x0); reg("pmu+0x3ec4 put back", PMU + 0x3ec4); }
    wr(CMU + 0x800, opt);
    log_hex("exceptions taken", faults);
rails:
    if (l18_mine) { acpm_lynx_rail_update(&c18, 0x3D, 0, 0xc0u, &t); log_str("LDO18S off again\n"); }
    if (l3_mine) { acpm_lynx_rail_update(&c3, 0x2E, 0, 0xc0u, &t); log_str("LDO3S off again\n"); }
end:
    stage("PCIEPROBE-END");
}
void lynxld_returned(void) {}
