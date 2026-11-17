/*
 * lynx touch SPI test v17: Goodix "nottingham" on SPI 0x10D10000 (PERIC1 USI0), polled, byte mode.
 * Sources: lynx FDT (pins, rails, cs-gpio), Google kernel/gs pinctrl-gs201.c (GPIO banks), cmucal-sfr.c (CMU_PERIC1),
 * spi-s3c64xx.c (SPI/USI registers and init order), goodix_brl_spi.c / goodix_brl_hw.c (framing, brl_power_on,
 * brl_dev_confirm), touch_bus_negotiator.c (bus owner AP = 0). Everything is switched back off at the end.
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
static void rmw(uint64_t a, uint32_t clr, uint32_t set) { wr(a, (rd(a) & ~clr) | set); }

/* GPIO banks (pinctrl-gs201.c): CON +0 (4 bits/pin), DAT +4, PUD +8 (4 bits/pin) */
#define GPA7   0x180E0020ull   /* gpa7-0: touch IRQ (input) */
#define GPA8   0x180E0040ull   /* gpa8-7: aoc2ap (input) */
#define GPH1   0x11840020ull   /* gph1-0: ap2aoc bus select (output, 0 = AP) */
#define GPP20  0x10C40000ull   /* gpp20-0/1 SPI CLK/MOSI (func 3), -2 MISO (func 3, pull-down), -3 CS (GPIO out) */
#define GPP23  0x10C40060ull   /* gpp23-2: touch reset */
/* CMU_PERIC1 0x10C00000 (cmucal-sfr.c) */
#define CMU    0x10C00000ull
#define MUX_USER 0x620u        /* bit 4: 0 = OSCCLK (24.576 MHz), 1 = CLKCMU_PERIC1_IP; bit 16 busy */
#define DIV_USI0 0x1808u       /* bits 3:0 ratio; bit 16 busy */
#define G_IPCLK  0x2030u
#define G_PCLK   0x2034u
#define G_RSTN   0x2084u       /* gates: bit 20 MANUAL, bit 21 CG_VAL */
#define SYSREG_USI 0x10C21000ull /* usi-offset 0: SW_CONF[2:0], SPI = 2 */
#define SPI    0x10D10000ull
#define CH_CFG 0x00u
#define CLK_CFG 0x04u
#define MODE_CFG 0x08u
#define SLAVE_SEL 0x0Cu
#define INT_EN 0x10u
#define STATUS 0x14u
#define TX_DATA 0x18u
#define RX_DATA 0x1Cu
#define PACKET_CNT 0x20u
#define PENDING_CLR 0x24u
#define SWAP_CFG 0x28u
#define FB_CLK 0x2Cu
#define USI_CON 0xC4u
#define USI_OPTION 0xC8u
#define QCH_USI0 0x3038u      /* CMU_PERIC1: bit 0 ENABLE, bit 1 CLOCK_REQ, bit 2 IGNORE_FORCE_PM_EN */

static struct acpm_cfg c; static struct acpm_trace t;
static void st(const char *k)
{
    log_str(k); log_str("\n");
    log_hex(" irq gpa7", rd(GPA7 + 4)); log_hex(" aoc2ap gpa8", rd(GPA8 + 4)); log_hex(" gph1 con/dat", ((uint64_t)rd(GPH1) << 32) | rd(GPH1 + 4));
    log_flush();
}
static int upd(uint32_t reg, uint32_t val, uint32_t mask)
{
    int e = acpm_lynx_rail_update(&c, reg, val, mask, &t);
    uint32_t v = 0xdead; acpm_pmic_read(&c, reg, &v, &t);
    log_hex("pmic reg", reg); log_hex(" err", (uint64_t)(int64_t)e); log_hex(" now", v); log_flush();
    return e;
}
static uint32_t xfer_status, xfer_got;
static void cs(int low) { rmw(GPP20 + 4, 1u << 3, low ? 0 : (1u << 3)); }

/* one full-duplex polled transfer, len <= 60 (FIFO 64) */
static int xfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    rmw(SPI + CH_CFG, 3u, 0);                         /* channel off */
    rmw(SPI + CH_CFG, 1u << 5, 1u << 5); rmw(SPI + CH_CFG, 1u << 5, 0);   /* SW reset: flush FIFOs */
    wr(SPI + PACKET_CNT, (1u << 16) | len);
    rmw(SPI + CH_CFG, 0, 3u);                         /* RX on | TX on */
    wr(SPI + SLAVE_SEL, 0);                           /* internal nCS active: the controller shifts only then (pins use the GPIO CS) */
    for (uint32_t i = 0; i < len; i++) wr(SPI + TX_DATA, tx[i]);
    uint32_t got = 0; uint64_t t0 = touch_ticks();
    while (got < len) {
        uint32_t lvl = (rd(SPI + STATUS) >> 15) & 0x7f;
        while (lvl-- && got < len) rx[got++] = (uint8_t)rd(SPI + RX_DATA);
        if (touch_ticks() - t0 > counter_freq() / 50) break;   /* 20 ms */
    }
    xfer_status = rd(SPI + STATUS); xfer_got = got;
    wr(SPI + SLAVE_SEL, 1u);
    rmw(SPI + CH_CFG, 3u, 0);
    wr(SPI + PACKET_CNT, 0);
    return got == len ? 0 : -1;
}
static int gdx_write(uint32_t addr, const uint8_t *d, uint32_t n)
{
    uint8_t tx[64], rx[64];
    tx[0] = 0xF0; tx[1] = addr >> 24; tx[2] = addr >> 16; tx[3] = addr >> 8; tx[4] = addr;
    for (uint32_t i = 0; i < n; i++) tx[5 + i] = d[i];
    cs(1); wait_us(5); int e = xfer(tx, rx, 5 + n); wait_us(5); cs(0);
    return e;
}
static int gdx_read(uint32_t addr, uint8_t *d, uint32_t n)
{
    uint8_t tx[64], rx[64];
    for (int i = 0; i < 64; i++) tx[i] = 0;
    tx[0] = 0xF1; tx[1] = addr >> 24; tx[2] = addr >> 16; tx[3] = addr >> 8; tx[4] = addr;
    tx[5] = tx[6] = tx[7] = 0xFF;      /* goodix_spi_read (not Berlin-A): 0xF1 + addr + 3 dummy bytes */
    cs(1); wait_us(5); int e = xfer(tx, rx, 8 + n); wait_us(5); cs(0);
    for (uint32_t i = 0; i < n; i++) d[i] = rx[8 + i];
    return e;
}
static void dump(const char *k, const uint8_t *d, uint32_t n)
{
    uint64_t v = 0;
    log_str(k);
    for (uint32_t i = 0; i < n; i++) { v = (v << 8) | d[i]; if ((i & 7) == 7 || i == n - 1) { log_hex(" ", v); v = 0; } }
    log_flush();
}

static int confirm(void)
{
    uint8_t aa[8], back[8];
    for (int i = 0; i < 8; i++) { aa[i] = 0xAA; back[i] = 0x55; }
    for (int tryn = 0; tryn < 3; tryn++) {
        int e1 = gdx_write(0x10000, aa, 8);
        int e2 = gdx_read(0x10000, back, 8);
        log_hex("confirm try", (uint64_t)tryn); log_hex(" werr", (uint64_t)(int64_t)e1); log_hex(" rerr", (uint64_t)(int64_t)e2);
        log_hex(" last xfer status", xfer_status); log_hex(" last xfer got", xfer_got);
        dump(" read back", back, 8);
        int ok = 1; for (int i = 0; i < 8; i++) ok &= back[i] == 0xAA;
        if (ok) { log_str("GOODIX DEVICE CONFIRMED\n"); log_flush(); return 1; }
        wait_us(5000);
    }
    return 0;
}

void lynxld_main(uint64_t fdt, uint64_t self)
{
    (void)fdt; (void)self;
    static struct acpm_chan h;
    log_init(); log_str("TSPI v20\n"); log_flush();
    acpm_cfg_init(&c);
    if (acpm_describe(&c, &h)) { log_hex("describe failed why", h.why); log_flush(); return; }
    log_hex("cmu mux_user", rd(CMU + MUX_USER)); log_hex("cmu div_usi0", rd(CMU + DIV_USI0));
    log_hex("cmu g_ipclk", rd(CMU + G_IPCLK)); log_hex("cmu g_pclk", rd(CMU + G_PCLK)); log_hex("cmu g_rstn", rd(CMU + G_RSTN));
    log_hex("gpp20 con", rd(GPP20)); log_flush();
    st("start");

    /* power, brl_power_on order */
    if (upd(0x44, 0x80, 0x80)) goto off;
    wait_us(3000);
    if (upd(0x45, 48, 0x3f) || upd(0x45, 0xc0, 0xc0)) goto off;
    wait_us(15000);

    /* bus to the AP: gph1-0 output low (touch_bus_negotiator: TBN_BUS_OWNER_AP = 0) */
    rmw(GPH1 + 4, 1u, 0); rmw(GPH1, 0xfu, 0x1u);
    wait_us(1000); st("bus -> AP");

    /* pins: CS high (inactive) as GPIO out; CLK/MOSI/MISO func 3; MISO pull-down */
    rmw(GPP20 + 4, 1u << 3, 1u << 3);
    rmw(GPP20, 0xffffu, 0x1333u);
    rmw(GPP20 + 8, 0xfu << 8, 0x1u << 8);
    /* clocks: OSCCLK, divide by 1, gates forced on */
    rmw(CMU + MUX_USER, 1u << 4, 0); rmw(CMU + DIV_USI0, 0xfu, 0);
    rmw(CMU + G_IPCLK, 0, (1u << 20) | (1u << 21)); rmw(CMU + G_PCLK, 0, (1u << 20) | (1u << 21)); rmw(CMU + G_RSTN, 0, (1u << 20) | (1u << 21));
    wait_us(100);
    log_hex("cmu mux_user now", rd(CMU + MUX_USER)); log_hex("cmu g_ipclk now", rd(CMU + G_IPCLK)); log_flush();
    /* USI in SPI mode, out of reset; s3c64xx_spi_hwinit */
    rmw(SYSREG_USI, 0x7u, 0x2u);
    wr(SPI + USI_CON, 0);
    wr(SPI + SLAVE_SEL, 1u);          /* SIG_INACT: CS is our GPIO */
    wr(SPI + INT_EN, 0);
    wr(SPI + MODE_CFG, 0);
    wr(SPI + PACKET_CNT, 0);
    wr(SPI + PENDING_CLR, 0x1e); wr(SPI + PENDING_CLR, 0);
    wr(SPI + SWAP_CFG, 0);
    wr(SPI + FB_CLK, 0);              /* samsung,spi-feedback-delay = <0> */
    wr(SPI + CH_CFG, 0);              /* mode 0, master */
    log_hex("usi sysreg", rd(SYSREG_USI)); log_hex("spi ch_cfg", rd(SPI + CH_CFG)); log_hex("spi status", rd(SPI + STATUS)); log_flush();

    /* release reset, then brl_dev_confirm */
    rmw(GPP23 + 4, 0, 1u << 2);
    wait_us(4000); st("reset released");
    int ok = 0;
    for (int stage = 0; stage < 3 && !ok; stage++) {
        if (stage == 1) { wr(SPI + USI_OPTION, (rd(SPI + USI_OPTION) & ~4u) | 2u); log_str("stage A: USI_OPTION CLKREQ_ON\n"); }
        if (stage == 2) { rmw(CMU + QCH_USI0, 1u, 2u); log_str("stage B: QCH disabled, CLOCK_REQ\n"); }
        wait_us(100);
        log_hex(" qch", rd(CMU + QCH_USI0)); log_hex(" usi_con", rd(SPI + USI_CON)); log_hex(" usi_option", rd(SPI + USI_OPTION));
        wr(SPI + MODE_CFG, 0x3ffu << 19);
        log_hex(" mode_cfg readback (want 0x1ff80000)", rd(SPI + MODE_CFG)); log_flush();
        ok = confirm();
    }
    wait_us(100000);
    if (ok) {
        uint8_t ver[48];
        gdx_read(0x10014, ver, 48);
        dump("fw version @0x10014", ver, 48);
        log_str(" as text: ");
        for (int i = 0; i < 48; i++) { char ch = (char)ver[i]; char s2[2] = { (ch >= 32 && ch < 127) ? ch : '.', 0 }; log_str(s2); }
        log_str("\n"); log_flush();
    }
    st("end");
off:
    rmw(CMU + QCH_USI0, 2u, 1u);
    rmw(GPP23 + 4, 1u << 2, 0);
    rmw(GPP20, 0xffffu, 0);           /* SPI pins back to inputs, as ABL left them */
    upd(0x45, 0, 0xc0);
    upd(0x44, 0, 0x80);
    log_str("TSPI-END\n"); log_flush();
}
void lynxld_returned(void) {}
