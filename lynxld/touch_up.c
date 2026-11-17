/*
 * lynxld: bring the Pixel 7a touch screen up before the kernel starts, as proven by the v16-v20 probes
 * (probes/tpower.c, probes/tspi.c): rails through ACPM, the bus from the AoC, pins, CMU_PERIC1 USI0 clock, USI/SPI, reset released,
 * Goodix brl_dev_confirm. Everything is left ON for the kernel's lynx_touch driver.
 */
#include <stdint.h>
#include "log.h"
#include "acpm/acpm.h"

uint64_t counter_freq(void) { uint64_t v; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v)); return v; }
uint64_t touch_ticks(void) { uint64_t v; __asm__ volatile("isb; mrs %0, cntpct_el0" : "=r"(v)); return v; }
static void wait_us(uint64_t us) { uint64_t t = touch_ticks(); while (touch_ticks() - t < us * counter_freq() / 1000000u) { } }
static uint32_t rd(uint64_t a) { return *(volatile uint32_t *)a; }
static void wr(uint64_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile("dsb sy" ::: "memory"); }
static void rmw(uint64_t a, uint32_t clr, uint32_t set) { wr(a, (rd(a) & ~clr) | set); }

#define GPH1   0x11840020ull
#define GPP20  0x10C40000ull
#define GPP23  0x10C40060ull
#define CMU    0x10C00000ull
#define SYSREG_USI 0x10C21000ull
#define SPI    0x10D10000ull

static struct acpm_cfg c;
static struct acpm_trace t;

static int rail(uint32_t reg, uint32_t val, uint32_t mask) { return acpm_lynx_rail_update(&c, reg, val, mask, &t); }
static void cs(int active) { rmw(GPP20 + 4, 1u << 3, active ? 0 : (1u << 3)); }

static int xfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    rmw(SPI + 0x00, 3u, 0);
    rmw(SPI + 0x00, 1u << 5, 1u << 5); rmw(SPI + 0x00, 1u << 5, 0);
    wr(SPI + 0x20, (1u << 16) | len);
    rmw(SPI + 0x00, 0, 3u);
    wr(SPI + 0x0C, 0);
    for (uint32_t i = 0; i < len; i++) wr(SPI + 0x18, tx[i]);
    uint32_t got = 0; uint64_t t0 = touch_ticks();
    while (got < len && touch_ticks() - t0 < counter_freq() / 50) {
        uint32_t lvl = (rd(SPI + 0x14) >> 15) & 0x7f;
        while (lvl-- && got < len) rx[got++] = (uint8_t)rd(SPI + 0x1C);
    }
    wr(SPI + 0x0C, 1u);
    rmw(SPI + 0x00, 3u, 0);
    wr(SPI + 0x20, 0);
    return got == len ? 0 : -1;
}

static int confirm(void)
{
    uint8_t tx[24], rx[24];
    tx[0] = 0xF0; tx[1] = 0; tx[2] = 1; tx[3] = 0; tx[4] = 0;
    for (int i = 0; i < 8; i++) tx[5 + i] = 0xAA;
    cs(1); wait_us(5); xfer(tx, rx, 13); wait_us(5); cs(0);
    for (int i = 0; i < 24; i++) tx[i] = 0;
    tx[0] = 0xF1; tx[1] = 0; tx[2] = 1; tx[3] = 0; tx[4] = 0; tx[5] = tx[6] = tx[7] = 0xFF;
    cs(1); wait_us(5); int e = xfer(tx, rx, 16); wait_us(5); cs(0);
    if (e) return 0;
    for (int i = 0; i < 8; i++) if (rx[8 + i] != 0xAA) return 0;
    return 1;
}

/* returns 1 when the chip answered */
int lynx_touch_up(void)
{
    static struct acpm_chan h;
    acpm_cfg_init(&c);
    if (acpm_describe(&c, &h)) { log_hex("touch: acpm describe failed", h.why); return 0; }
    if (rail(0x44, 0x80, 0x80)) { log_str("touch: LDO25M failed\n"); return 0; }
    wait_us(3000);
    if (rail(0x45, 48, 0x3f) || rail(0x45, 0xc0, 0xc0)) { log_str("touch: LDO26M failed\n"); return 0; }
    wait_us(15000);
    rmw(GPH1 + 4, 1u, 0); rmw(GPH1, 0xfu, 0x1u);                 /* bus to the AP */
    rmw(GPP20 + 4, 1u << 3, 1u << 3);
    rmw(GPP20, 0xffffu, 0x1333u);
    rmw(GPP20 + 8, 0xfu << 8, 0x1u << 8);
    rmw(CMU + 0x620, 1u << 4, 0); rmw(CMU + 0x1808, 0xfu, 0);
    rmw(CMU + 0x2030, 0, 3u << 20); rmw(CMU + 0x2034, 0, 3u << 20); rmw(CMU + 0x2084, 0, 3u << 20);
    wait_us(100);
    rmw(SYSREG_USI, 0x7u, 0x2u);
    wr(SPI + 0xC4, 0);
    wr(SPI + 0x0C, 1u); wr(SPI + 0x10, 0); wr(SPI + 0x08, 0); wr(SPI + 0x20, 0);
    wr(SPI + 0x24, 0x1e); wr(SPI + 0x24, 0); wr(SPI + 0x28, 0); wr(SPI + 0x2C, 0); wr(SPI + 0x00, 0);
    rmw(GPP23 + 4, 0, 1u << 2);                                   /* out of reset */
    wait_us(4000);
    for (int i = 0; i < 3; i++) {
        if (confirm()) {
            /* the confirm wrote the boot option: a reset pulse puts the chip back to its normal start (as the kernel's touch_reset) */
            rmw(GPP23 + 4, 1u << 2, 0); wait_us(2000); rmw(GPP23 + 4, 0, 1u << 2); wait_us(100000);
            log_str("touch: Goodix confirmed, reset, left on for the kernel\n");
            return 1;
        }
        wait_us(5000);
    }
    log_str("touch: Goodix did not answer\n");
    return 0;
}

/* The big clusters to their FDT max-freq through ACPM DVFS: CPUCL2 (X1) and CPUCL1 (A78); logs before/after. */
static void cluster_fast(uint32_t dom, uint32_t want)
{
    static struct acpm_trace dt;
    uint32_t khz = 0;
    int e = acpm_dvfs_get(&c, dom, &khz, &dt);
    log_hex("dvfs domain", dom); log_hex(" get err", (uint64_t)(int64_t)e); log_hex(" khz before", khz);
    e = acpm_dvfs_set(&c, dom, want, &dt); log_hex(" set err", (uint64_t)(int64_t)e);
    wait_us(2000);
    khz = 0; acpm_dvfs_get(&c, dom, &khz, &dt); log_hex(" khz after", khz); log_flush();
}
void lynx_cpu_fast(void)
{
    acpm_cfg_init(&c);
    /* v16 ran all seven cores at their FDT max-freq and the SoC's own thermal trip reset the phone 40 s into the boot
     * (bootreason reboot,thermal,tj). Nothing here throttles yet, so rows from the middle of Android's tables. */
    cluster_fast(4, 2048000u);
    cluster_fast(3, 1663000u);
    cluster_fast(2, 1197000u);
}
