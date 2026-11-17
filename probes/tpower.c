/* lynx touch power test v16: Google's brl_power_on() order with the lynx FDT rails, then the reset line; reads the
 * IRQ line along the way; switches everything back off at the end. PMIC writes only through acpm_lynx_rail_update(). */
#include <stdint.h>
#include "log.h"
#include "acpm.h"
int try_read32(uintptr_t a, uint32_t *v) { *v = *(volatile uint32_t *)a; return 1; }
int try_write32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile("dsb sy" ::: "memory"); return 1; }
uint64_t counter_freq(void) { uint64_t v; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v)); return v; }
uint64_t touch_ticks(void) { uint64_t v; __asm__ volatile("isb; mrs %0, cntpct_el0" : "=r"(v)); return v; }
static void wait_us(uint64_t us) { uint64_t t = touch_ticks(); while (touch_ticks() - t < us * counter_freq() / 1000000u) { } }
#define GPA7_DAT  0x180E0024ull   /* IRQ: gpa7-0 */
#define GPP23_DAT 0x10C40064ull   /* RESET: gpp23-2 (already an output, low) */
static uint32_t rd(uint64_t a) { return *(volatile uint32_t *)a; }
static struct acpm_cfg c; static struct acpm_trace t;
static void irq(const char *k) { log_str(k); log_hex(" irq(gpa7_dat)", rd(GPA7_DAT)); log_hex(" rst(gpp23_dat)", rd(GPP23_DAT)); log_flush(); }
static int upd(uint32_t reg, uint32_t val, uint32_t mask)
{
    int e = acpm_lynx_rail_update(&c, reg, val, mask, &t);
    uint32_t v = 0xdead; acpm_pmic_read(&c, reg, &v, &t);
    log_hex("upd reg", reg); log_hex(" val", val); log_hex(" mask", mask); log_hex(" err", (uint64_t)(int64_t)e); log_hex(" now", v); log_flush();
    return e;
}
void lynxld_main(uint64_t fdt, uint64_t self)
{
    (void)fdt; (void)self;
    static struct acpm_chan h;
    log_init(); log_str("TPOWER v16\n"); log_flush();
    acpm_cfg_init(&c);
    if (acpm_describe(&c, &h)) { log_hex("describe failed why", h.why); log_flush(); return; }
    irq("before");
    if (upd(0x44, 0x80, 0x80)) goto off;          /* iovdd: LDO25M on (1.8 V already set) */
    wait_us(3000);
    if (upd(0x45, 48, 0x3f)) goto off;            /* avdd: 3.0 V as the FDT says */
    if (upd(0x45, 0xc0, 0xc0)) goto off;          /* avdd: LDO26M on */
    wait_us(15000);
    irq("rails on");
    *(volatile uint32_t *)GPP23_DAT = rd(GPP23_DAT) | (1u << 2);   /* release reset */
    __asm__ volatile("dsb sy" ::: "memory");
    wait_us(4000); irq("reset released +4ms");
    wait_us(100000); irq("+100ms");
    wait_us(500000); irq("+600ms");
off:
    *(volatile uint32_t *)GPP23_DAT = rd(GPP23_DAT) & ~(1u << 2);  /* back in reset */
    __asm__ volatile("dsb sy" ::: "memory");
    upd(0x45, 0, 0xc0);
    upd(0x44, 0, 0x80);
    irq("all off");
    log_str("TPOWER-END\n"); log_flush();
}
void lynxld_returned(void) {}
