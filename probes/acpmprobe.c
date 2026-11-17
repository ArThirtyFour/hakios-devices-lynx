/* lynx ACPM probe v15: describe the PMIC IPC channel, then READ LDO25M (0x44) / LDO26M (0x45) through the APM. No PMIC write. */
#include <stdint.h>
#include "log.h"
#include "acpm.h"
int try_read32(uintptr_t a, uint32_t *v) { *v = *(volatile uint32_t *)a; return 1; }
int try_write32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile("dsb sy" ::: "memory"); return 1; }
uint64_t counter_freq(void) { uint64_t v; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v)); return v; }
uint64_t touch_ticks(void) { uint64_t v; __asm__ volatile("isb; mrs %0, cntpct_el0" : "=r"(v)); return v; }
uint64_t now(void) { uint64_t v; __asm__ volatile("isb; mrs %0, cntpct_el0" : "=r"(v)); return v; }
void lynxld_main(uint64_t fdt, uint64_t self)
{
    (void)fdt; (void)self;
    static struct acpm_cfg c; static struct acpm_chan h; static struct acpm_trace t;
    log_init(); log_str("ACPMPROBE v15\n"); log_flush();
    acpm_cfg_init(&c);
    int d = acpm_describe(&c, &h);
    log_hex("describe", (uint64_t)(int64_t)d); log_hex("why", h.why); log_hex("n_ap_ch", h.n); log_hex("tab", h.tab);
    log_hex("idx", h.idx); log_hex("qlen", h.qlen); log_hex("esz", h.esz); log_hex("rr", h.rr); log_hex("rf", h.rf); log_hex("sr", h.sr); log_hex("sf", h.sf);
    for (int i = 0; i < 8; i++) log_hex(" init", h.init[i]);
    log_flush();
    if (d) { log_str("ACPMPROBE-END (describe failed)\n"); log_flush(); return; }
    static const uint32_t regs[2] = { 0x44, 0x45 };
    for (int i = 0; i < 2; i++) {
        uint32_t v = 0xdead;
        int e = acpm_pmic_read(&c, regs[i], &v, &t);
        log_hex("reg", regs[i]); log_hex(" err", (uint64_t)(int64_t)e); log_hex(" val", v); log_flush();
    }
    log_str("ACPMPROBE-END\n"); log_flush();
}
void lynxld_returned(void) {}
