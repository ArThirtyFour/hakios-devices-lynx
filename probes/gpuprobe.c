/*
 * lynx GPU probe, step one: is the Mali-G710 there and awake as the bootloader leaves it?
 * Reads the two power domains, the GPU's identity, then the smallest thing that proves the GPU obeys: a soft
 * reset waited for, and its L2 cache powered on and off again. The MCU and its firmware are not touched.
 *
 * From the phone's FDT: /mali@28000000 (reg 0x28000000, 0x480000), power domains pd-g3d (0x18061E00) and
 * pd-embedded_g3d (0x18062000), gpu_pmu_status_reg_offset 0x2004 with gpu_pmu_status_local_pwr_mask 1 (the PMU is
 * at 0x18060000), /s2mpu_g3d@20080000. Android on this phone: "Mali-G710 7 cores r0p0 0x0A080602", kbase r54p3.
 * GPU registers: Linux's drivers/gpu/drm/panthor/panthor_regs.h (SOURCES.md).
 */
#include <stdint.h>
#include "log.h"

static uint64_t freq;
static uint64_t ticks(void) { uint64_t t; __asm__ volatile("isb; mrs %0,cntpct_el0" : "=r"(t)); return t; }
static void wait_us(uint64_t us) { uint64_t t = ticks(); while (ticks() - t < us * freq / 1000000u) { } }
static uint32_t rd(uint64_t a) { return *(volatile uint32_t *)a; }
static void wr(uint64_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile("dsb sy" ::: "memory"); }
static void stage(const char *s) { log_str(s); log_str("\n"); log_flush(); }

/* vec.S: a synchronous exception of ours comes here and the faulting instruction is skipped */
extern char probe_vectors[];
static volatile uint32_t faults;
void probe_fault(uint64_t esr, uint64_t elr, uint64_t far)
{
    if (faults++ < 6) { log_hex("EXCEPTION esr", esr); log_hex(" elr", elr); log_hex(" far", far); log_flush(); }
}
void probe_serror(uint64_t esr, uint64_t elr)
{
    log_hex("SERROR esr", esr); log_hex(" elr", elr); log_str("GPUPROBE-END (SError)\n"); log_flush();
}
static int tryrd(const char *k, uint64_t a)
{
    uint32_t before = faults, v;

    log_str(k); log_str(" ..."); log_flush();
    v = rd(a);
    if (faults != before) { log_str(" FAULT\n"); log_flush(); return 0; }
    log_hex(" ", v); log_flush();
    return 1;
}

#define PMU     0x18060000ull
#define GPU     0x28000000ull
#define S2MPU   0x20080000ull

/* 1 when the bits came within the time */
static int wait_for(uint64_t a, uint32_t mask, uint32_t want, uint32_t ms, const char *what)
{
    uint64_t t0 = ticks();
    uint32_t v = 0;

    while (ticks() - t0 < (uint64_t)ms * freq / 1000u) {
        v = rd(a);
        if ((v & mask) == want) break;
    }
    log_str(what); log_hex(" us", (ticks() - t0) * 1000000u / freq); log_hex(" value", v); log_flush();
    return (v & mask) == want;
}

void lynxld_main(uint64_t fdt, uint64_t self)
{
    (void)fdt; (void)self;
    uint32_t l2;

    __asm__ volatile("mrs %0,cntfrq_el0" : "=r"(freq));
    log_init(); stage("GPUPROBE v1");
    __asm__ volatile("msr vbar_el2, %0; isb" :: "r"((uint64_t)(uintptr_t)probe_vectors));

    /* the power domains, read only (an exynos-pd: CONFIGURATION +0, STATUS +4) */
    log_hex("pd-g3d conf", rd(PMU + 0x1E00)); log_hex("pd-g3d status", rd(PMU + 0x1E04));
    log_hex("pd-embedded_g3d conf", rd(PMU + 0x2000)); log_hex("pd-embedded_g3d status (bit 0: on)", rd(PMU + 0x2004)); log_flush();
    if (!(rd(PMU + 0x1E04) & 1u) || !(rd(PMU + 0x2004) & 1u)) { stage("a GPU power domain is off: nothing read"); goto end; }

    if (!tryrd("GPU_ID (Android: 0x0a080602?)", GPU + 0x000)) { stage("the GPU's registers cannot be read"); goto end; }
    tryrd("GPU_L2_FEATURES", GPU + 0x004); tryrd("GPU_CORE_FEATURES", GPU + 0x008); tryrd("GPU_TILER_FEATURES", GPU + 0x00C);
    tryrd("GPU_MEM_FEATURES", GPU + 0x010); tryrd("GPU_MMU_FEATURES", GPU + 0x014); tryrd("GPU_AS_PRESENT", GPU + 0x018);
    tryrd("GPU_CSF_ID", GPU + 0x01C); tryrd("GPU_INT_RAWSTAT", GPU + 0x020); tryrd("GPU_STATUS", GPU + 0x034);
    tryrd("GPU_FAULT_STATUS", GPU + 0x03C); tryrd("GPU_REVID", GPU + 0x280);
    tryrd("SHADER_PRESENT", GPU + 0x100); tryrd("TILER_PRESENT", GPU + 0x110); tryrd("L2_PRESENT", GPU + 0x120);
    tryrd("SHADER_READY", GPU + 0x140); tryrd("TILER_READY", GPU + 0x150); tryrd("L2_READY", GPU + 0x160);
    tryrd("COHERENCY_FEATURES", GPU + 0x300); tryrd("COHERENCY_PROTOCOL", GPU + 0x304);
    tryrd("MCU_CONTROL", GPU + 0x700); tryrd("MCU_STATUS", GPU + 0x704);
    if (faults) goto end;

    /* a soft reset: GPU_CMD 0x101, GPU_IRQ_RESET_COMPLETED is bit 8 of GPU_INT_RAWSTAT */
    wr(GPU + 0x028, 0);                         /* no interrupts: nothing here would take them */
    wr(GPU + 0x024, 0xffffffffu);
    wr(GPU + 0x030, 0x101);
    if (!wait_for(GPU + 0x020, 1u << 8, 1u << 8, 200, "soft reset")) { stage("THE GPU DID NOT RESET"); goto end; }
    wr(GPU + 0x024, 0xffffffffu);
    tryrd("GPU_STATUS after reset", GPU + 0x034); tryrd("L2_READY after reset", GPU + 0x160);

    /* the L2 cache on: L2_PWRON = L2_PRESENT, then L2_READY */
    l2 = rd(GPU + 0x120);
    wr(GPU + 0x1A0, l2);
    if (wait_for(GPU + 0x160, l2, l2, 200, "L2 power on")) stage("GPU ALIVE: reset obeyed, L2 powered");
    else stage("the L2 did not power on");
    tryrd("GPU_STATUS", GPU + 0x034); tryrd("GPU_INT_RAWSTAT", GPU + 0x020); tryrd("GPU_FAULT_STATUS", GPU + 0x03C);
    wr(GPU + 0x1E0, l2);                        /* and off again */
    wait_for(GPU + 0x160, l2, 0, 200, "L2 power off");

    /* last, because on the Pixel 8a a read of the GPU's S2MPU from here was an SError: is it ours to read on gs201? */
    if (tryrd("s2mpu_g3d VERSION", S2MPU + 0x60)) { tryrd("s2mpu_g3d CTRL0", S2MPU + 0x0); tryrd("s2mpu_g3d FAULT_STATUS", S2MPU + 0x2000); }
end:
    log_hex("exceptions taken", faults);
    stage("GPUPROBE-END");
    for (;;) __asm__ volatile("wfe");
}
void lynxld_returned(void) {}
