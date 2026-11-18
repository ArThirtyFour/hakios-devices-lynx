// SPDX-License-Identifier: MIT
/*
 * lynx GPU probe, step two: the Mali-G710's own processor (the MCU) started on Arm's CSF firmware, and one word
 * exchanged with it.
 *
 * What is done is what Linux's panthor does to bring a CSF GPU up (drivers/gpu/drm/panthor, "GPL-2.0 or MIT":
 * panthor_fw.c for the firmware file and the start, panthor_mmu.c for the address space, panthor_regs.h and
 * panthor_fw.h for the registers and the interface; v6.12, SOURCES.md), in the order it does it:
 *   - a soft reset, the L2 cache powered;
 *   - the firmware file read (magic 0xc3f13a6e, then entries; type 0 is a section: flags, the MCU's virtual range,
 *     where its bytes are in the file) and each section given memory of ours and mapped in address space 0 with
 *     page tables of the processors' AArch64 format; the protected-mode section is skipped, as panthor skips it;
 *   - address space 0 pointed at the tables (TRANSTAB, MEMATTR, TRANSCFG, the UPDATE command);
 *   - MCU_CONTROL = AUTO, then MCU_STATUS and the global-interface interrupt bit waited for;
 *   - the firmware's control block read at the start of the shared section (0x04000000): its version, where its
 *     input and output blocks are, how many groups it has;
 *   - a ping: GLB_PING toggled in the request word, doorbell 0 rung, the acknowledge word watched.
 * Then the MCU is stopped again, the L2 switched off and the GPU reset: nothing outlives the probe.
 *
 * Step one (gpuprobe.c) found on this phone: GPU_ID 0xa8620004, MMU_FEATURES 0x2830 (48 virtual, 40 physical bits),
 * both power domains on after the bootloader, the GPU's S2MPU not enabled.
 */
#include <stdint.h>
#include "log.h"

extern const uint8_t gpu_fw[], gpu_fw_end[];

static uint64_t freq;
static uint64_t ticks(void) { uint64_t t; __asm__ volatile("isb; mrs %0,cntpct_el0" : "=r"(t)); return t; }
static void wait_us(uint64_t us) { uint64_t t = ticks(); while (ticks() - t < us * freq / 1000000u) { } }
static void stage(const char *s) { log_str(s); log_str("\n"); log_flush(); }

extern char probe_vectors[];
static volatile uint32_t faults;
void probe_fault(uint64_t esr, uint64_t elr, uint64_t far)
{
    if (faults++ < 6) { log_hex("EXCEPTION esr", esr); log_hex(" elr", elr); log_hex(" far", far); log_flush(); }
}
void probe_serror(uint64_t esr, uint64_t elr)
{
    log_hex("SERROR esr", esr); log_hex(" elr", elr); log_str("GPUMCU-END (SError)\n"); log_flush();
}

#define PMU     0x18060000ull
#define GPU     0x28000000ull
static uint32_t rd(uint32_t off) { return *(volatile uint32_t *)(GPU + off); }
static void wr(uint32_t off, uint32_t v) { *(volatile uint32_t *)(GPU + off) = v; __asm__ volatile("dsb sy" ::: "memory"); }

/* panthor_regs.h */
#define GPU_INT_RAWSTAT 0x020
#define GPU_INT_CLEAR   0x024
#define GPU_INT_MASK    0x028
#define GPU_CMD         0x030
#define GPU_STATUS      0x034
#define GPU_FAULT_STATUS 0x03c
#define L2_PRESENT      0x120
#define SHADER_READY    0x140
#define L2_READY        0x160
#define L2_PWRON        0x1a0
#define L2_PWROFF       0x1e0
#define COHERENCY_ENABLE 0x304          /* kbase's name; 31 = no coherency protocol */
#define MCU_CONTROL     0x700
#define MCU_STATUS      0x704
#define JOB_INT_RAWSTAT 0x1000
#define JOB_INT_CLEAR   0x1004
#define JOB_INT_MASK    0x1008
#define MMU_INT_RAWSTAT 0x2000
#define MMU_INT_CLEAR   0x2004
#define MMU_INT_MASK    0x2008
#define AS0             0x2400          /* TRANSTAB +0, MEMATTR +8, COMMAND +0x18, FAULTSTATUS +0x1c, FAULTADDRESS +0x20, STATUS +0x28, TRANSCFG +0x30 */
#define DOORBELL0       0x80000

/* Memory of ours for the firmware's sections: 1 MiB (the seven sections taken are 896 KiB) inside what the bootloader
 * loaded this image into (the Image is 37.8 MB at 0x80000000; this far in there is only the old kernel's bytes, which a
 * probe has no use for). Not .bss: start.S reaches the end of .bss with an ADR, a megabyte at most. Zeroed as taken. */
#define POOL_PAGES 256u
#define pool ((uint8_t *)0x81800000ull)
static uint32_t pool_used;
static uint64_t tables[16][512] __attribute__((aligned(4096)));
static uint32_t tables_used = 1;        /* tables[0] is level 0 */

static uint8_t *take(uint32_t bytes)
{
    uint32_t pages = (bytes + 4095u) / 4096u;
    uint8_t *p;

    if (pool_used + pages > POOL_PAGES) return 0;
    p = pool + (uint64_t)pool_used * 4096u;
    pool_used += pages;
    for (uint64_t i = 0; i < (uint64_t)pages * 4096u; i += 8) *(volatile uint64_t *)(p + i) = 0;
    return p;
}

/* One 4 KiB page in the tables. Entries as Linux's io-pgtable-arm writes them for ARM_64_LPAE_S1 (what panthor uses):
 * tables valid | table; a page valid | page, the attribute index (0 not cached, 1 cached: MAIR 0x44 / 0xff), AP
 * unprivileged (bit 6) and read-only (bit 7), shareability (inner 3 for cached, outer 2 otherwise), AF, nG, XN. */
static int map_page(uint64_t va, uint64_t pa, uint64_t attrs)
{
    uint64_t *t = tables[0];

    for (int level = 0; level < 3; level++) {
        uint32_t i = (uint32_t)(va >> (39 - 9 * level)) & 511u;

        if (!(t[i] & 1u)) {
            if (tables_used >= 16) return 0;
            t[i] = (uint64_t)(uintptr_t)tables[tables_used++] | 3u;
        }
        t = (uint64_t *)(uintptr_t)(t[i] & 0x000000fffffff000ull);
    }
    t[(va >> 12) & 511u] = pa | attrs | 3u;
    return 1;
}

static uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

static uint8_t *shared;                 /* our memory behind the MCU's 0x04000000 */
static uint32_t shared_size;

/* panthor_fw_load(): 1 when every section is in place */
static int load_firmware(void)
{
    uint32_t size = (uint32_t)(gpu_fw_end - gpu_fw), end, off = 20;

    if (size < 20 || le32(gpu_fw) != 0xc3f13a6eu) { stage("not a CSF firmware file"); return 0; }
    end = le32(gpu_fw + 16);
    log_hex("firmware bytes", size); log_hex(" version major", gpu_fw[5]); log_hex(" minor", gpu_fw[4]);
    log_hex(" hash", le32(gpu_fw + 8)); log_hex(" entries end at", end); log_flush();
    if (end > size) return 0;
    while (off + 4 <= end) {
        uint32_t e = le32(gpu_fw + off), type = e & 0xff, esize = (e >> 8) & 0xff;

        if (esize < 4 || off + esize > end) { log_hex("bad entry at", off); return 0; }
        if (type == 0 && esize >= 24) {
            uint32_t flags = le32(gpu_fw + off + 4), vs = le32(gpu_fw + off + 8), ve = le32(gpu_fw + off + 12);
            uint32_t ds = le32(gpu_fw + off + 16), de = le32(gpu_fw + off + 20);
            uint64_t attrs = (1ull << 6) | (1ull << 10) | (1ull << 11);
            uint8_t *mem;

            log_hex("section va", vs); log_hex(" to", ve); log_hex(" flags", flags); log_hex(" data bytes", de - ds);
            if (flags & (1u << 5)) { log_str(" protected mode: skipped\n"); off += esize; continue; }
            if (ve <= vs || ((vs | ve) & 4095u) || de < ds || de > size || de - ds > ve - vs) { stage(" bad section"); return 0; }
            mem = take(ve - vs);
            if (!mem) { stage(" no memory left for it"); return 0; }
            for (uint32_t i = 0; i < de - ds; i++) mem[i] = gpu_fw[ds + i];
            if (!(flags & (1u << 1))) attrs |= 1ull << 7;               /* not writable */
            if (!(flags & (1u << 2))) attrs |= 1ull << 54;              /* not executable */
            if ((flags & (3u << 3)) == (1u << 3)) attrs |= (1ull << 2) | (3ull << 8);       /* cached: index 1, inner shareable */
            else attrs |= 2ull << 8;                                    /* anything else not cached, as panthor maps it */
            for (uint32_t o = 0; o < ve - vs; o += 4096u)
                if (!map_page(vs + o, (uint64_t)(uintptr_t)mem + o, attrs)) { stage(" no table left"); return 0; }
            log_hex(" at", (uint64_t)(uintptr_t)mem); log_flush();
            if (flags & (1u << 30)) { shared = mem; shared_size = ve - vs; if (vs != 0x04000000u) { stage("the shared section is not at 0x04000000"); return 0; } }
        }
        off += esize;
    }
    log_hex("pool pages used", pool_used); log_hex("tables used", tables_used); log_flush();
    return shared != 0;
}

static void state(const char *when)
{
    log_str(when); log_str("\n");
    log_hex(" MCU_STATUS (0 off, 1 on, 2 halt, 3 fatal)", rd(MCU_STATUS)); log_hex(" GPU_STATUS", rd(GPU_STATUS));
    log_hex(" GPU_INT_RAWSTAT", rd(GPU_INT_RAWSTAT)); log_hex(" GPU_FAULT_STATUS", rd(GPU_FAULT_STATUS));
    log_hex(" JOB_INT_RAWSTAT (bit 31: global interface)", rd(JOB_INT_RAWSTAT)); log_hex(" MMU_INT_RAWSTAT", rd(MMU_INT_RAWSTAT));
    log_hex(" AS0 STATUS", rd(AS0 + 0x28)); log_hex(" AS0 FAULTSTATUS", rd(AS0 + 0x1c));
    log_hex(" AS0 FAULTADDRESS", (uint64_t)rd(AS0 + 0x24) << 32 | rd(AS0 + 0x20));
    log_hex(" SHADER_READY", rd(SHADER_READY)); log_hex(" L2_READY", rd(L2_READY)); log_flush();
}

static int wait_reg(uint32_t off, uint32_t mask, uint32_t want, uint32_t ms)
{
    uint64_t t0 = ticks();

    while (ticks() - t0 < (uint64_t)ms * freq / 1000u) if ((rd(off) & mask) == want) return 1;
    return (rd(off) & mask) == want;
}

void lynxld_main(uint64_t fdt, uint64_t self)
{
    (void)fdt; (void)self;
    uint32_t l2, va_bits, in_va, out_va;
    uint64_t t0;
    int booted = 0;

    __asm__ volatile("mrs %0,cntfrq_el0" : "=r"(freq));
    log_init(); stage("GPUMCU v1");
    __asm__ volatile("msr vbar_el2, %0; isb" :: "r"((uint64_t)(uintptr_t)probe_vectors));
    if (!(*(volatile uint32_t *)(PMU + 0x1E04) & 1u) || !(*(volatile uint32_t *)(PMU + 0x2004) & 1u)) { stage("a GPU power domain is off"); goto end; }
    log_hex("GPU_ID", rd(0)); log_hex("GPU_MMU_FEATURES", rd(0x14)); log_flush();
    va_bits = rd(0x14) & 0xff;
    if (rd(0) >> 28 != 10 || va_bits != 48) { stage("not the GPU this was written for"); goto end; }

    wr(GPU_INT_MASK, 0); wr(JOB_INT_MASK, 0); wr(MMU_INT_MASK, 0);
    wr(GPU_INT_CLEAR, 0xffffffffu);
    wr(GPU_CMD, 0x101);                                         /* GPU_SOFT_RESET */
    if (!wait_reg(GPU_INT_RAWSTAT, 1u << 8, 1u << 8, 200)) { stage("the GPU did not reset"); goto end; }
    wr(GPU_INT_CLEAR, 0xffffffffu); wr(JOB_INT_CLEAR, 0xffffffffu); wr(MMU_INT_CLEAR, 0xffffffffu);
    wr(COHERENCY_ENABLE, 31);
    l2 = rd(L2_PRESENT);
    wr(L2_PWRON, l2);
    if (!wait_reg(L2_READY, l2, l2, 200)) { stage("the L2 did not power on"); goto off; }
    stage("reset, L2 on");

    if (!load_firmware()) goto off;

    /* address space 0: panthor_vm_active() -> panthor_mmu_as_enable(). MEMATTR is mair_to_memattr() of io-pgtable's
     * MAIR (0x44 not cached, 0xff cached, 0x04 device, 0xf4): 0x4c, 0x9f, 0x4c, 0x9c, and 0x4c for the unused four. */
    wr(AS0 + 0x00, (uint32_t)(uintptr_t)tables[0]); wr(AS0 + 0x04, (uint32_t)((uint64_t)(uintptr_t)tables[0] >> 32));
    wr(AS0 + 0x08, 0x9c4c9f4cu); wr(AS0 + 0x0c, 0x4c4c4c4cu);
    wr(AS0 + 0x30, 6u | ((55u - va_bits) << 6) | (2u << 24) | (1u << 30)); wr(AS0 + 0x34, 0);
    if (!wait_reg(AS0 + 0x28, 1u, 0, 100)) { stage("address space 0 is busy"); goto off; }
    wr(AS0 + 0x18, 1);                                          /* AS_COMMAND_UPDATE */
    if (!wait_reg(AS0 + 0x28, 1u, 0, 100)) { stage("address space 0 did not take the update"); goto off; }
    state("address space 0 set");

    /* panthor_fw_start() */
    t0 = ticks();
    wr(MCU_CONTROL, 2);                                         /* MCU_CONTROL_AUTO */
    while (ticks() - t0 < freq) {                               /* one second, as panthor waits */
        if ((rd(JOB_INT_RAWSTAT) & (1u << 31)) && rd(MCU_STATUS) == 1) { booted = 1; break; }
        if (rd(MCU_STATUS) == 3 || faults) break;
    }
    log_hex("MCU start: us", (ticks() - t0) * 1000000u / freq);
    state("after MCU_CONTROL = AUTO");
    if (!booted) { stage("THE MCU DID NOT BOOT"); goto stop; }

    /* struct panthor_fw_global_control_iface at the shared section's start */
    log_hex("GLB version", le32(shared + 0)); log_hex("GLB features", le32(shared + 4));
    in_va = le32(shared + 8); out_va = le32(shared + 12);
    log_hex("GLB input_va", in_va); log_hex("GLB output_va", out_va); log_hex("GLB group_num", le32(shared + 16));
    log_hex("GLB group_stride", le32(shared + 20)); log_hex("GLB instr_features", le32(shared + 28)); log_flush();
    stage("MCU BOOTED: the firmware runs");
    wr(JOB_INT_CLEAR, 1u << 31);

    if (in_va >= 0x04000000u && out_va >= 0x04000000u && in_va + 4 <= 0x04000000u + shared_size && out_va + 4 <= 0x04000000u + shared_size) {
        volatile uint32_t *req = (volatile uint32_t *)(shared + (in_va - 0x04000000u));
        volatile uint32_t *ack = (volatile uint32_t *)(shared + (out_va - 0x04000000u));
        uint32_t want;

        log_hex("GLB_REQ", *req); log_hex("GLB_ACK", *ack);
        want = (*req & ~(1u << 8)) | (~*ack & (1u << 8));      /* GLB_PING toggled against the acknowledge */
        *req = want;
        __asm__ volatile("dsb sy" ::: "memory");
        t0 = ticks();
        wr(DOORBELL0, 1);
        while (ticks() - t0 < freq / 2 && ((*ack ^ want) & (1u << 8))) { }
        log_hex("ping: us", (ticks() - t0) * 1000000u / freq); log_hex(" GLB_REQ", *req); log_hex(" GLB_ACK", *ack);
        log_hex(" JOB_INT_RAWSTAT", rd(JOB_INT_RAWSTAT)); log_flush();
        if (!((*ack ^ want) & (1u << 8))) stage("PING ANSWERED: the firmware listens to its interface");
        else stage("no answer to the ping");
    }
    state("at the end");
stop:
    wr(MCU_CONTROL, 0);
    wait_reg(MCU_STATUS, 0xffffffffu, 0, 200);
    log_hex("MCU stopped: MCU_STATUS", rd(MCU_STATUS));
off:
    wr(L2_PWROFF, rd(L2_PRESENT)); wait_us(1000);
    wr(GPU_INT_CLEAR, 0xffffffffu);
    wr(GPU_CMD, 0x101);
    wait_reg(GPU_INT_RAWSTAT, 1u << 8, 1u << 8, 200);
    wr(GPU_INT_CLEAR, 0xffffffffu); wr(JOB_INT_CLEAR, 0xffffffffu); wr(MMU_INT_CLEAR, 0xffffffffu);
end:
    log_hex("exceptions taken", faults);
    stage("GPUMCU-END");
    for (;;) __asm__ volatile("wfe");
}
void lynxld_returned(void) {}
