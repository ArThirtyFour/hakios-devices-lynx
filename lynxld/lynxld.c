/*
 * lynxld — HakiOS XNU loader for the Pixel 7a (lynx, gs201).
 *
 * Entered by ABL as a Linux arm64 Image (x0 = FDT, EL2, MMU off; proven on hardware), from `fastboot boot`:
 * nothing on the phone is written. In order:
 *   - brings up the USB gadget (usb/, 18d1:d00d) and takes the Darwin system image from the computer
 *     (tools/lynxrun) into RAM, feeding the bootloader's watchdogs meanwhile;
 *   - verifies the embedded LYNXPAY2 payload (XNU as GSPE + RAM disk + command line), lays the kernel out in the
 *     high DRAM bank, builds the Apple device tree and boot_args the way the upstream HakiOS xnuloader does;
 *   - keeps the panel the bootloader lit running (DECON0 trigger) on its own scanout buffer;
 *   - switches the watchdogs off for the kernel's run, starts the touch screen (touch_up.c), raises the CPU
 *     clusters' clocks through ACPM DVFS, and with the phone's firmware at hand starts Bluetooth (bt_up.c);
 *   - starts the kernel on a Cortex-X1 (PSCI CPU_ON, lynx_other_cpu_entry in start.S) in EL1.
 * The kernel's way back (XNULDRST), if it is asked for one (loader_return_after), lands in lynx_return_stub: the
 * kernel log goes into ramoops and over USB, the watchdogs come back on and reset the phone into Android.
 * Everything the loader says is in ramoops (log.c): /sys/fs/pstore/console-ramoops-0 after the next Android boot.
 */
#include <stdint.h>
#include <stddef.h>
#include "log.h"
#include "gspe-plan.h"
#include "usb/usbgadget.h"
#include "identity.h"   /* HakiOS xnuloader/identity.h: made from your own IPSW (xnuloader/mkidentity.sh); build.sh adds its directory */

#define LOADER_OFF       0x10000ull
#define PAYLOAD_OFF      0x200000ull
#define KVB              0xfffffff004000000ull
#define PHYS_BASE        0x880000000ull     /* DRAM bank 2: no static reservations in the lynx FDT */
#define WIN_SIZE         0x170000000ull     /* banks 2-4 (0x880000000-0xa00000000, no reservations), minus the framebuffer */
#define RESIDENT_OFF     0x08000000ull      /* the system image (md0), as upstream RESIDENT_RD_OFFSET */
#define FB_PA            0xfac00000ull      /* the panel's own scanout buffer (DPP0 DMA +0x40), live once DECON0's trigger is unmasked */
#define FB_W 1080u
#define FB_H 2400u
#define DECON0           0x1c240000ull
#define FB_STRIDE (FB_W * 4u)
#define POOL_PA  0xf8cdc000ull
#define POOL_LEN 0x8000u
#define HDR_PA   (POOL_PA + 0x4000u)
#define CHUNK    0x800000u
#define WDT0 0x10060000ull
#define WDT1 0x10070000ull
/* Seconds the kernel runs before it hands the phone back (loader_return_after); 0: never, the long power key ends the
 * run. build.sh: RETURN_AFTER=<seconds> for a test image that comes back to Android with its log by itself. */
#ifndef RETURN_AFTER_SECONDS
#define RETURN_AFTER_SECONDS 0
#endif
#define RETURN_STR2(x) #x
#define RETURN_STR(x) RETURN_STR2(x)
#define RETURN_AFTER RETURN_STR(RETURN_AFTER_SECONDS)
#define PAGE16K          0x4000ull
#define PRAM_BASE        0xfd600000ull      /* inside the reserved ramoops pmsg zone */
#define PRAM_SIZE        0x80000u
#define NVRAM_BANK_SIZE  0x2000u
#define BOOT_LINE_LENGTH 608
#define GIC_IO_BASE      0x10000000ull
#define GIC_IO_SIZE      0x10000000ull
#define GICD_OFF         0x00400000ull      /* 0x10400000, lynx FDT */
#define GICR_OFF         0x00440000ull      /* 0x10440000, lynx FDT */

void lynx_enter_el1(uint64_t entry_pa, uint64_t boot_args_pa);
void lynx_return_stub(void);
void lynx_other_cpu_entry(void);
int lynx_touch_up(void);
void lynx_cpu_fast(void);
#ifdef LYNX_BT          /* build.sh: the phone's Bluetooth firmware is in ../fw-lynx */
int lynx_bt_up(const uint8_t *bd_addr);
#define BT_ARG "lynx_bt=1 "
#else
#define BT_ARG ""
#endif
#define BOOT_MPIDR 0x700u   /* cpu@700: Cortex-X1, capacity 1024 (A55: 250) in the lynx FDT */
static volatile uint64_t other_go[2] __attribute__((aligned(64)));
static int64_t psci(uint64_t fn, uint64_t a1, uint64_t a2, uint64_t a3)
{
    register uint64_t x0 __asm__("x0") = fn, x1 __asm__("x1") = a1, x2 __asm__("x2") = a2, x3 __asm__("x3") = a3;
    __asm__ volatile("smc #0" : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3) :: "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
    return (int64_t)x0;
}

void *memset(void *d, int c, size_t n)
{
    uint8_t *p = d;
    if (!c && !((uintptr_t)p & 7)) {
        for (; n >= 8; n -= 8, p += 8) *(volatile uint64_t *)p = 0;
    }
    while (n--) *p++ = (uint8_t)c;
    return d;
}
void *memcpy(void *d, const void *s, size_t n)
{
    uint8_t *a = d; const uint8_t *b = s;
    if (!(((uintptr_t)a | (uintptr_t)b) & 7)) {
        for (; n >= 8; n -= 8, a += 8, b += 8) *(volatile uint64_t *)a = *(const volatile uint64_t *)b;
    }
    while (n--) *a++ = *b++;
    return d;
}
static size_t str_len(const char *s) { size_t n = 0; while (s[n]) n++; return n; }

struct boot_video { uint64_t v_baseAddr, v_display, v_rowBytes, v_width, v_height, v_depth; };
struct boot_args {
    uint16_t revision, version;
    uint64_t virtBase, physBase, memSize, topOfKernelData;
    struct boot_video video;
    uint32_t machineType;
    uint64_t deviceTreeP;
    uint32_t deviceTreeLength;
    char commandLine[BOOT_LINE_LENGTH];
    uint64_t bootFlags, memSizeActual;
};

/* ---- Apple device tree writer (upstream xnuloader) ---- */
struct dt_writer { uint8_t *base, *cur, *end; int overflow; uint32_t scratch[2]; uint32_t *counts[16]; int depth; };
static void dt_prop(struct dt_writer *w, const char *name, const void *data, uint32_t len)
{
    uint64_t padded = ((uint64_t)len + 3) & ~3ull;
    if (w->overflow || str_len(name) > 32 || (uint64_t)(w->end - w->cur) < 36 + padded) { w->overflow = 1; return; }
    memset(w->cur, 0, 32);
    memcpy(w->cur, name, str_len(name));
    *(volatile uint32_t *)(w->cur + 32) = len;
    memset(w->cur + 36, 0, (size_t)padded);
    memcpy(w->cur + 36, data, len);
    w->cur += 36 + padded;
    w->counts[w->depth - 1][0]++;
}
static void dt_str(struct dt_writer *w, const char *n, const char *v) { dt_prop(w, n, v, (uint32_t)str_len(v) + 1); }
static void dt_u32(struct dt_writer *w, const char *n, uint32_t v) { dt_prop(w, n, &v, 4); }
static void dt_u64(struct dt_writer *w, const char *n, uint64_t v) { dt_prop(w, n, &v, 8); }
static void dt_padded(struct dt_writer *w, const char *n, const char *v, uint32_t size)
{
    uint8_t f[64]; memset(f, 0, sizeof(f)); memcpy(f, v, str_len(v)); dt_prop(w, n, f, size);
}
static void dt_identity(struct dt_writer *w, const struct identity_prop *p, uint32_t count, const unsigned char *data)
{
    for (uint32_t i = 0; i < count; i++) dt_prop(w, p[i].name, data + p[i].offset, p[i].length);
}
static void dt_begin(struct dt_writer *w, const char *name)
{
    if (w->depth > 0) w->counts[w->depth - 1][1]++;
    if (w->overflow || (uint64_t)(w->end - w->cur) < 8) { w->overflow = 1; w->counts[w->depth++] = w->scratch; return; }
    w->counts[w->depth++] = (uint32_t *)w->cur;
    memset(w->cur, 0, 8);
    w->cur += 8;
    dt_str(w, "name", name);
}
static void dt_end(struct dt_writer *w) { w->depth--; }

static void nvram_chrp_header(uint8_t *h, uint8_t sig, uint32_t bytes, const char *name)
{
    uint16_t blocks = (uint16_t)(bytes / 16), sum;
    memset(h, 0, 16);
    h[0] = sig; h[2] = (uint8_t)blocks; h[3] = (uint8_t)(blocks >> 8);
    memcpy(h + 4, name, str_len(name));
    sum = h[0];
    for (int i = 2; i < 16; i++) sum += h[i];
    while (sum > 0xff) sum = (sum & 0xff) + (sum >> 8);
    h[1] = (uint8_t)sum;
}
static void build_nvram_image(uint8_t *image)
{
    uint32_t lo = 1, hi = 0, generation = 1;
    memset(image, 0, NVRAM_BANK_SIZE);
    nvram_chrp_header(image, 0x5a, 0x20, "nvram");
    nvram_chrp_header(image + 0x20, 0x70, NVRAM_BANK_SIZE - 0x20, "common");
    memcpy(image + 20, &generation, 4);
    for (uint32_t i = 20; i < NVRAM_BANK_SIZE; i++) { lo = (lo + image[i]) % 65521; hi = (hi + lo) % 65521; }
    uint32_t adler = (hi << 16) | lo;
    memcpy(image + 16, &adler, 4);
}

/* ---- payload ---- */
struct pay2 { char magic[8]; uint32_t gspe_len, gspe_crc, rd_len, rd_crc, cmd_len, pad; };
static uint32_t crc32(const uint8_t *p, uint64_t n)
{
    uint32_t c = 0xffffffffu;
    for (uint64_t i = 0; i < n; i++) { c ^= p[i]; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1))); }
    return ~c;
}
static uint64_t al16(uint64_t v) { return (v + 15) & ~15ull; }
static uint64_t al16k(uint64_t v) { return (v + PAGE16K - 1) & ~(PAGE16K - 1); }

static void pet(void);
static void dcache_civac(uint64_t lo, uint64_t hi)
{
    for (uint64_t a = lo & ~63ull; a < hi; a += 64) { __asm__ volatile("dc civac, %0" :: "r"(a) : "memory"); if (!(a & 0xffffff)) pet(); }
    __asm__ volatile("dsb sy" ::: "memory");
}

static void fail(const char *why) { log_str("ERR "); log_str(why); log_str("\n"); log_flush(); }

int try_read32(uintptr_t a, uint32_t *v) { *v = *(volatile uint32_t *)a; return 1; }
int try_write32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; return 1; }
static uint64_t ticks(void) { uint64_t v; __asm__ volatile("isb; mrs %0, cntpct_el0" : "=r"(v)); return v; }
static uint64_t freq(void) { uint64_t v; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v)); return v; }
static uint32_t rd32(uint64_t a) { return *(volatile uint32_t *)a; }
static void wr32(uint64_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static uint32_t wdt_saved[2];
static void pet(void)
{ if (rd32(WDT0) & 32u) wr32(WDT0 + 8, rd32(WDT0 + 4)); if (rd32(WDT1) & 32u) wr32(WDT1 + 8, rd32(WDT1 + 4)); }
static uint64_t lastpet;
static void service(void) { usbgadget_poll(); if (ticks() - lastpet > freq() / 4) { pet(); lastpet = ticks(); } }
static int usb_up(void)
{
    static struct usbgadget_hooks h;   /* the gadget keeps this pointer: it must outlive usb_up() */
    h.ticks = ticks; h.ticks_freq = freq();
    for (uint64_t i = 0; i < POOL_LEN; i += 8) *(volatile uint64_t *)(POOL_PA + i) = 0;
    if (usbgadget_init(POOL_PA, POOL_LEN, &h)) return -1;
    uint64_t t0 = ticks();
    while (!usbgadget_ready()) { service(); if (ticks() - t0 > 60 * freq()) return -1; }
    return 0;
}
static int send(const void *b, uint64_t n)
{
    const uint8_t *p = b;
    while (n) {
        size_t k = n > 4096 ? 4096 : (size_t)n;
        uint64_t t = ticks();
        while (!usbgadget_write((const char *)p, k)) { service(); if (ticks() - t > 5 * freq()) return -1; }
        p += k; n -= k;
    }
    return 0;
}
static int wait_read(uint32_t *got, uint64_t secs) { uint64_t t = ticks(); while (!usbgadget_read_done(got)) { service(); if (ticks() - t > secs * freq()) return -1; } return 0; }
static void put_u64(char *p, uint64_t v) { for (int i = 15; i >= 0; i--) { int d = (v >> (i * 4)) & 15; *p++ = (char)(d < 10 ? '0' + d : 'a' + d - 10); } }
/* the system image from the host: LYNXDL01 header (len, crc), then the bytes straight into the window */
static uint64_t receive_system(uint64_t dest)
{
    uint32_t got;
    if (send("LYNXDL READY\r\n", 14)) return 0;
    int rs = usbgadget_read_start(HDR_PA, 512);
    log_hex("hdr_arm", (uint64_t)(int64_t)rs); log_flush();
    if (rs) return 0;
    {   /* repeat READY every 2 s until the header arrives: the host may open the device late */
        uint64_t t0 = ticks(), last = t0;
        while (!usbgadget_read_done(&got)) {
            service();
            if (ticks() - last > 2 * freq()) { usbgadget_write("LYNXDL READY\r\n", 14); last = ticks(); }
            if (ticks() - t0 > 90 * freq()) { log_str("hdr timeout\n"); log_flush(); return 0; }
        }
    }
    log_hex("hdr_got", got); log_flush();
    const volatile uint8_t *hd = (const volatile uint8_t *)HDR_PA;
    const char m[8] = { 'L','Y','N','X','D','L','0','1' };
    for (int i = 0; i < 8; i++) if (hd[i] != (uint8_t)m[i]) return 0;
    uint64_t len = 0;
    for (int i = 0; i < 8; i++) len |= (uint64_t)hd[8 + i] << (8 * i);
    /* Leave room for the device tree, boot arguments and the kernel's bootstrap allocations. */
    if (!len || len > WIN_SIZE - RESIDENT_OFF - 0x08000000ull) return 0;
    uint64_t off = 0;
    while (off < len) {
        uint64_t rem = len - off; uint32_t n = rem > CHUNK ? CHUNK : (uint32_t)((rem + 511) & ~511ull);
        if (usbgadget_read_start(dest + off, n) || wait_read(&got, 30)) return 0;
        off += got;
        if (got < n) break;
    }
    return off >= len ? len : 0;
}

/*
 * The phone's own addresses, from the device tree the bootloader gave us: /chosen/config has "wlan_mac1" and
 * "bt_addr" as text ("5C:33:7B:..:..:..", what Android reads too). 1 with the six bytes, 0 if the tree has none
 * (the caller then keeps a made-up, locally administered address).
 */
static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static int hexval(uint8_t c) { return c >= '0' && c <= '9' ? c - '0' : (c | 32) >= 'a' && (c | 32) <= 'f' ? (c | 32) - 'a' + 10 : -1; }
static int name_is(const uint8_t *a, uint32_t room, const char *b) { while (*b) { if (!room || *a++ != (uint8_t)*b++) return 0; room--; } return room && *a == 0; }

static int fdt_mac(uint64_t fdt, const char *property, uint8_t out[6])
{
    const uint8_t *d = (const uint8_t *)(uintptr_t)fdt;
    uint32_t total, at, strings, depth = 0, chosen = 0, config = 0;

    if (fdt == 0 || (fdt & 3) || be32(d) != 0xd00dfeedu) return 0;
    total = be32(d + 4); at = be32(d + 8); strings = be32(d + 12);
    if (total < 40 || total > 0x400000 || at >= total || strings >= total) return 0;
    while (at + 4 <= total) {
        uint32_t token = be32(d + at);

        at += 4;
        if (token == 1) {                               /* a node: its name, padded */
            const uint8_t *name = d + at;

            depth++;
            if (depth == 2) chosen = name_is(name, total - at, "chosen");
            if (depth == 3) config = chosen && name_is(name, total - at, "config");
            while (at < total && d[at]) at++;
            if (at == total) return 0;
            at = (at + 4) & ~3u;
        } else if (token == 2) {
            if (depth == 3) config = 0;
            if (depth == 2) chosen = 0;
            if (depth-- == 0) return 0;
        } else if (token == 3) {                        /* a property: length, name offset, value */
            if (total - at < 8) return 0;
            uint32_t length = be32(d + at), offset = be32(d + at + 4);
            if (length > total - at - 8 || offset >= total - strings) return 0;
            uint32_t name = strings + offset;
            const uint8_t *v = d + at + 8;

            if (depth == 3 && config && length >= 17 && name_is(d + name, total - name, property)) {
                for (int i = 0; i < 6; i++) {
                    int h = hexval(v[3 * i]), l = hexval(v[3 * i + 1]);

                    if (h < 0 || l < 0 || (i < 5 && v[3 * i + 2] != ':')) return 0;
                    out[i] = (uint8_t)(h << 4 | l);
                }
                return (out[0] | out[1] | out[2] | out[3] | out[4] | out[5]) != 0 && !(out[0] & 1);
            }
            at = (at + 8 + length + 3) & ~3u;
        } else if (token != 4) {
            return 0;                                   /* the end (9), or not a tree */
        }
    }
    return 0;
}

void lynxld_main(uint64_t fdt, uint64_t self)
{
    uint64_t base = self - LOADER_OFF, mpidr, cntfrq;
    log_init();
    log_str("LYNXLD v21\n");
    log_hex("fdt", fdt); log_hex("imgbase", base);
    log_flush();

    /* -- payload -- */
    const struct pay2 *p = (const struct pay2 *)(base + PAYLOAD_OFF);
    const char m[8] = { 'L','Y','N','X','P','A','Y','2' };
    for (int i = 0; i < 8; i++) if (p->magic[i] != m[i]) { fail("payload magic"); return; }
    const uint8_t *g = (const uint8_t *)(p + 1);
    const uint8_t *rd = g + al16(p->gspe_len);
    const char *cmd = (const char *)(rd + al16(p->rd_len));
    if (p->cmd_len == 0 || p->cmd_len > 400 || cmd[p->cmd_len - 1] != 0) { fail("cmdline"); return; }
    if (crc32(g, p->gspe_len) != p->gspe_crc) { fail("gspe crc"); return; }
    if (crc32(rd, p->rd_len) != p->rd_crc) { fail("ramdisk crc"); return; }
    struct lynx_gspe_plan plan;
    if (lynx_gspe_validate(g, p->gspe_len, &plan) || plan.virtual_base != KVB) { fail("gspe invalid"); return; }
    log_str("payload ok\n");
    log_flush();
    pet(); lastpet = ticks();
    if (usb_up()) { fail("usb"); return; }
    log_str("usb up, waiting for the system image\n"); log_flush();
    uint64_t sys_pa = PHYS_BASE + RESIDENT_OFF;
    uint64_t sys_len = receive_system(sys_pa);
    if (!sys_len) { fail("system image download"); usbgadget_stop(); return; }
    if (rd32(sys_pa + 32) != 0x4253584eu) { fail("system image: no APFS NXSB"); usbgadget_stop(); return; }
    log_hex("system_pa", sys_pa); log_hex("system_len", sys_len); log_flush();
    send("SYSTEM OK\r\n", 11);

    /* -- layout -- */
    uint64_t slide = PHYS_BASE - KVB;               /* pa = va + slide */
    uint64_t kern_hi = al16k(KVB + plan.span + slide);
    uint64_t rd_pa = kern_hi, rd_size = al16k(p->rd_len);
    uint64_t sys_size = al16k(sys_len);
    if (rd_pa + rd_size > sys_pa) { fail("kernel overlaps system image"); return; }
    uint64_t dt_pa = sys_pa + sys_size, dt_max = 0x10000;
    uint64_t ba_pa = dt_pa + dt_max, top = ba_pa + PAGE16K;
    if (top - PHYS_BASE + 0x4000000ull > WIN_SIZE) { fail("window too small"); return; }

    /* -- segments -- */
    const uint8_t *kfile = g + 48 + (uint64_t)plan.segment_count * 32;
    for (uint32_t i = 0; i < plan.segment_count; i++) {
        const uint8_t *s = g + 48 + (uint64_t)i * 32;
        uint64_t vm, vsize, off, fsize;
        memcpy(&vm, s, 8); memcpy(&vsize, s + 8, 8); memcpy(&off, s + 16, 8); memcpy(&fsize, s + 24, 8);
        if (!vsize) continue;
        for (uint64_t o = 0; o < vsize; o += 0x100000) { uint64_t k = vsize - o < 0x100000 ? vsize - o : 0x100000; memset((void *)(uintptr_t)(vm + slide + o), 0, k); pet(); }
        for (uint64_t o = 0; o < fsize; o += 0x100000) { uint64_t k = fsize - o < 0x100000 ? fsize - o : 0x100000; memcpy((void *)(uintptr_t)(vm + slide + o), kfile + off + o, k); pet(); }
    }
    if (*(volatile uint32_t *)(uintptr_t)(KVB + 0x3004000ull + slide) != 0xfeedfacfu) { fail("text magic"); return; }
    log_hex("kernel_pa_lo", KVB + 0x3004000ull + slide); log_hex("kernel_pa_hi", kern_hi);
    log_flush();

    log_str("segments placed\n"); log_flush();
    memset((void *)(uintptr_t)rd_pa, 0, rd_size);
    memcpy((void *)(uintptr_t)rd_pa, rd, p->rd_len);
    log_str("md1 placed\n"); log_flush(); pet();

    /* -- return slot: checkpoint 0 = run the whole way -- */
    volatile uint64_t *slot = (volatile uint64_t *)(uintptr_t)(plan.stub + slide);
    if (!plan.stub || slot[0] != 0x545352444c554e58ull) { fail("XNULDRST slot"); return; }
    slot[1] = (uint64_t)(uintptr_t)&lynx_return_stub;
    slot[2] = 0;
    slot[3] = slide;
    log_hex("return_stub", slot[1]); log_flush();

    /* -- boot_args -- */
    struct boot_args *ba = (struct boot_args *)(uintptr_t)ba_pa;
    memset(ba, 0, sizeof(*ba));
    ba->revision = 2; ba->version = 2;
    ba->virtBase = KVB; ba->physBase = PHYS_BASE;
    ba->memSize = WIN_SIZE; ba->memSizeActual = WIN_SIZE;
    ba->topOfKernelData = top;
    for (uint64_t i = 0; i < (uint64_t)FB_STRIDE * FB_H; i += 8) { *(volatile uint64_t *)(FB_PA + i) = 0; if (!(i & 0xfffff)) pet(); }
    ba->video.v_baseAddr = FB_PA; ba->video.v_display = 1; ba->video.v_rowBytes = FB_STRIDE;
    ba->video.v_width = FB_W; ba->video.v_height = FB_H; ba->video.v_depth = 32;
    log_str("framebuffer cleared\n"); log_flush();
    uint32_t n = 0;
    for (const char *c = "debug=0x8 cpus=7 rd=md0 lynx_touch=1 lynx_battery=1 " BT_ARG "loader_return_after=" RETURN_AFTER " "; *c; c++) ba->commandLine[n++] = *c;
    for (const char *c = cmd; *c && n < BOOT_LINE_LENGTH - 1; c++) ba->commandLine[n++] = *c;

    /* -- device tree -- */
    __asm__ volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(cntfrq));
    struct dt_writer w = { .base = (uint8_t *)(uintptr_t)dt_pa, .cur = (uint8_t *)(uintptr_t)dt_pa,
                           .end = (uint8_t *)(uintptr_t)(dt_pa + dt_max) };
    uint8_t seed[64];
    for (unsigned i = 0; i < sizeof(seed); i++) seed[i] = (uint8_t)(i * 37 + (self >> (i & 7)));
    /* the phone's own addresses when the bootloader's tree has them; made-up, locally administered ones otherwise */
    static uint8_t wifi[6] = { 0x02, 0x4c, 0x59, 0x4e, 0x00, 0x01 }, bt[6] = { 0x02, 0x4c, 0x59, 0x4e, 0x00, 0x02 };
    uint8_t mac[6];
    if (fdt_mac(fdt, "wlan_mac1", mac)) for (int i = 0; i < 6; i++) wifi[i] = mac[i];
    if (fdt_mac(fdt, "bt_addr", mac)) for (int i = 0; i < 6; i++) bt[i] = mac[i];
    log_hex("wifi_mac", (uint64_t)wifi[0] << 40 | (uint64_t)wifi[1] << 32 | (uint64_t)wifi[2] << 24 | wifi[3] << 16 | wifi[4] << 8 | wifi[5]);
    log_hex("bt_mac", (uint64_t)bt[0] << 40 | (uint64_t)bt[1] << 32 | (uint64_t)bt[2] << 24 | bt[3] << 16 | bt[4] << 8 | bt[5]);
    static const uint8_t hash[48] = { 'l', 'y', 'n', 'x' }, behavior[16] = { 0 };
    uint8_t nvram[NVRAM_BANK_SIZE];
    build_nvram_image(nvram);

    dt_begin(&w, "device-tree");
    dt_identity(&w, identity_root, sizeof(identity_root) / sizeof(identity_root[0]), identity_root_data);
    dt_str(&w, "platform-name", "t8015");
    dt_padded(&w, "serial-number", "LYNX00PIXEL7A", 32);
    dt_padded(&w, "mlb-serial-number", "LYNX00PIXEL7AMLB00", 32);
    dt_padded(&w, "model-number", "MQAC2", 32);
    dt_padded(&w, "region-info", "LL/A", 32);
    dt_padded(&w, "regulatory-model-number", "A1865", 32);
    dt_u32(&w, "#address-cells", 2);
    dt_u32(&w, "#size-cells", 2);
    dt_begin(&w, "chosen");
    dt_prop(&w, "random-seed", seed, sizeof(seed));
    dt_u64(&w, "dram-base", PHYS_BASE);
    dt_u64(&w, "dram-size", WIN_SIZE);
    dt_u32(&w, "debug-enabled", 1);
    dt_u32(&w, "display-scale", 3);
    dt_u32(&w, "display-rotation", 0);
    dt_str(&w, "psci-method", "smc");
    dt_u64(&w, "unique-chip-id", 0x00004c594e580001ull);
    dt_u32(&w, "chip-id", 0x8015);
    dt_u32(&w, "board-id", 0x06);
    dt_prop(&w, "mac-address-wifi0", wifi, 6);
    dt_prop(&w, "mac-address-bluetooth0", bt, 6);
    dt_prop(&w, "boot-manifest-hash", hash, 48);
    dt_prop(&w, "software-behavior", behavior, 16);
    dt_str(&w, "firmware-version", "iBoot-8422.100.650");
    dt_u32(&w, "certificate-production-status", 1);
    dt_u32(&w, "certificate-security-mode", 1);
    dt_u32(&w, "effective-production-status-ap", 1);
    dt_u32(&w, "effective-security-mode-ap", 1);
    dt_u32(&w, "secure-boot", 1);
    dt_u32(&w, "production-cert", 1);
    dt_u32(&w, "development-cert", 0);
    dt_u32(&w, "embedded-panic-log-size", PRAM_SIZE);
    dt_u32(&w, "nvram-bank-size", NVRAM_BANK_SIZE);
    dt_u32(&w, "nvram-bank-count", 1);
    dt_u32(&w, "nvram-current-bank", 0);
    dt_prop(&w, "nvram-proxy-data", nvram, sizeof(nvram));
    dt_begin(&w, "memory-map");
    uint64_t sysreg[2] = { sys_pa, sys_size }, rdreg[2] = { rd_pa, rd_size };
    dt_prop(&w, "RAMDisk", sysreg, sizeof(sysreg));      /* md0: the system, the root */
    dt_prop(&w, "RAMDisk1", rdreg, sizeof(rdreg));       /* md1: root.apfs, mounted on /redwood */
    dt_end(&w);
    dt_end(&w);
    dt_begin(&w, "defaults"); dt_u32(&w, "kern.vm_compressor", 2); dt_end(&w);
    dt_begin(&w, "product");
    dt_identity(&w, identity_product, sizeof(identity_product) / sizeof(identity_product[0]), identity_product_data);
    dt_begin(&w, "haptics");
    dt_identity(&w, identity_haptics, sizeof(identity_haptics) / sizeof(identity_haptics[0]), identity_haptics_data);
    dt_end(&w);
    dt_end(&w);
    dt_begin(&w, "options"); dt_end(&w);
    memset((void *)(uintptr_t)PRAM_BASE, 0, PRAM_SIZE);
    uint64_t pram[2] = { PRAM_BASE, PRAM_SIZE };
    dt_begin(&w, "pram"); dt_prop(&w, "reg", pram, sizeof(pram)); dt_end(&w);
    dt_begin(&w, "cpus");
    /* the boot core (X1 0x700), then the other X1 and the two A78: each only if the firmware says it is off */
    static const uint32_t cores[7] = { BOOT_MPIDR, 0x600u, 0x500u, 0x400u, 0x100u, 0x200u, 0x300u };   /* core 0 is the loader's */
    uint32_t ncpu = 0;
    for (int i = 0; i < 7; i++) {
        char name[5] = { 'c', 'p', 'u', (char)('0' + ncpu), 0 };
        int64_t off = psci(0xC4000004u, cores[i], 0, 0);
        log_hex("core", cores[i]); log_hex(" affinity", (uint64_t)off);
        if (off != 1) continue;
        dt_begin(&w, name);
        dt_str(&w, "device_type", "cpu");
        dt_str(&w, "state", ncpu == 0 ? "running" : "waiting");
        dt_u32(&w, "reg", cores[i]);
        dt_u32(&w, "cpu-id", ncpu);
        dt_u64(&w, "timebase-frequency", cntfrq);
        dt_u64(&w, "fixed-frequency", cntfrq);
        dt_u64(&w, "clock-frequency", i < 2 ? 2048000000ull : i < 4 ? 1663000000ull : 1197000000ull);
        dt_u64(&w, "bus-frequency", 100000000ull);
        dt_u64(&w, "memory-frequency", 100000000ull);
        dt_u64(&w, "peripheral-frequency", 24000000ull);
        if (i >= 4) dt_u32(&w, "small-core", 1);            /* A55: the kernel starts these later */
        dt_end(&w);
        ncpu++;
    }
    log_hex("cpus for the kernel", ncpu); log_flush();
    dt_end(&w);
    uint64_t ranges[3] = { 0, GIC_IO_BASE, GIC_IO_SIZE };
    dt_begin(&w, "arm-io");
    dt_str(&w, "device_type", "lynx");
    dt_prop(&w, "ranges", ranges, sizeof(ranges));
    dt_u32(&w, "#address-cells", 2);
    dt_u32(&w, "#size-cells", 2);
    dt_begin(&w, "gic");
    dt_str(&w, "device_type", "interrupt-controller");
    dt_str(&w, "compatible", "arm,gic-v3");
    uint64_t gic[4] = { GICD_OFF, 0x10000, GICR_OFF, 0x100000 };
    dt_prop(&w, "reg", gic, sizeof(gic));
    dt_end(&w);
    dt_end(&w);
    dt_end(&w);
    if (w.overflow) { fail("device tree overflow"); return; }
    log_str("device tree built\n"); log_flush();
    ba->deviceTreeP = dt_pa - slide;
    ba->deviceTreeLength = (uint32_t)(w.cur - w.base);

    log_hex("dt_pa", dt_pa); log_hex("dt_len", ba->deviceTreeLength);
    log_hex("ba_pa", ba_pa); log_hex("top", top);
    log_str("cmdline: "); log_str(ba->commandLine); log_str("\n");
    log_hex("entry_pa", plan.entry + slide);
    log_str("JUMP\n");
    log_flush();

    log_str("cache maintenance\n"); log_flush();
    /* written with the MMU off (non-cacheable): drop any stale lines before XNU turns its caches on */
    send("JUMP\r\n", 6);
    { uint64_t t = ticks(); while (ticks() - t < freq() / 2) service(); }
    usbgadget_stop();
    /* the prebuilt kernel cannot feed gs201's watchdogs: switch them off for the run (Linux s3c2410wdt_stop),
       switched back on after the kernel returns */
    wdt_saved[0] = rd32(WDT0); wdt_saved[1] = rd32(WDT1);
    wr32(WDT0, wdt_saved[0] & ~0x21u); wr32(WDT1, wdt_saved[1] & ~0x21u);
    log_hex("wdt0_off", rd32(WDT0)); log_flush();
    dcache_civac(PHYS_BASE, top);
    dcache_civac(FB_PA, FB_PA + (uint64_t)FB_STRIDE * FB_H);
    dcache_civac(PRAM_BASE, PRAM_BASE + PRAM_SIZE);
    /* the panel: shadow update request (GLOBAL|CMP) then unmask the TE trigger, Google's decon_reg_update_req_and_unmask();
       from here DECON0 sends the buffer to the panel on every TE, whatever writes it */
    wr32(DECON0 + 0x50, rd32(DECON0 + 0x50) | (1u << 31) | (1u << 20));
    { uint64_t t = ticks(); while (ticks() - t < freq() / 100) { } }
    wr32(DECON0 + 0x30, (rd32(DECON0 + 0x30) & ~(1u << 4)) | 1u);
    log_hex("decon_trig", rd32(DECON0 + 0x30)); log_flush();
    log_hex("touch_up", (uint64_t)lynx_touch_up()); log_flush();
    lynx_cpu_fast();
#ifdef LYNX_BT
    {
        static uint8_t bt_mac[6];

        log_hex("bt_up", (uint64_t)lynx_bt_up(fdt_mac(fdt, "bt_addr", bt_mac) ? bt_mac : 0)); log_flush();
    }
#endif
    (void)mpidr;
    other_go[0] = plan.entry + slide;
    other_go[1] = ba_pa;
    dcache_civac((uint64_t)(uintptr_t)other_go, (uint64_t)(uintptr_t)other_go + 64);
    int64_t aff = psci(0xC4000004u, BOOT_MPIDR, 0, 0);          /* AFFINITY_INFO: 1 = off */
    log_hex("core7_affinity", (uint64_t)aff);
    int64_t r = psci(0xC4000003u, BOOT_MPIDR, (uint64_t)(uintptr_t)&lynx_other_cpu_entry, (uint64_t)(uintptr_t)other_go);
    log_hex("cpu_on", (uint64_t)r); log_flush();
    if (r != 0) {
        log_str("CPU_ON failed: the kernel runs on core 0\n"); log_flush();
        lynx_enter_el1(plan.entry + slide, ba_pa);
    }
    for (;;) __asm__ volatile("wfe");          /* core 0 sleeps; the kernel is core 7's */
}

void lynxld_returned(uint64_t log_pa, uint64_t log_len, uint64_t reason, uint64_t esr,
                     uint64_t elr, uint64_t far, uint64_t lr, uint64_t sp)
{
    log_str("\n== XNU RETURNED ==\n");
    log_hex("reason", reason); log_hex("esr", esr); log_hex("elr", elr); log_hex("far", far);
    log_hex("lr", lr); log_hex("sp", sp); log_hex("log_pa", log_pa); log_hex("log_len", log_len);
    log_flush();
    if (log_pa >= PHYS_BASE && log_pa < PHYS_BASE + WIN_SIZE && log_len) {
        if (log_len > 0x30000) log_len = 0x30000;
        dcache_civac(log_pa, log_pa + log_len);
        log_str("== KERNEL LOG ==\n");
        const volatile char *k = (const volatile char *)(uintptr_t)log_pa;
        for (uint64_t i = 0; i < log_len; i++) log_char(k[i] ? k[i] : '\n');
        log_str("\n== END ==\n");
    }
    log_flush();
    /* back from the kernel: the watchdogs on again (they reset the phone into Android in the end) */
    wr32(WDT0, wdt_saved[0]); wr32(WDT1, wdt_saved[1]); pet(); lastpet = ticks();
    if (usb_up()) { log_str("usb after return failed\n"); log_flush(); return; }
    char h[64] = "KLOG 0000000000000000\r\n";
    if (log_len > 0x30000) log_len = 0x30000;
    put_u64(h + 5, (log_pa >= PHYS_BASE && log_pa < PHYS_BASE + WIN_SIZE) ? log_len : 0);
    send(h, 23);
    if (log_pa >= PHYS_BASE && log_pa < PHYS_BASE + WIN_SIZE) {
        static uint8_t chunk[4096];
        for (uint64_t o = 0; o < log_len; o += sizeof(chunk)) {
            uint64_t k = log_len - o > sizeof(chunk) ? sizeof(chunk) : log_len - o;
            for (uint64_t i = 0; i < k; i++) chunk[i] = *(volatile uint8_t *)(log_pa + o + i);
            if (send(chunk, k)) break;
        }
    }
    char fh[64] = "FB 0000000000000000\r\n";
    put_u64(fh + 3, (uint64_t)FB_STRIDE * FB_H);
    send(fh, 21);
    dcache_civac(FB_PA, FB_PA + (uint64_t)FB_STRIDE * FB_H);
    {
        static uint8_t chunk[4096];
        const uint64_t total = (uint64_t)FB_STRIDE * FB_H;
        for (uint64_t o = 0; o < total; o += sizeof(chunk)) {
            uint64_t k = total - o > sizeof(chunk) ? sizeof(chunk) : total - o;
            for (uint64_t i = 0; i < k; i += 4) *(uint32_t *)(chunk + i) = *(volatile uint32_t *)(FB_PA + o + i);
            if (send(chunk, k)) break;
        }
    }
    send("END\r\n", 5);
    { uint64_t t = ticks(); while (ticks() - t < 3 * freq()) service(); }
    usbgadget_stop();
    log_str("returned: log and framebuffer sent\n"); log_flush();
}
