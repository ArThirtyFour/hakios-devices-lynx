/*
 * lynxld: start the Bluetooth half of the Pixel 7a's WCN6740 before the kernel does, as the btfw-v1 probe did
 * on the phone (probes/btfw.c): UART 0x181B0000 at 115200 8N1 with automatic flow control, power as
 * Google's btpower.c, rampatch and NVM as mainline btqca.c, HCI_Reset. The chip is left ON and running its firmware
 * for the kernel's lynx_bt driver, which only talks H4 on the same UART.
 *
 * Built only when the phone's own firmware is in ../fw-lynx (build.sh; bt_fw.S): it cannot be redistributed.
 * The NVM's tag 17 byte 1 (the baud rate after the reset; the file's 0x11 = 3.2 Mbaud) is sent as 0 = 115200,
 * and its tag 2 (the address: the file has 00:00:00:00:5a:ad) as the phone's own, which the bootloader's tree has.
 */
#include <stdint.h>
#include "log.h"

#define UART 0x181b0000ull
#define SYS  0x180204e4ull
#define GPA3 0x180d0060ull
#define GPA6 0x180e0000ull
#define GPP16 0x10840200ull
#define GPH2 0x14440000ull
#define CMU 0x18000000ull
#define DIV 0x1814
#define FORCE ((1u<<20)|(1u<<21))

extern const uint8_t bt_patch[], bt_patch_end[], bt_nvm[], bt_nvm_end[];

static uint32_t rd(uint64_t a) { return *(volatile uint32_t *)a; }
static void wr(uint64_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile("dsb sy" ::: "memory"); }
static void rmw(uint64_t a, uint32_t mask, uint32_t v) { wr(a, (rd(a) & ~mask) | v); }
static uint64_t freq;
static uint64_t ticks(void) { uint64_t t; __asm__ volatile("isb; mrs %0,cntpct_el0" : "=r"(t)); return t; }
static int elapsed(uint64_t t, uint32_t us) { return ticks() - t >= freq * us / 1000000; }
static void wait_us(uint32_t us) { uint64_t t = ticks(); while (!elapsed(t, us)) { } }

static uint8_t ev[260], seg[4 + 2 + 243];

static int get(uint8_t *b, uint64_t t0, uint32_t us)
{
    for (;;) {
        if (rd(UART + 0x18) & 0x1ff) { (void)rd(UART + 0x14); *b = (uint8_t)rd(UART + 0x24); return 1; }
        if (elapsed(t0, us)) return 0;
    }
}

/* one H4 event into ev[] (code, length, parameters): its length, or -1 */
static int event(uint32_t us)
{
    uint64_t t0 = ticks();
    uint8_t b;
    uint32_t n;

    do { if (!get(&b, t0, us)) return -1; } while (b != 0x04);
    if (!get(&ev[0], t0, us) || !get(&ev[1], t0, us)) return -1;
    for (n = 0; n < ev[1]; n++) if (!get(&ev[2 + n], t0, us)) return -1;
    return (int)(2 + n);
}

static int put(const uint8_t *p, uint32_t n)
{
    uint64_t t = ticks();

    for (uint32_t i = 0; i < n; i++) {
        while (rd(UART + 0x18) & (1u << 24)) if (elapsed(t, 300000)) return 0;
        wr(UART + 0x20, p[i]);
        t = ticks();
    }
    return 1;
}

/* the command-complete event of this command: its length, or -1 */
static int command(const uint8_t *cmd, uint32_t n, uint32_t us)
{
    if (!put(cmd, n)) return -1;
    for (int i = 0; i < 4; i++) {
        int len = event(us);

        if (len < 0) return -1;
        if (ev[0] == 0x0e && len >= 5 && ev[3] == cmd[1] && ev[4] == cmd[2]) return len;
    }
    return -1;
}

/* NVM only (zero_at given): the byte at zero_at goes as 0, and tag 2 (the address, six bytes, the last first) as ours */
#define NVM_ADDRESS 92u         /* tag 2 begins at 80: id, length 6, eight reserved bytes, then the address */
static const uint8_t *address;

static int download(const uint8_t *data, uint32_t size, int skip, uint32_t zero_at)
{
    uint32_t done = 0;

    while (done < size) {
        uint32_t n = size - done > 243 ? 243 : size - done;
        int last = done + n == size || n < 243;

        seg[0] = 0x01; seg[1] = 0x00; seg[2] = 0xfc; seg[3] = (uint8_t)(n + 2); seg[4] = 0x1e; seg[5] = (uint8_t)n;
        for (uint32_t i = 0; i < n; i++) {
            uint32_t at = done + i;

            seg[6 + i] = at == zero_at ? 0 : data[at];
            if (!skip && address && at >= NVM_ADDRESS && at < NVM_ADDRESS + 6) seg[6 + i] = address[5 - (at - NVM_ADDRESS)];
        }
        if (!put(seg, 6 + n)) return 0;
        done += n;
        if (!skip || last) {
            int len = event(last ? 5000000 : 1000000);

            if (len < 0 || (!skip && (ev[0] != 0x0e || len < 7 || ev[5] != 0x00 || ev[6] != 0x1e))) return 0;
        }
    }
    return 1;
}

static const uint8_t c_version[] = { 1, 0x00, 0xfc, 1, 0x19 };
static const uint8_t c_nolog[] = { 1, 0x17, 0xfc, 2, 0x14, 0x00 };
static const uint8_t c_reset[] = { 1, 0x03, 0x0c, 0 };

static void off(void)
{
    rmw(GPP16 + 4, 12, 0); wait_us(55000); rmw(GPH2 + 4, 1u << 5, 0);
}

/* 1: the firmware runs and answered HCI_Reset. bd_addr: the phone's own address (six bytes as they are read), or 0 */
int lynx_bt_up(const uint8_t *bd_addr)
{
    uint32_t psize = (uint32_t)(bt_patch_end - bt_patch), nsize = (uint32_t)(bt_nvm_end - bt_nvm), d, q;
    uint64_t t0;
    int n;

    __asm__ volatile("mrs %0,cntfrq_el0" : "=r"(freq));
    t0 = ticks();
    address = (bd_addr && bt_nvm[80] == 2 && bt_nvm[81] == 0 && bt_nvm[82] == 6) ? bd_addr : 0;
    if (bt_patch[0] != 1 || bt_patch[14] != 3 || bt_nvm[0] != 2 || bt_nvm[135] != 17 || bt_nvm[148] != 0x11) { log_str("bt: not the expected firmware files\n"); return 0; }
    if ((rd(CMU + 0x1000) & 3) != 1 || (rd(CMU + 0x1004) & 3) != 0 || !(rd(CMU + 0x2084) & (1u << 21))) { log_str("bt: APM clock source is not the 394 MHz one\n"); return 0; }
    rmw(CMU + 0x20ac, 0, FORCE); rmw(CMU + 0x20b0, 0, FORCE); rmw(CMU + 0x2100, 0, FORCE); rmw(CMU + 0x211c, 0, FORCE);
    rmw(CMU + 0x306c, 1, 2); wait_us(100);
    rmw(CMU + DIV, 0x7f, 1);                                    /* 394 MHz / 2 = 197 MHz, as Android */
    { uint64_t t = ticks(); while (rd(CMU + DIV) & (1u << 16)) if (elapsed(t, 10000)) { log_str("bt: clock divider busy\n"); return 0; } }
    rmw(SYS, 7, 1); wr(UART + 0xc4, 1); wait_us(1); wr(UART + 0xc4, 0); wait_us(1); wr(UART + 0xc8, 2);
    rmw(GPA3, 0xffff, 0x2222); rmw(GPA3 + 8, 0xffff, 0);
    d = 197000000u / 115200u; q = d / 16 - 1;
    if ((d & 15) != 15 && 197000000u / d - 115200u > 115200u - 197000000u / (d + 1)) d++;
    wr(UART + 0x04, 0); wr(UART + 0x38, 15);
    wr(UART + 0x00, 3); wr(UART + 0x28, q); wr(UART + 0x2c, d & 15);
    wr(UART + 0x08, 7); wait_us(10); wr(UART + 0x08, 1);
    wr(UART + 0x0c, 0x10);                                      /* automatic flow control */
    wr(UART + 0x04, 5);

    rmw(GPA6, 15u << 24, 0); rmw(GPA6 + 8, 15u << 24, 1u << 24);
    rmw(GPP16 + 4, (1u << 2) | (1u << 3), 0); rmw(GPP16, 0xff00, 0x1100);
    wait_us(100000);
    rmw(GPH2 + 4, 0, 1u << 5); rmw(GPH2, 15u << 20, 1u << 20); wait_us(6000);
    rmw(GPP16 + 4, 0, 1u << 2); wait_us(50000);
    rmw(GPP16 + 4, 0, 1u << 3);
    while (event(30000) >= 0) { }

    n = command(c_version, sizeof(c_version), 300000);
    if (n < 20 || ev[5] != 0x00 || ev[6] != 0x19) { log_str("bt: the chip does not answer\n"); off(); return 0; }
    if (ev[8] != bt_patch[16] || ev[9] != bt_patch[17] || ev[14] != bt_patch[18] || ev[15] != bt_patch[19]) { log_str("bt: firmware is for another chip\n"); off(); return 0; }
    if (!download(bt_patch, psize, 1, 0xffffffffu)) { log_str("bt: rampatch download failed\n"); off(); return 0; }
    wait_us(10000);
    if (!download(bt_nvm, nsize, 0, 148)) { log_str("bt: NVM download failed\n"); off(); return 0; }
    command(c_nolog, sizeof(c_nolog), 500000);
    n = command(c_reset, sizeof(c_reset), 2000000);
    if (n < 6 || ev[5] != 0) { log_str("bt: no answer to the reset after the download\n"); off(); return 0; }
    log_hex("bt: firmware running, left on for the kernel; ms", (ticks() - t0) * 1000u / freq);
    return 1;
}
