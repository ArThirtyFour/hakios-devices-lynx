/*
 * lynx Bluetooth step 2: firmware into the WCN6740 over its UART, then the first commands to the running firmware.
 * The UART / power bring-up is btprobe.c's (run on the phone as bt-v2: the chip answered at 115200). The download is
 * mainline Linux's drivers/bluetooth/btqca.c (SOURCES.md; "btqca.c.source" below is that file, "btqca" for short):
 *   qca_read_soc_version      01 00 fc 01 19                    -> command complete, 00 19, a length byte, the version
 *   qca_download_firmware     01 00 fc <n+2> 1e <n> <n bytes>   in segments of at most 243 (MAX_SIZE_PER_TLV_SEGMENT)
 *     rampatch hpbtfw21.tlv: its header says download_mode 3 (QCA_SKIP_EVT_VSE_CC): no answer to a segment but the last
 *     NVM hpnv21.bin: every segment answered, command complete with 00 1e (btqca.c.source:486-512)
 *   qca_disable_soc_logging   01 17 fc 02 14 00
 *   HCI_Reset                 01 03 0c 00
 *   qca_read_fw_build_info    01 00 fc 01 20
 * The NVM's tag 17 (HCI transport) byte 1 is the baud rate the firmware talks at after the reset: the file says 0x11
 * (QCA_BAUDRATE_3200000); Linux writes its own there (btqca.c.source:428-431), here 0 = QCA_BAUDRATE_115200. Byte 0's
 * 0x80 (in-band sleep) is NOT set as Linux does: nothing here speaks that protocol.
 * The chip throttles a download with its RTS: the UART's automatic flow control is on for it (UMCON bit 4).
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
#define IP 0x20ac
#define PC 0x20b0
#define RST 0x2100
#define QCH 0x306c
#define FORCE ((1u<<20)|(1u<<21))
#define LCON 0x00
#define CON 0x04
#define FCON 0x08
#define MCON 0x0c
#define TRSTAT 0x10
#define ERSTAT 0x14
#define FSTAT 0x18
#define MSTAT 0x1c
#define TX 0x20
#define RX 0x24
#define BRDIV 0x28
#define FRAC 0x2c
#define INTMASK 0x38
#define USICON 0xc4
#define USIOPT 0xc8

extern const uint8_t bt_patch[], bt_patch_end[], bt_nvm[], bt_nvm_end[];

static uint32_t rd(uint64_t a) { return *(volatile uint32_t *)a; }
static void wr(uint64_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile("dsb sy" ::: "memory"); }
static void rmw(uint64_t a, uint32_t mask, uint32_t v) { wr(a, (rd(a) & ~mask) | v); }
static uint64_t freq;
static uint64_t ticks(void) { uint64_t t; __asm__ volatile("isb; mrs %0,cntpct_el0" : "=r"(t)); return t; }
static int elapsed(uint64_t t, uint32_t us) { return ticks() - t >= freq * us / 1000000; }
static void wait_us(uint32_t us) { uint64_t t = ticks(); while (!elapsed(t, us)) { } }
static void stage(const char *s) { log_str(s); log_str("\n"); log_flush(); }

static void baud(uint32_t rate, uint32_t b)
{
    uint32_t d = rate / b, q = d / 16 - 1;

    if ((d & 15) != 15 && rate / d - b > b - rate / (d + 1)) d++;
    wr(UART + CON, 0); wr(UART + INTMASK, 15);
    wr(UART + LCON, 3); wr(UART + BRDIV, q); wr(UART + FRAC, d & 15);
    wr(UART + FCON, 7); wait_us(10); wr(UART + FCON, 1);
    wr(UART + MCON, 0x10);                  /* automatic flow control: TX waits for the chip's RTS */
    wr(UART + CON, 5);
    log_hex("baud", b); log_hex(" UBRDIV", q); log_hex(" UFRACVAL", d & 15); log_flush();
}

static uint32_t rx_errors, rx_stray;

/* 1 with the byte, 0 when the time is up */
static int get(uint8_t *b, uint64_t t0, uint32_t us)
{
    for (;;) {
        if (rd(UART + FSTAT) & 0x1ff) {
            rx_errors |= rd(UART + ERSTAT);
            *b = (uint8_t)rd(UART + RX);
            return 1;
        }
        if (elapsed(t0, us)) return 0;
    }
}

static uint8_t ev[260];

/* One H4 event into ev[] (event code, length, parameters); its length, or -1 when none came in time. */
static int event(uint32_t us)
{
    uint64_t t0 = ticks();
    uint8_t b;
    uint32_t n;

    for (;;) {
        if (!get(&b, t0, us)) return -1;
        if (b == 0x04) break;
        if (rx_stray++ < 16) log_hex("stray byte", b);
    }
    if (!get(&ev[0], t0, us) || !get(&ev[1], t0, us)) return -1;
    for (n = 0; n < ev[1]; n++) if (!get(&ev[2 + n], t0, us)) return -1;
    return (int)(2 + n);
}

static void show(const char *k, int n)
{
    uint64_t v = 0;

    log_str(k); log_hex(" length", (uint64_t)(int64_t)n);
    for (int i = 0; i < n && i < 64; i++) { v = (v << 8) | ev[i]; if ((i & 7) == 7 || i == n - 1) { log_hex("  ", v); v = 0; } }
    log_flush();
}

/* 1 when all of it left the FIFO; 0 when the chip held its RTS for 300 ms */
static int put(const uint8_t *p, uint32_t n)
{
    uint64_t t = ticks();

    for (uint32_t i = 0; i < n; i++) {
        while (rd(UART + FSTAT) & (1u << 24)) if (elapsed(t, 300000)) return 0;
        wr(UART + TX, p[i]);
        t = ticks();
    }
    return 1;
}

/* a command and its command-complete event; the event's length, or -1 */
static int command(const char *name, const uint8_t *cmd, uint32_t n, uint32_t us)
{
    int len;

    if (!put(cmd, n)) { log_str(name); stage(": TX stalled"); return -1; }
    for (int i = 0; i < 4; i++) {
        len = event(us);
        show(name, len);
        if (len < 0) return -1;
        if (ev[0] == 0x0e && len >= 5 && ev[3] == cmd[1] && ev[4] == cmd[2]) return len;    /* the command complete of this opcode */
    }
    return -1;
}

static uint8_t seg[4 + 2 + 243];

/* btqca.c.source:554-621. skip: QCA_SKIP_EVT_VSE_CC (only the last segment is answered). patch_at: the byte to send as 0. */
static int download(const char *name, const uint8_t *data, uint32_t size, int skip, uint32_t zero_at)
{
    uint32_t done = 0, segs = 0;
    uint64_t t0 = ticks();

    while (done < size) {
        uint32_t n = size - done > 243 ? 243 : size - done;
        int last = done + n == size || n < 243;

        seg[0] = 0x01; seg[1] = 0x00; seg[2] = 0xfc; seg[3] = (uint8_t)(n + 2); seg[4] = 0x1e; seg[5] = (uint8_t)n;
        for (uint32_t i = 0; i < n; i++) seg[6 + i] = done + i == zero_at ? 0 : data[done + i];
        if (!put(seg, 6 + n)) { log_str(name); log_hex(": TX stalled at", done); log_hex(" UMSTAT", rd(UART + MSTAT)); log_flush(); return 0; }
        done += n; segs++;
        if (!skip || last) {
            int len = event(last ? 5000000 : 1000000);

            if (skip) {                     /* the rampatch's one answer: shown whatever it is, only its absence ends the run */
                show("answer to the last rampatch segment", len);
                if (len < 0) return 0;
            } else if (len < 0 || ev[0] != 0x0e || len < 7 || ev[5] != 0x00 || ev[6] != 0x1e) {
                log_str(name); log_hex(": bad answer to segment", segs); show(" it was", len); return 0;
            }
        } else {
            uint8_t b;
            while ((rd(UART + FSTAT) & 0x1ff) && rx_stray < 64) { b = (uint8_t)rd(UART + RX); if (rx_stray++ < 16) log_hex("byte during download", b); }
        }
    }
    log_str(name); log_hex(": bytes sent", done); log_hex(" segments", segs);
    log_hex(" ms", (ticks() - t0) * 1000u / freq); log_flush();
    return 1;
}

static const uint8_t c_version[] = { 1, 0x00, 0xfc, 1, 0x19 };
static const uint8_t c_board[] = { 1, 0x00, 0xfc, 1, 0x23 };
static const uint8_t c_nolog[] = { 1, 0x17, 0xfc, 2, 0x14, 0x00 };
static const uint8_t c_reset[] = { 1, 0x03, 0x0c, 0 };
static const uint8_t c_build[] = { 1, 0x00, 0xfc, 1, 0x20 };
static const uint8_t c_bdaddr[] = { 1, 0x09, 0x10, 0 };
static const uint8_t c_localver[] = { 1, 0x01, 0x10, 0 };
static const uint8_t c_features[] = { 1, 0x03, 0x10, 0 };

void lynxld_main(uint64_t fdt, uint64_t self)
{
    (void)fdt; (void)self;
    __asm__ volatile("mrs %0,cntfrq_el0" : "=r"(freq));
    log_init(); stage("BTFW v1");
    uint32_t c16 = rd(GPP16), ch = rd(GPH2), ca = rd(GPA6), cu = rd(GPA3);
    uint32_t pa = rd(GPA6 + 8), pu = rd(GPA3 + 8), dw = rd(GPP16 + 4);
    uint32_t ip = rd(CMU + IP), pc = rd(CMU + PC), rs = rd(CMU + RST), qc = rd(CMU + QCH), dv = rd(CMU + DIV), sg = rd(CMU + 0x211c);
    uint32_t psize = (uint32_t)(bt_patch_end - bt_patch), nsize = (uint32_t)(bt_nvm_end - bt_nvm), sys;
    int n, running = 0;

    /* the files are what this probe was written for: rampatch type 1, download mode 3; NVM type 2, tag 17 at 147 */
    if (bt_patch[0] != 1 || bt_patch[14] != 3 || bt_nvm[0] != 2 || bt_nvm[135] != 17 || bt_nvm[148] != 0x11) { stage("firmware files are not the expected ones"); goto end; }
    if ((rd(CMU + 0x1000) & 3) != 1 || (rd(CMU + 0x1004) & 3) != 0 || !(rd(CMU + 0x2084) & (1u << 21))) { stage("APM clock source is not the 394 MHz one"); goto end; }
    rmw(CMU + IP, 0, FORCE); rmw(CMU + PC, 0, FORCE); rmw(CMU + RST, 0, FORCE); rmw(CMU + 0x211c, 0, FORCE);
    rmw(CMU + QCH, 1, 2); wait_us(100);
    sys = rd(SYS);
    rmw(CMU + DIV, 0x7f, 1);
    { uint64_t t = ticks(); while (rd(CMU + DIV) & (1u << 16)) if (elapsed(t, 10000)) { stage("DIV busy"); goto clocks; } }
    rmw(SYS, 7, 1); wr(UART + USICON, 1); wait_us(1); wr(UART + USICON, 0); wait_us(1); wr(UART + USIOPT, 2);
    rmw(GPA3, 0xffff, 0x2222); rmw(GPA3 + 8, 0xffff, 0);
    baud(197000000, 115200);

    rmw(GPA6, 15u << 24, 0); rmw(GPA6 + 8, 15u << 24, 1u << 24);
    rmw(GPP16 + 4, (1u << 2) | (1u << 3), 0); rmw(GPP16, 0xff00, 0x1100);
    wait_us(100000);
    rmw(GPH2 + 4, 0, 1u << 5); rmw(GPH2, 15u << 20, 1u << 20); wait_us(6000);
    rmw(GPP16 + 4, 0, 1u << 2); wait_us(50000);
    rmw(GPP16 + 4, 0, 1u << 3);
    log_hex("chip on: UMSTAT", rd(UART + MSTAT)); log_flush();
    while (event(30000) >= 0) { }

    n = command("version", c_version, sizeof(c_version), 300000);
    if (n < 20 || ev[5] != 0x00 || ev[6] != 0x19) { stage("no version"); goto tidy; }
    /* btqca.c.source: struct qca_btsoc_version one byte into the data: product (4), patch (2), rom (2), soc id (4) */
    log_hex("product", ev[8] | ev[9] << 8); log_hex("rom", ev[14] | ev[15] << 8);
    log_hex("soc id", ev[16] | ev[17] << 8 | ev[18] << 16 | (uint32_t)ev[19] << 24); log_flush();
    if (ev[8] != bt_patch[16] || ev[9] != bt_patch[17] || ev[14] != bt_patch[18] || ev[15] != bt_patch[19]) {
        stage("the rampatch is for another product / ROM build: nothing sent"); goto tidy;
    }

    if (!download("rampatch", bt_patch, psize, 1, 0xffffffffu)) goto tidy;
    wait_us(10000);
    command("board id", c_board, sizeof(c_board), 300000);
    if (!download("nvm", bt_nvm, nsize, 0, 148)) goto tidy;
    command("logging off", c_nolog, sizeof(c_nolog), 500000);
    n = command("HCI_Reset", c_reset, sizeof(c_reset), 2000000);
    if (n < 6 || ev[5] != 0) { stage("no answer to the reset after the download"); goto tidy; }
    running = 1;
    stage("FIRMWARE RUNNING");
    n = command("build info", c_build, sizeof(c_build), 500000);
    if (n > 8) {
        log_str("build: ");
        for (int i = 8; i < n; i++) { char s2[2] = { (ev[i] >= 32 && ev[i] < 127) ? (char)ev[i] : '.', 0 }; log_str(s2); }
        log_str("\n"); log_flush();
    }
    command("Read_BD_ADDR", c_bdaddr, sizeof(c_bdaddr), 500000);
    command("Read_Local_Version", c_localver, sizeof(c_localver), 500000);
    command("Read_Local_Features", c_features, sizeof(c_features), 500000);
tidy:
    log_hex("running", (uint64_t)running); log_hex("rx errors", rx_errors); log_hex("stray bytes", rx_stray);
    log_hex("UMSTAT", rd(UART + MSTAT)); log_flush();
    rmw(GPP16 + 4, 12, 0); wait_us(55000); rmw(GPH2 + 4, 1u << 5, 0);
    wr(UART + CON, 0); wr(UART + MCON, 0); wr(SYS, sys);
    rmw(GPP16 + 4, 1u << 3, dw & (1u << 3));
    rmw(GPP16, 0xff00, c16 & 0xff00); rmw(GPH2, 15u << 20, ch & (15u << 20));
    rmw(GPA6, 15u << 24, ca & (15u << 24)); rmw(GPA6 + 8, 15u << 24, pa & (15u << 24));
    rmw(GPA3, 0xffff, cu & 0xffff); rmw(GPA3 + 8, 0xffff, pu & 0xffff);
clocks:
    wr(CMU + DIV, dv); wr(CMU + QCH, qc); wr(CMU + RST, rs); wr(CMU + IP, ip); wr(CMU + PC, pc); wr(CMU + 0x211c, sg);
end:
    stage("BTFW-END");
    for (;;) __asm__ volatile("wfe");
}
void lynxld_returned(void) {}
