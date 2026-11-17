/* ramoops console writer (0xfd3ff000, reserved in lynx FDT; survives watchdog reset). */
#include "log.h"
#define LOG 0xfd3ff000ull
#define CAP 0x40000u
#define CNT (*(volatile uint32_t *)(LOG + 12 + CAP))
static volatile uint32_t *const hdr = (volatile uint32_t *)LOG;
static volatile uint8_t *const data = (volatile uint8_t *)(LOG + 12);
static void put(char c) { uint32_t n = CNT; if (n < CAP) { data[n] = (uint8_t)c; CNT = n + 1; } }
void log_init(void) { CNT = 0; }
void log_char(char c) { put(c); }
void log_str(const char *s) { while (*s) put(*s++); }
void log_hex(const char *k, uint64_t v)
{
    log_str(k); log_str("=0x");
    for (int i = 15; i >= 0; i--) { int d = (int)((v >> (i * 4)) & 15); put((char)(d < 10 ? '0' + d : 'a' + d - 10)); }
    put('\n');
}
void log_flush(void)
{
    uint32_t n = CNT;
    hdr[0] = 0x43474244u; hdr[1] = n; hdr[2] = n;
    __asm__ volatile("dsb sy" ::: "memory");
    for (uint64_t a = LOG; a < LOG + 12 + CAP + 64; a += 64) __asm__ volatile("dc civac, %0" :: "r"(a) : "memory");
    __asm__ volatile("dsb sy; isb" ::: "memory");
}
