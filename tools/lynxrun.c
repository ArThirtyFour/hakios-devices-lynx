/* Host side of lynxld: send the system image to the phone over USB (18d1:d00d), then, if the kernel ever returns to the
 * loader, collect the kernel log and the framebuffer.  cc -O2 -o lynxrun lynxrun.c -lusb-1.0;  lynxrun SYSTEM.apfs OUTDIR */
#include <libusb-1.0/libusb.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
static libusb_context *ctx;
static libusb_device_handle *h;
static unsigned char rb[65536]; static int rn, rp;
static void nap(long ms) { struct timespec t = { ms / 1000, (ms % 1000) * 1000000L }; nanosleep(&t, NULL); }
static int dev_open(int secs)
{
    if (h) { libusb_close(h); h = NULL; }
    for (int i = 0; i < secs * 4 && !h; i++) { h = libusb_open_device_with_vid_pid(ctx, 0x18d1, 0xd00d); if (!h) nap(250); }
    if (!h) return -1;
    libusb_set_auto_detach_kernel_driver(h, 1); libusb_claim_interface(h, 0); libusb_claim_interface(h, 1);
    libusb_control_transfer(h, 0x21, 0x22, 1, 0, NULL, 0, 1000);
    rn = rp = 0; return 0;
}
static int getb(int ms)
{
    while (rp >= rn) {
        int got = 0, r = libusb_bulk_transfer(h, 0x81, rb, sizeof rb, &got, ms);
        if (r == LIBUSB_ERROR_TIMEOUT && got == 0) return -1;
        if (r && r != LIBUSB_ERROR_TIMEOUT) return -2;
        rn = got; rp = 0;
    }
    return rb[rp++];
}
static int line(char *o, int cap, int ms)
{
    int n = 0, c;
    while ((c = getb(ms)) >= 0) { if (n < cap - 1) o[n++] = (char)c; if (c == '\n') break; }
    o[n] = 0; return c < 0 && n == 0 ? c : n;
}
static int blob(const char *path, uint64_t len)
{
    FILE *f = fopen(path, "wb"); uint64_t i = 0; int c;
    for (; i < len && (c = getb(5000)) >= 0; i++) fputc(c, f);
    fclose(f); return i == len ? 0 : -1;
}
int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: lynxrun SYSTEM.apfs OUTDIR\n"); return 2; }
    setvbuf(stdout, NULL, _IOLBF, 0);
    int collect = !strcmp(argv[1], "--collect");
    FILE *fp = collect ? NULL : fopen(argv[1], "rb"); if (!collect && !fp) { perror(argv[1]); return 2; }
    char l[256], p1[512], p2[512];
    libusb_init(&ctx);
    if (collect) goto after;
    fseek(fp, 0, SEEK_END); uint64_t len = (uint64_t)ftell(fp); fseek(fp, 0, SEEK_SET);
    if (dev_open(120)) { puts("lynxrun: no phone"); return 1; }
    l[0] = 0;
    for (int i = 0; i < 45 && !strstr(l, "READY"); i++) {   /* DTR again until the phone answers: it may not have been configured yet */
        libusb_control_transfer(h, 0x21, 0x22, 1, 0, NULL, 0, 1000);
        if (line(l, sizeof l, 2000) < 0) l[0] = 0;
    }
    if (!strstr(l, "READY")) { printf("lynxrun: no READY: %s\n", l); return 1; }
    printf("phone: %s", l);
    unsigned char hdr[512] = "LYNXDL01"; memcpy(hdr + 8, &len, 8);
    int done; const size_t CH = 0x800000; unsigned char *buf = malloc(CH);
    int hr = libusb_bulk_transfer(h, 0x01, hdr, 512, &done, 10000);
    if (hr) { printf("lynxrun: header failed: %s (done=%d)\n", libusb_error_name(hr), done); return 1; }
    struct timespec a, z; clock_gettime(CLOCK_MONOTONIC, &a);
    uint64_t sent = 0;
    while (sent < len) {
        size_t want = len - sent > CH ? CH : (size_t)(len - sent), r = fread(buf, 1, want, fp), pad = (r + 511) & ~(size_t)511;
        memset(buf + r, 0, pad - r);
        if (libusb_bulk_transfer(h, 0x01, buf, (int)pad, &done, 20000) || (size_t)done != pad) { printf("lynxrun: send failed at %llu\n", (unsigned long long)sent); return 1; }
        sent += r;
    }
    clock_gettime(CLOCK_MONOTONIC, &z);
    double s = (double)(z.tv_sec - a.tv_sec) + (z.tv_nsec - a.tv_nsec) / 1e9;
    printf("lynxrun: system image %llu bytes in %.1f s (%.1f MB/s)\n", (unsigned long long)sent, s, sent / s / 1e6);
    while (line(l, sizeof l, 180000) > 0) { printf("phone: %s", l); if (strstr(l, "JUMP")) break; }
    puts("lynxrun: kernel running; waiting for it to come back (loader_return_after)...");
    if (h) { libusb_close(h); h = NULL; }
    for (int i = 0; i < 120; i++) { libusb_device_handle *t = libusb_open_device_with_vid_pid(ctx, 0x18d1, 0xd00d); if (!t) break; libusb_close(t); nap(500); }
    puts("lynxrun: phone left USB (kernel running)");
after:
    if (dev_open(900)) { puts("lynxrun: phone did not come back on USB"); return 1; }
    for (;;) {
        if (line(l, sizeof l, 30000) < 0) { puts("lynxrun: stream ended"); break; }
        if (!strncmp(l, "KLOG ", 5)) { snprintf(p1, sizeof p1, "%s/kernel.log", argv[2]); uint64_t n = strtoull(l + 5, NULL, 16); printf("lynxrun: kernel log %llu bytes -> %s (%s)\n", (unsigned long long)n, p1, blob(p1, n) ? "SHORT" : "ok"); }
        else if (!strncmp(l, "FB ", 3)) { snprintf(p2, sizeof p2, "%s/fb.raw", argv[2]); uint64_t n = strtoull(l + 3, NULL, 16); printf("lynxrun: framebuffer %llu bytes -> %s (%s)\n", (unsigned long long)n, p2, blob(p2, n) ? "SHORT" : "ok"); }
        else if (strstr(l, "END")) { puts("lynxrun: END"); break; }
        else printf("phone: %s", l);
    }
    return 0;
}
