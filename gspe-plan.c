/* GSPE input is untrusted: validate fully before any loader memory writes. */
#include "gspe-plan.h"

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static uint64_t le64(const uint8_t *p)
{
    return (uint64_t)le32(p) | (uint64_t)le32(p + 4) << 32;
}

int lynx_gspe_validate(const void *blob, size_t length, struct lynx_gspe_plan *out)
{
    const uint8_t *b = blob;
    const uint8_t magic[8] = { 'G','S','P','E','0','1',0,0 };
    uint64_t base, entry, stub, end = 0;
    uint32_t n, size;
    size_t body;
    int entry_ok = 0, stub_ok;
    if (!b || !out || length < 48) return -1;
    for (unsigned i = 0; i < 8; i++) if (b[i] != magic[i]) return -1;
    n = le32(b + 12); size = le32(b + 32);
    base = le64(b + 16); entry = le64(b + 24); stub = le64(b + 40);
    if (le32(b + 8) != 48 || le32(b + 36) || !n || n > 32) return -1;
    body = 48 + (size_t)n * 32;
    if (body > length || size != length - body) return -1;
    stub_ok = stub == 0;
    for (uint32_t i = 0; i < n; i++) {
        const uint8_t *s = b + 48 + i * 32;
        uint64_t vm = le64(s), vsize = le64(s + 8), off = le64(s + 16), fsize = le64(s + 24);
        if (fsize > vsize || off > size || fsize > size - off) return -1;
        if (!vsize) continue;
        if (vm < base || vsize > UINT64_MAX - vm) return -1;
        uint64_t hi = vm + vsize;
        if (hi > end) end = hi;
        if (entry >= vm && entry < hi) entry_ok = 1;
        if (stub >= vm && stub < hi && hi - stub >= 32) stub_ok = 1;
        for (uint32_t j = 0; j < i; j++) {
            const uint8_t *t = b + 48 + j * 32;
            uint64_t other = le64(t), other_size = le64(t + 8);
            if (other_size && vm < other + other_size && other < hi) return -1;
        }
    }
    if (!entry_ok || !stub_ok || end <= base) return -1;
    out->virtual_base = base; out->entry = entry; out->stub = stub;
    out->span = end - base; out->segment_count = n; out->kernel_size = size;
    return 0;
}
