/* GSPE validation for the lynx XNU bring-up. No hardware access. */
#ifndef LYNX_GSPE_PLAN_H
#define LYNX_GSPE_PLAN_H
#include <stddef.h>
#include <stdint.h>

struct lynx_gspe_plan {
    uint64_t virtual_base, entry, stub, span;
    uint32_t segment_count, kernel_size;
};

/* Return zero only for a bounded, non-overlapping manifest. */
int lynx_gspe_validate(const void *blob, size_t length, struct lynx_gspe_plan *plan);
#endif
