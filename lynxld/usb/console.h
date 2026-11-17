/* lynxld: PixOS console API mapped onto the ramoops log. */
#pragma once
#include <stdint.h>
#include "../log.h"
static inline void con_str(const char *label, const char *s) { log_str(label); log_str(s); log_str("\n"); log_flush(); }
