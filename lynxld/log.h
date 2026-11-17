#ifndef LYNXLD_LOG_H
#define LYNXLD_LOG_H
#include <stdint.h>
void log_init(void);
void log_char(char c);
void log_str(const char *s);
void log_hex(const char *key, uint64_t v);
void log_flush(void);
#endif
