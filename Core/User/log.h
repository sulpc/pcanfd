#ifndef _LOG_H_
#define _LOG_H_

#include "util_types.h"

void log_init(void);
void log_process(void);
void log_printf(const char* fmt, ...);
void log_printk(const char* fmt, ...);
void log_bytesdump(const uint8_t* data, uint32_t len);

#endif
