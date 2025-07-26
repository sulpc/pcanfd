#include "log.h"
#include "util_ringbuffer.h"

#include <stdarg.h>   // va_list
#include <stdio.h>    // vsprintf
#include <string.h>

#include "stm32g4xx_hal.h"
#include "system.h"

#define LOG_BUFFER_SIZE 1024
#define LOG_LINE_SIZE   64
#define LOG_TX_SIZE     16

extern UART_HandleTypeDef huart1;
static util_ringbuffer_t  log_buffer;
static bool               log_inited = false;

#define uart_putc(c)                                                                                                   \
    do {                                                                                                               \
        huart1.Instance->TDR = c;                                                                                      \
        while (!(huart1.Instance->ISR & USART_ISR_TXE))                                                                \
            ;                                                                                                          \
    } while (0)

static void console_puts(const char* str)
{
    while (*str) {
        uart_putc(*str);
        str++;
    }
}
static void console_put(const uint8_t* data, uint16_t len)
{
    while (len) {
        uart_putc(*data);
        data++;
        len--;
    }
}

void log_init(void)
{
    static uint8_t log_data_buffer[LOG_BUFFER_SIZE];

    util_ringbuffer_init(&log_buffer, &log_data_buffer[0], LOG_BUFFER_SIZE);
    log_inited = true;
}

void log_printf(const char* fmt, ...)
{
    if (!log_inited) {
        return;
    }
    char    buffer[LOG_LINE_SIZE];
    va_list ap;

    va_start(ap, fmt);
    vsprintf(buffer, fmt, ap);
    va_end(ap);

    util_ringbuffer_put(&log_buffer, (uint8_t*)buffer, (uint16_t)strlen(buffer));
}

void log_printk(const char* fmt, ...)
{
    char    buffer[LOG_LINE_SIZE];
    va_list ap;

    va_start(ap, fmt);
    vsprintf(buffer, fmt, ap);
    va_end(ap);

    console_puts(buffer);
}

void log_process(void)
{
    static char buffer[LOG_TX_SIZE];

    if (!log_inited) {
        return;
    }

    uint16_t get = util_ringbuffer_get(&log_buffer, (uint8_t*)buffer, sizeof(buffer));

    console_put((uint8_t*)buffer, get);
}

void log_bytesdump(const uint8_t* data, uint32_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        log_printf("%02X ", data[i]);
    }
    log_printf("\n");
}
