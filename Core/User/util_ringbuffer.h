/**
 * @file util_ringbuffer.h
 * @author sulpc
 * @brief
 * @version 0.1
 * @date 2024-05-30
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _UTIL_RINGBUFFER_H_
#define _UTIL_RINGBUFFER_H_

#include "util_types.h"


typedef struct {
    uint8_t* buf;   // data buffer
    uint16_t cap;   // capacity
    uint16_t cnt;   // data count
    uint16_t rd;    // read position
    uint16_t wr;    // write position
} util_ringbuffer_t;

/*
Option 1: use a cnt to record the number of data
    when cnt==cap, the buffer is full
    when cnt==0, the buffer is empty

Option 2: without the extra variable  cnt
    when rd==wr, the buffer is empty
    when ((wr+1)%cap)==rd, the buffer is full
    the real size of buffer is cap-1, may not effective when want a size of 2^n
*/

#define util_ringbuffer_init(rb, buffer, size)                                                                         \
    do {                                                                                                               \
        (rb)->buf = (buffer);                                                                                          \
        (rb)->cap = (size);                                                                                            \
        (rb)->cnt = 0;                                                                                                 \
        (rb)->rd  = 0;                                                                                                 \
        (rb)->wr  = 0;                                                                                                 \
    } while (0)

#define util_ringbuffer_empty(rb) ((rb)->cnt == 0)           // ((rb)->rd == (rb)->wr)
#define util_ringbuffer_full(rb)  ((rb)->cnt == (rb)->cap)   // ((((rb)->wr + 1) % (rb)->cap) == (rb)->rd)

#define util_ringbuffer_getc(rb, data)                                                                                 \
    do {                                                                                                               \
        *(data) = (rb)->buf[(rb)->rd];                                                                                 \
        (rb)->rd++;                                                                                                    \
        (rb)->cnt--;                                                                                                   \
        if ((rb)->rd == (rb)->cap) {                                                                                   \
            (rb)->rd = 0;                                                                                              \
        }                                                                                                              \
    } while (0)

#define util_ringbuffer_putc(rb, data)                                                                                 \
    do {                                                                                                               \
        (rb)->buf[(rb)->wr] = data;                                                                                    \
        (rb)->wr++;                                                                                                    \
        (rb)->cnt++;                                                                                                   \
        if ((rb)->wr == (rb)->cap) {                                                                                   \
            (rb)->wr = 0;                                                                                              \
        }                                                                                                              \
    } while (0)


/**
 * @brief
 *
 * @param rb
 * @param data
 * @param size number of bytes want get
 * @return uint16_t number of bytes really get
 */
uint16_t util_ringbuffer_get(util_ringbuffer_t* rb, uint8_t* data, uint16_t size);

/**
 * @brief
 *
 * @param rb
 * @param data
 * @param size number of bytes want put
 * @return uint16_t number of bytes really put
 */
uint16_t util_ringbuffer_put(util_ringbuffer_t* rb, const uint8_t* data, uint16_t size);

#endif
