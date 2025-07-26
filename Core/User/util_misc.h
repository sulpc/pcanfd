/**
 * @file util_misc.h
 * @author sulpc
 * @brief
 * @version 0.1
 * @date 2024-05-30
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _UTIL_MISC_H_
#define _UTIL_MISC_H_

#include "util_types.h"
#include <ctype.h>    // isdigit
#include <math.h>     // powf
#include <setjmp.h>   //
#include <stdlib.h>   // atoi, atof, ...
#include <string.h>   // strcmp, memset, ...

#define util_nop(...)                 (void)0
#define util_unused(v)                (void)v
#define util_bref(bbuf, boff)         *((uint8_t*)(bbuf) + (boff))         // byte ref
#define util_cbref(bbuf, boff)        *((const uint8_t*)(bbuf) + (boff))   // const byte ref
#define util_arraylen(array)          (sizeof(array) / sizeof(array[0]))
#define util_fieldoffset(type, field) ((const uint8_t*)&((type*)0)->field - (const uint8_t*)0)
#define util_getbits(u, o, w)         (((uint32_t)(u) >> (o)) & ((1u << (w)) - 1u))

// value
#define util_abs(v)                   ((v) < 0 ? -(v) : (v))
#define util_min2(a, b)               ((a) < (b) ? (a) : (b))
#define util_min3(a, b, c)            ((a) < (b) ? util_min2(a, c) : util_min2(b, c))
#define util_max2(a, b)               ((a) > (b) ? (a) : (b))
#define util_max3(a, b, c)            ((a) > (b) ? util_max2(a, c) : util_max2(b, c))
#define util_checkvalue(v, min, max)  ((v) >= (min) && (v) <= (max))
#define util_limitvalue(v, min, max)  ((v) < (min)) ? (min) : (((v) > (max)) ? (max) : (v))

// bitmap
#define util_bitmask(n)               (1u << (n))
#define util_bitmapchk(bmbuf, n)      ((util_cbref(bmbuf, (n) >> 3) & util_bitmask((n) & 0x7u)) != 0u)

// endian
#define util_getbigendian2(buf)       (((uint16_t)util_cbref(buf, 0) <<  8) | ((uint16_t)util_cbref(buf, 1) <<  0))
#define util_getbigendian4(buf)       (((uint32_t)util_cbref(buf, 0) << 24) | ((uint32_t)util_cbref(buf, 1) << 16) |   \
                                       ((uint32_t)util_cbref(buf, 2) <<  8) | ((uint32_t)util_cbref(buf, 3) <<  0))
#define util_getlittleendian2(buf)    (((uint16_t)util_cbref(buf, 1) <<  8) | ((uint16_t)util_cbref(buf, 0) <<  0))
#define util_getlittleendian4(buf)    (((uint32_t)util_cbref(buf, 3) << 24) | ((uint32_t)util_cbref(buf, 2) << 16) |   \
                                       ((uint32_t)util_cbref(buf, 1) <<  8) | ((uint32_t)util_cbref(buf, 0) <<  0))
#define util_setbigendian2(buf, val)                                                                                   \
    do {                                                                                                               \
        util_bref(buf, 0) = ((uint16_t)(val) >> 8) & 0xffu;                                                            \
        util_bref(buf, 1) = ((uint16_t)(val) >> 0) & 0xffu;                                                            \
    } while (0)
#define util_setbigendian4(buf, val)                                                                                   \
    do {                                                                                                               \
        util_bref(buf, 0) = ((uint32_t)(val) >> 24) & 0xffu;                                                           \
        util_bref(buf, 1) = ((uint32_t)(val) >> 16) & 0xffu;                                                           \
        util_bref(buf, 2) = ((uint32_t)(val) >>  8) & 0xffu;                                                           \
        util_bref(buf, 3) = ((uint32_t)(val) >>  0) & 0xffu;                                                           \
    } while (0)

// fifo
#define util_fifo_create(fifo, capacity)                                                                               \
    struct {                                                                                                           \
        uint16_t _capacity;                                                                                            \
        uint16_t _count;                                                                                               \
        uint16_t _head;                                                                                                \
        uint16_t _tail;                                                                                                \
        void*    _data[capacity];                                                                                      \
    } fifo = {._capacity = capacity, ._count = 0, ._head = 0, ._tail = 0};
#define util_fifo_empty(fifo) (fifo._count == 0)
#define util_fifo_full(fifo)  (fifo._count == fifo._capacity)
#define util_fifo_peek(fifo)  (util_fifo_empty(fifo) ? nullptr : fifo._data[fifo._head])
#define util_fifo_push(fifo, pdata)                                                                                    \
    do {                                                                                                               \
        if (!util_fifo_full(fifo)) {                                                                                   \
            fifo._data[fifo._tail] = pdata;                                                                            \
            fifo._tail             = (fifo._tail + 1) % fifo._capacity;                                                \
            fifo._count++;                                                                                             \
        }                                                                                                              \
    } while (0)
#define util_fifo_pop(fifo)                                                                                            \
    do {                                                                                                               \
        if (!util_fifo_empty(fifo)) {                                                                                  \
            fifo._head = (fifo._head + 1) % fifo._capacity;                                                            \
            fifo._count--;                                                                                             \
        }                                                                                                              \
    } while (0)
#endif
