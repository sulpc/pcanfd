#include "util_ringbuffer.h"
#include "util_misc.h"

#define STUPID_COPY 0

uint16_t util_ringbuffer_get(util_ringbuffer_t* rb, uint8_t* data, uint16_t size)
{
    uint16_t copy = 0;
#if STUPID_COPY
    while (!util_ringbuffer_empty(rb) && copy < size) {
        util_ringbuffer_getc(rb, &data[copy]);
        copy++;
    }
#else
    if (!util_ringbuffer_empty(rb)) {
        if (rb->rd < rb->wr) {
            // -----rd++++++++++wr-----  +: data could get
            copy = util_min2(size, rb->wr - rb->rd);
            memcpy(data, rb->buf + rb->rd, copy);
            rb->rd += copy;
        } else {   // wr<=rd, rd==wr means full
            // +++++wr----------rd+++++  +: data could get
            if (size < rb->cap - rb->rd) {
                copy = size;
                memcpy(data, rb->buf + rb->rd, copy);
                rb->rd += copy;
            } else {
                copy = rb->cap - rb->rd;
                memcpy(data, rb->buf + rb->rd, copy);

                uint16_t copy2 = util_min2(size - copy, rb->wr);
                memcpy(data + copy, rb->buf, copy2);
                rb->rd = copy2;

                copy += copy2;
            }
        }
    }
    rb->cnt -= copy;
#endif
    return copy;
}


uint16_t util_ringbuffer_put(util_ringbuffer_t* rb, const uint8_t* data, uint16_t size)
{
    uint16_t copy = 0;

#if STUPID_COPY
    while (!util_ringbuffer_full(rb) && copy < size) {
        util_ringbuffer_putc(rb, data[copy]);
        copy++;
    }
#else
    size = util_min2(size, rb->cap - rb->cnt);

    if (!util_ringbuffer_full(rb)) {
        if (rb->wr < rb->rd) {
            // +++++wr----------rd+++++  -: data could put
            copy = util_min2(size, rb->rd - rb->wr);
            memcpy(rb->buf + rb->wr, data, copy);
            rb->wr += copy;
        } else {   // rd<=wr, rd==wr means empty
            // -----rd++++++++++wr-----  -: data could put
            if (size < rb->cap - rb->wr) {
                copy = size;
                memcpy(rb->buf + rb->wr, data, copy);
                rb->wr += copy;
            } else {
                copy = rb->cap - rb->wr;
                memcpy(rb->buf + rb->wr, data, copy);

                uint16_t copy2 = util_min2(size - copy, rb->rd);
                memcpy(rb->buf, data + copy, copy2);
                rb->wr = copy2;

                copy += copy2;
            }
        }
    }
    rb->cnt += copy;
#endif
    return copy;
}
