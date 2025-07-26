// tiny frame protocol

#include "util_types.h"

typedef struct {
    const void (*msg_cbk)(const uint8_t* data, uint16_t len);
    const void (*frm_cbk)(const uint8_t* data, uint16_t len);
    uint8_t* const msg_buf;     // used to parse raw frame
    uint8_t* const frm_buf;     // used to make raw frame
    const uint16_t frm_bufsz;   // used to make raw frame
    const uint16_t msg_bufsz;   // used to parse raw frame
    uint32_t       head;        // init to 0
    uint32_t       expect;      // init to 0
    uint32_t       recv;        // init to 0
} tfp_t;

uint16_t tfp_stream_feed(tfp_t* tfp, const uint8_t* data, uint16_t size);
void     tfp_make_frame(tfp_t* tfp, const uint8_t* data, uint16_t size);
