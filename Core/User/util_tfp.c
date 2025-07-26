#include "util_tfp.h"
#include "log.h"
#include "util_misc.h"

// f"0x{int.from_bytes(b'head', 'big'):08x}"

#define TFP_FRM_HEAD 0x68656164u   // 'head'
#define TFP_FRM_TAIL 0x7461696cu   // 'tail'

uint16_t tfp_stream_feed(tfp_t* tfp, const uint8_t* data, uint16_t size)
{
    if (tfp->msg_buf == nullptr || tfp->msg_bufsz == 0 || tfp->msg_cbk == nullptr) {
        return 0;
    }

    uint16_t eat = 0;

    // no head, find it
    if (tfp->head != TFP_FRM_HEAD) {
        for (; eat < size; eat++) {
            tfp->head <<= 8;
            tfp->head |= (uint32_t)data[eat];
            if (tfp->head == TFP_FRM_HEAD) {
                eat += 1;
                break;
            }
        }
    }
    // still no head, continue in next package
    if (tfp->head != TFP_FRM_HEAD) {
        return size;
    }

    // no length, length should be the next 2 bytes in pack buffer
    if (tfp->expect == 0 && size - eat >= 2) {
        tfp->expect = util_getbigendian2(data + eat) + 6;   // data + crc + tail
        if (tfp->expect > tfp->msg_bufsz - 6) {
            log_printf("!!! msg oversize\n");
            tfp->recv   = 0;
            tfp->expect = 0;
            tfp->head   = 0;
        }
        tfp->recv = 0;
        eat += 2;
    }

    if (tfp->expect != 0) {
        // get data from pack buffer
        uint16_t data_in_pack = util_min2(tfp->expect - tfp->recv, size - eat);
        memcpy(tfp->msg_buf + tfp->recv, data + eat, data_in_pack);
        tfp->recv += data_in_pack;
        eat += data_in_pack;

        if (tfp->recv == tfp->expect) {
            uint16_t crc  = util_getbigendian2(tfp->msg_buf + tfp->recv - 6);
            uint32_t tail = util_getbigendian4(tfp->msg_buf + tfp->recv - 4);

            if (tail == TFP_FRM_TAIL) {
                // todo: check crc
                if (crc == 0x0) {
                    tfp->msg_cbk(tfp->msg_buf, tfp->recv - 6);
                } else {
                    log_printf("!!! crc invalid\n");
                }
            } else {
                log_printf("!!! tail invalid\n");
            }
            tfp->recv   = 0;
            tfp->expect = 0;
            tfp->head   = 0;
        }
    }
    return eat;
}


void tfp_make_frame(tfp_t* tfp, const uint8_t* data, uint16_t size)
{
    if (tfp->frm_buf == nullptr || tfp->frm_bufsz == 0 || tfp->frm_cbk == nullptr) {
        return;
    }
    if (size + 12 > tfp->frm_bufsz) {
        log_printf("!!! msg oversize\n");
        return;
    }

    uint16_t crc = 0;   // todo

    util_setbigendian4(tfp->frm_buf, TFP_FRM_HEAD);              // head 4
    util_setbigendian2(tfp->frm_buf + 4, size);                  // len  2
    memcpy(tfp->frm_buf + 6, data, size);                        // data
    util_setbigendian2(tfp->frm_buf + 6 + size, crc);            // crc  2
    util_setbigendian4(tfp->frm_buf + 8 + size, TFP_FRM_TAIL);   // tail 4

    tfp->frm_cbk(tfp->frm_buf, size + 12);
}
