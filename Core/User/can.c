#include "can.h"

#include "log.h"
#include "stm32g4xx_hal.h"
#include "system.h"
#include "util_misc.h"
#include "util_ringbuffer.h"
#include <string.h>

#define CAN_TX_QUEUE_SIZE 128

typedef struct {
    uint32_t frq;
    uint32_t bitrate;
    uint8_t  sample;
    uint8_t  brp;     // bitrate prescaler
    uint8_t  tsync;   // always 1
    uint8_t  tseg1;
    uint8_t  tseg2;
} bitrate_config_t;

typedef struct {
    uint32_t id;
    bool     fdf;
    bool     ide;
    bool     brs;
    uint8_t  len;
    uint8_t  data[64];
} can_frame_t;

extern FDCAN_HandleTypeDef    hfdcan1;
static can_frame_t            frm_buffer[CAN_TX_QUEUE_SIZE];
static uint32_t               wr_pos, rd_pos, tx_num;
static bool                   connected         = false;
static can_err_notify_t       can_err_notify    = nullptr;
static can_tx_confirm_t       can_tx_confirm    = nullptr;
static can_rx_indicate_t      can_rx_indicate   = nullptr;
const static bitrate_config_t bitrate_configs[] = {
    // clock   = apb1
    // bitrate = 1 / (1 / (frq / brp) * (tsync + tseg1 + tseg1))
    // sample  = (tsync + tseg1) / (tsync + tseg1 + tseg1))
    // clang-format off
    // frq        bitrate  sample(%)  brp, tsync  tseg1  tseg2
    {  170000000, 500000,  82,        20,  1,     13,    3    },
    {  170000000, 1000000, 82,        10,  1,     13,    3    },
    {  170000000, 2000000, 82,        5,   1,     13,    3    },
    {  170000000, 5000000, 82,        2,   1,     13,    3    },
    // clang-format on
};


static uint8_t dlc_to_len(uint32_t dlc)
{
    static const uint8_t dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return dlc2len[dlc & 0x0f];
}

static uint32_t len_to_dlc(uint8_t len)
{
    if (len <= 8) {
        return len;
    }
    if (len <= 12) {
        return FDCAN_DLC_BYTES_12;
    }
    if (len <= 16) {
        return FDCAN_DLC_BYTES_16;
    }
    if (len <= 20) {
        return FDCAN_DLC_BYTES_20;
    }
    if (len <= 24) {
        return FDCAN_DLC_BYTES_24;
    }
    if (len <= 32) {
        return FDCAN_DLC_BYTES_32;
    }
    if (len <= 48) {
        return FDCAN_DLC_BYTES_48;
    }
    if (len <= 64) {
        return FDCAN_DLC_BYTES_64;
    }
    return 0;
}

static void can_rx_proc(void)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t               data[64];
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, data) == HAL_OK) {
            if (can_rx_indicate) {
                can_rx_indicate(0,                                      // bus
                                header.Identifier,                      // id
                                header.IdType == FDCAN_EXTENDED_ID,     // ide
                                header.FDFormat == FDCAN_FD_CAN,        // fdf
                                header.BitRateSwitch == FDCAN_BRS_ON,   // brs
                                data,                                   // data
                                dlc_to_len(header.DataLength)           // len
                );
            }
        }
    }
}

static void can_tx_proc(void)
{
    if (tx_num != 0 && HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
        FDCAN_TxHeaderTypeDef header = {0};   // important to init 0
        can_frame_t*          frame  = frm_buffer + rd_pos;

        rd_pos = (rd_pos + 1) % CAN_TX_QUEUE_SIZE;
        tx_num--;

        header.Identifier    = frame->id;
        header.BitRateSwitch = frame->brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
        header.IdType        = frame->ide ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
        header.FDFormat      = frame->fdf ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
        header.DataLength    = len_to_dlc(frame->len);
        header.TxFrameType   = FDCAN_DATA_FRAME;

        // log_printf("can%s dlc=%d\n", header.FDFormat == FDCAN_FD_CAN ? "fd" : "", header.DataLength);

        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, frame->data) != 0) {
            log_printf("send frame fail\n");
        }
    }
}

static void can_state_reset(void)
{
    wr_pos    = 0;
    rd_pos    = 0;
    tx_num    = 0;
    connected = false;
}

void can_process(void)
{
    if (connected) {
        can_rx_proc();
        can_tx_proc();
    }
}

int can_send(uint32_t id, bool ide, bool fdf, bool brs, const uint8_t* data, uint8_t len)
{
    if (tx_num == CAN_TX_QUEUE_SIZE) {
        return -1;
    }
    frm_buffer[wr_pos].id  = id;
    frm_buffer[wr_pos].ide = ide;
    frm_buffer[wr_pos].fdf = fdf;
    frm_buffer[wr_pos].brs = brs;
    frm_buffer[wr_pos].len = len;
    memcpy(frm_buffer[wr_pos].data, data, len);

    wr_pos = (wr_pos + 1) % CAN_TX_QUEUE_SIZE;
    tx_num++;
    return 0;
}

int can_config(uint32_t nomi_bitrate, uint8_t nomi_sample, uint32_t data_bitrate, uint8_t data_sample)
{
    log_printf("can init:\n  nomi: %6d-%d%%\n  data: %6d-%d%%\n",   //
               nomi_bitrate, nomi_sample, data_bitrate, data_sample);

    hfdcan1.Instance                = FDCAN1;
    hfdcan1.Init.ClockDivider       = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat        = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.Mode               = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause      = DISABLE;
    hfdcan1.Init.ProtocolException  = DISABLE;
    hfdcan1.Init.StdFiltersNbr      = 0;
    hfdcan1.Init.ExtFiltersNbr      = 0;
    hfdcan1.Init.TxFifoQueueMode    = FDCAN_TX_FIFO_OPERATION;

    int nomi_cfg = -1, data_cfg = -1;
    for (int i = 0; i < util_arraylen(bitrate_configs); i++) {
        if (bitrate_configs[i].bitrate == nomi_bitrate) {
            nomi_cfg = i;
        }
        if (bitrate_configs[i].bitrate == data_bitrate) {
            data_cfg = i;
        }
    }

    if (nomi_cfg < 0 || data_cfg < 0) {
        log_printf("bitrate not support\n");
        return -1;
    } else {
        hfdcan1.Init.NominalPrescaler     = bitrate_configs[nomi_cfg].brp;
        hfdcan1.Init.NominalSyncJumpWidth = bitrate_configs[nomi_cfg].tsync;
        hfdcan1.Init.NominalTimeSeg1      = bitrate_configs[nomi_cfg].tseg1;
        hfdcan1.Init.NominalTimeSeg2      = bitrate_configs[nomi_cfg].tseg2;
        hfdcan1.Init.DataPrescaler        = bitrate_configs[data_cfg].brp;
        hfdcan1.Init.DataSyncJumpWidth    = bitrate_configs[data_cfg].tsync;
        hfdcan1.Init.DataTimeSeg1         = bitrate_configs[data_cfg].tseg1;
        hfdcan1.Init.DataTimeSeg2         = bitrate_configs[data_cfg].tseg2;

        if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
            log_printf("can init fail\n");
            return -2;
        }
        return 0;
    }
}

int can_connect(void)
{
    log_printf("can connect\n");
    can_state_reset();
    // error notification
    // HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_LIST_PROTOCOL_ERROR, 0);
    HAL_FDCAN_Start(&hfdcan1);
    connected = true;
    return 0;
}

int can_unconnect(void)
{
    log_printf("can unconnect\n");
    connected = false;
    HAL_FDCAN_Stop(&hfdcan1);
    return 0;
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs)
{
    log_printf("Can ErrorStatus %08X\n", ErrorStatusITs);
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hfdcan)
{
    log_printf("Can Error\n");
}

int can_set_filter_mask(uint8_t bus, bool ide, uint32_t id, uint32_t mask)
{
    // todo: set filter
    // HAL_CAN_ConfigFilter(p_can, &filter)
    return 0;
}

void can_install_rx_callback(uint8_t bus, can_rx_indicate_t rx_indicate)
{
    can_rx_indicate = rx_indicate;
}

void can_install_tx_callback(uint8_t bus, can_tx_confirm_t tx_confirm)
{
    can_tx_confirm = tx_confirm;
}

void can_install_err_callback(uint8_t bus, can_err_notify_t err_notify)
{
    can_err_notify = err_notify;
}

void can_set_silent(uint8_t bus, bool silent)
{
    // todo
    // p_can->Init.Mode = silent_mode ? CAN_MODE_SILENT : CAN_MODE_NORMAL;
}

/* bxCAN does not support FDCAN ISO mode switch */
void can_set_iso_mode(uint8_t bus, bool iso_mode)
{
    // todo
}

void can_set_loopback(uint8_t bus, bool loopback)
{
    // todo
}

void can_set_bus_active(uint8_t bus, bool active)
{
    if (active) {
        can_connect();
    } else {
        can_unconnect();
    }
}

/* set predefined best values */
void can_set_bitrate(uint8_t bus, uint32_t bitrate, uint8_t sample, bool is_data_bitrate)
{
    static uint32_t nomi_bitrate = 500000;
    static uint32_t data_bitrate = 1000000;
    static uint8_t  nomi_sample  = 80;
    static uint8_t  data_sample  = 80;

    if (!is_data_bitrate) {
        // log_printf("pcan set nomi %d-%d%%\n", bitrate, sample);
        nomi_bitrate = bitrate;
        nomi_sample  = sample;
    } else {
        // log_printf("pcan set data %d-%d%%\n", bitrate, sample);
        data_bitrate = bitrate;
        data_sample  = sample;
        can_config(nomi_bitrate, nomi_sample, data_bitrate, data_sample);
    }
}
