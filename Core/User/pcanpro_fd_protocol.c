#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "pcan_usbpro_fw.h"
#include "pcanfd_usb_fw.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_protocol.h"
#include "pcanpro_led.h"
#include "pcanpro_usbd.h"

#include "util_misc.h"
#include "can.h"
#include "log.h"

#define CAN_CHANNEL_MAX     (1)

struct pcan_usbfd_fw_info
{
  uint16_t  size_of;        /* sizeof this */
  uint16_t  type;            /* type of this structure */
  uint8_t    hw_type;        /* Type of hardware (HW_TYPE_xxx) */
  uint8_t    bl_version[3];  /* Bootloader version */
  uint8_t    hw_version;      /* Hardware version (PCB) */
  uint8_t    fw_version[3];  /* Firmware version */
  uint32_t  dev_id[2];      /* "device id" per CAN */
  uint32_t  ser_no;          /* S/N */
  uint32_t  flags;          /* special functions */
  uint8_t   unk[8];
} __attribute__ ((packed));

static struct
{
  uint32_t device_nr;
  uint32_t last_bus_load_update;
  uint32_t last_time_sync;
  uint32_t last_time_flush;
  uint8_t  can_drv_loaded;
  uint8_t  lin_drv_loaded;

  struct
  {
    /* config */
    uint8_t   silient;
    uint8_t   bus_active;
    uint8_t   loopback;
    uint8_t   err_mask;
    uint32_t  channel_nr;
    uint16_t  opt_mask;

    uint8_t   led_is_busy;

    /* slow speed */
    struct ucan_timing_slow slow_br;
    /* can fd , data fast speed */
    struct ucan_timing_fast fast_br;

    /* clock */
    uint32_t can_clock;

    uint32_t nominal_qt;
    uint32_t data_qt;
    uint32_t tx_time_ns;
    uint32_t rx_time_ns;

    uint32_t bus_load;
  }
  can[CAN_CHANNEL_MAX];
}
pcan_device =
{
  .device_nr = 0xFFFFFFFF,

  .can[0] = 
  {
    .channel_nr = 0xFFFFFFFF,
    .can_clock = 80000000u
  },
  // .can[1] = 
  // {
  //   .channel_nr = 0xFFFFFFFF,
  //   .can_clock = 80000000u
  // },
};

#define PCAN_USB_DATA_BUFFER_SIZE   2048
static uint8_t resp_buffer[2][PCAN_USB_DATA_BUFFER_SIZE];
static uint8_t drv_load_packet[16];

static uint16_t data_pos = 0;
static uint8_t   data_buffer[PCAN_USB_DATA_BUFFER_SIZE];

void *pcan_data_alloc_buffer( uint16_t type, uint16_t size )
{
  uint16_t aligned_size = (size+(4-1))&(~(4-1));
  if( sizeof( data_buffer ) < (aligned_size+data_pos+4) )
    return (void*)0;
  struct ucan_msg *pmsg = (void*)&data_buffer[data_pos];

  pmsg->size = aligned_size;
  pmsg->type = type;
  pmsg->ts_low = pcan_timestamp_us();
  pmsg->ts_high = 0;

  data_pos += aligned_size;
  return pmsg;
}

static struct t_m2h_fsm resp_fsm[2] = 
{
  [0] = {
    .state = 0,
    .ep_addr = PCAN_USB_EP_CMDIN,
    .pdbuf = resp_buffer[0],
    .dbsize = PCAN_USB_DATA_BUFFER_SIZE,
  },
  [1] = {
    .state = 0,
    .ep_addr = PCAN_USB_EP_MSGIN_CH1,
    .pdbuf = resp_buffer[1],
    .dbsize = PCAN_USB_DATA_BUFFER_SIZE,
  }
};

/* low level requests */
uint8_t pcan_protocol_device_setup( USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req )
{
  switch( req->bRequest )
  {
    case USB_VENDOR_REQUEST_INFO:
      switch( req->wValue )
      {
        case USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE:
        {
          static struct pcan_usbfd_fw_info fwi =
          {
            .size_of = sizeof( struct pcan_usbfd_fw_info ),
            .type = 2,
            .hw_type = 1,
#if ( PCAN_X6 == 1 )
            .bl_version = { 1, 1, 0 },
            .hw_version = 3,
#else
            .bl_version = { 2, 1, 0 }, /* bootloader v > 2 support massstorage mode */
            .hw_version = 2,
#endif
            .fw_version = { 3, 2, 0 },
            .dev_id[0] = 0xFFFFFFFF,
            .dev_id[1] = 0xFFFFFFFF,
            .ser_no = 0xFFFFFFFF,
            .flags = 0x00000000,
            .unk = { 
              0x01, /* cmd_out */
              0x81, /* cmd_in */
              0x02, /* write */
              0x03, /* write */
              0x82, /* read */
              0x00,
              0x00,
              0x00 
              }
          };
          /* windows/linux has different struct size */
          fwi.size_of = req->wLength;
          fwi.dev_id[0] = pcan_device.can[0].channel_nr;
          // fwi.dev_id[1] = pcan_device.can[1].channel_nr;
          fwi.ser_no = pcan_device.device_nr;
          return USBD_CtlSendData( pdev,  (void*)&fwi, fwi.size_of );
        }
        default:
          assert(0);
        break;
      }
      break;
    case USB_VENDOR_REQUEST_FKT:
      switch( req->wValue )
      {
        case USB_VENDOR_REQUEST_wVALUE_SETFKT_BOOT:
          break;
        case USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED:
        {
          USBD_CtlPrepareRx( pdev, drv_load_packet, 16 );
          return USBD_OK;
        }
        break;
        default:
          assert(0);
          break;
      }
      break;
    case USB_VENDOR_REQUEST_ZERO:
      break;
    default:
      USBD_CtlError( pdev, req );
      return USBD_FAIL; 
  }

  return USBD_FAIL;
}

void pcan_ep0_receive( void )
{
  if( drv_load_packet[0] == 0 )
  {
    pcan_flush_ep( PCAN_USB_EP_MSGIN_CH1 );
    pcan_flush_ep( PCAN_USB_EP_CMDIN );
    pcan_device.can_drv_loaded = drv_load_packet[1];
    pcan_led_set_mode( LED_STAT, LED_MODE_BLINK_SLOW, 0xFFFFFFFF );
  }
  else
    pcan_device.lin_drv_loaded = drv_load_packet[1];
}

static inline uint32_t can_msg_time(bool ide, bool brs, uint8_t len, uint32_t nt, uint32_t dt)
{
    const uint32_t data_bits    = (uint32_t)len << 3;
    const uint32_t control_bits = ide ? 67 : 47;
    if (brs)
        return (control_bits * nt) + (data_bits * dt);
    else
        return (control_bits + data_bits) * nt;
}

void pcan_protocol_rx_frame( uint8_t channel, uint32_t id, bool ide, bool fdf, bool brs, const uint8_t* data, uint8_t len )
{
  static const uint8_t pcan_fd_len2dlc[] = 
  {
    0, 1, 2, 3, 4, 5, 6, 7, 8,	/* 0 - 8 */
    9, 9, 9, 9,	    /* 9 - 12 */
    10, 10, 10, 10,	    /* 13 - 16 */
    11, 11, 11, 11,	    /* 17 - 20 */
    12, 12, 12, 12,	    /* 21 - 24 */
    13, 13, 13, 13, 13, 13, 13, 13,	/* 25 - 32 */
    14, 14, 14, 14, 14, 14, 14, 14,	/* 33 - 40 */
    14, 14, 14, 14, 14, 14, 14, 14,	/* 41 - 48 */
    15, 15, 15, 15, 15, 15, 15, 15,	/* 49 - 56 */
    15, 15, 15, 15, 15, 15, 15, 15	/* 57 - 64 */
  };
  struct ucan_rx_msg *pcan_msg = pcan_data_alloc_buffer( UCAN_MSG_CAN_RX, sizeof(struct ucan_rx_msg));
  if( !pcan_msg )
    return;

  if( len > 64 )
    len = 64;

  // if( !(pmsg->flags & MSG_FLAG_ECHO) )
  {
    uint32_t nqt = pcan_device.can[channel].nominal_qt;
    uint32_t dqt = pcan_device.can[channel].data_qt;
    pcan_device.can[channel].rx_time_ns += can_msg_time( ide, brs, len, nqt, dqt );
    if( !pcan_device.can[channel].led_is_busy )
    {
      pcan_led_set_mode( channel ? LED_CH1_RX:LED_CH0_RX, LED_MODE_BLINK_FAST, 237 );
    }
  }

  pcan_msg->channel_dlc = UCAN_MSG_CHANNEL_DLC( channel, pcan_fd_len2dlc[len] );
  pcan_msg->client = 0; // pmsg->dummy;
  pcan_msg->flags = 0;
  pcan_msg->tag_low = 0;
  pcan_msg->tag_high = 0;

  /* we support only regular frames */
  // if( pmsg->flags & MSG_FLAG_RTR )
  //   pcan_msg->flags |= UCAN_MSG_RTR;
  if( ide )
    pcan_msg->flags |= UCAN_MSG_EXT_ID;
  if( fdf )
  {
    pcan_msg->flags |= UCAN_MSG_EXT_DATA_LEN;
    if( brs )
      pcan_msg->flags |= UCAN_MSG_BITRATE_SWITCH;
    // if( pmsg->flags & MSG_FLAG_ESI )
    //   pcan_msg->flags |= UCAN_MSG_ERROR_STATE_IND;
  }
  // if( pmsg->flags & MSG_FLAG_ECHO )
  // {
  //   pcan_msg->flags |= UCAN_MSG_API_SRR | UCAN_MSG_HW_SRR;
  //   pcan_msg->ts_low = pcan_timestamp_us();
  // }
  // else
  {
    pcan_msg->ts_low = pcan_timestamp_us();
  }

  pcan_msg->can_id = id;
  memcpy( pcan_msg->d, data, len );
}

void pcan_protocol_tx_frame_cb( uint8_t channel, uint32_t id, bool ide, bool fdf, bool brs, const uint8_t* data, uint8_t len )
{
  if( !pcan_device.can[channel].led_is_busy )
  {
    pcan_led_set_mode( channel ? LED_CH1_TX:LED_CH0_TX, LED_MODE_BLINK_FAST, 237 );
  }

  uint32_t nqt = pcan_device.can[channel].nominal_qt;
  uint32_t dqt = pcan_device.can[channel].data_qt;
  pcan_device.can[channel].tx_time_ns += can_msg_time( ide, brs, len, nqt, dqt );
}

int pcan_protocol_tx_frame( struct ucan_tx_msg *pmsg )
{
  static const uint8_t pcan_fd_dlc2len[] = 
  {
    0, 1, 2, 3, 4, 5, 6, 7,
    8, 12, 16, 20, 24, 32, 48, 64
  };

  uint8_t  channel;
  uint32_t id;
  uint8_t  len;
  bool     ide = false;
  bool     fdf = false;
  bool     brs = false;

  channel = UCAN_MSG_CHANNEL(pmsg);

  if( channel >= CAN_CHANNEL_MAX )
    return -1;

  id = pmsg->can_id;

  /* CAN-FD frame */
  if( pmsg->flags & UCAN_MSG_EXT_DATA_LEN )
  {
    fdf = true;
    if( pmsg->flags & UCAN_MSG_BITRATE_SWITCH )
      brs = true;
    if( pmsg->flags & UCAN_MSG_ERROR_STATE_IND )
      return 0; // ! todo: ignore

    len = pcan_fd_dlc2len[UCAN_MSG_DLC(pmsg)];
  }
  else
  {
    len = UCAN_MSG_DLC(pmsg);
  }

  /* TODO: process UCAN_MSG_SINGLE_SHOT, UCAN_MSG_HW_SRR, UCAN_MSG_ERROR_STATE_IND */
  if( pmsg->flags & UCAN_MSG_RTR )
    return 0; // ! todo: ignore

  if( pmsg->flags & UCAN_MSG_EXT_ID )
    ide = true;

  if( pmsg->flags & (/*UCAN_MSG_API_SRR|*/UCAN_MSG_HW_SRR) )
  {
    return 0; // ! todo: ignore
  }

  if( can_send(id, ide, fdf, brs, pmsg->d, len) != 0 )
  {
    /* TODO: tx queue overflow ? */
    ;
  }
  return 0;
}

static int pcan_protocol_send_status( uint8_t channel, uint8_t status )
{
  struct ucan_status_msg *ps = pcan_data_alloc_buffer( UCAN_MSG_STATUS, sizeof( struct ucan_status_msg ) );
  if( !ps )
    return -1;

  ps->channel_p_w_b = channel&0x0f;
  ps->channel_p_w_b |= (status&0x0f)<<4;
  return 0;
}

int pcan_protocol_set_baudrate( uint8_t channel, uint16_t brp, uint16_t tseg1, uint8_t tseg2, uint8_t sjw, bool is_data_bitrate )
{
  uint32_t bitrate = ((pcan_device.can[channel].can_clock) / brp) / (1 + tseg1 + tseg2);
  uint8_t  sample  = (1 + tseg1) * 100 / (1 + tseg1 + tseg2);

  // log_printf("pcan %s: frp=%d, brp=%d,tseg1=%d,tseg2=%d,sjw=%d\n", is_data_bitrate ? "data" : "nomi", pcan_device.can[channel].can_clock, brp, tseg1, tseg2, sjw);

  can_set_bitrate( channel, bitrate, sample, is_data_bitrate );

  if( is_data_bitrate )
  {
    pcan_device.can[channel].data_qt = 1000000000u/bitrate;
  }
  else
  {
    pcan_device.can[channel].nominal_qt = 1000000000u/bitrate;
  }

  return 0;
}

static void pcan_protocol_process_cmd( uint8_t *ptr, uint16_t size )
{
  struct ucan_command *pcmd = (void*)ptr;

  while( size >= sizeof( struct ucan_command ) )
  {
    switch( UCAN_CMD_OPCODE( pcmd ) )
    {
      case UCAN_CMD_NOP:
        break;
      case UCAN_CMD_RESET_MODE:
      {
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          can_set_bus_active( UCAN_CMD_CHANNEL(pcmd) , 0 );
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].bus_active = 0;

          /* update ISO mode only on inactive bus */
          uint8_t bus_iso_mode = ( pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].opt_mask & UCAN_OPTION_ISO_MODE ) != 0;
          can_set_iso_mode( UCAN_CMD_CHANNEL(pcmd), bus_iso_mode );
        }
      }
        break;
      case UCAN_CMD_NORMAL_MODE:
      {
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          if( pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].silient  )
          {
            can_set_silent( UCAN_CMD_CHANNEL(pcmd) , 0 );
            pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].silient = 0;
          }
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].bus_active = 1;
          can_set_bus_active( UCAN_CMD_CHANNEL(pcmd) , 1 );
          pcan_protocol_send_status( UCAN_CMD_CHANNEL(pcmd), 0 );
        }
      }
        break;
      case UCAN_CMD_LISTEN_ONLY_MODE:
      {
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          can_set_silent( UCAN_CMD_CHANNEL(pcmd) , 1 );
          can_set_bus_active( UCAN_CMD_CHANNEL(pcmd) , 1 );
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].silient = 1;
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].bus_active = 1;
          pcan_protocol_send_status( UCAN_CMD_CHANNEL(pcmd), 0 );
        }
      }
        break;
      case UCAN_CMD_TIMING_SLOW:
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          struct ucan_timing_slow *ptiming = (void*)pcmd;
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].slow_br = *ptiming;

          pcan_protocol_set_baudrate( UCAN_CMD_CHANNEL(pcmd), ptiming->brp + 1, ptiming->tseg1 + 1, ptiming->tseg2 + 1, (ptiming->sjw_t & 0x0f)+1, false);
        }
        break;
      /* only for CAN-FD */
      case UCAN_CMD_TIMING_FAST:
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          struct ucan_timing_fast *ptiming = (void*)pcmd;
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].fast_br = *ptiming;

          pcan_protocol_set_baudrate( UCAN_CMD_CHANNEL(pcmd), ptiming->brp + 1, ptiming->tseg1 + 1, ptiming->tseg2 + 1, (ptiming->sjw & 0x0f)+1, true);
        }
        break;
      case UCAN_CMD_SET_STD_FILTER:
        if( UCAN_CMD_CHANNEL(pcmd) >= CAN_CHANNEL_MAX )
          break;
        can_set_filter_mask( UCAN_CMD_CHANNEL(pcmd), 0, 0, 0 );
        break;
      case UCAN_CMD_RESERVED2:
        break;
      case UCAN_CMD_FILTER_STD:
        if( UCAN_CMD_CHANNEL(pcmd) >= CAN_CHANNEL_MAX )
          break;
        can_set_filter_mask( UCAN_CMD_CHANNEL(pcmd), 0, 0, 0 );
        break;
      case UCAN_CMD_TX_ABORT:
        break;
      case UCAN_CMD_WR_ERR_CNT:
        break;
      case UCAN_CMD_SET_EN_OPTION:
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          struct ucan_option *popt = (void*)pcmd;
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].opt_mask |= popt->mask;
        }
        break;
      case UCAN_CMD_CLR_DIS_OPTION:
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          struct ucan_option *popt = (void*)pcmd;
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].opt_mask &= ~popt->mask;
        }
        break;
      case UCAN_CMD_SET_ERR_GEN1:
        break;
      case UCAN_CMD_SET_ERR_GEN2:
        break;
      case UCAN_CMD_DIS_ERR_GEN:
        break;
      case UCAN_CMD_RX_BARRIER:
        break;
      case UCAN_CMD_SET_ERR_GEN_S:
        break;
      case UCAN_USB_CMD_CLK_SET:
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          struct ucan_usb_clock *pclock = (void*)pcmd;
          switch( pclock->mode )
          {
            default:
            case UCAN_USB_CLK_80MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 80000000u;
            break;
            case UCAN_USB_CLK_60MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 60000000u;
            break;
            case UCAN_USB_CLK_40MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 40000000u;
            break;
            case UCAN_USB_CLK_30MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 30000000u;
            break;
            case UCAN_USB_CLK_24MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 24000000u;
            break;
            case UCAN_USB_CLK_20MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 20000000u;
            break;
          }
        }
        break;
      case UCAN_USB_CMD_LED_SET:
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          struct ucan_usb_led *pled = (void*)pcmd;
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].led_is_busy = pled->mode;
          pcan_led_set_mode( UCAN_CMD_CHANNEL(pcmd), pled->mode, 0xFFFFFFFF );
        }
        break;
      case UCAN_USB_CMD_DEVID_SET:
      {
        struct ucan_usb_device_id *pdevid = (void*)pcmd;
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].channel_nr = pdevid->device_id;
        }
        break;
      }
      case 0x87: /* CAN FD ISO MODE, 0xff - enable, 0x55 - disable  */
      {
        /* do nothing here */
        break;
      }
      case UCAN_CMD_END_OF_COLLECTION:
        return;
      default:
        assert( 0 );
        break;
    }

    size -= sizeof( struct ucan_command );
    ++pcmd;
  }
}

void pcan_protocol_process_data( uint8_t ep, uint8_t *ptr, uint16_t size )
{
  if( ep == 1 )
  {
    pcan_protocol_process_cmd( ptr, size );
    return;
  }
  static union {
      struct ucan_tx_msg txmsg;
      uint8_t            data[sizeof(struct ucan_tx_msg)];
  } rxbuf;
  static uint16_t     rxlen  = 0;
  static uint16_t     msglen = 0;
  struct ucan_tx_msg* pmsg = nullptr;
  uint16_t            eat  = 0;

  while (size) {
    if (size < 4) {
        break;
    }

    if (rxlen == 0 || msglen == 0) {
        pmsg = (void*)ptr;
        if (pmsg->size == 0 || pmsg->type == 0) {
            break;
        }
        msglen = pmsg->size;
        if (msglen < size) {
            rxlen = msglen;
            eat   = msglen;
        } else {
            rxlen = 0;
            eat   = 0;
        }
    }

    if (rxlen < msglen){
        eat = util_min2(size, msglen - rxlen);
        memcpy(rxbuf.data + rxlen, ptr, eat);
        rxlen += eat;
        if (rxlen >= msglen) {
            pmsg = &rxbuf.txmsg;
        }
    }

    if (rxlen >= msglen) {
      rxlen  = 0;
      msglen = 0;

      switch( pmsg->type )
      {
        //to host only
        //UCAN_MSG_ERROR:
        //UCAN_MSG_BUSLOAD:
        case UCAN_MSG_CAN_TX:
          pcan_protocol_tx_frame( (struct ucan_tx_msg *)pmsg );
          break;
        case UCAN_MSG_CAN_TX_PAUSE:
          /* TODO: */
          break;
        case UCAN_CMD_END_OF_COLLECTION:
        case 0xffff:
          return;
        default:
          assert( 0 );
          break;  
      }
    }

    size -= eat;
    ptr += eat;
  }
}

void pcan_protocol_init( void )
{
  can_install_rx_callback( 0, pcan_protocol_rx_frame );
  can_install_tx_callback( 0, pcan_protocol_tx_frame_cb );
}

void pcan_protocol_poll( void )
{
  uint32_t ts_ms = pcan_timestamp_millis();
  uint32_t ts_us = pcan_timestamp_us();

  /* flush data */
  if( data_pos > 0 )
  {
    /* endmark */
    *(uint32_t*)&data_buffer[data_pos] = 0x00000000;
    uint16_t flush_size = data_pos + 4;
    /* align to 64 */
    flush_size += (64-1);
    flush_size &= ~(64-1);
    int res = pcan_flush_data( &resp_fsm[1], data_buffer, flush_size );
    if( res )
    { 
      data_pos = 0;
      pcan_device.last_time_flush = ts_us;
    }
  }
#if 0
  else
  {
    ts_us -= pcan_device.last_time_flush;
    if( pcan_device.last_time_flush && ( ts_us > 800 ) )
    {
      int res = pcan_flush_data( &resp_fsm[1], 0, 0 );
      if( res )
      {
        pcan_device.last_time_flush = 0;
      }
    }
  }
  #endif

  /* timesync part */
  if( !pcan_device.can_drv_loaded )
    return;

  /* update bus load each 250ms */
  if( ( ts_ms - pcan_device.last_bus_load_update ) >= 250u )
  {
    pcan_device.last_bus_load_update = ts_ms;
    for( int i = 0; i < CAN_CHANNEL_MAX; i++ )
    {
      uint32_t total_ns = pcan_device.can[i].tx_time_ns + pcan_device.can[i].rx_time_ns;
      /* get bus in percents 0 - 100% */
      pcan_device.can[i].bus_load = total_ns / (250000000u/100u);

      pcan_device.can[i].tx_time_ns = 0;
      pcan_device.can[i].rx_time_ns = 0;
    }
  }

  if( ( ts_ms - pcan_device.last_time_sync ) < 1000u )
    return;
  struct ucan_usb_ts_msg *pts = pcan_data_alloc_buffer( UCAN_USB_MSG_CALIBRATION, sizeof( struct ucan_usb_ts_msg ) );
  if( !pts )
    return;

  pts->usb_frame_index = pcan_usb_frame_number();
  pts->unused = 0;
  pcan_device.last_time_sync = ts_ms;

  for( int i = 0; i < CAN_CHANNEL_MAX; i++ )
  {
    if( !(pcan_device.can[i].opt_mask & UCAN_OPTION_BUSLOAD) )
      continue;
    if( !pcan_device.can[i].bus_active )
      continue;
    struct ucan_bus_load_msg *pbs = pcan_data_alloc_buffer(  UCAN_MSG_BUSLOAD, sizeof( struct ucan_bus_load_msg ) );
    if( !pbs )
      return;

    pbs->channel = i;
    /* 0 ... 100% => 0 ... 4095 */
    pbs->bus_load = (pcan_device.can[i].bus_load*4095u)/100u;
  }
  
}
