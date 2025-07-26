#pragma once
#include <stdint.h>
#include "usbd_def.h"

#define PCAN_FS_MAX_BULK_PACKET_SIZE  (64)
#define PCAN_HS_MAX_BULK_PACKET_SIZE  (512)

#define PCAN_USB_VENDOR_ID        0x0483
#define PCAN_USBPROFD_PRODUCT_ID  0x0011
#define PCAN_USBPRO_PRODUCT_ID    0x000d

/* PCAN-USB Endpoints */
#define PCAN_USB_EP_CMDOUT        0x01
#define PCAN_USB_EP_CMDIN         0x81
#define PCAN_USB_EP_MSGOUT_CH1    0x02
#define PCAN_USB_EP_MSGIN_CH1     0x82
#define PCAN_USB_EP_MSGOUT_CH2    0x03
#define PCAN_USB_EP_MSGIN_CH2     0x83


#if ( PCAN_PRO_FD ) || ( PCAN_FD ) || ( PCAN_X6)
#define PCAN_DATA_PACKET_SIZE (256)
#define PCAN_CMD_PACKET_SIZE  (128)
#elif ( PCAN_PRO )
#define PCAN_DATA_PACKET_SIZE (64)
#define PCAN_CMD_PACKET_SIZE  (512)
#else
#error Oops
#endif

struct t_class_data
{
  int tx_flow_control_in_use;
  uint8_t ep_tx_in_use[15];
  uint8_t cmd_ep_buffer[PCAN_CMD_PACKET_SIZE];
  uint8_t data1_ep_buffer[PCAN_DATA_PACKET_SIZE];
  uint8_t data2_ep_buffer[PCAN_DATA_PACKET_SIZE];
};

struct t_m2h_fsm
{
  uint8_t   state;
  uint8_t   ep_addr;
  uint8_t   *pdbuf;
  uint32_t  dbsize;
  uint32_t  total_tx;
};

extern USBD_ClassTypeDef usbd_pcanpro;

void pcan_usb_device_poll( void );
uint16_t pcan_usb_frame_number( void );
int pcan_flush_ep( uint8_t ep );
int pcan_flush_data( struct t_m2h_fsm *pfsm, void *src, int size );
