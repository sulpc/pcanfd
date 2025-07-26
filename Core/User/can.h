#ifndef _CAN_H_
#define _CAN_H_

#include "util_types.h"

typedef void (*can_rx_indicate_t)(uint8_t bus, uint32_t id, bool ide, bool fdf, bool brs, const uint8_t* data,
                                  uint8_t len);
typedef void (*can_tx_confirm_t)(uint8_t bus, uint32_t id, bool ide, bool fdf, bool brs, const uint8_t* data,
                                 uint8_t len);
typedef void (*can_err_notify_t)(uint8_t bus, uint32_t err);

int  can_config(uint32_t nomi_bitrate, uint8_t nomi_sample, uint32_t data_bitrate, uint8_t data_sample);
int  can_connect(void);
int  can_unconnect(void);
int  can_send(uint32_t id, bool ide, bool fdf, bool brs, const uint8_t* data, uint8_t len);
void can_process(void);
void can_set_bitrate(uint8_t bus, uint32_t bitrate, uint8_t sample, bool is_data_bitrate);
void can_set_bus_active(uint8_t bus, bool active);
void can_set_silent(uint8_t bus, bool silent);
void can_set_iso_mode(uint8_t bus, bool iso_mode);
void can_set_loopback(uint8_t bus, bool loopback);
int  can_set_filter_mask(uint8_t bus, bool ide, uint32_t id, uint32_t mask);
void can_install_err_callback(uint8_t bus, can_err_notify_t err_notify);
void can_install_rx_callback(uint8_t bus, can_rx_indicate_t rx_indicate);
void can_install_tx_callback(uint8_t bus, can_tx_confirm_t tx_confirm);

#endif
