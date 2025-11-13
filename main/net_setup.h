#ifndef __NET_SETUP_H__
#define __NET_SETUP_H__

#include "esp_err.h"

void net_setup();
esp_err_t custom_wifi_deinit();
esp_err_t open_wifi_rx_cb(void *buffer, uint16_t len, void *eb);

#endif