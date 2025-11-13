#ifndef __WIFI_MAC_H__
#define __WIFI_MAC_H__

#include "wifi_driver.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define MAC_SIZE 6


typedef union mac_addr {
    uint8_t mac_bytes[6];
} mac_addr_t;

typedef struct wifi_mac_frame_hdr {
    struct mac_frame_control {
        unsigned    protocol_version    : 2;
        unsigned    type            : 2;
        unsigned    sub_type        : 4;
        unsigned    to_ds           : 1;
        unsigned    from_ds         : 1;
        unsigned    _flags:6;        
    } __attribute__((packed)) frame_control;
    // 
    uint16_t  duration_id;
    mac_addr_t address1;
    mac_addr_t address2;
    mac_addr_t address3;
    struct mac_sequence_control {
        unsigned    fragment_number     : 4;
        unsigned    sequence_number     : 12;
    } __attribute__((packed)) sequence_control;
} __attribute__((packed)) wifi_mac_frame_hdr_t;


// Template callback function similar to the standard wifi RX CB for compatibility
typedef esp_err_t (*wifi_mac_rx_cb_t)(void *buffer, uint16_t len, void *eb);

// Perform packet encapsulation and queue for transmission
// wifi_if is unused. It is only there to maintain compatibility with the drivers
esp_err_t wifi_mac_send_frame(uint32_t wifi_if, uint8_t *pkt, uint32_t len);

void wifi_mac_handle_rx(uint8_t* pkt, uint32_t len);

void wifi_mac_task(void);

void wifi_mac_set_rxcb(wifi_mac_rx_cb_t wifi_cb);

extern QueueHandle_t driver_event_q_hdl;
extern StaticQueue_t driver_event_q;

extern QueueHandle_t mac_event_q_hdl;
extern StaticQueue_t mac_event_q;

#endif