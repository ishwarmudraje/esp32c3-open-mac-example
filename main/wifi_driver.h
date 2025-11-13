#ifndef __HW_TASK_H__
#define __HW_TASK_H__

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Register definitions
// MAC registers
#define WIFI_MAC_BASE 0x60033000
#define WIFI_BASE_RX_DSCR 0x60033088
#define WIFI_NEXT_RX_DSCR 0x6003308c
#define WIFI_LAST_RX_DSCR 0x60033090
#define WIFI_LAST_RXBUF_CONFIG 0x6003309c
#define WIFI_MAC_BITMASK_084 0x60033084
#define WIFI_TX_CONFIG 0x60033d04
#define WIFI_MAC_CTRL 0x60033ca0
#define WIFI_TX_PLCP0 0x60033d08
#define WIFI_TX_PLCP1 0x600342f8
#define WIFI_TX_PLCP1_2 0x600342fc
#define WIFI_TX_HTSIG 0x60034310
#define WIFI_TX_PLCP2 0x60034314
#define WIFI_TX_DURATION 0x60034318
#define WIFI_TX_STATUS 0x60033cb0
#define WIFI_TX_CLR 0x60033cac
#define WIFI_TX_GET_ERR 0x60033ca8
#define WIFI_TX_CLR_ERR 0x60033ca4
#define WIFI_INT_STATUS_GET 0x60033c3c
#define WIFI_INT_STATUS_CLR 0x60033c40
#define PWR_INT_STATUS_GET 0x60035118
#define PWR_INT_STATUS_CLR 0x6003511c
#define WIFI_MAC_CCA_REG 0x60033c50
#define WIFI_OFFSET_REG 0x60033c64


// Intererupt registers
#define INTR_SRC_MAC 0x600c2000
#define INTR_SRC_PWR 0x600c2008
#define INTR_ENABLE_REG 0x600c2104 // Writing a 1 to corresponding bit enables and writing 0 disables
#define INTR_STATUS_REG 0x600c00F8

#define WIFI_INTR_NUMBER 1
#define SYSTICK_INTR_NUMBER 7 // Tick 
#define TIMER_ALARM_NUMBER 3
#define TASK_WDT_NUMBER 9

#define WIFI_FRAME_SIZE 1534 // MTU + headers + FCS

typedef struct dma_list_item {
	uint16_t size : 12;
	uint16_t length : 12;
	uint8_t _unknown : 6;
	uint8_t has_data : 1;
	uint8_t owner : 1;
	void* packet;
	struct dma_list_item* next;
} __attribute__((packed,aligned(4))) dma_list_item_t;

// Data types for queueing
typedef enum {
	INTR_ENTRY,
	TX_ENTRY,
	RX_ENTRY
} hardware_queue_entry_type_t;

typedef struct
{
	uint32_t interrupt_received;
} intr_queue_entry_t;

typedef struct
{
	uint8_t* packet;
	uint32_t len;
} tx_queue_entry_t;

typedef struct {
	hardware_queue_entry_type_t type;
	union {
		intr_queue_entry_t intr;
		tx_queue_entry_t tx;
		tx_queue_entry_t rx;
	} content;
} hardware_queue_entry_t;

// RTOS functions
extern bool pp_post(uint32_t requestnum, uint32_t argument);

// Send 802.11 packet over the interface
void wifi_transmit_packet(uint8_t *pkt, uint32_t len);

// Receive 802.11 packet over the interface
uint32_t wifi_receive_packet(uint8_t* pkt);

void wifi_hardware_task(void);

void wifi_get_bssid(uint8_t* mac_buffer);

// eb argument is not used as there is only one EB.
// Kept for compatibility and for future implementations using EB pools
// like the official binaries
void wifi_free_eb(uint8_t *eb);

extern uint8_t wifi_frame_buffer[WIFI_FRAME_SIZE];
extern uint8_t s_frame_buffer_used; // Makeshift mutex

#endif