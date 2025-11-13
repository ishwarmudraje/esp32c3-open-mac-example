#include "wifi_driver.h"

#include "esp_attr.h"
#include "riscv/interrupt.h"

#include "esp_log.h"

#include "esp_wifi.h"

#include "rom/ets_sys.h"

#include "string.h"

#define BUFFER_SIZE 10
#define RX_BUFFER_AMOUNT 10

// Declare the TX DMA list item.
// Only using one since we do not have high traffic
dma_list_item_t tx_item = {
    .size = 83,
    .length = 95,
    ._unknown = 32,
    .has_data = 1,
    .owner = 1,
    .packet = NULL, // Set pointer before passing pointer to PLCP0 register
    .next = NULL
};

static const char* TAG = "wifi_driver";

typedef uint8_t rx_pkt_t[1600];

// Statically declare all rx dma items
// rx_pkt_buffers will be used by the respective DMA items
rx_pkt_t rx_pkt_buffers[RX_BUFFER_AMOUNT];

dma_list_item_t* rx_chain_begin = NULL;
dma_list_item_t* rx_chain_last = NULL;

dma_list_item_t rx_item_array[RX_BUFFER_AMOUNT];

// Makeshift single frame buffer for Rx handling
// MAC layer uses it to pass onto IP layer
uint8_t wifi_frame_buffer[WIFI_FRAME_SIZE];
uint8_t s_frame_buffer_used = false; // Makeshift mutex

// Static queues and handles
// Driver queue is written to by:
//      a. The driver when an interrupt is received
//      b. The MAC layer when packet needs to be Tx'd
QueueHandle_t driver_event_q_hdl;
StaticQueue_t driver_event_q;

// MAC event queue is written to by the driver when 
// packet is Rx'd
QueueHandle_t mac_event_q_hdl;
StaticQueue_t mac_event_q;

uint8_t driver_q_buffer[BUFFER_SIZE * sizeof(hardware_queue_entry_t)];
uint8_t mac_q_buffer[BUFFER_SIZE * sizeof(hardware_queue_entry_t)];

// MAC init (source: ghidra)
void wifi_hw_init(){
    // ic_mac_init decompilation
    uint32_t mac_val = REG_READ(WIFI_MAC_CTRL);
    REG_WRITE(WIFI_MAC_CTRL, mac_val &  0xff00efff);
}

// MAC deinit (source: ghidra)
void wifi_hw_deinit(){
    // ic_mac_init decompilation
    uint32_t mac_val = REG_READ(WIFI_MAC_CTRL);
    REG_WRITE(WIFI_MAC_CTRL, mac_val | 0xff1000);
}

// Method to disable rx (source: ghidra)
void wifi_disable_rx(){
    uint32_t value = REG_READ(WIFI_MAC_BITMASK_084);
    REG_WRITE(WIFI_MAC_BITMASK_084, value & 0x7fffffff);
    return;
}

// Method to enable rx (source: ghidra)
void wifi_enable_rx(){
    uint32_t value = REG_READ(WIFI_MAC_BITMASK_084);
    REG_WRITE(WIFI_MAC_BITMASK_084, value | 0x80000000);
    return;
}

// Setting base address of the receive DMA buffer
static inline void set_base_rx_addr(dma_list_item_t * addr){
    REG_WRITE(WIFI_BASE_RX_DSCR, (uint32_t)addr & 0xfffff);
}

void update_rx_chain() {
    // Update rx chain. But, this does not really load the "next" address
    // properly. Maybe it has a different function
    uint32_t value = REG_READ(WIFI_MAC_BITMASK_084);
	REG_WRITE(WIFI_MAC_BITMASK_084, value |= 0x1);
	// Wait for confirmation from hardware
	while (REG_READ(WIFI_MAC_BITMASK_084) & 0x1);
}

// Currently setting up RX is only pointing to teh rx DMA struct.
// Further work needed if we want to avoid esp_wifi_init and esp_wifi_start
void wifi_setup_rx() {

    // Get rid of previous buffers. L33t H4xx0r method. 
    // Probably using esp-open-mac's init method is better
    dma_list_item_t* last_dscr = REG_READ(WIFI_LAST_RX_DSCR) | 0x3fc00000;
    last_dscr->next = NULL;
    
    dma_list_item_t* next_dscr = REG_READ(WIFI_NEXT_RX_DSCR) | 0x3fc00000;
    next_dscr->next = NULL;

	// This function sets up the linked list needed for the Wi-Fi MAC RX functionality
	dma_list_item_t* prev = NULL;
	for (int i = 0; i < RX_BUFFER_AMOUNT; i++) {
		dma_list_item_t* item = &rx_item_array[RX_BUFFER_AMOUNT - i - 1];
        // ESP_LOGI(TAG, "rx dscr set to %p", item);
		item->has_data = 0;
		item->owner = 1;
		item->size = 1600;
		item->length = item->size;

		uint8_t* packet = (uint8_t *)&rx_pkt_buffers[i]; // TODO verify that this does not need to be bigger
		item->packet = packet;
		item->next = prev;
		prev = item;
		if (!rx_chain_last) {
			rx_chain_last = item;
		}
	}
	set_base_rx_addr(prev);
	rx_chain_begin = prev;
    update_rx_chain();
}

// Clear the slot for the next transmission when TX is complete
// Call when 0x80 interrupt is raised by the wifi peripheral
static void processTxComplete() {
	uint32_t txq_state_complete = REG_READ(WIFI_TX_STATUS);
	if (txq_state_complete == 0) {
		return;
	}
    // Clear the slot using a mask
    // Set the corresponding bit of WIFI_TX_CLR
    // and clear the bit of WIFI_TX_STATUS
	uint32_t slot = 31 - __builtin_clz(txq_state_complete);
	uint32_t clear_mask = 1 << slot;
    uint32_t wifi_tx_clr = REG_READ(WIFI_TX_CLR);
	REG_WRITE(WIFI_TX_CLR,  wifi_tx_clr |= clear_mask);
    REG_WRITE(WIFI_TX_STATUS, txq_state_complete &= ~clear_mask);
    s_frame_buffer_used = false;
}


// esp32-open-mac interrupt handler
void IRAM_ATTR wifi_interrupt_handler(void){
    uint32_t mac_cause = REG_READ(WIFI_INT_STATUS_GET);

    // This probably means the power management peripheral triggered
    // the interrupt as this interrupt is shared between the MAC and PWR peripherals
    if(mac_cause == 0){
        return;
    }

    REG_WRITE(WIFI_INT_STATUS_CLR, mac_cause);

    hardware_queue_entry_t intr_queue_entry;    
    intr_queue_entry.type = INTR_ENTRY;
    intr_queue_entry.content.intr.interrupt_received = mac_cause;
    xQueueSendFromISR(driver_event_q_hdl, &intr_queue_entry, pdFALSE);
    
    return;
}

void wifi_setup_interrupt(){
    // ic_set_interrupt_handler() in ghidra
    // Mask out power interrupt and the MAC interrupt sources (temporarily)
    // From decompilation of intr_matrix_set. Both are mapped to CPU interrupt number 1
    // Disable the wifi CPU interrupt and renable after setting the handler
    REG_WRITE(INTR_SRC_MAC, 0);
    REG_WRITE(INTR_SRC_PWR, 0);

    // Disable all interrupt sources. A bit hacky but works
    uint32_t value = REG_READ(INTR_ENABLE_REG);
    REG_WRITE(INTR_ENABLE_REG, 0);
    intr_handler_set(WIFI_INTR_NUMBER, (intr_handler_t)wifi_interrupt_handler, 0);
    REG_WRITE(INTR_ENABLE_REG, value);

    // Unmask the interrupt source again. So called "routing"
    REG_WRITE(INTR_SRC_MAC, WIFI_INTR_NUMBER);
    // Mask this interrupt for now. Results in a constant interval interrupt at what seems to be the TBTT
    // REG_WRITE(INTR_SRC_PWR, WIFI_INTR_NUMBER);

}

// Enabling TX ORs the MS Byte of PLCP0 register to 0xc
static inline void enable_tx(){
    // Finally enable tx
    uint32_t cfg_val = REG_READ(WIFI_TX_PLCP0);
    REG_WRITE(WIFI_TX_PLCP0, cfg_val | 0xc0000000);
}

// Raw TX/RX only works when proprietary task (ppTask) is running
// Killing it most likely deletes data containing
// some settings that the peripheral needs
// The proprietary posts to the queue monitored by ppTask triggering various actions
// ppTask is given nothing to do as we override the interrupt 
// So, hopefully it never executes anything meaningful to interfere with out HW process
void wifi_raw_tx(dma_list_item_t* tx_item){

    uint32_t cfg_val = REG_READ(WIFI_TX_CONFIG);
    REG_WRITE(WIFI_TX_CONFIG, cfg_val | 0xa);

    // Write the address of DMA struct to PLCP0 and set the 6th byte to 6 (why?) and 7th byte to 1 (why?)
    REG_WRITE(WIFI_TX_PLCP0, (((uint32_t)tx_item & 0xfffff) | 0x600000) | 0x01000000);

    // Using esp32-open-mac's implementation for reference
    uint32_t rate = WIFI_PHY_RATE_54M;
    uint32_t is_n_enabled = (rate >= 16);

    REG_WRITE(WIFI_TX_PLCP1, 0x10000000 | (tx_item->length & 0xfff) | ((rate & 0x1f) << 12) | ((is_n_enabled & 0b1) << 25));
    REG_WRITE(WIFI_TX_PLCP1_2, 0x00100000);

    // Write 0x00000020 (why?)
    cfg_val = REG_READ(WIFI_TX_PLCP2);
    // REG_WRITE(WIFI_TX_PLCP2, (cfg_val & (0xffffc03f | 0x00000020))); // Using the proprietary code
    REG_WRITE(WIFI_TX_PLCP2, cfg_val | 0x10); // Using the proprietary code
    
    // Duration to 0 (Why?)
    REG_WRITE(WIFI_TX_DURATION, 0x00);

    // // Configure EDCA
    cfg_val = REG_READ(WIFI_TX_CONFIG);
    cfg_val = cfg_val | 0x02000000;
    // cfg_val = cfg_val | (0xffffefff & 0x00003000);
    REG_WRITE(WIFI_TX_CONFIG, cfg_val);

    enable_tx();
}

// only needed to handle the power interrupt
static void wifi_process_timeout(void){
    uint32_t value = (REG_READ(WIFI_TX_GET_ERR) >> 16) && 0xff;
    uint32_t slot = 31 - __builtin_clz(value);
    uint32_t tx_value = REG_READ(WIFI_TX_PLCP0);
    REG_WRITE(WIFI_TX_PLCP0, tx_value & 0xbfffffff);
    uint32_t reg_value = REG_READ(WIFI_TX_CLR_ERR); 
    REG_WRITE(WIFI_TX_CLR_ERR, ((slot && 0xff) << 16) | (reg_value & 0xff00ffff));
}

// Create a DMA struct and pass it raw_tx function
void wifi_transmit_packet(uint8_t *pkt, uint32_t len){
    // Probably need to handle access with a mutex
    tx_item.packet = pkt;
    tx_item.length = len;
    tx_item.size = len + 32; // 32 bytes more??
    wifi_raw_tx(&tx_item);
}

void wifi_handle_rx() {
	// print_rx_chain(rx_chain_begin);
	dma_list_item_t* current = rx_chain_begin;
	// This is a workaround for when we receive a lot of packets; otherwise we get stuck in this function,
	// handling packets for all eternity
	// This is much less of a problem now that we implement hardware filtering
	int received = 0;

	while (current && current->has_data) {
		dma_list_item_t* next = current->next;
		//TODO enable interrupt?

		received++;
        ESP_LOGW(TAG, "There are %d packets received so far", received);
        if(received >= RX_BUFFER_AMOUNT){
            ESP_LOGE(TAG, "RX buffers full. Recycle them to continue RX");
        }

		// update rx chain
		rx_chain_begin = next;
		current->next = NULL;

        hardware_queue_entry_t rx_queue_entry;
        rx_queue_entry.type = RX_ENTRY;
        rx_queue_entry.content.rx.packet = current->packet;
        rx_queue_entry.content.rx.len = current->length;
        if(mac_event_q_hdl == NULL){
            ESP_LOGE(TAG, "MAC queue not created");
        }else{
            ESP_LOGI(TAG, "Queueing RX entry");
            xQueueSendToBack(mac_event_q_hdl, &rx_queue_entry, 0);
            //TODO disable interrupt?
            current = next;
            if (received > RX_BUFFER_AMOUNT) {
                goto out;
		    }
        }
	
	}
	out:
	// TODO enable interrupt
}

// Free up the DMA item for reuse
void wifi_recycle_dma_item(dma_list_item_t* item){
    item->length = item->size;
	item->has_data = 0;
	if (rx_chain_begin) {
		rx_chain_last->next = item;
		update_rx_chain();
		if (REG_READ(WIFI_NEXT_RX_DSCR) == 0x3ff00000) {
			dma_list_item_t* last_dscr = (dma_list_item_t*)(REG_READ(WIFI_LAST_RX_DSCR));
			if (item == last_dscr) {
				rx_chain_last = item;
			} else {
				assert(last_dscr->next != 0);
				set_base_rx_addr(last_dscr->next);
				rx_chain_last = item;
			}
		} else {
			rx_chain_last = item;
		}
	} else {
		rx_chain_begin = item;
		set_base_rx_addr(item);
		rx_chain_last = item;
	}

}

// Read the bssid configured during association
void wifi_get_bssid(uint8_t* mac_buffer){
    uint32_t mac_p1 = REG_READ(WIFI_MAC_BASE);
    uint32_t mac_p2 = REG_READ(WIFI_MAC_BASE + 4);

    memcpy(mac_buffer, (uint8_t *)&mac_p1, 4);
    memcpy(mac_buffer + 4, (uint8_t *)&mac_p2, 2);
}

// Free the DMA item for new packet reception
void wifi_free_eb(uint8_t *eb){
    for(int i = 0; i < RX_BUFFER_AMOUNT; i++){
        if(eb == (uint8_t *)&rx_pkt_buffers[i][0]){
            wifi_recycle_dma_item(&rx_item_array[i]);
            return;
        }
    }
}

// Run this task. The interrupt pushes events to a queue
// that this task monitors. All queues are statically allocated
// to ensure analyzability.
// For compatibility, make sure the packet queueing task
// already has the WiFi MAC layer headers added
void wifi_hardware_task(){

    // create event queue for sharing of both TX and RX tasks together
    driver_event_q_hdl = xQueueCreateStatic(BUFFER_SIZE, sizeof(hardware_queue_entry_t), driver_q_buffer, &driver_event_q);
    mac_event_q_hdl = xQueueCreateStatic(BUFFER_SIZE, sizeof(hardware_queue_entry_t), mac_q_buffer, &mac_event_q);

    // Setup our own interrupts
    wifi_hw_deinit();
    wifi_setup_interrupt();
    wifi_setup_rx();
    wifi_hw_init();

    hardware_queue_entry_t hw_queue_entry;
    uint32_t cause;

    while(1){
        // Polls the queue for entries sent by the interrupt handler
        if(xQueueReceive(driver_event_q_hdl, &hw_queue_entry, 0)){
            if(hw_queue_entry.type == TX_ENTRY){
                ESP_LOGI(TAG, "Sending queued packet");
                wifi_transmit_packet(hw_queue_entry.content.tx.packet, hw_queue_entry.content.tx.len);

            }else if(hw_queue_entry.type == INTR_ENTRY){
                cause = hw_queue_entry.content.intr.interrupt_received;

                if(cause & 0x80){
                    // Packet received
                    ESP_LOGI(TAG, "TX complete");
                    processTxComplete();
                }
                
                if(cause & 0x800){
                    ESP_LOGE(TAG, "Watchdog panic");
                }
                
                if (cause & 0x600000) {
					// TODO this is bad, we should reboot
					ESP_LOGE(TAG, "something bad, we should reboot");
				}
				
                if (cause & 0x1004020) {
                    // Moved to level debug. Otherwise, too many messages
					ESP_LOGD(TAG, "received message");		
                    wifi_handle_rx();
				}
                
                if (cause & 0x80000) {
					ESP_LOGE(TAG, "lmacProcessAllTxTimeout");
                    s_frame_buffer_used = false;
                    wifi_process_timeout();
				}

				if (cause & 0x100) {
					ESP_LOGE(TAG, "lmacProcessCollisions");
				}
            }else{
                ESP_LOGE(TAG, "RX entry in the wrong queue");
            }
        }
        // suspend task briefly for IDLE task
        vTaskDelay(50 / portTICK_PERIOD_MS); 
    }
}