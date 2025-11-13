#include "wifi_mac.h"
#include "esp_err.h"
#include "esp_log.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define BUFFER_SIZE 10
#define MTU 1500

static const char* TAG = "wifi_mac";

mac_addr_t bssid_addr;

uint8_t mac_frame_rx_buffer[MTU];

wifi_mac_rx_cb_t rx = NULL;

// Open 802.11 send function. Sends normal data frames (no QoS)
esp_err_t wifi_mac_send_frame(uint32_t wifi_if, uint8_t *pkt, uint32_t len){
    // Encapsulating with WiFi header
    if(s_frame_buffer_used){
        return ESP_ERR_NO_MEM;
    }
    s_frame_buffer_used = true;

    // Add buffers
    wifi_mac_frame_hdr_t * wifi_hdr_ptr = (wifi_mac_frame_hdr_t *)wifi_frame_buffer;

    // Frame control
    wifi_hdr_ptr->frame_control.protocol_version = 0;
    wifi_hdr_ptr->frame_control.type = 2; // Data frame
    wifi_hdr_ptr->frame_control.from_ds = 0;
    wifi_hdr_ptr->frame_control.to_ds = 1;
    wifi_hdr_ptr->frame_control.sub_type = 0;

    // Rest of the header
    wifi_hdr_ptr->duration_id = 0x0000; // Reversed due to endianness

    // Since packet is going to the DS, address1 is the RA = BSSID
    // address2 is the SA = transmitter address, address3 is the destination address
    memcpy(&wifi_hdr_ptr->address2, pkt + MAC_SIZE, MAC_SIZE);
    memcpy(&wifi_hdr_ptr->address3, pkt, MAC_SIZE);
    memcpy(&wifi_hdr_ptr->address1, (uint8_t *)&bssid_addr, MAC_SIZE);

    wifi_hdr_ptr->sequence_control.fragment_number = 0;
    wifi_hdr_ptr->sequence_control.sequence_number = 0;

    // LLC and other headers
    wifi_frame_buffer[sizeof(wifi_mac_frame_hdr_t)] = 0xaa;
    wifi_frame_buffer[sizeof(wifi_mac_frame_hdr_t) + 1] = 0xaa;
    wifi_frame_buffer[sizeof(wifi_mac_frame_hdr_t) + 2] = 0x03;
    wifi_frame_buffer[sizeof(wifi_mac_frame_hdr_t) + 3] = 0x00;
    wifi_frame_buffer[sizeof(wifi_mac_frame_hdr_t) + 4] = 0x00;
    wifi_frame_buffer[sizeof(wifi_mac_frame_hdr_t) + 5] = 0x00;

    memcpy(&wifi_frame_buffer[0] + sizeof(wifi_mac_frame_hdr_t) + 6, pkt + 12, len - 12);

    // Set the FCS to 0
    memset(&wifi_frame_buffer[0] + sizeof(wifi_frame_buffer) + 6 + len - 12, 0, 4);
    
    // Queue it
    hardware_queue_entry_t tx_queue_entry;
    tx_queue_entry.type = TX_ENTRY;
    tx_queue_entry.content.tx.packet = &wifi_frame_buffer[0];
    tx_queue_entry.content.tx.len = len + sizeof(wifi_mac_frame_hdr_t) + 4 + 6 - 12;
    xQueueSendToBack(driver_event_q_hdl, &tx_queue_entry, 0);
    return ESP_OK;

}

void wifi_mac_handle_rx(uint8_t* pkt, uint32_t len){

    // Copy the packet in 802.11 format. First 48 bytes are from the 
    // ESP's internal radiotap header format. Hence discarded for now
    // @TODO: Need fix for differentiating QoS from regular data pkts
    wifi_mac_frame_hdr_t* mac_hdr = (wifi_mac_frame_hdr_t *)(pkt + 48);

    // Packet is coming from DS. address1 is the destination address (RA = DA)
    // address 3 is the source address (SA)
    memcpy(&mac_frame_rx_buffer[0], &mac_hdr->address1.mac_bytes[0], 6);
    memcpy(&mac_frame_rx_buffer[6], &mac_hdr->address3.mac_bytes[0], 6);

    // Copy rest of the packet. Note +2 is due to receiving QoS data frames.
    // This may have to be made more flexible in future
    memcpy(&mac_frame_rx_buffer[12], pkt + sizeof(wifi_mac_frame_hdr_t) + 54 + 2, len - sizeof(wifi_mac_frame_hdr_t) - 6);

    // Compensating for QOS frames with +2. Need a more robust method
    uint32_t eth_pkt_size = len - sizeof(wifi_mac_frame_hdr_t) - 54 + 2 * MAC_SIZE - 6;

    // Callback from the IP stack
    if(rx == NULL){
        ESP_LOGE(TAG, "Wifi RX handler not set...");
        wifi_free_eb(pkt);
    }else{
        // eb is "freed" by the callback
        if(mac_frame_rx_buffer[12] == 0x08){
            ESP_LOGD(TAG, "IPv4 packet");
            rx(mac_frame_rx_buffer,  eth_pkt_size, pkt);
        }else{
            // For now simply print out the packet and free eb
            ESP_LOGD(TAG, "Non IPv4 packet");
            wifi_free_eb(pkt);
        }
    }
}

void wifi_mac_set_rxcb(wifi_mac_rx_cb_t wifi_cb){
    rx = wifi_cb;
}

void wifi_mac_task(){

    hardware_queue_entry_t rx_entry;

    wifi_get_bssid(&bssid_addr.mac_bytes[0]);

    // Poll the queue for received packets
    while(1){
        if(xQueueReceive(mac_event_q_hdl, &rx_entry, 0)){
            wifi_mac_handle_rx(rx_entry.content.rx.packet, rx_entry.content.rx.len);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}