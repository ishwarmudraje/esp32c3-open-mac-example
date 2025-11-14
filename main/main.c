/*custom - Transactional networking stack development.
ESP32 - Firmware for transmitting a UDP packet bypassing the esp_netif()
library. Depends on only FreeRTOS and the standard ESP32 libraries 
*/

// Standard libraries
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"

#include "net_setup.h"

// WiFi driver library
#include "esp_private/wifi.h"
#include "esp_wifi.h"

// Event library
#include "esp_event.h"

#include "esp_log.h"

// ESP error definitions
#include "esp_err.h"

#include "portmacro.h"
#include "wifi_mac.h"
#include "wifi_driver.h"

// Defining connection parameters
#define SCAN_AUTH_MODE_THRESHOLD ESP_WIFI_AUTH_WPA2_PSK // Not used since it is default
#define MAX_RETRY 5

#define STACK_SIZE 4096

StackType_t hwStack[STACK_SIZE], macStack[STACK_SIZE], udpStack[STACK_SIZE];

StaticTask_t hwTaskBuffer, macTaskBuffer, udpTaskBuffer;
TaskHandle_t hwTaskHandle = NULL, macTaskHandle = NULL, udpTaskHandle = NULL;


esp_err_t ret;

const uint8_t packet[] = {
    // MAC layer
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // Destination MAC address
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // Source MAC address
    
    0x08, 0x00, // protocol type - IPV4
    
    // IPv4 header
    0x45, //version :4 (obv) with IHL = 5
    0x00, // DSCP and ECN
    0x00, 0x22, // Total length (33 bytes), IPv4 Header + UDP Header + "Hello"
    0x00, 0x00, // Identification and fragmentation data
    0x00, 0x00, // Flags and fragment offset
    0x05, // TTL
    0x11, // Protocol type UDP 
    0x33, 0x49, // Header checksum (update after calculating length of total packet)
    0xc0, 0xa8, 0x00, 0x7A, // Source IP address - Arbitrary IP address for the ESP-32
    0xc0, 0xa8, 0x00, 0xb8, // Destination IP address - Laptop's IP address (assigned through DHCP by the router)

    // UDP header - 8 bytes
    0x1f, 0x45, // Source port - Both source and destination ports are random since it is UDP
    0xf0, 0xf0, // Destination port
    0x00, 0x0e, // Length
    0x00, 0x00,  // Checksum - calculate using the pseudo ipv4 header
    'h', 'e', 'l', 'l', 'o', '\n' // The message :)

};


static void task(void *pvParameters) {

    while (true) {
        vTaskDelay(1000/portTICK_PERIOD_MS);

        wifi_mac_send_frame(1, (uint8_t *)&packet[0], sizeof(packet));

    }
}



void app_main(){

    net_setup();

    // 200ms for the task to be successfully terminated (not sure if necessary)
    vTaskDelay(200/portTICK_PERIOD_MS);

    // Slightly higher prio for mac_task otherwise keeps getting preempted in favor of HW task
    // and packet backlog is not cleared in time
    ESP_LOGI("app_main", "Starting hw_task");
    hwTaskHandle = xTaskCreateStaticPinnedToCore(&wifi_hardware_task, "hw_task", STACK_SIZE, NULL, 21, hwStack, &hwTaskBuffer, 0);   
    ESP_LOGI("app_main", "Starting mac_task");
    macTaskHandle = xTaskCreateStaticPinnedToCore(&wifi_mac_task, "mac_task", STACK_SIZE, NULL, 23, macStack, &macTaskBuffer, 0);   

    ESP_LOGI("app_main", "Starting ping task");
    xTaskCreateStaticPinnedToCore(&task, "udp_task", STACK_SIZE, NULL, 6, udpStack, &udpTaskBuffer, 0);

}