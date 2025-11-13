#include "net_setup.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "esp_mac.h"

#include "wifi_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char* TAG = "net_setup";

wifi_config_t wifi_config = {
    .sta = {
            .ssid = "custom",
            .password = "",
        },
};

uint8_t s_retry_num = 0;
bool ns_wifi_connected = false;
SemaphoreHandle_t ns_wifi_conn_sem = NULL;

esp_err_t custom_wifi_init(){
    esp_err_t result;

    // Disabling nvs in configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = false;
    result = esp_wifi_init(&cfg);
    ns_wifi_conn_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(ns_wifi_conn_sem);
    return result;

}

esp_err_t custom_wifi_deinit(){
    // Disable wifi
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_stop());
    return esp_wifi_deinit();
}

esp_err_t custom_start_wifi_sta(wifi_config_t* cfg){
    esp_err_t result;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, cfg));
    result = esp_wifi_start();
    return result;
}

void wifi_event_handlers(void *arg, esp_event_base_t event_base,
                    int32_t event_id, void* event_data)
{
    // Check the event base for a WiFi event. If device disconnects, retry until MAX_RETRY is exceeded
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
        ESP_ERROR_CHECK(esp_wifi_connect());
        ESP_LOGI(TAG, "successfully initiated connection to wifi");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
        ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");
        if (s_retry_num < 5) {
            ESP_LOGI(TAG, "retrying connect");
            ESP_ERROR_CHECK(esp_wifi_connect());
            s_retry_num++;
            ESP_LOGI(TAG, "retrying connect finished");
        }
    } else if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_CONNECTED) {
            ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED");
            if(ns_wifi_conn_sem == NULL){
                ESP_LOGE(TAG, "init_wifi before setting event handlers!");
            }
            else{
                if(xSemaphoreTake(ns_wifi_conn_sem, (TickType_t)10) == pdTRUE){
                    ESP_LOGI(TAG, "Wifi connected!");
                    ns_wifi_connected = true;
                    xSemaphoreGive(ns_wifi_conn_sem);
                }else{
                    ESP_LOGE(TAG, "Something went wrong");
                }

            }
            // ns_wifi_connected = true;
            /* TODO: why is this not defined? should be 41 */
            /* } else if (event_id == WIFI_EVENT_HOME_CHANNEL_CHANGE) { */
            /*     ESP_LOGI(TAG, "WIFI_EVENT_HOME_CHANNEL_CHANGE"); */
        } else {
            ESP_LOGW(TAG, "unexpected WIFI_EVENT id %d", (int) event_id);
        }
    } else {
        ESP_LOGW(TAG, "unexpected event base %p id %d", (void *) event_base, (int) event_id);
    }
}

// Create callback function for WiFi reception
// buffer is Local packet buffer to read from the L2 buffer and free it
// Open wifi stack's free buffer works differently but kept here to keep it similar
esp_err_t open_wifi_rx_cb(void *buffer, uint16_t len, void *eb){

    // Free the rx buffer (open mac)
    wifi_free_eb(eb);
        for(int i = 0; i < len; i++){
        printf("%02x ", *((uint8_t *)buffer+i));
    }
    printf("\n");

    return ESP_OK;
}

esp_err_t read_sta_mac(mac_addr_t* mac){

    // Read mac address and store in the 
    // mac_addr_t struct. Originally stored in efuse read by the 
    // wifi firmware at the beginning
    esp_err_t ret;    
    ret = esp_read_mac((uint8_t *)mac, ESP_MAC_WIFI_STA);
    return ret;
}

void net_setup() {
    // Init and connect to wifi
    ESP_ERROR_CHECK(custom_wifi_init());

    // No power save mode
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_event_handler_instance_t instance_any_id;
    
    // register the event hanndler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handlers,
                                                        NULL,
                                                        &instance_any_id));
    
    ESP_ERROR_CHECK(custom_start_wifi_sta(&wifi_config));
    ESP_ERROR_CHECK(esp_wifi_internal_reg_rxcb(WIFI_IF_STA, (wifi_rxcb_t)open_wifi_rx_cb));

    // Wait until connected
    while(1){
        if(ns_wifi_conn_sem == NULL){
            ESP_LOGE(TAG, "init_wifi before setting event handlers!");
        }

        // Wait until WiFi is connected. custom_strart_wifi_sta() only starts the handshake with AP
        if(xSemaphoreTake(ns_wifi_conn_sem, (TickType_t)10) == pdTRUE){
            if(ns_wifi_connected){
                break;
            }else{
                ESP_LOGI(TAG, "Waiting for connection");
            }
            xSemaphoreGive(ns_wifi_conn_sem);
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    xSemaphoreGive(ns_wifi_conn_sem);

    // Kill proprietary task and take over tx/rx
    ESP_LOGW(TAG, "Wifi connected. Killing proprietary task");
    pp_post(0xf, 0);
    
    // Remove this line if proprietary drivers are used
    wifi_mac_set_rxcb((wifi_mac_rx_cb_t)open_wifi_rx_cb);
}
