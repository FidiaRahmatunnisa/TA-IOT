#include <string.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ESP2_SLAVE";

// MAC ESP1 (Master) tujuan kirim data
uint8_t master_mac[6] = {0xa8, 0x42, 0xe3, 0x5b, 0xca, 0xc8};

typedef struct {
    uint64_t timestamp;
} espnow_data_t;

// Global offset untuk sinkronisasi waktu
static int64_t offset = 0;
static int offset_calculated = 0;

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi diinisialisasi dan mulai dalam mode STA");
}

static void espnow_send_timestamp_task(void *arg)
{
    espnow_data_t data_to_send;

    while (1) {
        if (offset_calculated) {
            data_to_send.timestamp = esp_timer_get_time() + offset;
        } else {
            data_to_send.timestamp = esp_timer_get_time();
        }

        esp_err_t result = esp_now_send(master_mac, (uint8_t *)&data_to_send, sizeof(data_to_send));
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Kirim timestamp ke ESP1: %llu", data_to_send.timestamp);
        } else {
            ESP_LOGW(TAG, "Gagal kirim timestamp, err: %d", result);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(uint64_t)) {
        uint64_t ts_master_received;
        memcpy(&ts_master_received, data, sizeof(ts_master_received));

        uint64_t ts_slave_now = esp_timer_get_time();

        offset = (int64_t)ts_master_received - (int64_t)ts_slave_now;
        offset_calculated = 1;

        ESP_LOGI(TAG, "Offset waktu disetel: %lld us", offset);
    }
}

void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
}

void app_main(void)
{
    ESP_LOGI(TAG, "Mulai ESP2 SLAVE");
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();

    espnow_init();  // pastikan sudah register recv callback

    esp_now_peer_info_t peerInfo = {0};
    memcpy(peerInfo.peer_addr, master_mac, 6);
    peerInfo.channel = 1;       // pastikan channel sama dengan ESP1
    peerInfo.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    ESP_LOGI(TAG, "Peer ESP1 didaftarkan");

    xTaskCreate(espnow_send_timestamp_task, "espnow_send_timestamp_task", 2048, NULL, 5, NULL);
}

