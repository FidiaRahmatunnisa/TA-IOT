#include <string.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "ESP1_MASTER";

// MAC ESP2 (Slave)
uint8_t slave_mac[6] = {0x6c, 0xc8, 0x40, 0x33, 0xf5, 0xa0};

// Struktur data mic3 (harus sama dengan di ESP2)
typedef struct __attribute__((packed)) {
    uint64_t timestamp;
    float ste;
    float zcr;
    int peak;
    int mode;
} mic3_data_t;

static uint64_t start_time_us;

void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi diinisialisasi dan mulai dalam mode STA");
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    uint64_t now_since_start = esp_timer_get_time() - start_time_us;

    if (len == sizeof(uint64_t)) {
        // Paket sinkronisasi waktu
        uint64_t ts_received;
        memcpy(&ts_received, data, sizeof(ts_received));

        // Tampilkan log hanya di 20 detik pertama
        if (now_since_start < 20000000ULL) {
            ESP_LOGI(TAG, "[SYNC] Terima timestamp dari %s: %llu", macStr, ts_received);
        }
    } 
    else if (len == sizeof(mic3_data_t)) {
        // Paket data mic3
        mic3_data_t mic_data;
        memcpy(&mic_data, data, sizeof(mic_data));

        ESP_LOGI(TAG, "[MIC3] Dari %s: ts=%llu STE=%.2f ZCR=%.4f Peak=%d Mode=%d",
                 macStr, mic_data.timestamp, mic_data.ste, mic_data.zcr, mic_data.peak, mic_data.mode);
    }
    else {
        ESP_LOGW(TAG, "Panjang data tidak dikenali dari %s: %d", macStr, len);
    }
}

void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // Daftarkan peer ESP2
    esp_now_peer_info_t peerInfo = {0};
    memcpy(peerInfo.peer_addr, slave_mac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    ESP_LOGI(TAG, "Peer ESP2 didaftarkan");
}

void app_main1(void)
{
    ESP_LOGI(TAG, "Mulai ESP1 MASTER");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();

    // Set WiFi channel ke 1 supaya sama dengan ESP2
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    espnow_init();

    // Catat waktu mulai untuk kontrol 20 detik logging sync
    start_time_us = esp_timer_get_time();

    // Kirim pesan sync request ke ESP2 supaya mulai kirim data
    const char *sync_req = "SYNC_START";
    esp_err_t send_result = esp_now_send(slave_mac, (uint8_t *)sync_req, strlen(sync_req));
    if (send_result == ESP_OK) {
        ESP_LOGI(TAG, "SYNC request dikirim ke ESP2");
    } else {
        ESP_LOGW(TAG, "Gagal kirim SYNC request (%d)", send_result);
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
