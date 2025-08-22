// mic3_raw_logger.c
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_now.h"


// ---------------- CONFIG ----------------
#define SAMPLE_RATE     96000
#define I2S_READ_LEN    1024
#define I2S_PORT        I2S_NUM_0
#define MIC3_SCK        26
#define MIC3_WS         25
#define MIC3_SD         22

//-----------------------------------------
static const char *TAG = "ESP2_SLAVE";
static int32_t i2s_buf[I2S_READ_LEN];

// MAC ESP1 (Master) tujuan kirim data
uint8_t master_mac[6] = {0xa8, 0x42, 0xe3, 0x5b, 0xca, 0xc8};

typedef struct {
    uint64_t timestamp;
} espnow_data_t;

typedef struct {
    uint64_t timestamp; // hasil sinkronisasi
    float ste;
    float zcr;
    int32_t peak;
} mic3_packet_t;

// Global offset untuk sinkronisasi waktu
static int64_t offset = 0;
static int offset_calculated = 0;

//-----------------------------------------
static mic3_packet_t mic3_data_global;
static SemaphoreHandle_t mic_data_mutex;
static int sync_phase_done = 0; // 0 = sync only, 1 = send mic data

// ---- I2S init ----
void init_i2s_mic3() {
    i2s_config_t cfg = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    i2s_driver_install(I2S_PORT, &cfg, 0, NULL);

    i2s_pin_config_t pins = {
        .bck_io_num = MIC3_SCK,
        .ws_io_num = MIC3_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = MIC3_SD,
        .mck_io_num = I2S_PIN_NO_CHANGE
    };
    i2s_set_pin(I2S_PORT, &pins);
    i2s_zero_dma_buffer(I2S_PORT);

    ESP_LOGI(TAG, "I2S initialized (SR=%d, frame=%d)", SAMPLE_RATE, I2S_READ_LEN);
}

// ---- compute STE, ZCR, PEAK ----
void compute_ste_zcr_peak(int32_t *buf, int len, float *ste_out, float *zcr_out, int *peak_out) {
    double ste = 0.0;
    int zcr = 0, last_sign = 0, peak = 0;

    for (int i = 0; i < len; ++i) {
        int16_t sample = buf[i] >> 14;  // downscale ke 16-bit
        ste += (double)sample * (double)sample;
        int sign = (sample >= 0) ? 1 : -1;
        if (i > 0 && sign != last_sign) zcr++;
        last_sign = sign;
        int amp = abs(sample);
        if (amp > peak) peak = amp;
    }

    *ste_out = (float)(ste / len);
    *zcr_out = (float)zcr / len;
    *peak_out = peak;
}

// ---- Main Task ----
void mic3_task(void *arg) {
    while (1) {
        size_t bytes;
        float ste, zcr; 
        int peak;

        // Baca data dari mic
        i2s_read(I2S_PORT, i2s_buf, sizeof(i2s_buf), &bytes, portMAX_DELAY);

        // Hitung nilai STE, ZCR, PEAK
        compute_ste_zcr_peak(i2s_buf, I2S_READ_LEN, &ste, &zcr, &peak);

        // Ambil timestamp (mikrodetik sejak boot)
        // int64_t now_us = esp_timer_get_time(); //---> ini esp2 ini, bukan sinkronisasi
        uint64_t ts_sync = offset_calculated ? (esp_timer_get_time() + offset) : esp_timer_get_time(); //----> ini sinkronisasi 

        // Tampilkan log raw --- esp2 ini, bukan sinkronisasi
        // ESP_LOGI(TAG, "TIME=%lld us | STE=%.2f | PEAK=%d | ZCR=%.4f", now_us, ste, peak, zcr);

        xSemaphoreTake(mic_data_mutex, portMAX_DELAY);
        mic3_data_global.timestamp = ts_sync;
        mic3_data_global.ste = ste;
        mic3_data_global.peak = peak;
        mic3_data_global.zcr = zcr;
        xSemaphoreGive(mic_data_mutex);

        // ini sudah sinkronisasi ini
        // ESP_LOGI(TAG, "TIME=%llu us | STE=%.2f | PEAK=%d | ZCR=%.4f", ts_sync, ste, peak, zcr);
    }
}

// ------ ESP SEND ------
// Callback resmi ESPNOW (dipanggil setelah kirim data)
static void espnow_send_cb_real(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Data terkirim ke %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        ESP_LOGW(TAG, "Gagal kirim data");
    }
}

// Task untuk kirim data terus-menerus
static void espnow_send_task(void *arg) {
    uint64_t start_time = esp_timer_get_time();

    while (1) {
        uint64_t now_us = esp_timer_get_time();

        if (!sync_phase_done && (now_us - start_time) > 20 * 1000000ULL) {
            sync_phase_done = 1;
            ESP_LOGI(TAG, "Sinkronisasi selesai, mulai kirim data mic");
        }

        if (sync_phase_done) {
            mic3_packet_t packet;
            xSemaphoreTake(mic_data_mutex, portMAX_DELAY);
            packet = mic3_data_global;
            xSemaphoreGive(mic_data_mutex);

            // esp_now_send(master_mac, (uint8_t *)&packet, sizeof(packet));
            esp_now_send(master_mac, (uint8_t*)&mic3_data_global, sizeof(mic3_data_global));
        } else {
            uint64_t ts_sync = offset_calculated ? (esp_timer_get_time() + offset) : esp_timer_get_time();
            esp_now_send(master_mac, (uint8_t *)&ts_sync, sizeof(ts_sync));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --------- RECEV ESP ---------
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

//------- esp-now INIT ---------
void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());

    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb_real));
  

    esp_now_peer_info_t peerInfo = {0};
    memcpy(peerInfo.peer_addr, master_mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Gagal menambahkan peer");
    } else {
        ESP_LOGI(TAG, "Peer ESP1 berhasil ditambahkan");
    }
}

// ---- app_main ----
void app_main2(void)
{
    nvs_flash_init();
    espnow_init();
    init_i2s_mic3();

    mic_data_mutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(mic3_task, "mic3_task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(espnow_send_task, "espnow_send_task", 4096, NULL, 5, NULL, 0);
}

