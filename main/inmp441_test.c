#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"
#include "hal/gpio_hal.h"

//==== WiFi dan HTTP ====
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#define WIFI_SSID "HoYo"
#define WIFI_PASS "JaYONaEcelyo"
#define SERVER_URL "http://98.81.56.30/receive.php"

// === Konfigurasi Umum ===
#define SAMPLE_RATE     96000
#define I2S_READ_LEN    1024
#define SILENT_LIMIT    3
#define STE_THRESHOLD   200000.0f
#define ZCR_THRESHOLD   0.2f

// === Threshold Per Mikrofon ===
#define THRESHOLD1      100
#define THRESHOLD2      100
#define THRESHOLD3      100

// === I2S Pin Mic1 / Mic3 (I2S_NUM_0) ===
#define MIC13_WS        25
#define MIC13_SCK       26
#define MIC1_SD         22
#define MIC3_SD         32

// === I2S Pin Mic2 (I2S_NUM_1) ===
#define MIC2_WS         27
#define MIC2_SCK        14
#define MIC2_SD         33

int32_t mic1_buf[I2S_READ_LEN], mic2_buf[I2S_READ_LEN], mic3_buf[I2S_READ_LEN];
volatile int peak1 = 0, peak2 = 0, peak3 = 0;
volatile uint64_t ts1 = 0, ts2 = 0, ts3 = 0;
volatile uint64_t base_time = 0;
volatile bool event_started = false;
volatile bool ts1_recorded = false, ts2_recorded = false, ts3_recorded = false;
int silent_counter = 0;
float ste1 = 0, ste2 = 0, ste3 = 0;
float zcr1 = 0, zcr2 = 0, zcr3 = 0;

bool wifi_ready = false;

void wifi_init() {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    printf("[WIFI] Menghubungkan ke %s ...\n", WIFI_SSID);
    esp_wifi_connect();

    wifi_ap_record_t ap_info;
    while (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
        printf(".");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    printf("\n[WIFI] Terhubung ke %s, sinyal: %d dBm\n", ap_info.ssid, ap_info.rssi);
    printf("[INFO] ESP32 siap mengirim data ke server!\n");
    wifi_ready = true;  // <--- Tambahkan ini
}


void encode_spaces(const char *src, char *dst, size_t dst_size) {
    int j = 0;
    for (int i = 0; src[i] != '\0' && j < dst_size - 1; i++) {
        if (src[i] == ' ' && j < dst_size - 3) {
            dst[j++] = '%';
            dst[j++] = '2';
            dst[j++] = '0';
        } else {
            dst[j++] = src[i];
        }
    }
    dst[j] = '\0';
}

void kirim_data_http(uint64_t ts1, uint64_t ts2, uint64_t ts3, const char *lokasi) {
    if (!wifi_ready) {
        printf("[HTTP] Belum terhubung WiFi, data tidak dikirim.\n");
        return;
    }

    char lokasi_encoded[64];
    encode_spaces(lokasi, lokasi_encoded, sizeof(lokasi_encoded));

    char url[256];
    snprintf(url, sizeof(url),
             SERVER_URL "?ts1=%llu&ts2=%llu&ts3=%llu&lokasi=%s",
             ts1, ts2, ts3, lokasi_encoded);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 3000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        printf("[HTTP] Gagal inisialisasi client.\n");
        return;
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        printf("[HTTP] Data terkirim: %s\n", url);
    } else {
        printf("[HTTP] Gagal kirim data: %s\n", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

void init_i2s(i2s_port_t port, int bck, int ws, int din) {
    i2s_config_t config = {
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
    i2s_driver_install(port, &config, 0, NULL);

    i2s_pin_config_t pin_config = {
        .bck_io_num = bck,
        .ws_io_num = ws,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = din,
        .mck_io_num = I2S_PIN_NO_CHANGE
    };
    i2s_set_pin(port, &pin_config);
    i2s_zero_dma_buffer(port);
}

void remap_i2s_input(int gpio_num) {
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
    gpio_iomux_in(gpio_num, I2S0I_DATA_IN0_IDX);
}

void compute_ste_zcr(int32_t *buf, float *ste_out, float *zcr_out) {
    float ste = 0;
    int zcr = 0;
    int last_sign = 0;
    for (int i = 0; i < I2S_READ_LEN; i++) {
        int16_t sample = buf[i] >> 14;
        ste += sample * sample;
        int sign = (sample >= 0) ? 1 : -1;
        if (i > 0 && sign != last_sign) zcr++;
        last_sign = sign;
    }
    *ste_out = ste / I2S_READ_LEN;
    *zcr_out = (float)zcr / I2S_READ_LEN;
}

void process_signal(int32_t *buf, int *peak, uint64_t *ts, bool *recorded, uint64_t base_time, int threshold, float *ste, float *zcr) {
    int max = 0, idx = 0;
    for (int i = 0; i < I2S_READ_LEN; i++) {
        int amp = abs(buf[i] >> 14);
        if (amp > max) { max = amp; idx = i; }
    }
    *peak = max;
    compute_ste_zcr(buf, ste, zcr);

    uint64_t t_now = esp_timer_get_time();
    uint64_t t_arrival = t_now - ((I2S_READ_LEN - idx) * 1000000ULL / SAMPLE_RATE);

    if (!(*recorded) && max > threshold && *ste > STE_THRESHOLD && *zcr < ZCR_THRESHOLD && base_time > 0 && t_arrival >= base_time) {
        *ts = t_arrival - base_time;
        *recorded = true;
    }
}

void read_mic13_task(void *arg) {
    size_t bytes_read;
    bool baca_mic1 = true;
    while (1) {
        if (baca_mic1) {
            remap_i2s_input(MIC1_SD);
            i2s_read(I2S_NUM_0, mic1_buf, sizeof(mic1_buf), &bytes_read, portMAX_DELAY);
            process_signal(mic1_buf, &peak1, &ts1, &ts1_recorded, base_time, THRESHOLD1, &ste1, &zcr1);

            remap_i2s_input(MIC3_SD);
            i2s_read(I2S_NUM_0, mic3_buf, sizeof(mic3_buf), &bytes_read, portMAX_DELAY);
            process_signal(mic3_buf, &peak3, &ts3, &ts3_recorded, base_time, THRESHOLD3, &ste3, &zcr3);
        } else {
            remap_i2s_input(MIC3_SD);
            i2s_read(I2S_NUM_0, mic3_buf, sizeof(mic3_buf), &bytes_read, portMAX_DELAY);
            process_signal(mic3_buf, &peak3, &ts3, &ts3_recorded, base_time, THRESHOLD3, &ste3, &zcr3);

            remap_i2s_input(MIC1_SD);
            i2s_read(I2S_NUM_0, mic1_buf, sizeof(mic1_buf), &bytes_read, portMAX_DELAY);
            process_signal(mic1_buf, &peak1, &ts1, &ts1_recorded, base_time, THRESHOLD1, &ste1, &zcr1);
        }
        baca_mic1 = !baca_mic1;
    }
}

void read_mic2_task(void *arg) {
    size_t bytes_read;
    while (1) {
        i2s_read(I2S_NUM_1, mic2_buf, sizeof(mic2_buf), &bytes_read, portMAX_DELAY);
        process_signal(mic2_buf, &peak2, &ts2, &ts2_recorded, base_time, THRESHOLD2, &ste2, &zcr2);
    }
}

void monitor_task(void *arg) {
    while (1) {
        if (!event_started &&
            ((peak1 > THRESHOLD1 && ste1 > STE_THRESHOLD && zcr1 < ZCR_THRESHOLD) ||
             (peak2 > THRESHOLD2 && ste2 > STE_THRESHOLD && zcr2 < ZCR_THRESHOLD) ||
             (peak3 > THRESHOLD3 && ste3 > STE_THRESHOLD && zcr3 < ZCR_THRESHOLD))) {
            base_time = esp_timer_get_time();
            ts1 = ts2 = ts3 = 0;
            ts1_recorded = ts2_recorded = ts3_recorded = false;
            event_started = true;
        }

        const char *lokasi = "BELUM TERDETEKSI";
        if (ts1 > 0 || ts2 > 0 || ts3 > 0) {
            if (ts1 <= ts2 && ts1 <= ts3 && ts1 > 0) lokasi = "DEKAT MIC1";
            else if (ts2 <= ts1 && ts2 <= ts3 && ts2 > 0) lokasi = "DEKAT MIC2";
            else if (ts3 <= ts1 && ts3 <= ts2 && ts3 > 0) lokasi = "DEKAT MIC3";
        }

        uint64_t show1 = ts1_recorded ? ts1 : 0;
        uint64_t show2 = ts2_recorded ? ts2 : 0;
        uint64_t show3 = ts3_recorded ? ts3 : 0;

         //tes sini
        // printf("Mic1[Peak=%d, STE=%.1f, ZCR=%.2f, T=+%llu µs] | Mic2[Peak=%d, STE=%.1f, ZCR=%.2f, T=+%llu µs] | Mic3[Peak=%d, STE=%.1f, ZCR=%.2f, T=+%llu µs] => Lokasi: %s\n",
        //     peak1, ste1, zcr1, show1,
        //     peak2, ste2, zcr2, show2,
        //     peak3, ste3, zcr3, show3,
        //     lokasi);

        printf("%llu,%llu,%llu,%s\n", show1, show2, show3, lokasi);

        // if (ts1_recorded || ts2_recorded || ts3_recorded) {
        //     kirim_data_http(show1, show2, show3, lokasi);
        // }

        // Kirim data ke server SETIAP LOOP (terdeteksi atau tidak)
        kirim_data_http(show1, show2, show3, lokasi);

        if (peak1 < THRESHOLD1 && peak2 < THRESHOLD2 && peak3 < THRESHOLD3) {
            silent_counter++;
            if (silent_counter >= SILENT_LIMIT) {
                base_time = 0;
                ts1 = ts2 = ts3 = 0;
                ts1_recorded = ts2_recorded = ts3_recorded = false;
                event_started = false;
                silent_counter = 0;
            }
        } else {
            silent_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main() {
    wifi_init();

    init_i2s(I2S_NUM_0, MIC13_SCK, MIC13_WS, MIC1_SD);
    init_i2s(I2S_NUM_1, MIC2_SCK, MIC2_WS, MIC2_SD);

    xTaskCreatePinnedToCore(read_mic13_task, "mic13", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(read_mic2_task, "mic2", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(monitor_task, "monitor", 4096, NULL, 1, NULL, 0);
}
