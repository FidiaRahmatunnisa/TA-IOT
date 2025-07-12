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

// === Konfigurasi Umum ===
#define SAMPLE_RATE     96000
#define I2S_READ_LEN    1024

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

// === Fungsi Switching GPIO Matrix Input ===
void remap_i2s_input(int gpio_num) {
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
    gpio_iomux_in(gpio_num, I2S0I_DATA_IN0_IDX);
}

// === Task Gabungan Mic1 dan Mic3 (via I2S0) ===
void read_mic13_task(void *arg) {
    size_t bytes_read;
    bool baca_mic1_dulu = true;

    while (1) {
        int peak = 0, idx = 0;

        if (baca_mic1_dulu) {
            // === Mic1 ===
            remap_i2s_input(MIC1_SD);
            i2s_read(I2S_NUM_0, mic1_buf, sizeof(mic1_buf), &bytes_read, portMAX_DELAY);
            uint64_t t1 = esp_timer_get_time();
            peak = 0, idx = 0;
            for (int i = 0; i < I2S_READ_LEN; i++) {
                int amp = abs(mic1_buf[i] >> 14);
                if (amp > peak) { peak = amp; idx = i; }
            }
            peak1 = peak;
            ts1 = t1 - ((I2S_READ_LEN - idx) * 1000000ULL / SAMPLE_RATE);

            // === Mic3 ===
            remap_i2s_input(MIC3_SD);
            i2s_read(I2S_NUM_0, mic3_buf, sizeof(mic3_buf), &bytes_read, portMAX_DELAY);
            t1 = esp_timer_get_time();
            peak = 0, idx = 0;
            for (int i = 0; i < I2S_READ_LEN; i++) {
                int amp = abs(mic3_buf[i] >> 14);
                if (amp > peak) { peak = amp; idx = i; }
            }
            peak3 = peak;
            ts3 = t1 - ((I2S_READ_LEN - idx) * 1000000ULL / SAMPLE_RATE);
        } else {
            // === Mic3 ===
            remap_i2s_input(MIC3_SD);
            i2s_read(I2S_NUM_0, mic3_buf, sizeof(mic3_buf), &bytes_read, portMAX_DELAY);
            uint64_t t1 = esp_timer_get_time();
            peak = 0, idx = 0;
            for (int i = 0; i < I2S_READ_LEN; i++) {
                int amp = abs(mic3_buf[i] >> 14);
                if (amp > peak) { peak = amp; idx = i; }
            }
            peak3 = peak;
            ts3 = t1 - ((I2S_READ_LEN - idx) * 1000000ULL / SAMPLE_RATE);

            // === Mic1 ===
            remap_i2s_input(MIC1_SD);
            i2s_read(I2S_NUM_0, mic1_buf, sizeof(mic1_buf), &bytes_read, portMAX_DELAY);
            t1 = esp_timer_get_time();
            peak = 0, idx = 0;
            for (int i = 0; i < I2S_READ_LEN; i++) {
                int amp = abs(mic1_buf[i] >> 14);
                if (amp > peak) { peak = amp; idx = i; }
            }
            peak1 = peak;
            ts1 = t1 - ((I2S_READ_LEN - idx) * 1000000ULL / SAMPLE_RATE);
        }

        baca_mic1_dulu = !baca_mic1_dulu;
    }
}

// === Task Mic2 (via I2S1) ===
void read_mic2_task(void *arg) {
    size_t bytes_read;
    while (1) {
        i2s_read(I2S_NUM_1, mic2_buf, sizeof(mic2_buf), &bytes_read, portMAX_DELAY);
        uint64_t t1 = esp_timer_get_time();
        int peak = 0, idx = 0;
        for (int i = 0; i < I2S_READ_LEN; i++) {
            int amp = abs(mic2_buf[i] >> 14);
            if (amp > peak) { peak = amp; idx = i; }
        }
        peak2 = peak;
        ts2 = t1 - ((I2S_READ_LEN - idx) * 1000000ULL / SAMPLE_RATE);
    }
}

// === Monitoring Task ===
void monitor_task(void *arg) {
    while (1) {

        const char* prediksi_lokasi;
        if (ts1 <= ts2 && ts1 <= ts3) {
            prediksi_lokasi = "DEKAT MIC1";
        } else if (ts2 <= ts1 && ts2 <= ts3) {
            prediksi_lokasi = "DEKAT MIC2";
        } else {
            prediksi_lokasi = "DEKAT MIC3";
        }

        printf("Mic1: Peak=%d @ %llu µs | Mic2: Peak=%d @ %llu µs | Mic3: Peak=%d @ %llu µs | %s\n",
               peak1, ts1, peak2, ts2, peak3, ts3, prediksi_lokasi);

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void app_main() {
    init_i2s(I2S_NUM_0, MIC13_SCK, MIC13_WS, MIC1_SD);  // akan diremap ke Mic3 juga
    init_i2s(I2S_NUM_1, MIC2_SCK, MIC2_WS, MIC2_SD);

    xTaskCreatePinnedToCore(read_mic13_task, "mic13", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(read_mic2_task, "mic2", 4096, NULL, 1, NULL, 1);
    xTaskCreate(monitor_task, "monitor", 4096, NULL, 1, NULL);
}
