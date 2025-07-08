#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_timer.h"

// === Konfigurasi Umum ===
#define SAMPLE_RATE     48000
#define I2S_READ_LEN    1024
#define I2S_BITS        I2S_BITS_PER_SAMPLE_32BIT

// === Pin Mic1 (I2S_NUM_0) ===
#define MIC1_WS         25
#define MIC1_SCK        26
#define MIC1_SD         22

// === Pin Mic2 (I2S_NUM_1) ===
#define MIC2_WS         27
#define MIC2_SCK        14
#define MIC2_SD         33

// === Pin Mic3 (juga pakai I2S_NUM_0 dengan switching) ===
#define MIC3_SD         32

// === Buffer dan Variabel Global ===
int32_t mic1_buf[I2S_READ_LEN];
int32_t mic2_buf[I2S_READ_LEN];
int32_t mic3_buf[I2S_READ_LEN];

volatile int peak1 = 0;
volatile int peak2 = 0;
volatile int peak3 = 0;
volatile uint64_t ts1 = 0;
volatile uint64_t ts2 = 0;
volatile uint64_t ts3 = 0;

// === Fungsi Inisialisasi I2S ===
void init_i2s(i2s_port_t port, int bck, int ws, int din) {
    i2s_config_t config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = bck,
        .ws_io_num = ws,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = din,
        .mck_io_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(port, &config, 0, NULL);
    i2s_set_pin(port, &pin_config);
    i2s_zero_dma_buffer(port);
}

// === Fungsi untuk Switching Input I2S Data In ===
void switch_i2s_input(i2s_port_t port, int new_din_gpio, int bck, int ws) {
    i2s_pin_config_t new_pins = {
        .bck_io_num = bck,
        .ws_io_num = ws,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = new_din_gpio,
        .mck_io_num = I2S_PIN_NO_CHANGE
    };
    i2s_set_pin(port, &new_pins);
}

// === Task Baca Mic1 & Mic3 Bergantian ===
void read_mic1_and_mic3_task(void *arg) {
    size_t bytes_read;
    while (1) {
        // === Baca Mic1 ===
        switch_i2s_input(I2S_NUM_0, MIC1_SD, MIC1_SCK, MIC1_WS);
        uint64_t t0 = esp_timer_get_time();
        i2s_read(I2S_NUM_0, mic1_buf, sizeof(mic1_buf), &bytes_read, portMAX_DELAY);

        int local_peak1 = 0;
        int peak_idx1 = 0;
        for (int i = 0; i < I2S_READ_LEN; i++) {
            int amp = abs(mic1_buf[i] >> 14);
            if (amp > local_peak1) {
                local_peak1 = amp;
                peak_idx1 = i;
            }
        }
        peak1 = local_peak1;
        ts1 = t0 + ((uint64_t)peak_idx1 * 1000000ULL / SAMPLE_RATE);

        // === Baca Mic3 ===
        switch_i2s_input(I2S_NUM_0, MIC3_SD, MIC1_SCK, MIC1_WS);
        uint64_t t3 = esp_timer_get_time();
        i2s_read(I2S_NUM_0, mic3_buf, sizeof(mic3_buf), &bytes_read, portMAX_DELAY);

        int local_peak3 = 0;
        int peak_idx3 = 0;
        for (int i = 0; i < I2S_READ_LEN; i++) {
            int amp = abs(mic3_buf[i] >> 14);
            if (amp > local_peak3) {
                local_peak3 = amp;
                peak_idx3 = i;
            }
        }
        peak3 = local_peak3;
        ts3 = t3 + ((uint64_t)peak_idx3 * 1000000ULL / SAMPLE_RATE);
    }
}

// === Task Baca Mic2 (I2S_NUM_1) ===
void read_mic2_task(void *arg) {
    size_t bytes_read;
    while (1) {
        uint64_t t2_start = esp_timer_get_time();
        i2s_read(I2S_NUM_1, mic2_buf, sizeof(mic2_buf), &bytes_read, portMAX_DELAY);

        int local_peak2 = 0;
        int peak_idx2 = 0;
        for (int i = 0; i < I2S_READ_LEN; i++) {
            int amp = abs(mic2_buf[i] >> 14);
            if (amp > local_peak2) {
                local_peak2 = amp;
                peak_idx2 = i;
            }
        }
        peak2 = local_peak2;
        ts2 = t2_start + ((uint64_t)peak_idx2 * 1000000ULL / SAMPLE_RATE);
    }
}

// === Task Monitoring Output Serial ===
void monitor_task(void *arg) {
    while (1) {
        printf("Mic1: Peak=%d @ %llu µs | Mic2: Peak=%d @ %llu µs | Mic3: Peak=%d @ %llu µs | ∆1-2: %lld µs | ∆1-3: %lld µs\n",
               peak1, ts1, peak2, ts2, peak3, ts3,
               (int64_t)(ts2 - ts1),
               (int64_t)(ts3 - ts1));
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// === Entry Point ===
void app_main() {
    init_i2s(I2S_NUM_0, MIC1_SCK, MIC1_WS, MIC1_SD);  // Mic1 & Mic3
    init_i2s(I2S_NUM_1, MIC2_SCK, MIC2_WS, MIC2_SD);  // Mic2

    xTaskCreatePinnedToCore(read_mic1_and_mic3_task, "mic1mic3", 4096, NULL, 1, NULL, 0);  // Core 0
    xTaskCreatePinnedToCore(read_mic2_task, "mic2", 4096, NULL, 1, NULL, 1);              // Core 1
    xTaskCreate(monitor_task, "monitor", 4096, NULL, 1, NULL);
}
