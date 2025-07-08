#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_timer.h"

// === Konfigurasi ===
#define SAMPLE_RATE     48000
#define I2S_READ_LEN    1024
#define I2S_BITS        I2S_BITS_PER_SAMPLE_32BIT

// === Pin I2S Mic1 (I2S_NUM_0) ===
#define MIC1_WS         25
#define MIC1_SCK        26
#define MIC1_SD         22

// === Pin I2S Mic2 (I2S_NUM_1) ===
#define MIC2_WS         27
#define MIC2_SCK        14
#define MIC2_SD         33

// === Buffer dan Variabel Global ===
int32_t mic1_buf[I2S_READ_LEN];
int32_t mic2_buf[I2S_READ_LEN];

volatile int peak1 = 0;
volatile int peak2 = 0;
volatile uint64_t ts1 = 0;
volatile uint64_t ts2 = 0;

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

// === Task Pembaca Mic1 ===
void read_mic1_task(void *arg) {
    size_t bytes_read;
    while (1) {
        uint64_t t0 = esp_timer_get_time();
        i2s_read(I2S_NUM_0, mic1_buf, sizeof(mic1_buf), &bytes_read, portMAX_DELAY);

        int local_peak = 0;
        int peak_idx = 0;
        for (int i = 0; i < I2S_READ_LEN; i++) {
            int amp = abs(mic1_buf[i] >> 14);
            if (amp > local_peak) {
                local_peak = amp;
                peak_idx = i;
            }
        }

        peak1 = local_peak;
        ts1 = t0 + ((uint64_t)peak_idx * 1000000ULL / SAMPLE_RATE);
    }
}

// === Task Pembaca Mic2 ===
void read_mic2_task(void *arg) {
    size_t bytes_read;
    while (1) {
        uint64_t t0 = esp_timer_get_time();
        i2s_read(I2S_NUM_1, mic2_buf, sizeof(mic2_buf), &bytes_read, portMAX_DELAY);

        int local_peak = 0;
        int peak_idx = 0;
        for (int i = 0; i < I2S_READ_LEN; i++) {
            int amp = abs(mic2_buf[i] >> 14);
            if (amp > local_peak) {
                local_peak = amp;
                peak_idx = i;
            }
        }

        peak2 = local_peak;
        ts2 = t0 + ((uint64_t)peak_idx * 1000000ULL / SAMPLE_RATE);
    }
}

// === Task Monitoring Hasil ===
void monitor_task(void *arg) {
    while (1) {
        printf("Mic1: Peak=%d @ %llu µs | Mic2: Peak=%d @ %llu µs | Selisih = %lld µs\n",
               peak1, ts1, peak2, ts2, (int64_t)(ts2 - ts1));
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// === Entry Point Aplikasi ===
void app_main() {
    init_i2s(I2S_NUM_0, MIC1_SCK, MIC1_WS, MIC1_SD);  // Mic1
    init_i2s(I2S_NUM_1, MIC2_SCK, MIC2_WS, MIC2_SD);  // Mic2

    xTaskCreatePinnedToCore(read_mic1_task, "mic1", 4096, NULL, 1, NULL, 0);  // Core 0
    xTaskCreatePinnedToCore(read_mic2_task, "mic2", 4096, NULL, 1, NULL, 1);  // Core 1
    xTaskCreate(monitor_task, "monitor", 4096, NULL, 1, NULL);
}
