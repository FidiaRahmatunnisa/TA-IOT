#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

// === Konfigurasi Umum ===
#define SAMPLE_RATE     96000
#define I2S_READ_LEN    1024
#define THRESHOLD       100
#define STE_THRESHOLD   200000.0f
#define ZCR_THRESHOLD   0.2f

// === I2S0 Mic1 ===
#define MIC1_SCK        26
#define MIC1_WS         25
#define MIC1_SD         22

// === I2S1 Mic2 ===
#define MIC2_SCK        14
#define MIC2_WS         27
#define MIC2_SD         33

// === UART ===
#define UART_NUM        UART_NUM_1
#define UART_RXD        16 //RX2
#define UART_TXD        17 //TX2
#define BUF_SIZE        1024

int32_t mic1_buf[I2S_READ_LEN];
int32_t mic2_buf[I2S_READ_LEN];

float ste1 = 0, zcr1 = 0;
float ste2 = 0, zcr2 = 0;

int peak1 = 0, peak2 = 0;

volatile uint64_t ts1 = 0, ts2 = 0, ts3 = 0;
volatile uint64_t base_time = 0;

// === Status Flag ===
volatile bool mic1_ready = false;
volatile bool mic2_ready = false;
volatile bool mic3_ready = false;

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

void init_i2s_mic1() {
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
    i2s_pin_config_t pin_config = {
        .bck_io_num = MIC1_SCK,
        .ws_io_num = MIC1_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = MIC1_SD,
        .mck_io_num = I2S_PIN_NO_CHANGE
    };
    i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_0);
}

void init_i2s_mic2() {
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
    i2s_pin_config_t pin_config = {
        .bck_io_num = MIC2_SCK,
        .ws_io_num = MIC2_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = MIC2_SD,
        .mck_io_num = I2S_PIN_NO_CHANGE
    };
    i2s_driver_install(I2S_NUM_1, &config, 0, NULL);
    i2s_set_pin(I2S_NUM_1, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_1);
}

void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TXD, UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Tunggu sebentar agar ESP2 siap
    vTaskDelay(pdMS_TO_TICKS(500));

    // Kirim sinyal START
    const char *start_signal = "START";
    uart_write_bytes(UART_NUM, start_signal, strlen(start_signal));
}

void send_start_signal() {
    const char *start_signal = "START\n";
    uart_write_bytes(UART_NUM, start_signal, strlen(start_signal));
    printf("📡 START signal sent to ESP2\n");
}

void mic1_task(void *arg) {
    while (1) {
        size_t bytes_read;
        i2s_read(I2S_NUM_0, mic1_buf, sizeof(mic1_buf), &bytes_read, portMAX_DELAY);
        compute_ste_zcr(mic1_buf, &ste1, &zcr1);

        int max = 0, idx = 0;
        for (int i = 0; i < I2S_READ_LEN; i++) {
            int amp = abs(mic1_buf[i] >> 14);
            if (amp > max) { max = amp; idx = i; }
        }
        peak1 = max;

        if (ts1 == 0 && peak1 > THRESHOLD && ste1 > STE_THRESHOLD && zcr1 < ZCR_THRESHOLD) {
            uint64_t t_us = (idx * 1000000ULL) / SAMPLE_RATE;
            ts1 = (esp_timer_get_time() - base_time) + t_us;
            mic1_ready = true;
            printf("Mic1 TOA: %06llu µs\n", ts1);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void mic2_task(void *arg) {
    while (1) {
        size_t bytes_read;
        i2s_read(I2S_NUM_1, mic2_buf, sizeof(mic2_buf), &bytes_read, portMAX_DELAY);
        compute_ste_zcr(mic2_buf, &ste2, &zcr2);

        int max = 0, idx = 0;
        for (int i = 0; i < I2S_READ_LEN; i++) {
            int amp = abs(mic2_buf[i] >> 14);
            if (amp > max) { max = amp; idx = i; }
        }
        peak2 = max;

        if (ts2 == 0 && peak2 > THRESHOLD && ste2 > STE_THRESHOLD && zcr2 < ZCR_THRESHOLD) {
            uint64_t t_us = (idx * 1000000ULL) / SAMPLE_RATE;
            ts2 = (esp_timer_get_time() - base_time) + t_us;
            mic2_ready = true;
            printf("Mic2 TOA: %06llu µs\n", ts2);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void uart_receive_task(void *arg) {
    uint8_t data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            if (sscanf((char*)data, "M3:%llu", &ts3) == 1) {
                mic3_ready = true;
                printf("Mic3 TOA : %06llu µs\n", ts3);

                // Jika ketiganya sudah siap
                if (mic1_ready && mic2_ready) {
                    // printf("✔ TS1:%06llu, TS2:%06llu, TS3:%06llu\n", ts1, ts2, ts3);
                    printf("uart oke!\n");

                    // Reset semuanya
                    ts1 = ts2 = ts3 = 0;
                    mic1_ready = mic2_ready = mic3_ready = false;
                    base_time = esp_timer_get_time();
                } else {
                    printf("⏳ Menunggu Mic1 dan Mic2...\n");
                }
            } else {
                printf("⚠ Data UART tidak sesuai format: %s\n", data);
            }
        }
    }
}

void test_esp1() {
    init_i2s_mic1();
    init_i2s_mic2();
    init_uart();

    send_start_signal(); // 🔁 Tambahan baru di sini
    base_time = esp_timer_get_time();

    xTaskCreatePinnedToCore(mic1_task, "mic1", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(mic2_task, "mic2", 4096, NULL, 1, NULL, 1);
    xTaskCreate(uart_receive_task, "uart_rx", 4096, NULL, 1, NULL);
}

