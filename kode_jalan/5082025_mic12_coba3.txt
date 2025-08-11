#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/semphr.h"

// === Variabel Global ===
uint64_t toa_mic1 = 0;
uint64_t toa_mic2 = 0;
SemaphoreHandle_t toa_mutex;
float ste_mic1 = 0, ste_mic2 = 0;
float zcr_mic1 = 0, zcr_mic2 = 0;
int peak_mic1 = 0, peak_mic2 = 0;

// === Konfigurasi Umum ===
#define SAMPLE_RATE     96000
#define I2S_READ_LEN    1024
//===umum
// #define THRESHOLD       100
// #define STE_THRESHOLD   200000.0f
// #define ZCR_THRESHOLD   0.2f

// === Threshold Berdasarkan Mean dan StdDev UMUM===
// #define STE_MEAN        16000
// #define STE_STDDEV      3000
// #define STE_THRESHOLD   (STE_MEAN + 2 * STE_STDDEV)  

// #define PEAK_MEAN       5000
// #define PEAK_STDDEV     2000
// #define PEAK_THRESHOLD  (PEAK_MEAN + 2 * PEAK_STDDEV)  

// #define ZCR_MEAN        40
// #define ZCR_STDDEV      10
// #define ZCR_THRESHOLD   (ZCR_MEAN - 2 * ZCR_STDDEV)  

//===treshold-mic1
#define STE_MEAN1        1596092.83
#define STE_STDDEV1      9796849.95
#define STE_THRESHOLD1   (STE_MEAN1 + 2 * STE_STDDEV1)  

#define PEAK_MEAN1       876.52
#define PEAK_STDDEV1     2357.43
#define PEAK_THRESHOLD1  (PEAK_MEAN1 + 2 * PEAK_STDDEV1)  

#define ZCR_MEAN1        0.07
#define ZCR_STDDEV1      0.06
#define ZCR_THRESHOLD1   (ZCR_MEAN1 - 2 * ZCR_STDDEV1)  

//===treshold-mic2
#define STE_MEAN2        47216.37
#define STE_STDDEV2      273742.36
#define STE_THRESHOLD2   (STE_MEAN2 + 2 * STE_STDDEV2)  

#define PEAK_MEAN2       210.79
#define PEAK_STDDEV2     451.95
#define PEAK_THRESHOLD2  (PEAK_MEAN2 + 2 * PEAK_STDDEV2)  

#define ZCR_MEAN2        0.04
#define ZCR_STDDEV2      0.05
#define ZCR_THRESHOLD2   (ZCR_MEAN2 - 2 * ZCR_STDDEV2)  

// === Pin I2S Mic1 (I2S0) ===
#define MIC1_SCK        26
#define MIC1_WS         25
#define MIC1_SD         22

// === Pin I2S Mic2 (I2S1) ===
#define MIC2_SCK        14
#define MIC2_WS         27
#define MIC2_SD         33

// === UART (ESP → UART Monitor / ESP lain) ===
#define UART_PORT_NUM   UART_NUM_1
#define UART_TXD        17
#define UART_RXD        16
#define UART_BUF_SIZE   1024

// === Buffer Mic1 & Mic2 ===
int32_t mic1_buf[I2S_READ_LEN], mic2_buf[I2S_READ_LEN];

// === Fungsi bantu STE dan ZCR ===
void compute_ste_zcr(int32_t *buf, float *ste_out, float *zcr_out, int *peak_out, int *idx_out) {
    float ste = 0;
    int zcr = 0;
    int last_sign = 0, peak = 0, idx = 0;

    for (int i = 0; i < I2S_READ_LEN; i++) {
        int16_t sample = buf[i] >> 14;
        ste += sample * sample;
        int sign = (sample >= 0) ? 1 : -1;
        if (i > 0 && sign != last_sign) zcr++;
        last_sign = sign;

        int amp = abs(sample);
        if (amp > peak) { peak = amp; idx = i; }
    }

    *ste_out = ste / I2S_READ_LEN;
    *zcr_out = (float)zcr / I2S_READ_LEN;
    *peak_out = peak;
    *idx_out = idx;
}

// === Inisialisasi I2S (Mic1 dan Mic2) ===
void init_i2s(i2s_port_t port, int sck, int ws, int sd) {
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
        .bck_io_num = sck,
        .ws_io_num = ws,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = sd,
        .mck_io_num = I2S_PIN_NO_CHANGE
    };
    i2s_driver_install(port, &config, 0, NULL);
    i2s_set_pin(port, &pin_config);
    i2s_zero_dma_buffer(port);
}

// === Inisialisasi UART ===
void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// === Task Mic1 ===
void mic1_task(void *arg) {
    size_t bytes_read;
    while (1) {
        i2s_read(I2S_NUM_0, mic1_buf, sizeof(mic1_buf), &bytes_read, portMAX_DELAY);
        float ste, zcr;
        int peak, idx;
        compute_ste_zcr(mic1_buf, &ste, &zcr, &peak, &idx);
        //===Waktu gabungan idx-global => UNTUK TEST
        uint64_t now = esp_timer_get_time(); // waktu saat buffer diterima
        uint64_t ts = now - ((I2S_READ_LEN - idx) * 1000000ULL / SAMPLE_RATE);

        if (peak > PEAK_THRESHOLD1 && ste > STE_THRESHOLD1 && zcr < ZCR_THRESHOLD1) {
        //===Waktu gabungan idx-global => UNTUK SERIUS
        // uint64_t now = esp_timer_get_time(); // waktu saat buffer diterima
        // uint64_t ts = now - ((I2S_READ_LEN - idx) * 1000000ULL / SAMPLE_RATE);

        if (xSemaphoreTake(toa_mutex, portMAX_DELAY)) {
            toa_mic1 = ts;
            ste_mic1 = ste;
            zcr_mic1 = zcr;
            peak_mic1 = peak;
            xSemaphoreGive(toa_mutex);
        }

        //===print perTASK
        // char buffer[64];
        // snprintf(buffer, sizeof(buffer), "M1:%06llu\n", ts);
        // uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
        // printf("Terkirim: %s", buffer);

        vTaskDelay(pdMS_TO_TICKS(500));
    }else{
          printf("TRESHOLD-1 mungkin besar jadi tidak ada nilai yang keluar\n");
           //===lengkap = waktu + treshold
                char buffer[256];
                snprintf(buffer, sizeof(buffer),
                    "M1:%06llu | STE:%.2f ZCR:%.2f PEAK:%d\n",
                    ts, ste, zcr, peak);
                uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
                printf("%s", buffer);
    }

        }
}

// === Task Mic2 ===
void mic2_task(void *arg) {
    size_t bytes_read;
    while (1) {
        i2s_read(I2S_NUM_1, mic2_buf, sizeof(mic2_buf), &bytes_read, portMAX_DELAY);
        float ste, zcr;
        int peak, idx;
        compute_ste_zcr(mic2_buf, &ste, &zcr, &peak, &idx);
        //===Waktu gabungan idx-global
        uint64_t now = esp_timer_get_time(); // waktu saat buffer diterima
        uint64_t ts = now - ((I2S_READ_LEN - idx) * 1000000ULL / SAMPLE_RATE);

        if (peak > PEAK_THRESHOLD2 && ste > STE_THRESHOLD2 && zcr < ZCR_THRESHOLD2) {
        //===Waktu gabungan idx-global => UNTUK SERIUS
        // uint64_t now = esp_timer_get_time(); // waktu saat buffer diterima
        // uint64_t ts = now - ((I2S_READ_LEN - idx) * 1000000ULL / SAMPLE_RATE);

        if (xSemaphoreTake(toa_mutex, portMAX_DELAY)) {
            toa_mic2 = ts;
            ste_mic2 = ste;
            zcr_mic2 = zcr;
            peak_mic2 = peak;
            xSemaphoreGive(toa_mutex);
        }

        //===print perTASK
        // char buffer[64];
        // snprintf(buffer, sizeof(buffer), "M2:%06llu\n", ts);
        // uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
        // printf("Terkirim: %s", buffer);

        vTaskDelay(pdMS_TO_TICKS(500));
    }else{
        printf("TRESHOLD-2 mungkin besar jadi tidak ada nilai yang keluar\n");
         //===lengkap = waktu + treshold
                char buffer[256];
                snprintf(buffer, sizeof(buffer),
                    "M2:%06llu | STE:%.2f ZCR:%.2f PEAK:%d \n ",
                    ts, ste, zcr, peak);
                uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
                printf("%s", buffer);
    }

    }
}

void monitor_task(void *arg) {
    uint64_t t1 = 0, t2 = 0;
    while (1) {
        if (xSemaphoreTake(toa_mutex, pdMS_TO_TICKS(10))) {
            t1 = toa_mic1;
            t2 = toa_mic2;
            xSemaphoreGive(toa_mutex);

            if (t1 > 0 && t2 > 0) {
                char lokasi[32];

                if (t1 < t2) {
                    strcpy(lokasi, "Dekat Mic1");
                } else if (t2 < t1) {
                    strcpy(lokasi, "Dekat Mic2");
                } else {
                    strcpy(lokasi, "Tengah");
                }

                //===hanya waktu => hidupkan kalau if-peak-ste-zcr HIDUP
                char buffer[128];
                snprintf(buffer, sizeof(buffer), "M1:%06llu, M2:%06llu, Lokasi: %s\n", t1, t2, lokasi);
                uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));  
                printf("%s", buffer);  

                //===lengkap = waktu + treshold
                // char buffer[256];
                // snprintf(buffer, sizeof(buffer),
                //     "M1:%06llu | STE:%.2f ZCR:%.2f PEAK:%d | "
                //     "M2:%06llu | STE:%.2f ZCR:%.2f PEAK:%d | Lokasi: %s\n",
                //     t1, ste_mic1, zcr_mic1, peak_mic1,
                //     t2, ste_mic2, zcr_mic2, peak_mic2,
                //     lokasi);
                // uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
                // printf("%s", buffer);

                //===print minimalis ke python
                // printf("%llu,%.2f,%.2f,%d,%llu,%.2f,%.2f,%d,%s\n",
                // t1, ste_mic1, zcr_mic1, peak_mic1,
                // t2, ste_mic2, zcr_mic2, peak_mic2,
                // lokasi);

                toa_mic1 = 0;
                toa_mic2 = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// === Main Aplikasi ===
void test_esp1() {
    toa_mutex = xSemaphoreCreateMutex();
    init_uart();
    init_i2s(I2S_NUM_0, MIC1_SCK, MIC1_WS, MIC1_SD);
    init_i2s(I2S_NUM_1, MIC2_SCK, MIC2_WS, MIC2_SD);
    xTaskCreatePinnedToCore(mic1_task, "mic1_task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(mic2_task, "mic2_task", 4096, NULL, 1, NULL, 0);
    xTaskCreate(monitor_task, "monitor_task", 4096, NULL, 4, NULL);
}
