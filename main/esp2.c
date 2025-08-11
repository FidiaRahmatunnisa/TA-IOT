// mic3_calib_and_detect_auto.c
// ESP-IDF single-mic (Mic3) code with 4 auto modes + NVS storage

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

// ---------------- CONFIG ----------------
#define N_EVENTS        100
#define SAMPLE_RATE     96000
#define I2S_READ_LEN    1024
#define I2S_PORT        I2S_NUM_0
#define MIC3_SCK        26
#define MIC3_WS         25
#define MIC3_SD         22
#define MODE_DELAY_MS   5000  // jeda antar mode

// Mode enum
typedef enum {
    MODE_DETECTION = 0,
    MODE_CAL_NOISE = 1,
    MODE_CAL_TARGET = 2,
    MODE_CAL_RESULT = 3
} mic_mode_t;

// ----------------------------------------
static const char *TAG = "mic3";
static int32_t i2s_buf[I2S_READ_LEN];
static mic_mode_t current_mode = MODE_CAL_NOISE;

typedef struct {
    double mean;
    double std;
} stat_t;

typedef struct {
    double ste_thr;
    double peak_thr;
    double zcr_thr;
} threshold_t;

nvs_handle_t nvs_handle_app;

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
        int16_t sample = buf[i] >> 14;
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

// ---- compute mean & std ----
void compute_mean_std(const float *arr, int n, double *mean_out, double *std_out) {
    double sum = 0;
    for (int i = 0; i < n; i++) sum += arr[i];
    double mean = sum / n;
    double s = 0;
    for (int i = 0; i < n; i++) s += (arr[i] - mean) * (arr[i] - mean);
    *mean_out = mean;
    *std_out = sqrt(s / (n - 1));
}

// ---- Save/Load NVS (double) ----
void save_double(const char *key, double value) {
    uint64_t raw;
    memcpy(&raw, &value, sizeof(raw));  // copy bit pattern
    nvs_set_u64(nvs_handle_app, key, raw);
    nvs_commit(nvs_handle_app);
}

double load_double(const char *key) {
    uint64_t raw = 0;
    nvs_get_u64(nvs_handle_app, key, &raw);
    double value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

void save_stat(const char *key_mean, const char *key_std, stat_t val) {
    save_double(key_mean, val.mean);
    save_double(key_std, val.std);
}

void load_stat(const char *key_mean, const char *key_std, stat_t *val) {
    val->mean = load_double(key_mean);
    val->std  = load_double(key_std);
}

void save_threshold(threshold_t thr) {
    save_double("thr_ste", thr.ste_thr);
    save_double("thr_peak", thr.peak_thr);
    save_double("thr_zcr", thr.zcr_thr);
}

void load_threshold(threshold_t *thr) {
    thr->ste_thr  = load_double("thr_ste");
    thr->peak_thr = load_double("thr_peak");
    thr->zcr_thr  = load_double("thr_zcr");
}


// ---- Main Task ----
void mic3_task(void *arg) {
    float ste_list[N_EVENTS], zcr_list[N_EVENTS], peak_list[N_EVENTS];
    int collected = 0;
    threshold_t thr;
    stat_t noise_ste, noise_peak, noise_zcr;
    stat_t target_ste, target_peak, target_zcr;

    while (1) {
        size_t bytes;
        float ste, zcr; int peak;
        i2s_read(I2S_PORT, i2s_buf, sizeof(i2s_buf), &bytes, portMAX_DELAY);
        compute_ste_zcr_peak(i2s_buf, I2S_READ_LEN, &ste, &zcr, &peak);

        if (current_mode == MODE_CAL_NOISE || current_mode == MODE_CAL_TARGET) {
            if (collected < N_EVENTS) {
                ste_list[collected] = ste;
                zcr_list[collected] = zcr;
                peak_list[collected] = peak;
                collected++;
                ESP_LOGI(TAG, "CAL[%d] STE=%.2f ZCR=%.4f PEAK=%d", collected, ste, zcr, peak);
                vTaskDelay(pdMS_TO_TICKS(100));
            } else {
                stat_t s_ste, s_peak, s_zcr;
                compute_mean_std(ste_list, N_EVENTS, &s_ste.mean, &s_ste.std);
                compute_mean_std(peak_list, N_EVENTS, &s_peak.mean, &s_peak.std);
                compute_mean_std(zcr_list, N_EVENTS, &s_zcr.mean, &s_zcr.std);

                if (current_mode == MODE_CAL_NOISE) {
                    save_stat("noise_ste_m", "noise_ste_s", s_ste);
                    save_stat("noise_peak_m", "noise_peak_s", s_peak);
                    save_stat("noise_zcr_m", "noise_zcr_s", s_zcr);
                    ESP_LOGI(TAG, "Noise stats saved");
                } else {
                    save_stat("target_ste_m", "target_ste_s", s_ste);
                    save_stat("target_peak_m", "target_peak_s", s_peak);
                    save_stat("target_zcr_m", "target_zcr_s", s_zcr);
                    ESP_LOGI(TAG, "Target stats saved");
                }
                collected = 0;
                vTaskDelay(pdMS_TO_TICKS(MODE_DELAY_MS));
                current_mode++;
            }
        }
        else if (current_mode == MODE_CAL_RESULT) {
            load_stat("noise_ste_m", "noise_ste_s", &noise_ste);
            load_stat("noise_peak_m", "noise_peak_s", &noise_peak);
            load_stat("noise_zcr_m", "noise_zcr_s", &noise_zcr);
            load_stat("target_ste_m", "target_ste_s", &target_ste);
            load_stat("target_peak_m", "target_peak_s", &target_peak);
            load_stat("target_zcr_m", "target_zcr_s", &target_zcr);

            thr.ste_thr  = noise_ste.mean + 0.5 * (target_ste.mean - noise_ste.mean);
            thr.peak_thr = noise_peak.mean + 0.5 * (target_peak.mean - noise_peak.mean);
            thr.zcr_thr  = fmax(0.0, noise_zcr.mean - 0.5 * noise_zcr.std);

            save_threshold(thr);
            ESP_LOGI(TAG, "Threshold saved: STE=%.2f PEAK=%.2f ZCR=%.4f",
                     thr.ste_thr, thr.peak_thr, thr.zcr_thr);

            vTaskDelay(pdMS_TO_TICKS(MODE_DELAY_MS));
            current_mode = MODE_DETECTION;
        }
        else if (current_mode == MODE_DETECTION) {
            load_threshold(&thr);
            // output hanya deteksi
            if (peak > thr.peak_thr && ste > thr.ste_thr && zcr < thr.zcr_thr) {
                ESP_LOGI(TAG, "Event DETECTED: STE=%.2f PEAK=%d ZCR=%.4f", ste, peak, zcr);
            }
            // berbagai jenis output
            // if (peak > thr.peak_thr && ste > thr.ste_thr && zcr < thr.zcr_thr) {
            //     ESP_LOGI(TAG, "Event DETECTED: STE=%.2f PEAK=%d ZCR=%.4f -> SESUAI",
            //             ste, peak, zcr);
            // } else {
            //     ESP_LOGI(TAG, "Event NOT DETECTED: STE=%.2f PEAK=%d ZCR=%.4f -> TIDAK SESUAI",
            //             ste, peak, zcr);
            // }
        }
    }
}

// ---- app_main ----
void app_main2(void) {
    nvs_flash_init();
    nvs_open("storage", NVS_READWRITE, &nvs_handle_app);
    init_i2s_mic3();
    xTaskCreatePinnedToCore(mic3_task, "mic3_task", 8192, NULL, 5, NULL, 1);
}
