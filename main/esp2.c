// mic3_calib_and_detect.c
// ESP-IDF single-mic (Mic3) code with calibration mode & detection
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_timer.h"
#include "esp_log.h"

// ---------------- CONFIG ----------------
// Mode kalibrasi = 1 untuk mengumpulkan sample, Mode Deteksi 0 untuk  normal
#define CALIBRATE       1

// Jika CALIBRATE==0, gunakan threshold di bawah ini (isi dari hasil kalibrasi)
#define STE_THRESHOLD   10359952.65//200000.0f
#define PEAK_THRESHOLD  3328.6//100
#define ZCR_THRESHOLD   0.0000f//0.20f

// Jumlah event yang dikumpulkan saat kalibrasi
#define N_EVENTS        100

// I2S / audio config
#define SAMPLE_RATE     96000
#define I2S_READ_LEN    1024   // jumlah sample per frame
#define I2S_PORT        I2S_NUM_0

// Pin Mic3 (sesuaikan wiring)
#define MIC3_SCK        26
#define MIC3_WS         25
#define MIC3_SD         22

// ----------------------------------------

static const char *TAG = "mic3";

static int32_t i2s_buf[I2S_READ_LEN];

// ---- fungsi bantu: inisialisasi I2S ----
void init_i2s_mic3()
{
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

// ---- compute STE, ZCR, PEAK, idx_peak ----
void compute_ste_zcr_peak(int32_t *buf, int len, float *ste_out, float *zcr_out, int *peak_out, int *idx_out)
{
    double ste = 0.0;
    int zcr = 0;
    int last_sign = 0;
    int peak = 0;
    int idx = 0;

    for (int i = 0; i < len; ++i) {
        // INMP441 32-bit left aligned; shift to 16-bit approx
        int16_t sample = buf[i] >> 14; // from previous convention
        ste += (double)sample * (double)sample;
        int sign = (sample >= 0) ? 1 : -1;
        if (i > 0 && sign != last_sign) zcr++;
        last_sign = sign;

        int amp = abs(sample);
        if (amp > peak) { peak = amp; idx = i; }
    }

    *ste_out = (float)(ste / len);
    *zcr_out = (float)zcr / len;
    *peak_out = peak;
    *idx_out = idx;
}

// ---- helper statistik sederhana ----
static void compute_mean_std(const float *arr, int n, double *mean_out, double *std_out)
{
    if (n <= 0) { *mean_out = 0; *std_out = 0; return; }
    double sum = 0;
    for (int i = 0; i < n; ++i) sum += arr[i];
    double mean = sum / n;
    double s = 0;
    for (int i = 0; i < n; ++i) {
        double d = arr[i] - mean;
        s += d * d;
    }
    double var = (n > 1) ? (s / (n - 1)) : 0.0;
    *mean_out = mean;
    *std_out = sqrt(var);
}

// ---- TASK utama: mode kalibrasi atau deteksi ----
void mic3_main_task(void *arg)
{
    // buffer untuk kalibrasi (jika CALIBRATE)
#if CALIBRATE
    float *ste_list = malloc(sizeof(float) * N_EVENTS);
    float *zcr_list = malloc(sizeof(float) * N_EVENTS);
    float *peak_list = malloc(sizeof(float) * N_EVENTS);
    int collected = 0;
    ESP_LOGI(TAG, "Running in CALIBRATION mode. Collecting %d events...", N_EVENTS);
#else
    ESP_LOGI(TAG, "Running in DETECTION mode. Using thresholds: STE=%.1f, PEAK=%d, ZCR=%.3f",
             STE_THRESHOLD, (int)PEAK_THRESHOLD, ZCR_THRESHOLD);
#endif

    bool event_started = false;
    uint64_t event_start_time = 0;
    bool sent_for_this_event = false;

    while (1) {
        size_t bytes = 0;
        // read frame (blocking)
        i2s_read(I2S_PORT, i2s_buf, sizeof(i2s_buf), &bytes, portMAX_DELAY);
        // compute features
        float ste, zcr;
        int peak, idx;
        compute_ste_zcr_peak(i2s_buf, I2S_READ_LEN, &ste, &zcr, &peak, &idx);

        // compute approximate time offset of peak within the buffer (microseconds)
        uint64_t idx_offset_us = ((uint64_t)idx * 1000000ULL) / SAMPLE_RATE;
        uint64_t now = esp_timer_get_time();
        // we will set event_start_time when event starts (for relative TOA)

#if CALIBRATE
        // Simple trigger condition to register an event sample during calibration:
        // Use a permissive peak threshold to capture impulsive events (tune if needed)
        const int CAL_PEAK_TRIGGER = 50;
        if (!event_started && peak > CAL_PEAK_TRIGGER) {
            event_started = true;
            event_start_time = now;
            sent_for_this_event = false;
        }

        if (event_started && !sent_for_this_event && peak > (CAL_PEAK_TRIGGER/2)) {
            // register sample
            if (collected < N_EVENTS) {
                ste_list[collected] = ste;
                zcr_list[collected] = zcr;
                peak_list[collected] = (float)peak;
                collected++;
                ESP_LOGI(TAG, "CAL[%3d] -> STE: %.2f | ZCR: %.4f | PEAK: %d | idx:%d | idx_us:%llu",
                         collected, ste, zcr, peak, idx, idx_offset_us);
            }
            sent_for_this_event = true;
            // small delay to avoid multiple samples for same impact
            vTaskDelay(pdMS_TO_TICKS(250));
            event_started = false;
        }

        if (collected >= N_EVENTS) {
            // compute stats & print recommendation
            double ste_mean, ste_std, zcr_mean, zcr_std, peak_mean, peak_std;
            compute_mean_std(ste_list, N_EVENTS, &ste_mean, &ste_std);
            compute_mean_std(zcr_list, N_EVENTS, &zcr_mean, &zcr_std);
            compute_mean_std(peak_list, N_EVENTS, &peak_mean, &peak_std);

            ESP_LOGI(TAG, "=== KALIBRASI SELESAI (%d sampel) ===", N_EVENTS);
            ESP_LOGI(TAG, "STE mean=%.2f std=%.2f", ste_mean, ste_std);
            ESP_LOGI(TAG, "ZCR mean=%.4f std=%.4f", zcr_mean, zcr_std);
            ESP_LOGI(TAG, "PEAK mean=%.2f std=%.2f", peak_mean, peak_std);

            // rekomendasi threshold: mean + k*std (k = 2 untuk STE/PEAK), ZCR choose mean - k*std
            double ste_reco = ste_mean + 0.5 * ste_std;
            double peak_reco = peak_mean + 0.5 * peak_std;
            double zcr_reco = zcr_mean - 0.5 * zcr_std;
            if (zcr_reco < 0.0) zcr_reco = 0.0;

            ESP_LOGI(TAG, "-> Rekomendasi THRESHOLD: STE=%.2f , PEAK=%.1f , ZCR=%.4f",
                     ste_reco, peak_reco, zcr_reco);

            ESP_LOGI(TAG, "Salin nilai rekomendasi ini ke konstanta deteksi (CALIBRATE=0) untuk uji nyata.");
            // free mem dan berhenti kalibrasi
            free(ste_list); free(zcr_list); free(peak_list);
            // sleep forever supaya tidak mengulang
            while (1) vTaskDelay(pdMS_TO_TICKS(1000));
        }

#else
        // DETECTION MODE
        // Trigger start: semua kondisi terpenuhi
        if (!event_started &&
            peak > PEAK_THRESHOLD &&
            ste > STE_THRESHOLD &&
            zcr < ZCR_THRESHOLD) {
            event_started = true;
            sent_for_this_event = false;
            event_start_time = now;
            ESP_LOGI(TAG, "Event START detected (peak=%d ste=%.2f zcr=%.4f)", peak, ste, zcr);
        }

        if (event_started && !sent_for_this_event &&
            peak > PEAK_THRESHOLD &&
            ste > STE_THRESHOLD &&
            zcr < ZCR_THRESHOLD) {

            uint64_t toa_rel = (now - event_start_time) + idx_offset_us;
            ESP_LOGI(TAG, "TOA Mic3: %llu us | peak=%d | STE=%.2f | ZCR=%.4f | idx=%d (idx_us=%llu)",
                     toa_rel, peak, ste, zcr, idx, idx_offset_us);
            sent_for_this_event = true;
        }

        // reset jika suara hilang
        if (event_started && peak < (PEAK_THRESHOLD / 2) && ste < (STE_THRESHOLD * 0.5f)) {
            event_started = false;
            sent_for_this_event = false;
            // small delay to avoid bounce
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        // juga print nilai raw tiap beberapa frame agar operator bisa lihat perubahan lingkungan
        static int frame_counter = 0;
        frame_counter++;
        if ((frame_counter % 10) == 0) {
            printf("RAW: STE=%.2f | ZCR=%.4f | PEAK=%d | idx=%d\n", ste, zcr, peak, idx);
        }
#endif

        // sedikit delay -> menurunkan beban CPU (tune sesuai kebutuhan)
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ---- app_main ----
void app_main(void)
{
    init_i2s_mic3();
    xTaskCreatePinnedToCore(mic3_main_task, "mic3_main", 4096, NULL, 5, NULL, 1);
}
