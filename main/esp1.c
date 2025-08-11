// mic1-mic2_calib_and_detect_auto.c
// ESP-IDF single-mic (Mic1 dan Mic2) code with 4 auto modes + NVS storage
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/i2s.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

// ---------------- CONFIG ----------------
#define N_EVENTS                100
#define SAMPLE_RATE             96000
#define I2S_READ_LEN            1024
#define CAL_SAMPLE_INTERVAL_MS  100

// Pins I2S Mic1 (I2S_NUM_0)
#define MIC1_SCK    26
#define MIC1_WS     25
#define MIC1_SD     22

// Pins I2S Mic2 (I2S_NUM_1)
#define MIC2_SCK    14
#define MIC2_WS     27
#define MIC2_SD     33

// Event bits for sync
#define EVT_MIC1_CALIB_DONE (1<<0)
#define EVT_MIC2_CALIB_DONE (1<<1)

static const char *TAG = "dual_mic_sync";

static EventGroupHandle_t eg;
static SemaphoreHandle_t feat_mutex;
static nvs_handle_t nvs_handle_app = 0;

// Feature struct
typedef struct {
    float ste;
    float zcr;
    int peak;
    int idx;
    uint64_t ts_us;
} feat_t;

// Threshold struct
typedef struct {
    double ste_thr;
    double peak_thr;
    double zcr_thr;
} thr_t;

// Kalibrasi buffer dan counter mic1
static float m1_ste[N_EVENTS], m1_zcr[N_EVENTS], m1_peak[N_EVENTS];
static int m1_col = 0;
static bool m1_done = false;

// Kalibrasi buffer dan counter mic2
static float m2_ste[N_EVENTS], m2_zcr[N_EVENTS], m2_peak[N_EVENTS];
static int m2_col = 0;
static bool m2_done = false;

// Latest features dari mic1 dan mic2
static feat_t last_m1 = {0}, last_m2 = {0};

// ======= Helper functions =======

static void init_i2s_port(i2s_port_t port, int bck, int ws, int sd)
{
    i2s_config_t cfg = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    i2s_driver_install(port, &cfg, 0, NULL);
    i2s_pin_config_t pin_cfg = {
        .bck_io_num = bck,
        .ws_io_num = ws,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = sd,
        .mck_io_num = I2S_PIN_NO_CHANGE
    };
    i2s_set_pin(port, &pin_cfg);
    i2s_zero_dma_buffer(port);
}

static void compute_ste_zcr_peak(int32_t *buf, int len, float *ste_out, float *zcr_out, int *peak_out, int *idx_out)
{
    double ste = 0.0;
    int zcr = 0, last_sign = 0, peak = 0, idx = 0;
    for (int i=0;i<len;i++){
        int16_t s = buf[i] >> 14;
        ste += (double)s * (double)s;
        int sign = (s >= 0) ? 1 : -1;
        if (i>0 && sign != last_sign) zcr++;
        last_sign = sign;
        int amp = abs(s);
        if (amp > peak) { peak = amp; idx = i; }
    }
    *ste_out = (float)(ste / len);
    *zcr_out = (float)zcr / len;
    *peak_out = peak;
    *idx_out = idx;
}

static void mean_std(const float *arr, int n, double *mean_out, double *std_out)
{
    if (n<=0) { *mean_out=0; *std_out=0; return; }
    double sum=0;
    for (int i=0;i<n;i++) sum += arr[i];
    double mean = sum / n;
    double s=0;
    for (int i=0;i<n;i++) {
        double d = arr[i]-mean;
        s += d*d;
    }
    *mean_out = mean;
    *std_out = (n>1) ? sqrt(s/(n-1)) : 0.0;
}

static esp_err_t nvs_save_double(const char *key, double v)
{
    uint64_t raw;
    memcpy(&raw, &v, sizeof(raw));
    return nvs_set_u64(nvs_handle_app, key, raw);
}
static double nvs_load_double(const char *key)
{
    uint64_t raw=0;
    if (nvs_get_u64(nvs_handle_app, key, &raw) != ESP_OK) return 0.0;
    double v; memcpy(&v, &raw, sizeof(v)); return v;
}

static void save_stat(const char *keyprefix, double mean, double std)
{
    char a[64], b[64];
    snprintf(a,sizeof(a), "%s_mean", keyprefix);
    snprintf(b,sizeof(b), "%s_std", keyprefix);
    nvs_save_double(a, mean);
    nvs_save_double(b, std);
}

static void save_thr(const char *prefix, thr_t t)
{
    char a[64], b[64], c[64];
    snprintf(a,sizeof(a), "%s_thr_ste", prefix);
    snprintf(b,sizeof(b), "%s_thr_peak", prefix);
    snprintf(c,sizeof(c), "%s_thr_zcr", prefix);
    nvs_save_double(a, t.ste_thr);
    nvs_save_double(b, t.peak_thr);
    nvs_save_double(c, t.zcr_thr);
    nvs_commit(nvs_handle_app);
}

static void load_thr(const char *prefix, thr_t *t)
{
    char a[64], b[64], c[64];
    snprintf(a,sizeof(a), "%s_thr_ste", prefix);
    snprintf(b,sizeof(b), "%s_thr_peak", prefix);
    snprintf(c,sizeof(c), "%s_thr_zcr", prefix);
    t->ste_thr  = nvs_load_double(a);
    t->peak_thr = nvs_load_double(b);
    t->zcr_thr  = nvs_load_double(c);
}

// ======= Mic1 Kalibrasi task =======
static void mic1_calib_task(void *arg)
{
    ESP_LOGI(TAG, "mic1_calib_task started");

    // Initialize I2S for mic1
    init_i2s_port(I2S_NUM_0, MIC1_SCK, MIC1_WS, MIC1_SD);

    int32_t *buf = malloc(sizeof(int32_t)*I2S_READ_LEN);
    if (!buf) {
        ESP_LOGE(TAG, "malloc fail mic1");
        vTaskDelete(NULL);
    }

    // Step 1: kalibrasi noise lingkungan mic1
    m1_col = 0;
    while (m1_col < N_EVENTS) {
        size_t bytes_read;
        i2s_read(I2S_NUM_0, buf, sizeof(int32_t)*I2S_READ_LEN, &bytes_read, portMAX_DELAY);
        float ste, zcr; int peak, idx;
        compute_ste_zcr_peak(buf, I2S_READ_LEN, &ste, &zcr, &peak, &idx);

        m1_ste[m1_col] = ste; m1_zcr[m1_col] = zcr; m1_peak[m1_col] = (float)peak;
        m1_col++;
        ESP_LOGI(TAG, "mic1 noise CAL[%d] STE=%.2f ZCR=%.4f PEAK=%d", m1_col, ste, zcr, peak);
        vTaskDelay(pdMS_TO_TICKS(CAL_SAMPLE_INTERVAL_MS));
    }
    ESP_LOGI(TAG, "mic1 noise calib done");

    // Simpan statistik noise mic1
    double mean_ste, std_ste, mean_zcr, std_zcr, mean_peak, std_peak;
    mean_std(m1_ste, N_EVENTS, &mean_ste, &std_ste);
    mean_std(m1_zcr, N_EVENTS, &mean_zcr, &std_zcr);
    mean_std(m1_peak, N_EVENTS, &mean_peak, &std_peak);
    save_stat("m1_noise_ste", mean_ste, std_ste);
    save_stat("m1_noise_zcr", mean_zcr, std_zcr);
    save_stat("m1_noise_peak", mean_peak, std_peak);
    vTaskDelay(pdMS_TO_TICKS(5000)); //jeda 5 detik

    // Step 2: kalibrasi target mic1
    m1_col = 0;
    while (m1_col < N_EVENTS) {
        size_t bytes_read;
        i2s_read(I2S_NUM_0, buf, sizeof(int32_t)*I2S_READ_LEN, &bytes_read, portMAX_DELAY);
        float ste, zcr; int peak, idx;
        compute_ste_zcr_peak(buf, I2S_READ_LEN, &ste, &zcr, &peak, &idx);

        m1_ste[m1_col] = ste; m1_zcr[m1_col] = zcr; m1_peak[m1_col] = (float)peak;
        m1_col++;
        ESP_LOGI(TAG, "mic1 target CAL[%d] STE=%.2f ZCR=%.4f PEAK=%d", m1_col, ste, zcr, peak);
        vTaskDelay(pdMS_TO_TICKS(CAL_SAMPLE_INTERVAL_MS));
    }
    ESP_LOGI(TAG, "mic1 target calib done");

    // Simpan statistik target mic1
    mean_std(m1_ste, N_EVENTS, &mean_ste, &std_ste);
    mean_std(m1_zcr, N_EVENTS, &mean_zcr, &std_zcr);
    mean_std(m1_peak, N_EVENTS, &mean_peak, &std_peak);
    save_stat("m1_target_ste", mean_ste, std_ste);
    save_stat("m1_target_zcr", mean_zcr, std_zcr);
    save_stat("m1_target_peak", mean_peak, std_peak);

   // Hitung threshold mic1 dan simpan dengan std deviasi
    thr_t thr1;
    thr1.ste_thr  = mean_ste + 1.0 * std_ste;   // contoh margin 1 x std deviasi
    thr1.peak_thr = mean_peak + 1.0 * std_peak;
    thr1.zcr_thr  = fmax(0.0, mean_zcr - 0.5 * std_zcr);
    save_thr("m1", thr1);
    ESP_LOGI(TAG, "MIC1 thresholds: STE=%.2f, PEAK=%.2f, ZCR=%.2f", thr1.ste_thr, thr1.peak_thr, thr1.zcr_thr);

    ESP_LOGI(TAG, "mic1 threshold saved");

    free(buf);
    m1_done = true;
    xEventGroupSetBits(eg, EVT_MIC1_CALIB_DONE);

    vTaskDelete(NULL);
}

// ======= Mic2 Kalibrasi task =======
static void mic2_calib_task(void *arg)
{
    ESP_LOGI(TAG, "mic2_calib_task waiting mic1");

    // Tunggu sampai mic1 selesai kalibrasi
    xEventGroupWaitBits(eg, EVT_MIC1_CALIB_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "mic2_calib_task started");

    // Initialize I2S for mic2
    init_i2s_port(I2S_NUM_1, MIC2_SCK, MIC2_WS, MIC2_SD);

    int32_t *buf = malloc(sizeof(int32_t)*I2S_READ_LEN);
    if (!buf) {
        ESP_LOGE(TAG, "malloc fail mic2");
        vTaskDelete(NULL);
    }

    // Step 1: kalibrasi noise lingkungan mic2
    m2_col = 0;
    while (m2_col < N_EVENTS) {
        size_t bytes_read;
        i2s_read(I2S_NUM_1, buf, sizeof(int32_t)*I2S_READ_LEN, &bytes_read, portMAX_DELAY);
        float ste, zcr; int peak, idx;
        compute_ste_zcr_peak(buf, I2S_READ_LEN, &ste, &zcr, &peak, &idx);

        m2_ste[m2_col] = ste; m2_zcr[m2_col] = zcr; m2_peak[m2_col] = (float)peak;
        m2_col++;
        ESP_LOGI(TAG, "mic2 noise CAL[%d] STE=%.2f ZCR=%.4f PEAK=%d", m2_col, ste, zcr, peak);
        vTaskDelay(pdMS_TO_TICKS(CAL_SAMPLE_INTERVAL_MS));
    }
    ESP_LOGI(TAG, "mic2 noise calib done");

    // Simpan statistik noise mic2
    double mean_ste, std_ste, mean_zcr, std_zcr, mean_peak, std_peak;
    mean_std(m2_ste, N_EVENTS, &mean_ste, &std_ste);
    mean_std(m2_zcr, N_EVENTS, &mean_zcr, &std_zcr);
    mean_std(m2_peak, N_EVENTS, &mean_peak, &std_peak);
    save_stat("m2_noise_ste", mean_ste, std_ste);
    save_stat("m2_noise_zcr", mean_zcr, std_zcr);
    save_stat("m2_noise_peak", mean_peak, std_peak);
    vTaskDelay(pdMS_TO_TICKS(5000));// jeda 5 detik

    // Step 2: kalibrasi target mic2
    m2_col = 0;
    while (m2_col < N_EVENTS) {
        size_t bytes_read;
        i2s_read(I2S_NUM_1, buf, sizeof(int32_t)*I2S_READ_LEN, &bytes_read, portMAX_DELAY);
        float ste, zcr; int peak, idx;
        compute_ste_zcr_peak(buf, I2S_READ_LEN, &ste, &zcr, &peak, &idx);

        m2_ste[m2_col] = ste; m2_zcr[m2_col] = zcr; m2_peak[m2_col] = (float)peak;
        m2_col++;
        ESP_LOGI(TAG, "mic2 target CAL[%d] STE=%.2f ZCR=%.4f PEAK=%d", m2_col, ste, zcr, peak);
        vTaskDelay(pdMS_TO_TICKS(CAL_SAMPLE_INTERVAL_MS));
    }
    ESP_LOGI(TAG, "mic2 target calib done");

    // Simpan statistik target mic2
    mean_std(m2_ste, N_EVENTS, &mean_ste, &std_ste);
    mean_std(m2_zcr, N_EVENTS, &mean_zcr, &std_zcr);
    mean_std(m2_peak, N_EVENTS, &mean_peak, &std_peak);
    save_stat("m2_target_ste", mean_ste, std_ste);
    save_stat("m2_target_zcr", mean_zcr, std_zcr);
    save_stat("m2_target_peak", mean_peak, std_peak);

    // Hitung threshold mic2 dan simpan dengan std deviasi
    thr_t thr2;
    thr2.ste_thr  = mean_ste + 1.0 * std_ste;
    thr2.peak_thr = mean_peak + 1.0 * std_peak;
    thr2.zcr_thr  = fmax(0.0, mean_zcr - 0.5 * std_zcr);
    save_thr("m2", thr2);
    ESP_LOGI(TAG, "MIC2 thresholds: STE=%.2f, PEAK=%.2f, ZCR=%.2f", thr2.ste_thr, thr2.peak_thr, thr2.zcr_thr);

    ESP_LOGI(TAG, "mic2 threshold saved");

    free(buf);
    m2_done = true;
    xEventGroupSetBits(eg, EVT_MIC2_CALIB_DONE);

    vTaskDelete(NULL);
}

// ======= Mic1 Reader Task =======
static void mic1_reader_task(void *arg)
{
    int32_t *buf = malloc(sizeof(int32_t)*I2S_READ_LEN);
    if (!buf) {
        ESP_LOGE(TAG, "malloc fail mic1_reader");
        vTaskDelete(NULL);
    }

    while(1) {
        size_t bytes_read;
        i2s_read(I2S_NUM_0, buf, sizeof(int32_t)*I2S_READ_LEN, &bytes_read, portMAX_DELAY);

        float ste, zcr;
        int peak, idx;
        compute_ste_zcr_peak(buf, I2S_READ_LEN, &ste, &zcr, &peak, &idx);

        uint64_t now = esp_timer_get_time();

        xSemaphoreTake(feat_mutex, portMAX_DELAY);
        last_m1.ste = ste; last_m1.zcr = zcr; last_m1.peak = peak; last_m1.idx = idx; last_m1.ts_us = now;
        xSemaphoreGive(feat_mutex);

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    free(buf);
    vTaskDelete(NULL);
}

// ======= Mic2 Reader Task =======
static void mic2_reader_task(void *arg)
{
    int32_t *buf = malloc(sizeof(int32_t)*I2S_READ_LEN);
    if (!buf) {
        ESP_LOGE(TAG, "malloc fail mic2_reader");
        vTaskDelete(NULL);
    }

    while(1) {
        size_t bytes_read;
        i2s_read(I2S_NUM_1, buf, sizeof(int32_t)*I2S_READ_LEN, &bytes_read, portMAX_DELAY);

        float ste, zcr;
        int peak, idx;
        compute_ste_zcr_peak(buf, I2S_READ_LEN, &ste, &zcr, &peak, &idx);

        uint64_t now = esp_timer_get_time();

        xSemaphoreTake(feat_mutex, portMAX_DELAY);
        last_m2.ste = ste; last_m2.zcr = zcr; last_m2.peak = peak; last_m2.idx = idx; last_m2.ts_us = now;
        xSemaphoreGive(feat_mutex);

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    free(buf);
    vTaskDelete(NULL);
}

// ======= Detection task =======
static void detection_task(void *arg)
{
    ESP_LOGI(TAG, "Detection task waiting for calibration done");
    // Tunggu sampai kalibrasi mic1 dan mic2 selesai
    xEventGroupWaitBits(eg, EVT_MIC1_CALIB_DONE | EVT_MIC2_CALIB_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Detection task started");

    thr_t thr1, thr2;

    // Baca threshold kalibrasi sekali saja di awal
    load_thr("m1", &thr1);
    load_thr("m2", &thr2);

    while (1) {
        feat_t f1, f2;

        // Ambil data fitur dengan proteksi semaphore
        if (xSemaphoreTake(feat_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            f1 = last_m1;
            f2 = last_m2;
            xSemaphoreGive(feat_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to take feat_mutex");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Deteksi jika melewati threshold
        bool d1 = (f1.peak > thr1.peak_thr) && (f1.ste > thr1.ste_thr) && (f1.zcr < thr1.zcr_thr);
        bool d2 = (f2.peak > thr2.peak_thr) && (f2.ste > thr2.ste_thr) && (f2.zcr < thr2.zcr_thr);

        if (d1 || d2) {
            // Hitung waktu kedatangan suara (TOA) relatif
            uint64_t idx_off1 = ((uint64_t)f1.idx * 1000000ULL) / SAMPLE_RATE;
            uint64_t idx_off2 = ((uint64_t)f2.idx * 1000000ULL) / SAMPLE_RATE;

            uint64_t toa1 = (f1.ts_us >= idx_off1) ? (f1.ts_us - idx_off1) : f1.ts_us;
            uint64_t toa2 = (f2.ts_us >= idx_off2) ? (f2.ts_us - idx_off2) : f2.ts_us;

            if (d1 && (!d2 || toa1 <= toa2)) {
                ESP_LOGI(TAG, "[DETECT] Mic1 FIRST | TOA=%llu us | STE=%.2f PEAK=%d ZCR=%.4f",
                         toa1, f1.ste, f1.peak, f1.zcr);
            } else if (d2 && (!d1 || toa2 < toa1)) {
                ESP_LOGI(TAG, "[DETECT] Mic2 FIRST | TOA=%llu us | STE=%.2f PEAK=%d ZCR=%.4f",
                         toa2, f2.ste, f2.peak, f2.zcr);
            } else{ESP_LOGI(TAG, "TIDAK TERDETEKSI");}

            // Delay lebih panjang sebagai debounce supaya tidak double detect
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            // Delay singkat agar CPU tidak busy dan watchdog tidak terpicu
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ======= APP MAIN =======
void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    if (nvs_open("calib", NVS_READWRITE, &nvs_handle_app) != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed");
    }

    eg = xEventGroupCreate();
    feat_mutex = xSemaphoreCreateMutex();

    // Mulai kalibrasi mic1 dulu
    xTaskCreate(mic1_calib_task, "mic1_calib", 8192, NULL, 6, NULL);

    // Tunggu mic1 selesai
    xEventGroupWaitBits(eg, EVT_MIC1_CALIB_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5000));  // Delay 5 detik

    // Baru kalibrasi mic2
    xTaskCreate(mic2_calib_task, "mic2_calib", 8192, NULL, 6, NULL);

    // Tunggu mic2 selesai
    xEventGroupWaitBits(eg, EVT_MIC2_CALIB_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5000));  // Delay 5 detik

    // Setelah kalibrasi selesai, baru mulai pembaca mic1 dan mic2 secara paralel
    xTaskCreate(mic1_reader_task, "mic1_reader", 4096, NULL, 5, NULL);
    xTaskCreate(mic2_reader_task, "mic2_reader", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Delay 5 detik sebelum mulai task detection");
    vTaskDelay(pdMS_TO_TICKS(5000));  // Delay 5 detik

    // Mulai task deteksi setelah kalibrasi dan pembaca siap
    xTaskCreate(detection_task, "detection", 8192, NULL, 7, NULL);

    ESP_LOGI(TAG, "System started: calibration done, readers and detection started");
}

