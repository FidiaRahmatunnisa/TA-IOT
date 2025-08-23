// ==== ESP1 MASTER (mic1-mic2-mic3, 4 modes on master) ====

#include <string.h>
#include <math.h>    
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"   
#include "freertos/event_groups.h"
#include "freertos/semphr.h"     
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2s.h"


// ---------------- CONFIG --------------
// #define N_EVENTS                100
#define SAMPLE_RATE             96000
#define I2S_READ_LEN            1024
#define CAL_SAMPLE_INTERVAL_MS  100
#define N_EVENTS        30
#define MODE_DELAY_MS   5000

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

static const char *TAG = "ESP1_MASTER";

// MAC ESP2 (Slave)
static uint8_t slave_mac[6] = {0x6c, 0xc8, 0x40, 0x33, 0xf5, 0xa0};

// === Struktur data dari ESP2 (HARUS SAMA di ESP2) ===
typedef struct __attribute__((packed)) {
    uint64_t timestamp;
    float    ste;
    float    zcr;
    int32_t  peak;
    int      mode;     
} mic3_data_t;

// === Mode Kalibrasi ===
typedef enum {
    MODE_DETECTION = 0, //dihilangkan karena akan dilakukan bersama
    MODE_CAL_NOISE = 1,
    MODE_CAL_TARGET = 2,
    MODE_CAL_RESULT = 3
} mic_mode_t;

// === variabel untuk mean-std ===
typedef struct {
    double mean;
    double std;
} stat_t;

// === Globals ===
static nvs_handle_t nvs_handle_app;
static uint64_t start_time_us;
static mic_mode_t current_mode = MODE_CAL_NOISE;
static EventGroupHandle_t eg;
static SemaphoreHandle_t feat_mutex;
static nvs_handle_t nvs_handle_app1 = 0;

// Queue 1-slot untuk sampel mic3 (pakai overwrite agar selalu dapat yang terbaru)
static QueueHandle_t mic3_queue = NULL;

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

// === variabel untuk ste-zcr-peak ===
// typedef struct {
//     double ste_thr;
//     double peak_thr;
//     double zcr_thr;
// } threshold_t;

// Kalibrasi buffer dan counter mic1
static float m1_ste[N_EVENTS], m1_zcr[N_EVENTS], m1_peak[N_EVENTS];
static int m1_col = 0;
static bool m1_done = false;

// Kalibrasi buffer dan counter mic2
static float m2_ste[N_EVENTS], m2_zcr[N_EVENTS], m2_peak[N_EVENTS];
static int m2_col = 0;
static bool m2_done = false;

// Latest features dari mic1 dan mic2
static feat_t last_m1 = {0}, last_m2 = {0},  last_m3 = {0};
static bool calib_done = false;

// ==== WiFi init ====
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi STA started");
}

// ======= DUAL I2S init =======
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

// ===== task STE-ZCR-PEAK
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


// ==== mean/std ====
// ini yang dari mic1mic2
static void compute_mean_std(const float *arr, int n, double *mean_out, double *std_out)
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

// ==== LOAD & SAVE
//Fungsi dasar NVS untuk double
static esp_err_t nvs_save_double(const char *key, double v)
{
    uint64_t raw;
    memcpy(&raw, &v, sizeof(raw));
    return nvs_set_u64(nvs_handle_app, key, raw);
}

static double nvs_load_double(const char *key)
{
    uint64_t raw = 0;
    if (nvs_get_u64(nvs_handle_app, key, &raw) != ESP_OK) return 0.0;
    double v;
    memcpy(&v, &raw, sizeof(raw));
    return v;
}

//Fungsi gabungan save/load stat
// prefix: "mic1", "mic2", "mic3"
static void save_stat(const char *prefix, stat_t s)
{
    char key_mean[64], key_std[64];
    snprintf(key_mean, sizeof(key_mean), "%s_mean", prefix);
    snprintf(key_std,  sizeof(key_std),  "%s_std", prefix);
    nvs_save_double(key_mean, s.mean);
    nvs_save_double(key_std,  s.std);
    nvs_commit(nvs_handle_app);
}

static void load_stat(const char *prefix, stat_t *s)
{
    char key_mean[64], key_std[64];
    snprintf(key_mean, sizeof(key_mean), "%s_mean", prefix);
    snprintf(key_std,  sizeof(key_std),  "%s_std", prefix);
    s->mean = nvs_load_double(key_mean);
    s->std  = nvs_load_double(key_std);
}

//Fungsi gabungan save/load threshold
static void save_thr(const char *prefix, thr_t t)
{
    char key_ste[64], key_peak[64], key_zcr[64];
    snprintf(key_ste,  sizeof(key_ste),  "%s_thr_ste", prefix);
    snprintf(key_peak, sizeof(key_peak), "%s_thr_peak", prefix);
    snprintf(key_zcr,  sizeof(key_zcr),  "%s_thr_zcr", prefix);

    nvs_save_double(key_ste,  t.ste_thr);
    nvs_save_double(key_peak, t.peak_thr);
    nvs_save_double(key_zcr,  t.zcr_thr);
    nvs_commit(nvs_handle_app);
}

static void load_thr(const char *prefix, thr_t *t)
{
    char key_ste[64], key_peak[64], key_zcr[64];
    snprintf(key_ste,  sizeof(key_ste),  "%s_thr_ste", prefix);
    snprintf(key_peak, sizeof(key_peak), "%s_thr_peak", prefix);
    snprintf(key_zcr,  sizeof(key_zcr),  "%s_thr_zcr", prefix);

    t->ste_thr  = nvs_load_double(key_ste);
    t->peak_thr = nvs_load_double(key_peak);
    t->zcr_thr  = nvs_load_double(key_zcr);
}

// ==== ESPNOW RECV CALLBACK ====
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    uint64_t now_since_start = esp_timer_get_time() - start_time_us;

    if (len == sizeof(uint64_t)) {
        // Paket sinkronisasi waktu
        uint64_t ts_received;
        memcpy(&ts_received, data, sizeof(ts_received));
        if (now_since_start < 20000000ULL) {
            ESP_LOGI(TAG, "[SYNC] Terima timestamp dari %s: %llu", macStr, (unsigned long long)ts_received);
        }
        return;
    }

    if (len == sizeof(mic3_data_t)) {
        mic3_data_t pkt;
        memcpy(&pkt, data, sizeof(pkt));

        // Simpan sampel terbaru ke queue (overwrite agar tidak pernah penuh)
        if (mic3_queue) {
            // Queue length = 1 → boleh gunakan xQueueOverwrite
            xQueueOverwrite(mic3_queue, &pkt);
        }

        // (Opsional) log singkat 20 detik pertama
        if (now_since_start < 20000000ULL) {
            ESP_LOGI(TAG, "[SYNC] Terima timestamp dari %s: %llu", macStr,(unsigned long long)pkt.timestamp);
        }
        return;
    }

    ESP_LOGW(TAG, "Paket tidak dikenali dari %s: len=%d", macStr, len);
}

// ==== ESPNOW INIT ====
static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    esp_now_peer_info_t peerInfo = {0};
    memcpy(peerInfo.peer_addr, slave_mac, 6);
    peerInfo.channel = 1;   // samakan channel
    peerInfo.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));

    ESP_LOGI(TAG, "Peer ESP2 didaftarkan");
}

// ======= Mic1 MODE / Kalibrasi task =======
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
    compute_mean_std(m1_ste, N_EVENTS, &mean_ste, &std_ste);
    compute_mean_std(m1_zcr, N_EVENTS, &mean_zcr, &std_zcr);
    compute_mean_std(m1_peak, N_EVENTS, &mean_peak, &std_peak);
    // save_stat("m1_noise_ste", mean_ste, std_ste);
    // save_stat("m1_noise_zcr", mean_zcr, std_zcr);
    // save_stat("m1_noise_peak", mean_peak, std_peak);
    // Simpan statistik noise
    stat_t noise_ste = {mean_ste, std_ste};
    stat_t noise_zcr = {mean_zcr, std_zcr};
    stat_t noise_peak = {mean_peak, std_peak};
    save_stat("mic1_noise_ste", noise_ste);
    save_stat("mic1_noise_zcr", noise_zcr);
    save_stat("mic1_noise_peak", noise_peak);
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
    compute_mean_std(m1_ste, N_EVENTS, &mean_ste, &std_ste);
    compute_mean_std(m1_zcr, N_EVENTS, &mean_zcr, &std_zcr);
    compute_mean_std(m1_peak, N_EVENTS, &mean_peak, &std_peak);
    // save_stat("m1_target_ste", mean_ste, std_ste);
    // save_stat("m1_target_zcr", mean_zcr, std_zcr);
    // save_stat("m1_target_peak", mean_peak, std_peak);
    // Simpan statistik target
    stat_t target_ste = {mean_ste, std_ste};
    stat_t target_zcr = {mean_zcr, std_zcr};
    stat_t target_peak = {mean_peak, std_peak};
    save_stat("mic1_target_ste", target_ste);
    save_stat("mic1_target_zcr", target_zcr);
    save_stat("mic1_target_peak", target_peak);

   // Step : 3 Hitung threshold mic1 dan simpan dengan std deviasi
    thr_t thr1;
    thr1.ste_thr  = mean_ste + 1.0 * std_ste;
    thr1.peak_thr = mean_peak + 1.0 * std_peak;
    thr1.zcr_thr  = fmax(0.0, mean_zcr - 0.5 * std_zcr);
    save_thr("mic1", thr1);
    ESP_LOGI(TAG, "MIC1 thresholds: STE=%.2f, PEAK=%.2f, ZCR=%.2f", thr1.ste_thr, thr1.peak_thr, thr1.zcr_thr);

    ESP_LOGI(TAG, "mic1 threshold saved");

    free(buf);
    m1_done = true;
    xEventGroupSetBits(eg, EVT_MIC1_CALIB_DONE);

    vTaskDelete(NULL);
}

// ======= Mic2 MODE / Kalibrasi task =======
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
    compute_mean_std(m2_ste, N_EVENTS, &mean_ste, &std_ste);
    compute_mean_std(m2_zcr, N_EVENTS, &mean_zcr, &std_zcr);
    compute_mean_std(m2_peak, N_EVENTS, &mean_peak, &std_peak);
    // save_stat("m2_noise_ste", mean_ste, std_ste);
    // save_stat("m2_noise_zcr", mean_zcr, std_zcr);
    // save_stat("m2_noise_peak", mean_peak, std_peak);
    // Simpan statistik noise
    stat_t noise_ste = {mean_ste, std_ste};
    stat_t noise_zcr = {mean_zcr, std_zcr};
    stat_t noise_peak = {mean_peak, std_peak};
    save_stat("mic2_noise_ste", noise_ste);
    save_stat("mic2_noise_zcr", noise_zcr);
    save_stat("mic2_noise_peak", noise_peak);
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
    compute_mean_std(m2_ste, N_EVENTS, &mean_ste, &std_ste);
    compute_mean_std(m2_zcr, N_EVENTS, &mean_zcr, &std_zcr);
    compute_mean_std(m2_peak, N_EVENTS, &mean_peak, &std_peak);
    // save_stat("m2_target_ste", mean_ste, std_ste);
    // save_stat("m2_target_zcr", mean_zcr, std_zcr);
    // save_stat("m2_target_peak", mean_peak, std_peak);
    // Simpan statistik target
    stat_t target_ste = {mean_ste, std_ste};
    stat_t target_zcr = {mean_zcr, std_zcr};
    stat_t target_peak = {mean_peak, std_peak};
    save_stat("mic2_target_ste", target_ste);
    save_stat("mic2_target_zcr", target_zcr);
    save_stat("mic2_target_peak", target_peak);

    // Step 3 : Hitung threshold mic2 dan simpan dengan std deviasi
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

// ==== MIC3 MODE TASK (kalibrasi/deteksi mic3 di ESP1) ====
static void mode_task(void *arg)
{
    float ste_list[N_EVENTS], zcr_list[N_EVENTS], peak_list[N_EVENTS];
    int collected = 0;
    thr_t thr;
    stat_t noise_ste, noise_peak, noise_zcr;
    stat_t target_ste, target_peak, target_zcr;

    // Pastikan kita punya queue
    configASSERT(mic3_queue != NULL);

    while (1) {
        mic3_data_t sample;

        // Tunggu sampel mic3 dari ESP2
        if (xQueueReceive(mic3_queue, &sample, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        float ste  = sample.ste;
        float zcr  = sample.zcr;
        int   peak = sample.peak;

        if (current_mode == MODE_CAL_NOISE || current_mode == MODE_CAL_TARGET) {
            if (collected < N_EVENTS) {
                ste_list[collected]  = ste;
                zcr_list[collected]  = zcr;
                peak_list[collected] = (float)peak;
                collected++;

                ESP_LOGI(TAG, "CAL[%s] %d/%d t_ESP2=%llu  STE=%.2f  ZCR=%.4f  PEAK=%d",
                         (current_mode == MODE_CAL_NOISE ? "NOISE" : "TARGET"),
                         collected, N_EVENTS, (unsigned long long)sample.timestamp, ste, zcr, peak);
            } else {
                stat_t s_ste, s_peak, s_zcr;
                compute_mean_std(ste_list,  N_EVENTS, &s_ste.mean,  &s_ste.std);
                compute_mean_std(peak_list, N_EVENTS, &s_peak.mean, &s_peak.std);
                compute_mean_std(zcr_list,  N_EVENTS, &s_zcr.mean,  &s_zcr.std);

                // ✅ Debug print sebelum disimpan
                ESP_LOGI(TAG, "DEBUG: s_ste.mean=%.2f s_peak.mean=%.2f s_zcr.mean=%.4f", s_ste.mean, s_peak.mean, s_zcr.mean);

                if (current_mode == MODE_CAL_NOISE) {
                    // save_stat("noise_ste_m",  "noise_ste_s",  s_ste);
                    // save_stat("noise_peak_m", "noise_peak_s", s_peak);
                    // save_stat("noise_zcr_m",  "noise_zcr_s",  s_zcr);
                    save_stat("mic3_noise_ste", s_ste);
                    save_stat("mic3_noise_peak", s_peak);
                    save_stat("mic3_noise_zcr", s_zcr);
                    ESP_LOGI(TAG, "Noise stats saved");
                } else { // MODE_CAL_TARGET
                    // save_stat("target_ste_m",  "target_ste_s",  s_ste);
                    // save_stat("target_peak_m", "target_peak_s", s_peak);
                    // save_stat("target_zcr_m",  "target_zcr_s",  s_zcr);
                    save_stat("mic3_target_ste", s_ste);
                    save_stat("mic3_target_peak", s_peak);
                    save_stat("mic3_target_zcr", s_zcr);
                    ESP_LOGI(TAG, "Target stats saved");
                }

                collected = 0;
                vTaskDelay(pdMS_TO_TICKS(MODE_DELAY_MS));
                current_mode++;
                ESP_LOGI(TAG, "Switched mode -> %d", current_mode);
            }
        }
        else if (current_mode == MODE_CAL_RESULT) {
            // load_stat("noise_ste_m",  "noise_ste_s",  &noise_ste);
            // load_stat("noise_peak_m", "noise_peak_s", &noise_peak);
            // load_stat("noise_zcr_m",  "noise_zcr_s",  &noise_zcr);
            // load_stat("target_ste_m",  "target_ste_s",  &target_ste);
            // load_stat("target_peak_m", "target_peak_s", &target_peak);
            // load_stat("target_zcr_m",  "target_zcr_s",  &target_zcr);
            ESP_LOGI(TAG, "DEBUG: about to load mic3_target_ste");
            // beri waktu NVS commit stabil
            vTaskDelay(pdMS_TO_TICKS(500)); // 50–100 ms biasanya cukup

            load_stat("mic3_noise_ste",  &noise_ste);
            load_stat("mic3_noise_peak", &noise_peak);
            load_stat("mic3_noise_zcr",  &noise_zcr);

            load_stat("mic3_target_ste",  &target_ste);
            load_stat("mic3_target_peak", &target_peak);
            load_stat("mic3_target_zcr",  &target_zcr);

            // ✅ Debug print setelah load
                ESP_LOGI(TAG, "DEBUG loaded noise_ste.mean=%.2f, noise_peak.mean=%.2f, noise_zcr.mean=%.4f", 
                        noise_ste.mean, noise_peak.mean, noise_zcr.mean);
                ESP_LOGI(TAG, "DEBUG loaded target_ste.mean=%.2f, target_peak.mean=%.2f, target_zcr.mean=%.4f", 
                        target_ste.mean, target_peak.mean, target_zcr.mean);

            thr.ste_thr  = noise_ste.mean  + 0.5 * (target_ste.mean  - noise_ste.mean);
            thr.peak_thr = noise_peak.mean + 0.5 * (target_peak.mean - noise_peak.mean);
            thr.zcr_thr  = fmax(0.0, noise_zcr.mean - 0.5 * noise_zcr.std);

            // save_threshold(thr);
            save_thr("mic3", thr);
            ESP_LOGI(TAG, "t_ESP2=%llu | Threshold saved: STE=%.2f  PEAK=%.2f  ZCR=%.4f",
                     (unsigned long long)sample.timestamp, thr.ste_thr, thr.peak_thr, thr.zcr_thr);

            vTaskDelay(pdMS_TO_TICKS(MODE_DELAY_MS));
            current_mode = MODE_DETECTION;
            ESP_LOGI(TAG, "Switched mode -> MODE_DETECTION");
            // ✅ Kalibrasi Mic3 selesai, set flag
            calib_done = true;
        }
        // ini akan dimasukan ke task yang sama dengan mic1 dan mic2
        else if (current_mode == MODE_DETECTION) {
            // load_threshold(&thr);
            // if ((peak > thr.peak_thr) && (ste > thr.ste_thr) && (zcr < thr.zcr_thr)) {
            //     ESP_LOGI(TAG, "t_ESP2=%llu | Event DETECTED: STE=%.2f  PEAK=%d  ZCR=%.4f", (unsigned long long)sample.timestamp, ste, peak, zcr);
            // }
            xQueueOverwrite(mic3_queue, &sample);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
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

    thr_t thr1, thr2, thr3;

    // Baca threshold kalibrasi sekali saja di awal
    load_thr("m1", &thr1);
    load_thr("m2", &thr2);
    load_thr("m3", &thr3);

    while (1) {
        feat_t f1, f2, f3;

        // Ambil data fitur dengan proteksi semaphore
        if (xSemaphoreTake(feat_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            f1 = last_m1;
            f2 = last_m2;
            f3 = last_m3;
            xSemaphoreGive(feat_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to take feat_mutex");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Deteksi jika melewati threshold
        bool d1 = (f1.peak > thr1.peak_thr) && (f1.ste > thr1.ste_thr) && (f1.zcr < thr1.zcr_thr);
        bool d2 = (f2.peak > thr2.peak_thr) && (f2.ste > thr2.ste_thr) && (f2.zcr < thr2.zcr_thr);
        bool d3 = (f3.peak > thr3.peak_thr) && (f3.ste > thr3.ste_thr) && (f3.zcr < thr3.zcr_thr);

        if (d1 || d2) {
            // Hitung waktu kedatangan suara (TOA) relatif
            uint64_t idx_off1 = ((uint64_t)f1.idx * 1000000ULL) / SAMPLE_RATE;
            uint64_t idx_off2 = ((uint64_t)f2.idx * 1000000ULL) / SAMPLE_RATE;
            uint64_t idx_off3 = ((uint64_t)f3.idx * 1000000ULL) / SAMPLE_RATE;

            uint64_t toa1 = (f1.ts_us >= idx_off1) ? (f1.ts_us - idx_off1) : f1.ts_us;
            uint64_t toa2 = (f2.ts_us >= idx_off2) ? (f2.ts_us - idx_off2) : f2.ts_us;
            uint64_t toa3 = (f3.ts_us >= idx_off3) ? (f3.ts_us - idx_off3) : f3.ts_us;

           // Tentukan siapa yang pertama
            if (d1 && (!d2 || toa1 <= toa2) && (!d3 || toa1 <= toa3)) {
                ESP_LOGI(TAG, "[DETECT] Mic1 FIRST | TOA=%llu us | STE=%.2f PEAK=%d ZCR=%.4f",
                        toa1, f1.ste, f1.peak, f1.zcr);
            } else if (d2 && (!d1 || toa2 < toa1) && (!d3 || toa2 <= toa3)) {
                ESP_LOGI(TAG, "[DETECT] Mic2 FIRST | TOA=%llu us | STE=%.2f PEAK=%d ZCR=%.4f",
                        toa2, f2.ste, f2.peak, f2.zcr);
            } else if (d3 && (!d1 || toa3 < toa1) && (!d2 || toa3 < toa2)) {
                ESP_LOGI(TAG, "[DETECT] Mic3 FIRST | TOA=%llu us | STE=%.2f PEAK=%d ZCR=%.4f",
                        toa3, f3.ste, f3.peak, f3.zcr);
            } else {
                ESP_LOGI(TAG, "TIDAK TERDETEKSI");
            }

            // Delay lebih panjang sebagai debounce supaya tidak double detect
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            // Delay singkat agar CPU tidak busy dan watchdog tidak terpicu
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ==== APP MAIN ====
void app_main(void)
{
    ESP_LOGI(TAG, "Mulai ESP1 MASTER");

    // 1️⃣ Inisialisasi NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(nvs_open("calib", NVS_READWRITE, &nvs_handle_app));

    // 2️⃣ Inisialisasi EventGroup & Mutex
    eg = xEventGroupCreate();
    feat_mutex = xSemaphoreCreateMutex();

    // 3️⃣ Inisialisasi WiFi & ESPNOW
    wifi_init();
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    espnow_init();

    // 4️⃣ Inisialisasi Queue mic3
    mic3_queue = xQueueCreate(1, sizeof(mic3_data_t));
    configASSERT(mic3_queue != NULL);

    // 5️⃣ Sinkronisasi dengan ESP2 (mic3)
    start_time_us = esp_timer_get_time();
    const char *sync_req = "SYNC_START";
    esp_err_t send_result = esp_now_send(slave_mac, (uint8_t *)sync_req, strlen(sync_req));
    if (send_result == ESP_OK) {
        ESP_LOGI(TAG, "SYNC request dikirim ke ESP2");
    } else {
        ESP_LOGW(TAG, "Gagal kirim SYNC request (%d)", send_result);
    }

    // Tunggu 20 detik sampai ESP2 siap
    vTaskDelay(pdMS_TO_TICKS(20000));
    ESP_LOGI(TAG, "Sinkronisasi selesai, ESP2 siap");

    // 6️⃣ Mulai kalibrasi mic1
    xTaskCreate(mic1_calib_task, "mic1_calib", 8192, NULL, 6, NULL);
    xEventGroupWaitBits(eg, EVT_MIC1_CALIB_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5000));

    // 7️⃣ Mulai kalibrasi mic2
    xTaskCreate(mic2_calib_task, "mic2_calib", 8192, NULL, 6, NULL);
    xEventGroupWaitBits(eg, EVT_MIC2_CALIB_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5000));

    // 9️⃣ Mulai mode_task untuk mic3 (kalibrasi & mode deteksi)
    // Mode_task menunggu mic1 & mic2 kalibrasi sebelum hitung threshold gabungan
    ESP_LOGI(TAG, "Mulai mode_task mic3");
    xTaskCreatePinnedToCore(mode_task, "mode_task", 12288, NULL, 6, NULL, 1);

    // 8️⃣ Mulai reader mic1 & mic2
    xTaskCreate(mic1_reader_task, "mic1_reader", 4096, NULL, 5, NULL);
    xTaskCreate(mic2_reader_task, "mic2_reader", 4096, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Tunggu sampai kalibrasi selesai
    while(!calib_done) {
        vTaskDelay(pdMS_TO_TICKS(10)); // tunggu 10 ms
    }

    // 🔟 Mulai task deteksi gabungan (mic1, mic2, mic3)
    xTaskCreate(detection_task, "detection", 8192, NULL, 7, NULL);

    // 11️⃣ Idle loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

