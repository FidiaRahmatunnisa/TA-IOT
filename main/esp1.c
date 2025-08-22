// ==== ESP1 MASTER (mic3-only, 4 modes on master) ====

#include <string.h>
#include <math.h>                  // sqrt, fmax
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"        // queue for mic3 samples
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"

#define N_EVENTS        100
#define MODE_DELAY_MS   5000

static const char *TAG = "ESP1_MASTER";

// MAC ESP2 (Slave)
static uint8_t slave_mac[6] = {0x6c, 0xc8, 0x40, 0x33, 0xf5, 0xa0};

// === Struktur data dari ESP2 (HARUS SAMA di ESP2) ===
typedef struct __attribute__((packed)) {
    uint64_t timestamp;
    float    ste;
    float    zcr;
    int32_t  peak;
    int      mode;     // jika ESP2 mengirimkan mode-nya, boleh diabaikan di ESP1
} mic3_data_t;

typedef enum {
    MODE_DETECTION = 0,
    MODE_CAL_NOISE = 1,
    MODE_CAL_TARGET = 2,
    MODE_CAL_RESULT = 3
} mic_mode_t;

typedef struct {
    double mean;
    double std;
} stat_t;

typedef struct {
    double ste_thr;
    double peak_thr;
    double zcr_thr;
} threshold_t;

// === Globals ===
static nvs_handle_t nvs_handle_app;
static uint64_t start_time_us;
static mic_mode_t current_mode = MODE_CAL_NOISE;

// Queue 1-slot untuk sampel mic3 (pakai overwrite agar selalu dapat yang terbaru)
static QueueHandle_t mic3_queue = NULL;

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

// ==== Utils: mean/std & NVS helpers ====
static void compute_mean_std(const float *arr, int n, double *mean_out, double *std_out) {
    double sum = 0;
    for (int i = 0; i < n; i++) sum += arr[i];
    double mean = sum / n;
    double s = 0;
    for (int i = 0; i < n; i++) s += (arr[i] - mean) * (arr[i] - mean);
    *mean_out = mean;
    *std_out  = (n > 1) ? sqrt(s / (n - 1)) : 0.0;
}

static void save_double(const char *key, double value) {
    uint64_t raw;
    memcpy(&raw, &value, sizeof(raw));  // copy bit pattern
    ESP_ERROR_CHECK(nvs_set_u64(nvs_handle_app, key, raw));
    ESP_ERROR_CHECK(nvs_commit(nvs_handle_app));
}

static double load_double(const char *key) {
    uint64_t raw = 0;
    esp_err_t err = nvs_get_u64(nvs_handle_app, key, &raw);
    if (err != ESP_OK) raw = 0;
    double value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static void save_stat(const char *key_mean, const char *key_std, stat_t val) {
    save_double(key_mean, val.mean);
    save_double(key_std,  val.std);
}

static void load_stat(const char *key_mean, const char *key_std, stat_t *val) {
    val->mean = load_double(key_mean);
    val->std  = load_double(key_std);
}

static void save_threshold(threshold_t thr) {
    save_double("thr_ste",  thr.ste_thr);
    save_double("thr_peak", thr.peak_thr);
    save_double("thr_zcr",  thr.zcr_thr);
}

static void load_threshold(threshold_t *thr) {
    thr->ste_thr  = load_double("thr_ste");
    thr->peak_thr = load_double("thr_peak");
    thr->zcr_thr  = load_double("thr_zcr");
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

// ==== MODE TASK (kalibrasi/deteksi mic3 di ESP1) ====
static void mode_task(void *arg)
{
    float ste_list[N_EVENTS], zcr_list[N_EVENTS], peak_list[N_EVENTS];
    int collected = 0;
    threshold_t thr;
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

                if (current_mode == MODE_CAL_NOISE) {
                    save_stat("noise_ste_m",  "noise_ste_s",  s_ste);
                    save_stat("noise_peak_m", "noise_peak_s", s_peak);
                    save_stat("noise_zcr_m",  "noise_zcr_s",  s_zcr);
                    ESP_LOGI(TAG, "Noise stats saved");
                } else { // MODE_CAL_TARGET
                    save_stat("target_ste_m",  "target_ste_s",  s_ste);
                    save_stat("target_peak_m", "target_peak_s", s_peak);
                    save_stat("target_zcr_m",  "target_zcr_s",  s_zcr);
                    ESP_LOGI(TAG, "Target stats saved");
                }

                collected = 0;
                vTaskDelay(pdMS_TO_TICKS(MODE_DELAY_MS));
                current_mode++;
                ESP_LOGI(TAG, "Switched mode -> %d", current_mode);
            }
        }
        else if (current_mode == MODE_CAL_RESULT) {
            load_stat("noise_ste_m",  "noise_ste_s",  &noise_ste);
            load_stat("noise_peak_m", "noise_peak_s", &noise_peak);
            load_stat("noise_zcr_m",  "noise_zcr_s",  &noise_zcr);
            load_stat("target_ste_m",  "target_ste_s",  &target_ste);
            load_stat("target_peak_m", "target_peak_s", &target_peak);
            load_stat("target_zcr_m",  "target_zcr_s",  &target_zcr);

            thr.ste_thr  = noise_ste.mean  + 0.5 * (target_ste.mean  - noise_ste.mean);
            thr.peak_thr = noise_peak.mean + 0.5 * (target_peak.mean - noise_peak.mean);
            thr.zcr_thr  = fmax(0.0, noise_zcr.mean - 0.5 * noise_zcr.std);

            save_threshold(thr);
            ESP_LOGI(TAG, "t_ESP2=%llu | Threshold saved: STE=%.2f  PEAK=%.2f  ZCR=%.4f",
                     (unsigned long long)sample.timestamp, thr.ste_thr, thr.peak_thr, thr.zcr_thr);

            vTaskDelay(pdMS_TO_TICKS(MODE_DELAY_MS));
            current_mode = MODE_DETECTION;
            ESP_LOGI(TAG, "Switched mode -> MODE_DETECTION");
        }
        else if (current_mode == MODE_DETECTION) {
            load_threshold(&thr);
            if ((peak > thr.peak_thr) && (ste > thr.ste_thr) && (zcr < thr.zcr_thr)) {
                ESP_LOGI(TAG, "t_ESP2=%llu | Event DETECTED: STE=%.2f  PEAK=%d  ZCR=%.4f", (unsigned long long)sample.timestamp, ste, peak, zcr);
            }
        }
    }
}

// ==== APP MAIN ====
void app_main(void)
{
    ESP_LOGI(TAG, "Mulai ESP1 MASTER");

    // NVS init (harus sebelum nvs_open)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle_app));

    // WiFi + ESPNOW
    wifi_init();
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE)); // samakan channel dengan ESP2
    espnow_init();

    // Queue untuk sampel mic3 (1 slot → overwrite newest)
    mic3_queue = xQueueCreate(1, sizeof(mic3_data_t));
    configASSERT(mic3_queue != NULL);

    // Mulai waktu start
    start_time_us = esp_timer_get_time();

    // Kirim request sinkronisasi ke ESP2
    const char *sync_req = "SYNC_START";
    esp_err_t send_result = esp_now_send(slave_mac, (uint8_t *)sync_req, strlen(sync_req));
    if (send_result == ESP_OK) {
        ESP_LOGI(TAG, "SYNC request dikirim ke ESP2");
    } else {
        ESP_LOGW(TAG, "Gagal kirim SYNC request (%d)", send_result);
    }

    // Setelah 20 detik, baru start mode_task
    vTaskDelay(pdMS_TO_TICKS(20000));
    ESP_LOGI(TAG, "Sinkronisasi selesai, mulai MODE1 (CAL_NOISE)");
    xTaskCreatePinnedToCore(mode_task, "mode_task", 12288, NULL, 6, NULL, 1);

    // Idle
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
