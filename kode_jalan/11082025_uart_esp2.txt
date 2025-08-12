#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include "esp_timer.h"

#define UART_NUM        UART_NUM_1
#define UART_TX_PIN     17    // TX ESP2 ke RX ESP1
#define UART_RX_PIN     16    // RX ESP2 dari TX ESP1
#define BUF_SIZE        1024

#define LED_GPIO        2     // Pin LED indikator di ESP2

static const char *TAG = "ESP2_UART_Slave";

void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
}

void led_init() {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0); // LED mati awalnya
}

void uart_receive_task(void *arg) {
    uint8_t buf[BUF_SIZE];
    size_t len = 0;

    static int offset_calculated = 0;
    static int64_t offset = 0;  // offset = ts_esp1 - ts_esp2

    while (1) {
        ESP_LOGI(TAG, "Menunggu data UART...");
        len = uart_read_bytes(UART_NUM, buf, sizeof(uint64_t), 100 / portTICK_PERIOD_MS);
        if (len == sizeof(uint64_t)) {
            uint64_t ts_esp1_received;
            memcpy(&ts_esp1_received, buf, sizeof(uint64_t));
            ESP_LOGI(TAG, "Terima timestamp referensi ESP1: %llu", ts_esp1_received);

            if (!offset_calculated) {
                uint64_t ts_esp2_now = esp_timer_get_time();
                offset = (int64_t)ts_esp1_received - (int64_t)ts_esp2_now;
                offset_calculated = 1;
                ESP_LOGI(TAG, "Offset waktu dihitung: %lld us", offset);
            }

            // LED indikator menyala sebentar
            gpio_set_level(LED_GPIO, 1);

            // Simulasi waktu deteksi suara lokal di ESP2 sekarang
            uint64_t ts_esp2_local = esp_timer_get_time();

            // Sesuaikan waktu lokal ESP2 ke waktu ESP1
            uint64_t ts_esp2_adjusted = (uint64_t)((int64_t)ts_esp2_local + offset);

            ESP_LOGI(TAG, "Timestamp ESP2 yang sudah disesuaikan: %llu", ts_esp2_adjusted);

            // Kirim balik timestamp ESP2 yang sudah disesuaikan ke ESP1
            int wlen = uart_write_bytes(UART_NUM, (const char*)&ts_esp2_adjusted, sizeof(ts_esp2_adjusted));
            if (wlen == sizeof(ts_esp2_adjusted)) {
                ESP_LOGI(TAG, "Kirim timestamp olahan ke ESP1 berhasil");
            } else {
                ESP_LOGW(TAG, "Gagal kirim timestamp olahan ke ESP1");
            }

            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_GPIO, 0);
        } else if (len > 0) {
            ESP_LOGW(TAG, "Data diterima kurang dari 8 byte: %d", len);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Memulai ESP2 UART Slave dengan LED indikator");
    uart_init();
    led_init();
    xTaskCreate(uart_receive_task, "uart_receive_task", 4096, NULL, 5, NULL);
}
