#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

#define UART_NUM        UART_NUM_1
#define UART_TX_PIN     17    // TX ESP1 ke RX ESP2
#define UART_RX_PIN     16    // RX ESP1 dari TX ESP2
#define BUF_SIZE        1024

static const char *TAG = "ESP1_UART_Master";

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

// Task kirim timestamp tiap 100 ms
void send_time_task(void *arg) {
    while (1) {
        uint64_t t = esp_timer_get_time(); // waktu mikrodetik sejak ESP menyala
        int len = uart_write_bytes(UART_NUM, (const char*)&t, sizeof(t));
        if (len == sizeof(t)) {
            ESP_LOGI(TAG, "Kirim timestamp: %llu", t);
        } else {
            ESP_LOGW(TAG, "Gagal kirim data UART");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task terima ACK dari ESP2
void uart_receive_ack_task(void *arg) {
    uint8_t buf[64];
    int len;
    uint64_t timestamp_from_esp2 = 0;

    while (1) {
        len = uart_read_bytes(UART_NUM, buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
        if (len >= sizeof(uint64_t)) {
            // Ambil 8 byte pertama sebagai timestamp
            memcpy(&timestamp_from_esp2, buf, sizeof(uint64_t));
            ESP_LOGI(TAG, "Terima timestamp sinkron dari ESP2: %llu", timestamp_from_esp2);
        } else if (len > 0) {
            // Jika data kurang dari 8 byte, tampilkan sebagai string saja
            ESP_LOGI(TAG, "Terima data dari ESP2: '%.*s'", len, buf);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main1(void) {
    uart_init();
    xTaskCreate(send_time_task, "send_time_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_receive_ack_task, "uart_receive_ack_task", 2048, NULL, 5, NULL);
}
