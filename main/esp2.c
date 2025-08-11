#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

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
    uint8_t buf[64];
    size_t len = 0;
    uint64_t timestamp = 0;

    while (1) {
        len = uart_read_bytes(UART_NUM, buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Bytes diterima: %d", len);
        if (len > 0) {
            if (len >= sizeof(uint64_t)) {
                memcpy(&timestamp, buf, sizeof(uint64_t));
                ESP_LOGI(TAG, "Terima timestamp: %llu", timestamp);

                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(LED_GPIO, 0);

                const char ack_msg[] = "esp2 terima kok\n";
                int wlen = uart_write_bytes(UART_NUM, ack_msg, strlen(ack_msg));
                if (wlen == strlen(ack_msg)) {
                    ESP_LOGI(TAG, "Kirim ACK string ke ESP1 berhasil");
                } else {
                    ESP_LOGW(TAG, "Gagal kirim ACK string ke ESP1");
                }
            } else {
                ESP_LOGW(TAG, "Data diterima kurang dari 8 byte: %d", len);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



void app_main(void) {
    ESP_LOGI(TAG, "Memulai ESP2 UART Slave dengan LED indikator");
    uart_init();
    led_init();
    xTaskCreate(uart_receive_task, "uart_receive_task", 2048, NULL, 5, NULL);
}
