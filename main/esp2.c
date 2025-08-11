#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_GPIO 2  // Pin LED di ESP2 (sesuaikan jika perlu)

static const char *TAG = "LED_TEST";

void app_main(void) {
    ESP_LOGI(TAG, "Mulai tes LED blinking");

    // Inisialisasi pin LED sebagai output
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO, 1);  // LED nyala
        vTaskDelay(pdMS_TO_TICKS(100)); // Tunggu 100 ms

        gpio_set_level(LED_GPIO, 0);  // LED mati
        vTaskDelay(pdMS_TO_TICKS(100)); // Tunggu 100 ms
    }
}
