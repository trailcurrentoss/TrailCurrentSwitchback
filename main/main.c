#include "board.h"
#include "relay.h"
#include "can_handler.h"
#include "wifi_config.h"
#include "discovery.h"
#include "ota.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef SWITCHBACK_ADDRESS
#define SWITCHBACK_ADDRESS 0
#endif

static const char *TAG = "main";

static void init_digital_inputs(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    for (int i = 0; i < NUM_DIN; i++) {
        io_conf.pin_bit_mask = 1ULL << DIN_PINS[i];
        gpio_config(&io_conf);
    }

    ESP_LOGI(TAG, "8 digital inputs initialized");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== TrailCurrent Switchback ===");
    ESP_LOGI(TAG, "CAN-Controlled 8-Channel Relay Module");
    ESP_LOGI(TAG, "Board: Waveshare ESP32-S3-ETH-8DI-8RO-C");

    // Initialize NVS and load WiFi credentials
    ESP_ERROR_CHECK(wifi_config_init());

    char ssid[33] = {0};
    char password[64] = {0};
    if (wifi_config_load(ssid, sizeof(ssid), password, sizeof(password))) {
        ESP_LOGI(TAG, "WiFi credentials loaded from NVS");
    } else {
        ESP_LOGI(TAG, "No WiFi credentials — OTA disabled until provisioned via CAN");
    }

    // Initialize relay outputs via TCA9554 I2C expander
    ESP_ERROR_CHECK(relay_init());

    // Initialize digital inputs
    init_digital_inputs();

    // Initialize discovery and OTA (must be after wifi_config_init)
    discovery_init();
    ota_init();

    ESP_LOGI(TAG, "Switchback address: %d (Toggle CAN: 0x%02X, Status CAN: 0x%02X)",
             SWITCHBACK_ADDRESS,
             CAN_ID_TOGGLE_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_STATUS_BASE + SWITCHBACK_ADDRESS);

    // Initialize CAN bus
    ESP_ERROR_CHECK(can_handler_init());

    ESP_LOGI(TAG, "=== Setup Complete ===");

    // Run CAN handler on a dedicated task
    xTaskCreatePinnedToCore(can_handler_task, "can_task", 4096, NULL, 5, NULL, 1);
}
