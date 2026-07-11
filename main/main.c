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
#ifdef SWITCHBACK_VARIANT_GNSS
#include "gnss.h"
#endif

#ifndef SWITCHBACK_ADDRESS
#define SWITCHBACK_ADDRESS 0
#endif

static const char *TAG = "main";

#ifdef SWITCHBACK_VARIANT_GNSS
// Bridge from the gnss RX task straight into the CAN handler's snapshot.
// Fires once per NMEA sentence (RMC + GGA at 1 Hz) — no polling.
static void on_gnss_update(const gnss_data_t *data)
{
    can_handler_publish_gnss(data);
}
#endif  // SWITCHBACK_VARIANT_GNSS

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

#if defined(SWITCHBACK_VARIANT_AFTLINE)
    ESP_LOGI(TAG, "Switchback+Aftline addr=%d (Toggle 0x%02X, Status 0x%02X, TrailerStatus 0x3A)",
             SWITCHBACK_ADDRESS,
             CAN_ID_TOGGLE_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_STATUS_BASE + SWITCHBACK_ADDRESS);
#elif defined(SWITCHBACK_VARIANT_PICKET) && defined(SWITCHBACK_VARIANT_GNSS)
    ESP_LOGI(TAG, "Switchback+Picket+GNSS addr=%d (Toggle 0x%02X, Status 0x%02X, Input 0x%02X, GNSS 0x06-0x09)",
             SWITCHBACK_ADDRESS,
             CAN_ID_TOGGLE_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_STATUS_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_INPUT_BASE + SWITCHBACK_ADDRESS);
#elif defined(SWITCHBACK_VARIANT_PICKET)
    ESP_LOGI(TAG, "Switchback+Picket addr=%d (Toggle 0x%02X, Status 0x%02X, Input 0x%02X)",
             SWITCHBACK_ADDRESS,
             CAN_ID_TOGGLE_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_STATUS_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_INPUT_BASE + SWITCHBACK_ADDRESS);
#else
    ESP_LOGI(TAG, "Switchback (base) addr=%d (Toggle 0x%02X, Status 0x%02X, no DI broadcast)",
             SWITCHBACK_ADDRESS,
             CAN_ID_TOGGLE_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_STATUS_BASE + SWITCHBACK_ADDRESS);
#endif

    // Initialize CAN bus
    ESP_ERROR_CHECK(can_handler_init());

    // Run CAN handler on a dedicated task
    xTaskCreatePinnedToCore(can_handler_task, "can_task", 4096, NULL, 5, NULL, 1);

#ifdef SWITCHBACK_VARIANT_GNSS
    // DFRobot Gravity GNSS module in UART mode — 9600 baud NMEA. Header pin
    // wiring: D/T (module TX) → ESP RX; C/R (module RX) → ESP TX. UART1 via
    // the GPIO matrix keeps UART0/console untouched.
    ESP_ERROR_CHECK(gnss_init(SWITCHBACK_GNSS_UART_NUM,
                              SWITCHBACK_GNSS_UART_TX,
                              SWITCHBACK_GNSS_UART_RX,
                              SWITCHBACK_GNSS_BAUD,
                              on_gnss_update));
#endif

    ESP_LOGI(TAG, "=== Setup Complete ===");
}
