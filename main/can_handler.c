#include "can_handler.h"
#include "board.h"
#include "relay.h"
#include "wifi_config.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "can";

static void send_status(void)
{
    twai_message_t msg = {
        .identifier = CAN_ID_STATUS,
        .data_length_code = 1,
        .data = { relay_get_states() },
    };
    twai_transmit(&msg, pdMS_TO_TICKS(10));
}

static void handle_ota_trigger(const uint8_t *data)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    // Compare last 3 bytes of MAC
    if (data[0] == mac[3] && data[1] == mac[4] && data[2] == mac[5]) {
        ESP_LOGI(TAG, "OTA trigger matched — OTA not yet implemented in ESP-IDF port");
        // TODO: implement esp_https_ota based OTA
    } else {
        ESP_LOGD(TAG, "OTA trigger ignored (MAC mismatch)");
    }
}

static void handle_rx_message(const twai_message_t *msg)
{
    if (msg->rtr) return;

    switch (msg->identifier) {
    case CAN_ID_OTA:
        if (msg->data_length_code >= 3) {
            handle_ota_trigger(msg->data);
        }
        break;

    case CAN_ID_WIFI_CONFIG:
        if (msg->data_length_code >= 1) {
            wifi_config_handle_can(msg->data, msg->data_length_code);
        }
        break;

    case CAN_ID_TOGGLE:
        if (msg->data_length_code >= 1) {
            uint8_t ch = msg->data[0];
            if (ch < NUM_RELAYS) {
                relay_toggle(ch);
            } else if (msg->data_length_code >= 2) {
                relay_set_all(msg->data[1] != 0);
            }
        }
        break;

    default:
        break;
    }
}

esp_err_t can_handler_init(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "CAN bus initialized (500 kbps, NO_ACK, TX=%d RX=%d)",
             CAN_TX_PIN, CAN_RX_PIN);
    return ESP_OK;
}

void can_handler_task(void *arg)
{
    int64_t last_status_us = 0;
    const int64_t status_interval_us = 33000; // ~30 Hz

    while (1) {
        // Try to receive a message with a short timeout
        twai_message_t rx_msg;
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(5)) == ESP_OK) {
            handle_rx_message(&rx_msg);
        }

        // Check wifi config timeout
        wifi_config_check_timeout();

        // Send status at ~30 Hz
        int64_t now = esp_timer_get_time();
        if (now - last_status_us >= status_interval_us) {
            last_status_us = now;
            send_status();
        }
    }
}
