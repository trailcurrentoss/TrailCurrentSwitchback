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

#define STATUS_TX_INTERVAL_MS  33    // ~30 Hz
#define TX_PROBE_INTERVAL_MS   2000  // slow probe when no peers detected

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

esp_err_t can_handler_init(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
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

    ESP_LOGI(TAG, "CAN bus initialized (500 kbps, NORMAL, TX=%d RX=%d)",
             CAN_TX_PIN, CAN_RX_PIN);
    return ESP_OK;
}

void can_handler_task(void *arg)
{
    uint32_t alerts = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS |
                      TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL |
                      TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                      TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_TX_FAILED |
                      TWAI_ALERT_TX_SUCCESS;
    twai_reconfigure_alerts(alerts, NULL);

    typedef enum { TX_ACTIVE, TX_PROBING } tx_state_t;
    bool bus_off = false;
    tx_state_t tx_state = TX_ACTIVE;
    int tx_fail_count = 0;
    const int TX_FAIL_THRESHOLD = 3;
    int64_t last_tx_us = 0;
    const int64_t tx_period_us = STATUS_TX_INTERVAL_MS * 1000LL;
    const int64_t tx_probe_period_us = TX_PROBE_INTERVAL_MS * 1000LL;

    while (1) {
        uint32_t triggered;
        twai_read_alerts(&triggered, pdMS_TO_TICKS(10));

        // --- Bus error handling ---
        if (triggered & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "TWAI bus-off, initiating recovery");
            bus_off = true;
            twai_initiate_recovery();
            continue;
        }
        if (triggered & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(TAG, "TWAI bus recovered, restarting");
            twai_start();
            bus_off = false;
            tx_fail_count = 0;
            tx_state = TX_PROBING;
        }
        if (triggered & TWAI_ALERT_ERR_PASS) {
            ESP_LOGW(TAG, "TWAI error passive (no peers ACKing?)");
        }
        if (triggered & TWAI_ALERT_TX_FAILED) {
            if (tx_state == TX_ACTIVE) {
                tx_fail_count++;
                if (tx_fail_count >= TX_FAIL_THRESHOLD) {
                    tx_state = TX_PROBING;
                    ESP_LOGW(TAG, "TWAI no peers detected, entering slow probe");
                }
            }
        }
        if (triggered & TWAI_ALERT_TX_SUCCESS) {
            if (tx_state == TX_PROBING) {
                tx_state = TX_ACTIVE;
                tx_fail_count = 0;
                ESP_LOGI(TAG, "TWAI probe ACK'd, peer detected, resuming normal TX");
            }
            tx_fail_count = 0;
        }

        // --- Drain received messages ---
        if (triggered & TWAI_ALERT_RX_DATA) {
            if (tx_state == TX_PROBING) {
                tx_state = TX_ACTIVE;
                tx_fail_count = 0;
                ESP_LOGI(TAG, "TWAI peer detected via RX, resuming normal TX");
            }
            twai_message_t msg;
            while (twai_receive(&msg, 0) == ESP_OK) {
                if (msg.rtr) continue;

                switch (msg.identifier) {
                case CAN_ID_OTA:
                    if (msg.data_length_code >= 3) {
                        handle_ota_trigger(msg.data);
                    }
                    break;

                case CAN_ID_WIFI_CONFIG:
                    if (msg.data_length_code >= 1) {
                        wifi_config_handle_can(msg.data, msg.data_length_code);
                    }
                    break;

                case CAN_ID_TOGGLE:
                    if (msg.data_length_code >= 1) {
                        uint8_t ch = msg.data[0];
                        if (ch < NUM_RELAYS) {
                            relay_toggle(ch);
                        } else if (msg.data_length_code >= 2) {
                            relay_set_all(msg.data[1] != 0);
                        }
                    }
                    break;

                default:
                    break;
                }
            }
        }

        // Check wifi config timeout
        wifi_config_check_timeout();

        // --- Periodic status transmit ---
        int64_t now = esp_timer_get_time();
        int64_t effective_period = (tx_state == TX_PROBING) ? tx_probe_period_us : tx_period_us;
        if (!bus_off && (now - last_tx_us >= effective_period)) {
            last_tx_us = now;

            twai_message_t tx_msg = {
                .identifier = CAN_ID_STATUS,
                .data_length_code = 1,
                .data = { relay_get_states() },
            };
            twai_transmit(&tx_msg, 0);
        }
    }
}
