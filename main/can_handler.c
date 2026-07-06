#include "can_handler.h"
#include "can_common.h"
#include "board.h"
#include "relay.h"
#include "wifi_config.h"
#include "discovery.h"
#include "ota.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#ifdef SWITCHBACK_VARIANT_AFTLINE
#include "esp_adc/adc_oneshot.h"
#endif

static const char *TAG = "can";

// Module address (0-2) — set at build time: idf.py build -DSWITCHBACK_ADDRESS=1
#ifndef SWITCHBACK_ADDRESS
#define SWITCHBACK_ADDRESS 0
#endif
#if SWITCHBACK_ADDRESS < 0 || SWITCHBACK_ADDRESS > 2
#error "SWITCHBACK_ADDRESS must be 0-2"
#endif

#define STATUS_TX_INTERVAL_MS  33    // ~30 Hz (relay status)
#define TX_PROBE_INTERVAL_MS   2000  // slow probe when no peers detected

#ifdef SWITCHBACK_VARIANT_PICKET
#define INPUT_TX_INTERVAL_MS   200   // 5 Hz (DI broadcast — matches Picket)
#define INPUT_DEBOUNCE_MS      50    // matches Picket

// Read the 8 digital inputs into a Picket-format bitmask.
// Bit i = DIN_PINS[i] state. HIGH (1) = open (no current through opto), LOW (0) = closed.
// In dry-contact mode, a reed switch wired between DIn and DGND closes the opto when the
// magnet is present, pulling the GPIO LOW. This matches Picket's "1 = open, 0 = closed".
static uint8_t read_digital_inputs(void)
{
    uint8_t state = 0;
    for (int i = 0; i < NUM_DIN; i++) {
        if (gpio_get_level(DIN_PINS[i]) == 1) {
            state |= (1 << i);
        }
    }
    return state;
}
#endif  // SWITCHBACK_VARIANT_PICKET

#ifdef SWITCHBACK_VARIANT_AFTLINE
#define AFTLINE_TX_INTERVAL_MS 33    // 30 Hz — matches standalone Aftline
#define CAN_ID_TRAILER_STATUS  0x3A  // DBC BO_ 58 TrailerStatus, TX=Aftline

static adc_oneshot_unit_handle_t s_aftline_adc = NULL;

static uint16_t aftline_read_voltage_mv(void)
{
    if (!s_aftline_adc) return 0;
    int raw = 0;
    if (adc_oneshot_read(s_aftline_adc, AFTLINE_ADC_CHANNEL, &raw) != ESP_OK) {
        return 0;
    }
    // ESP32-S3 ADC is 12-bit, 0-3.3V range with DB_12 attenuation.
    uint32_t mv_at_pin = ((uint32_t)raw * 3300) / 4095;
    uint32_t mv = mv_at_pin * AFTLINE_ADC_DIVIDER_RATIO;
    if (mv > 65535) mv = 65535;
    return (uint16_t)mv;
}
#endif  // SWITCHBACK_VARIANT_AFTLINE

esp_err_t can_handler_init(void)
{
    esp_err_t ret = can_common_init(CAN_TX_PIN, CAN_RX_PIN);
    if (ret != ESP_OK) return ret;

#ifdef SWITCHBACK_VARIANT_AFTLINE
    adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = AFTLINE_ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &s_aftline_adc));
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_aftline_adc, AFTLINE_ADC_CHANNEL, &chan_cfg));
    ESP_LOGI(TAG, "CAN addr=%d toggle=0x%02X status=0x%02X TrailerStatus=0x%02X (aftline)",
             SWITCHBACK_ADDRESS,
             CAN_ID_TOGGLE_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_STATUS_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_TRAILER_STATUS);
#elif defined(SWITCHBACK_VARIANT_PICKET)
    ESP_LOGI(TAG, "CAN addr=%d toggle=0x%02X status=0x%02X input=0x%02X (picket)",
             SWITCHBACK_ADDRESS,
             CAN_ID_TOGGLE_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_STATUS_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_INPUT_BASE + SWITCHBACK_ADDRESS);
#else
    ESP_LOGI(TAG, "CAN addr=%d toggle=0x%02X status=0x%02X (base, no DI broadcast)",
             SWITCHBACK_ADDRESS,
             CAN_ID_TOGGLE_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_STATUS_BASE + SWITCHBACK_ADDRESS);
#endif
    return ESP_OK;
}

void can_handler_task(void *arg)
{
    // Configure alerts BEFORE any bus activity so no error transitions are missed.
    twai_reconfigure_alerts(CAN_COMMON_ALERTS, NULL);

    // Alerts are armed above — any TX failure is caught by the state machine.
    can_common_version_broadcast();

    typedef enum { TX_ACTIVE, TX_PROBING } tx_state_t;
    bool bus_off = false;
    tx_state_t tx_state = TX_ACTIVE;
    int tx_fail_count = 0;
    const int TX_FAIL_THRESHOLD = 3;
    int64_t last_tx_us = 0;
    const int64_t tx_period_us = STATUS_TX_INTERVAL_MS * 1000LL;
    const int64_t tx_probe_period_us = TX_PROBE_INTERVAL_MS * 1000LL;

#ifdef SWITCHBACK_VARIANT_PICKET
    int64_t last_input_tx_us = 0;
    const int64_t input_tx_period_us = INPUT_TX_INTERVAL_MS * 1000LL;

    // Reed-switch / DI debounce state (Picket-format clone)
    uint8_t last_di_raw = read_digital_inputs();
    uint8_t di_debounced = last_di_raw;
    uint8_t last_logged_di = ~di_debounced;  // force first log
    int64_t last_di_change_us = esp_timer_get_time();
#endif

#ifdef SWITCHBACK_VARIANT_AFTLINE
    int64_t last_aftline_tx_us = 0;
    const int64_t aftline_tx_period_us = AFTLINE_TX_INTERVAL_MS * 1000LL;
#endif

    while (1) {
        uint32_t triggered;
        twai_read_alerts(&triggered, pdMS_TO_TICKS(10));

        // --- Bus error handling ---
        if (triggered & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "TWAI bus-off, initiating recovery");
            bus_off = true;
            twai_initiate_recovery();
            // No continue — fall through so RX_DATA in the same poll is still processed.
        }
        if (triggered & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(TAG, "TWAI bus recovered, restarting");
            twai_start();
            bus_off = false;
            tx_fail_count = 0;
            tx_state = TX_PROBING;
            // Version broadcast deferred until first TX_SUCCESS or RX_DATA confirms a peer.
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
                can_common_version_broadcast();
                ESP_LOGI(TAG, "TWAI probe ACK'd, peer detected, resuming normal TX");
            }
            tx_fail_count = 0;
        }

        // --- Drain received messages ---
        if (triggered & TWAI_ALERT_RX_DATA) {
            if (tx_state == TX_PROBING) {
                tx_state = TX_ACTIVE;
                tx_fail_count = 0;
                can_common_version_broadcast();
                ESP_LOGI(TAG, "TWAI peer detected via RX, resuming normal TX");
            }
            twai_message_t msg;
            while (twai_receive(&msg, 0) == ESP_OK) {
                if (msg.rtr) continue;

                switch (msg.identifier) {
                case CAN_ID_OTA:
                    if (msg.data_length_code >= 3) {
                        ota_handle_trigger(msg.data, msg.data_length_code);
                    }
                    break;

                case CAN_ID_WIFI_CONFIG:
                    if (msg.data_length_code >= 1) {
                        wifi_config_handle_can(msg.data, msg.data_length_code);
                    }
                    break;

                case CAN_ID_DISCOVERY_TRIGGER:
                    discovery_handle_trigger();
                    break;

                case (CAN_ID_TOGGLE_BASE + SWITCHBACK_ADDRESS):
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

        int64_t now = esp_timer_get_time();

#ifdef SWITCHBACK_VARIANT_PICKET
        // --- Debounce digital inputs (Picket-format) ---
        uint8_t raw_di = read_digital_inputs();
        if (raw_di != last_di_raw) {
            last_di_raw = raw_di;
            last_di_change_us = now;
        }
        if ((now - last_di_change_us) >= ((int64_t)INPUT_DEBOUNCE_MS * 1000)) {
            di_debounced = last_di_raw;
        }

        // --- Per-input picket state log (on debounced change) ---
        // Reed magnet present (cabinet closed) → opto closed → GPIO LOW → bit = 0
        // Reed magnet absent  (cabinet open)   → opto open   → GPIO HIGH → bit = 1
        if (di_debounced != last_logged_di) {
            last_logged_di = di_debounced;
            ESP_LOGI(TAG,
                "picket DI=0x%02X | DI1:Cabinet %s | DI2:Cabinet %s | DI3:Cabinet %s | DI4:Cabinet %s | DI5:Cabinet %s | DI6:Cabinet %s | DI7:Cabinet %s | DI8:Cabinet %s",
                di_debounced,
                (di_debounced & 0x01) ? "open" : "closed",
                (di_debounced & 0x02) ? "open" : "closed",
                (di_debounced & 0x04) ? "open" : "closed",
                (di_debounced & 0x08) ? "open" : "closed",
                (di_debounced & 0x10) ? "open" : "closed",
                (di_debounced & 0x20) ? "open" : "closed",
                (di_debounced & 0x40) ? "open" : "closed",
                (di_debounced & 0x80) ? "open" : "closed");
        }
#endif  // SWITCHBACK_VARIANT_PICKET

        // --- Periodic relay-status transmit (~30 Hz) ---
        int64_t effective_period = (tx_state == TX_PROBING) ? tx_probe_period_us : tx_period_us;
        if (!bus_off && (now - last_tx_us >= effective_period)) {
            last_tx_us = now;

            twai_message_t tx_msg = {
                .identifier = CAN_ID_STATUS_BASE + SWITCHBACK_ADDRESS,
                .data_length_code = 1,
                .data = { relay_get_states() },
            };
            twai_transmit(&tx_msg, 0);
        }

#ifdef SWITCHBACK_VARIANT_PICKET
        // --- Periodic DI broadcast (5 Hz, Picket-format) ---
        int64_t input_effective_period = (tx_state == TX_PROBING) ? tx_probe_period_us : input_tx_period_us;
        if (!bus_off && (now - last_input_tx_us >= input_effective_period)) {
            last_input_tx_us = now;

            twai_message_t input_msg = {
                .identifier = CAN_ID_INPUT_BASE + SWITCHBACK_ADDRESS,
                .data_length_code = 2,
                .data = {
                    di_debounced,   // DIN1-DIN8 (matches PicketStatus DoorStatus1to8)
                    0x00,            // Reserved (PicketStatus DoorStatus9to12 — unused on Switchback)
                },
            };
            twai_transmit(&input_msg, 0);
        }
#endif  // SWITCHBACK_VARIANT_PICKET

#ifdef SWITCHBACK_VARIANT_AFTLINE
        // --- Periodic TrailerStatus broadcast (30 Hz, DBC BO_ 58, 0x3A) ---
        // Opto is active-low when trailer wire carries voltage: signal = (gpio_get_level == 0).
        // No debounce — turn signals blink at 1-2 Hz and must pass through cleanly.
        int64_t aftline_effective_period = (tx_state == TX_PROBING) ? tx_probe_period_us : aftline_tx_period_us;
        if (!bus_off && (now - last_aftline_tx_us >= aftline_effective_period)) {
            last_aftline_tx_us = now;

            uint8_t flags = 0;
            if (gpio_get_level(AFTLINE_DI_CONNECTED)      == 0) flags |= 0x80;
            if (gpio_get_level(AFTLINE_DI_LEFT_TURN)      == 0) flags |= 0x40;
            if (gpio_get_level(AFTLINE_DI_RIGHT_TURN)     == 0) flags |= 0x20;
            if (gpio_get_level(AFTLINE_DI_RUNNING_LIGHTS) == 0) flags |= 0x10;
            if (gpio_get_level(AFTLINE_DI_BRAKES)         == 0) flags |= 0x08;

            uint16_t voltage_mv = aftline_read_voltage_mv();

            twai_message_t trailer_msg = {
                .identifier = CAN_ID_TRAILER_STATUS,
                .data_length_code = 3,
                .data = {
                    flags,
                    (uint8_t)((voltage_mv >> 8) & 0xFF),
                    (uint8_t)(voltage_mv & 0xFF),
                },
            };
            twai_transmit(&trailer_msg, 0);
        }
#endif  // SWITCHBACK_VARIANT_AFTLINE
    }
}
