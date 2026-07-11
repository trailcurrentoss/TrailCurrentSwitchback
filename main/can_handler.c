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
#ifdef SWITCHBACK_VARIANT_GNSS
#include <string.h>
#include "gnss.h"
#endif
#ifdef SWITCHBACK_VARIANT_SCD41
#include "sensors.h"
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

#ifdef SWITCHBACK_VARIANT_GNSS
#define GNSS_TX_INTERVAL_MS    33    // 30 Hz — matches standalone Bearing

// Snapshot buffer for the CAN TX task. Written by the main-task GNSS poll
// via can_handler_publish_gnss, drained by the TX loop under a spinlock so
// no consumer ever sees mixed minute/second across a fix rollover.
static portMUX_TYPE g_gnss_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t g_gnss_year;
static uint8_t  g_gnss_month, g_gnss_day, g_gnss_hour, g_gnss_minute, g_gnss_second;
static float    g_gnss_latitude, g_gnss_longitude;
static double   g_gnss_altitude, g_gnss_speed, g_gnss_course;
static uint8_t  g_gnss_satellites, g_gnss_mode;

void can_handler_publish_gnss(const gnss_data_t *data)
{
    // Suppress obviously-invalid date/time (module reports 2000-00-00 during
    // cold start before it has any satellites) — retain the last good stamp
    // so downstream consumers don't jump backwards on every reboot.
    bool date_valid =
        data->year  >= 2025 && data->year  <= 2099 &&
        data->month >= 1    && data->month <= 12 &&
        data->day   >= 1    && data->day   <= 31 &&
        data->hour  <= 23 &&
        data->minute <= 59 &&
        data->second <= 60;

    taskENTER_CRITICAL(&g_gnss_mux);
    if (date_valid) {
        g_gnss_year   = data->year;
        g_gnss_month  = data->month;
        g_gnss_day    = data->day;
        g_gnss_hour   = data->hour;
        g_gnss_minute = data->minute;
        g_gnss_second = data->second;
    }
    g_gnss_latitude   = data->latitude;
    g_gnss_longitude  = data->longitude;
    g_gnss_altitude   = data->altitude;
    g_gnss_satellites = data->satellites;
    g_gnss_speed      = data->speed;
    g_gnss_course     = data->course;
    g_gnss_mode       = data->gnss_mode;
    taskEXIT_CRITICAL(&g_gnss_mux);
}

// Bearing wire format — sign flag + 24-bit unsigned |value|*10000.
static void encode_lat_lon(float value, uint8_t out[4])
{
    out[0] = (value < 0) ? 1 : 0;
    if (value < 0) value = -value;
    uint32_t scaled = (uint32_t)(value * 10000.0f + 0.5f);
    out[1] = (scaled >> 16) & 0xFF;
    out[2] = (scaled >> 8) & 0xFF;
    out[3] = scaled & 0xFF;
}
#endif  // SWITCHBACK_VARIANT_GNSS

#ifdef SWITCHBACK_VARIANT_SCD41
#define ENV_TX_INTERVAL_MS     1000  // 1 Hz — matches standalone Borealis

static portMUX_TYPE g_env_mux = portMUX_INITIALIZER_UNLOCKED;
static int8_t   g_env_temp_c_int;
static int8_t   g_env_temp_f_int;
static uint16_t g_env_humidity_scaled;   // %RH × 100
static uint16_t g_env_co2_ppm;
static uint16_t g_env_voc_index;         // 0 = never seen; 1-500 valid range
static bool     g_env_valid;

void can_handler_publish_env(const scd41_data_t *data)
{
    if (!data || !data->valid) return;
    float tF = data->temperature_c * 9.0f / 5.0f + 32.0f;
    taskENTER_CRITICAL(&g_env_mux);
    g_env_temp_c_int      = (int8_t)(data->temperature_c + 0.5f);
    g_env_temp_f_int      = (int8_t)(tF + 0.5f);
    g_env_humidity_scaled = (uint16_t)(data->humidity * 100.0f);
    g_env_co2_ppm         = data->co2_ppm;
    g_env_valid           = true;
    taskEXIT_CRITICAL(&g_env_mux);
}

void can_handler_publish_voc(const sgp40_data_t *data)
{
    if (!data || !data->valid) return;
    taskENTER_CRITICAL(&g_env_mux);
    g_env_voc_index = data->voc_index;
    taskEXIT_CRITICAL(&g_env_mux);
}

// --- Safety snapshot (CO + alarm flags) ---
static portMUX_TYPE g_safety_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t g_co_ppm;
static bool     g_safety_valid;

void can_handler_publish_co(const co_data_t *data)
{
    if (!data || !data->valid) return;
    uint16_t co_rounded = (uint16_t)(data->ppm + 0.5f);
    taskENTER_CRITICAL(&g_safety_mux);
    g_co_ppm       = co_rounded;
    g_safety_valid = true;
    taskEXIT_CRITICAL(&g_safety_mux);
}

// Recompute the alarm bitmask from the latest environmental + safety values.
// Called from the TX loop under both spinlocks so the flags are consistent
// with what actually gets transmitted.
static uint8_t compute_alarm_flags(uint16_t co_ppm, uint16_t co2_ppm, uint16_t voc_index)
{
    uint8_t f = 0;
    if (co_ppm >= CO_PPM_ALARM)        f |= ALARM_FLAG_CO_ALARM;
    else if (co_ppm >= CO_PPM_WARNING) f |= ALARM_FLAG_CO_WARN;

    if (co2_ppm >= CO2_PPM_ALARM)         f |= ALARM_FLAG_CO2_ALARM;
    else if (co2_ppm >= CO2_PPM_WARNING)  f |= ALARM_FLAG_CO2_WARN;

    if (voc_index >= VOC_INDEX_ALARM) f |= ALARM_FLAG_VOC_ALARM;
    return f;
}
#endif  // SWITCHBACK_VARIANT_SCD41

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
#elif defined(SWITCHBACK_VARIANT_PICKET) && defined(SWITCHBACK_VARIANT_GNSS)
    ESP_LOGI(TAG, "CAN addr=%d toggle=0x%02X status=0x%02X input=0x%02X GNSS=0x%02X/0x%02X/0x%02X/0x%02X (picket_gnss)",
             SWITCHBACK_ADDRESS,
             CAN_ID_TOGGLE_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_STATUS_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_INPUT_BASE + SWITCHBACK_ADDRESS,
             CAN_ID_DATETIME, CAN_ID_SAT_SPEED, CAN_ID_ALTITUDE, CAN_ID_LATLON);
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

    // Per-second heartbeat log — visible even with no CAN peer on the bus.
    // Shows relay byte + TX state (+ DI/aftline state on those variants) so
    // an operator can confirm bench operation without a CAN analyzer.
    int64_t last_heartbeat_us = 0;
    const int64_t heartbeat_period_us = 1000000LL;

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

#ifdef SWITCHBACK_VARIANT_GNSS
    int64_t last_gnss_tx_us = 0;
    const int64_t gnss_tx_period_us = GNSS_TX_INTERVAL_MS * 1000LL;
    // Periodic "tx heartbeat" log so we can see the module is running even
    // when the CAN bus is disconnected (TX_FAILED silently keeps firing).
    int64_t last_gnss_log_us = 0;
    const int64_t gnss_log_period_us = 1000000LL;
#endif

#ifdef SWITCHBACK_VARIANT_SCD41
    int64_t last_env_tx_us = 0;
    const int64_t env_tx_period_us = ENV_TX_INTERVAL_MS * 1000LL;
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

        // --- Per-second heartbeat log (visible without a CAN peer) ---
        if (now - last_heartbeat_us >= heartbeat_period_us) {
            last_heartbeat_us = now;
            const char *tx_str = bus_off ? "bus_off"
                              : (tx_state == TX_PROBING) ? "probing" : "active";
#if defined(SWITCHBACK_VARIANT_PICKET)
            ESP_LOGI(TAG, "hb: relay=0x%02X di=0x%02X tx=%s",
                     relay_get_states(), di_debounced, tx_str);
#else
            ESP_LOGI(TAG, "hb: relay=0x%02X tx=%s", relay_get_states(), tx_str);
#endif
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

#ifdef SWITCHBACK_VARIANT_GNSS
        // --- Periodic GNSS broadcast (30 Hz, Bearing-format 0x06-0x09) ---
        int64_t gnss_effective_period = (tx_state == TX_PROBING) ? tx_probe_period_us : gnss_tx_period_us;
        if (!bus_off && (now - last_gnss_tx_us >= gnss_effective_period)) {
            last_gnss_tx_us = now;

            uint16_t year;
            uint8_t month, day, hour, minute, second;
            float lat, lon;
            double alt, spd, crs;
            uint8_t sats, mode;
            taskENTER_CRITICAL(&g_gnss_mux);
            year = g_gnss_year;
            month = g_gnss_month; day = g_gnss_day;
            hour = g_gnss_hour; minute = g_gnss_minute; second = g_gnss_second;
            lat = g_gnss_latitude; lon = g_gnss_longitude;
            alt = g_gnss_altitude; spd = g_gnss_speed; crs = g_gnss_course;
            sats = g_gnss_satellites; mode = g_gnss_mode;
            taskEXIT_CRITICAL(&g_gnss_mux);

            twai_message_t m_dt = {
                .identifier = CAN_ID_DATETIME,
                .data_length_code = 7,
                .data = {
                    (year >> 8) & 0xFF, year & 0xFF,
                    month, day, hour, minute, second
                }
            };

            uint16_t speed_scaled = (uint16_t)(spd * 100.0);
            uint16_t course_scaled = (uint16_t)(crs * 10.0 + 0.5);
            twai_message_t m_nav = {
                .identifier = CAN_ID_SAT_SPEED,
                .data_length_code = 6,
                .data = {
                    sats,
                    (speed_scaled >> 8) & 0xFF, speed_scaled & 0xFF,
                    (course_scaled >> 8) & 0xFF, course_scaled & 0xFF,
                    mode
                }
            };

            uint32_t alt_scaled = (uint32_t)(alt * 100.0);
            twai_message_t m_alt = {
                .identifier = CAN_ID_ALTITUDE,
                .data_length_code = 4,
                .data = {
                    (alt_scaled >> 24) & 0xFF,
                    (alt_scaled >> 16) & 0xFF,
                    (alt_scaled >> 8) & 0xFF,
                    alt_scaled & 0xFF
                }
            };

            uint8_t lat_enc[4], lon_enc[4];
            encode_lat_lon(lat, lat_enc);
            encode_lat_lon(lon, lon_enc);
            twai_message_t m_pos = {
                .identifier = CAN_ID_LATLON,
                .data_length_code = 8,
                .data = {
                    lat_enc[0], lat_enc[1], lat_enc[2], lat_enc[3],
                    lon_enc[0], lon_enc[1], lon_enc[2], lon_enc[3]
                }
            };

            twai_transmit(&m_dt, 0);
            twai_transmit(&m_nav, 0);
            twai_transmit(&m_alt, 0);
            twai_transmit(&m_pos, 0);

            // Heartbeat log every ~1 s so an operator watching the console
            // (without a CAN analyzer / peer on the bus) can confirm the
            // fresh GNSS snapshot is what would go on the wire.
            if (now - last_gnss_log_us >= gnss_log_period_us) {
                last_gnss_log_us = now;
                ESP_LOGI(TAG,
                    "gnss tx: %04u-%02u-%02u %02u:%02u:%02u sats=%u mode=%u lat=%.5f lon=%.5f alt=%.2fm spd=%.2fkt crs=%.1f (tx=%s)",
                    year, month, day, hour, minute, second, sats, mode,
                    (double)lat, (double)lon, alt, spd, crs,
                    (tx_state == TX_PROBING) ? "probing" : "active");
            }
        }
#endif  // SWITCHBACK_VARIANT_GNSS

#ifdef SWITCHBACK_VARIANT_SCD41
        // --- Periodic environmental broadcast (1 Hz, Borealis-format 0x1F) ---
        int64_t env_effective_period = (tx_state == TX_PROBING) ? tx_probe_period_us : env_tx_period_us;
        if (!bus_off && (now - last_env_tx_us >= env_effective_period)) {
            last_env_tx_us = now;

            int8_t   temp_c, temp_f;
            uint16_t hum, co2, voc;
            bool valid;
            taskENTER_CRITICAL(&g_env_mux);
            valid  = g_env_valid;
            temp_c = g_env_temp_c_int;
            temp_f = g_env_temp_f_int;
            hum    = g_env_humidity_scaled;
            co2    = g_env_co2_ppm;
            voc    = g_env_voc_index;
            taskEXIT_CRITICAL(&g_env_mux);

            if (valid) {
                // Layout matches Borealis EnvironmentalStatus. VOC bytes stay
                // zero until the first SGP40 measurement completes.
                twai_message_t env_msg = {
                    .identifier = CAN_ID_ENVIRONMENTAL,
                    .data_length_code = 8,
                    .data = {
                        (uint8_t)temp_c,
                        (uint8_t)temp_f,
                        (hum >> 8) & 0xFF, hum & 0xFF,
                        (co2 >> 8) & 0xFF, co2 & 0xFF,
                        (voc >> 8) & 0xFF, voc & 0xFF,
                    },
                };
                twai_transmit(&env_msg, 0);
            }

            // Safety frame piggybacks on the same 1 Hz cadence. Alarm flags
            // combine CO (SEN0466) with CO2 and VOC (already snapshotted
            // above), so they're always consistent with the 0x1F payload we
            // just sent.
            uint16_t co_ppm;
            bool safety_valid;
            taskENTER_CRITICAL(&g_safety_mux);
            safety_valid = g_safety_valid;
            co_ppm       = g_co_ppm;
            taskEXIT_CRITICAL(&g_safety_mux);

            if (safety_valid) {
                uint8_t flags = compute_alarm_flags(co_ppm, co2, voc);
                // Bytes 2-3 reserved for LPG Rs/R0 × 1000 (Borealis MQ-6);
                // stay zero because Switchback has no MQ-6.
                twai_message_t safety_msg = {
                    .identifier = CAN_ID_SAFETY,
                    .data_length_code = 8,
                    .data = {
                        (co_ppm >> 8) & 0xFF, co_ppm & 0xFF,
                        0x00, 0x00,
                        flags,
                        0x00, 0x00, 0x00,
                    },
                };
                twai_transmit(&safety_msg, 0);
            }
        }
#endif  // SWITCHBACK_VARIANT_SCD41

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
