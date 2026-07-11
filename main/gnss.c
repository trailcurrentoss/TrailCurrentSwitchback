#include "gnss.h"

// GNSS support is compiled only into the picket_gnss variant. Every other
// Switchback build ships this file as an empty translation unit.
#ifdef SWITCHBACK_VARIANT_GNSS

#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// The DFRobot Gravity GNSS (TEL0157) in UART mode does NOT stream NMEA.
// It uses a proprietary register-access protocol over serial, identical
// in layout to the I2C interface — only the transport differs:
//
//   Read  reg N of length L : TX [ N & 0x7F, L ]         → RX L bytes
//   Write reg N of length L : TX [ N | 0x80, data... ]   → (no response)
//
// Every register (year, month, lat, lon, altitude, GNSS mode, sleep mode,
// RGB mode, ID) is the same as the I2C map. The module stays silent until
// asked — earlier "0 bytes" sniffer results were correct because we
// weren't sending any queries.

// Register map — matches DFRobot_GNSS library.
#define REG_YEAR_H        0
#define REG_LAT_1         7
#define REG_LON_DIS       12
#define REG_LON_1         13
#define REG_LAT_DIS       18
#define REG_USE_STAR      19
#define REG_ALT_H         20
#define REG_SOG_H         23
#define REG_COG_H         26
#define REG_DATA_LEN      29    // number of bytes in the year-through-COG block

#define REG_ID            30
#define REG_GNSS_MODE     34
#define REG_SLEEP_MODE    35
#define REG_RGB_MODE      36

#define GNSS_DEVICE_ID    0x20  // library expects this value back from REG_ID
#define ENABLE_POWER      0x00
#define RGB_ON            0x05
#define RGB_OFF           0x02

#define GNSS_UART_RX_BUF  512
// Matches DFRobot's Python driver (raspberrypi/DFRobot_GNSS.py) — 50 ms
// post-write delay so the module has time to prepare its response, then a
// 1-second overall read timeout since the C++ Arduino driver also polls
// for up to 500-1000 ms before giving up.
#define GNSS_POST_WRITE_MS 50
#define GNSS_READ_TIMEOUT pdMS_TO_TICKS(1000)

static const char *TAG = "gnss";

static portMUX_TYPE     s_mux = portMUX_INITIALIZER_UNLOCKED;
static gnss_data_t      s_latest;
static gnss_update_cb_t s_on_update = NULL;
static int              s_uart_num = -1;

// ---------------------------------------------------------------------------
// Low-level register access (DFRobot proprietary UART protocol)
// ---------------------------------------------------------------------------

static esp_err_t gnss_write_reg(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t cmd = reg | 0x80;
    if (uart_write_bytes(s_uart_num, &cmd, 1) != 1) return ESP_FAIL;
    if (len && uart_write_bytes(s_uart_num, data, len) != (int)len) return ESP_FAIL;
    uart_wait_tx_done(s_uart_num, pdMS_TO_TICKS(100));
    return ESP_OK;
}

static esp_err_t gnss_read_reg(uint8_t reg, uint8_t *out, size_t len)
{
    uart_flush_input(s_uart_num);
    uint8_t cmd[2] = { (uint8_t)(reg & 0x7F), (uint8_t)len };
    if (uart_write_bytes(s_uart_num, cmd, 2) != 2) return ESP_FAIL;
    uart_wait_tx_done(s_uart_num, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(GNSS_POST_WRITE_MS));
    int got = uart_read_bytes(s_uart_num, out, len, GNSS_READ_TIMEOUT);
    if (got != (int)len) {
        ESP_LOGW(TAG, "readReg(0x%02X, len=%u) got %d bytes", reg, (unsigned)len, got);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Fix acquisition
// ---------------------------------------------------------------------------

static esp_err_t gnss_read_fix(gnss_data_t *out)
{
    uint8_t buf[REG_DATA_LEN];
    esp_err_t err = gnss_read_reg(REG_YEAR_H, buf, REG_DATA_LEN);
    if (err != ESP_OK) return err;

    out->year   = ((uint16_t)buf[0] << 8) | buf[1];
    out->month  = buf[2];
    out->day    = buf[3];
    out->hour   = buf[4];
    out->minute = buf[5];
    out->second = buf[6];

    uint8_t lat_dd = buf[7];
    uint8_t lat_mm = buf[8];
    uint32_t lat_frac = ((uint32_t)buf[9] << 16) | ((uint32_t)buf[10] << 8) | buf[11];
    float lat_deg = (float)lat_dd + ((float)lat_mm + (float)lat_frac / 100000.0f) / 60.0f;
    out->latitude = (buf[REG_LAT_DIS] == 'S') ? -lat_deg : lat_deg;

    uint8_t lon_dd = buf[13];
    uint8_t lon_mm = buf[14];
    uint32_t lon_frac = ((uint32_t)buf[15] << 16) | ((uint32_t)buf[16] << 8) | buf[17];
    float lon_deg = (float)lon_dd + ((float)lon_mm + (float)lon_frac / 100000.0f) / 60.0f;
    out->longitude = (buf[REG_LON_DIS] == 'W') ? -lon_deg : lon_deg;

    out->satellites = buf[19];
    out->altitude   = (double)((((uint16_t)buf[20] & 0x7F) << 8) | buf[21]) + (double)buf[22] / 100.0;
    out->speed      = (double)((((uint16_t)buf[23] & 0x7F) << 8) | buf[24]) + (double)buf[25] / 100.0;
    out->course     = (double)((((uint16_t)buf[26] & 0x7F) << 8) | buf[27]) + (double)buf[28] / 100.0;

    uint8_t mode;
    if (gnss_read_reg(REG_GNSS_MODE, &mode, 1) == ESP_OK) {
        out->gnss_mode = mode;
    } else {
        out->gnss_mode = 0;
    }
    return ESP_OK;
}

static void gnss_task(void *arg)
{
    // Bring the module out of sleep and select the constellation. These are
    // fire-and-forget writes — no response bytes are expected.
    uint8_t v;
    v = ENABLE_POWER;
    gnss_write_reg(REG_SLEEP_MODE, &v, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    v = 7;  // GPS + BeiDou + GLONASS
    gnss_write_reg(REG_GNSS_MODE, &v, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    v = RGB_ON;
    gnss_write_reg(REG_RGB_MODE, &v, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Probe once per second by trying to read REG_ID (always 0x20). This
    // makes the "did the round-trip work at all?" question obvious in the
    // logs — you don't need to wait for a GPS fix to see if the pipeline
    // is alive.
    int64_t last_probe = 0;
    const int64_t probe_period_us = 1000000LL;

    while (1) {
        int64_t now = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS * 1000LL;
        if (now - last_probe >= probe_period_us) {
            last_probe = now;
            uint8_t id = 0xFF;
            esp_err_t err = gnss_read_reg(REG_ID, &id, 1);
            ESP_LOGI(TAG, "probe REG_ID: err=%d id=0x%02X %s",
                     err, id,
                     (err == ESP_OK && id == GNSS_DEVICE_ID) ? "OK" : "no response");
        }

        gnss_data_t fresh;
        memset(&fresh, 0, sizeof(fresh));
        esp_err_t err = gnss_read_fix(&fresh);
        if (err == ESP_OK) {
            bool date_valid =
                fresh.year  >= 2025 && fresh.year  <= 2099 &&
                fresh.month >= 1    && fresh.month <= 12 &&
                fresh.day   >= 1    && fresh.day   <= 31 &&
                fresh.hour  <= 23   && fresh.minute <= 59 && fresh.second <= 60;

            taskENTER_CRITICAL(&s_mux);
            if (date_valid) {
                s_latest.year   = fresh.year;
                s_latest.month  = fresh.month;
                s_latest.day    = fresh.day;
                s_latest.hour   = fresh.hour;
                s_latest.minute = fresh.minute;
                s_latest.second = fresh.second;
            }
            s_latest.latitude   = fresh.latitude;
            s_latest.longitude  = fresh.longitude;
            s_latest.altitude   = fresh.altitude;
            s_latest.satellites = fresh.satellites;
            s_latest.speed      = fresh.speed;
            s_latest.course     = fresh.course;
            s_latest.gnss_mode  = fresh.gnss_mode;
            gnss_data_t snap = s_latest;
            taskEXIT_CRITICAL(&s_mux);

            if (s_on_update) s_on_update(&snap);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t gnss_init(int uart_num, int tx_pin, int rx_pin, int baud_rate,
                    gnss_update_cb_t on_update)
{
    s_uart_num  = uart_num;
    s_on_update = on_update;
    memset(&s_latest, 0, sizeof(s_latest));

    uart_config_t cfg = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(uart_num, GNSS_UART_RX_BUF, GNSS_UART_RX_BUF, 0, NULL, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(uart_num, &cfg);
    if (err != ESP_OK) return err;
    err = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    // Probe: read the ID register. Should return 0x20.
    uint8_t id = 0;
    for (int attempt = 0; attempt < 5; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(100));  // give the module time to settle
        if (gnss_read_reg(REG_ID, &id, 1) == ESP_OK && id == GNSS_DEVICE_ID) {
            break;
        }
        ESP_LOGW(TAG, "probe attempt %d: id=0x%02X (want 0x%02X)", attempt + 1, id, GNSS_DEVICE_ID);
    }
    if (id != GNSS_DEVICE_ID) {
        ESP_LOGE(TAG, "GNSS module ID probe failed after 5 attempts — will keep trying in the task");
        // Don't abort — the task retries. Init still returns OK so the rest
        // of the firmware comes up.
    } else {
        ESP_LOGI(TAG, "DFRobot GNSS UART online (id=0x%02X @ %d baud, tx=GPIO%d rx=GPIO%d)",
                 id, baud_rate, tx_pin, rx_pin);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(gnss_task, "gnss", 4096, NULL, 4, NULL, 0);
    if (ok != pdPASS) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t gnss_read(gnss_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    taskENTER_CRITICAL(&s_mux);
    *data = s_latest;
    taskEXIT_CRITICAL(&s_mux);
    return ESP_OK;
}

#endif  // SWITCHBACK_VARIANT_GNSS
