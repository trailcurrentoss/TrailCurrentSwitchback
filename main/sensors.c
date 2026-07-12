#include "sensors.h"

// SCD41 support is only compiled into variants that define SWITCHBACK_VARIANT_SCD41
// (currently just picket_gnss). Everything else ships an empty translation unit
// so we don't drag the driver into base/picket/aftline binaries.
#ifdef SWITCHBACK_VARIANT_SCD41

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "sensors";

static i2c_master_dev_handle_t s_scd41_dev = NULL;
static i2c_master_dev_handle_t s_sgp40_dev = NULL;
static i2c_master_dev_handle_t s_co_dev = NULL;

// SCD41 command opcodes (from Sensirion datasheet)
#define SCD41_CMD_START_PERIODIC  0x21B1
#define SCD41_CMD_READ_MEAS       0xEC05
#define SCD41_CMD_GET_DATA_READY  0xE4B8
#define SCD41_CMD_STOP_PERIODIC   0x3F86

// Sensirion CRC-8 (poly 0x31, init 0xFF). Same primitive SGP40, SHT4x, and
// friends use — kept local so this file has no dependency on the wider
// Borealis sensors module.
static uint8_t sensirion_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

static esp_err_t sensirion_send_cmd(uint16_t cmd)
{
    if (!s_scd41_dev) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = { (cmd >> 8) & 0xFF, cmd & 0xFF };
    return i2c_master_transmit(s_scd41_dev, buf, 2, 1000);
}

esp_err_t sensors_init(i2c_master_bus_handle_t bus)
{
    if (!bus) {
        ESP_LOGE(TAG, "sensors_init called with NULL bus (relay_init must run first)");
        return ESP_ERR_INVALID_ARG;
    }
    if (s_scd41_dev) { i2c_master_bus_rm_device(s_scd41_dev); s_scd41_dev = NULL; }
    if (s_sgp40_dev) { i2c_master_bus_rm_device(s_sgp40_dev); s_sgp40_dev = NULL; }
    if (s_co_dev)    { i2c_master_bus_rm_device(s_co_dev);    s_co_dev = NULL; }

    i2c_device_config_t scd41_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD41_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bus, &scd41_cfg, &s_scd41_dev) != ESP_OK) {
        ESP_LOGE(TAG, "SCD41 attach failed");
        s_scd41_dev = NULL;
    } else {
        ESP_LOGI(TAG, "SCD41 attached (0x%02X)", SCD41_I2C_ADDR);
    }

    i2c_device_config_t sgp40_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SGP40_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bus, &sgp40_cfg, &s_sgp40_dev) != ESP_OK) {
        ESP_LOGE(TAG, "SGP40 attach failed");
        s_sgp40_dev = NULL;
    } else {
        ESP_LOGI(TAG, "SGP40 attached (0x%02X)", SGP40_I2C_ADDR);
    }

    i2c_device_config_t co_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SEN0466_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bus, &co_cfg, &s_co_dev) != ESP_OK) {
        ESP_LOGE(TAG, "SEN0466 attach failed");
        s_co_dev = NULL;
    } else {
        ESP_LOGI(TAG, "SEN0466 CO sensor attached (0x%02X)", SEN0466_I2C_ADDR);
    }

    // Non-fatal on individual device miss — any surviving sensor keeps
    // contributing to its slice of the CAN broadcasts.
    return (s_scd41_dev || s_sgp40_dev || s_co_dev) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t scd41_start_periodic(void)
{
    esp_err_t err = sensirion_send_cmd(SCD41_CMD_START_PERIODIC);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SCD41 start_periodic failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "SCD41 periodic measurement started (5s update cadence)");
    }
    return err;
}

static bool scd41_data_ready(void)
{
    if (sensirion_send_cmd(SCD41_CMD_GET_DATA_READY) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(2));
    uint8_t resp[3];
    if (i2c_master_receive(s_scd41_dev, resp, 3, 1000) != ESP_OK) return false;
    if (sensirion_crc8(resp, 2) != resp[2]) return false;
    uint16_t status = ((uint16_t)resp[0] << 8) | resp[1];
    // Lower 11 bits == 0 means no data available yet
    return (status & 0x07FF) != 0;
}

scd41_data_t scd41_read(void)
{
    scd41_data_t out = { .valid = false };
    if (!s_scd41_dev) return out;
    if (!scd41_data_ready()) return out;

    if (sensirion_send_cmd(SCD41_CMD_READ_MEAS) != ESP_OK) return out;
    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t buf[9];
    if (i2c_master_receive(s_scd41_dev, buf, sizeof(buf), 1000) != ESP_OK) return out;

    if (sensirion_crc8(&buf[0], 2) != buf[2] ||
        sensirion_crc8(&buf[3], 2) != buf[5] ||
        sensirion_crc8(&buf[6], 2) != buf[8]) {
        ESP_LOGW(TAG, "SCD41 CRC error");
        return out;
    }

    uint16_t raw_co2 = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_t   = ((uint16_t)buf[3] << 8) | buf[4];
    uint16_t raw_rh  = ((uint16_t)buf[6] << 8) | buf[7];

    out.co2_ppm       = raw_co2;
    out.temperature_c = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    out.humidity      = 100.0f * ((float)raw_rh / 65535.0f);
    out.valid = true;
    return out;
}

// ---------------------------------------------------------------------------
// SGP40 — Sensirion VOC sensor (raw signal + simple index estimator)
// ---------------------------------------------------------------------------

#define SGP40_CMD_MEASURE_RAW  0x260F

// Same simple estimator Borealis uses. Not the full Sensirion gas_index_algorithm
// (BSD-3, drop-in via component manager) — this is a running-baseline EMA that
// tracks "is air quality changing" without needing 24 h of warm-up. Replace when
// long-term stability matters.
static uint16_t s_sgp40_baseline = 30000;
static bool     s_sgp40_baseline_seeded = false;

static uint16_t sgp40_estimate_index(uint16_t sraw)
{
    if (!s_sgp40_baseline_seeded) {
        s_sgp40_baseline = sraw;
        s_sgp40_baseline_seeded = true;
    }
    // 1/4096 step ≈ 17 minute time constant at 1 Hz sampling.
    int32_t delta_b = (int32_t)sraw - (int32_t)s_sgp40_baseline;
    s_sgp40_baseline = (uint16_t)((int32_t)s_sgp40_baseline + (delta_b / 4096));

    int32_t deviation = (int32_t)sraw - (int32_t)s_sgp40_baseline;
    int32_t idx = 100 + deviation / 50;
    if (idx < 1)   idx = 1;
    if (idx > 500) idx = 500;
    return (uint16_t)idx;
}

sgp40_data_t sgp40_measure(float humidity_pct, float temperature_c)
{
    sgp40_data_t out = { .valid = false };
    if (!s_sgp40_dev) return out;

    uint16_t rh_ticks, t_ticks;
    if (humidity_pct < 0.0f || humidity_pct > 100.0f) {
        rh_ticks = 0x8000;  // 50 %RH default
        t_ticks  = 0x6666;  // 25 °C default
    } else {
        rh_ticks = (uint16_t)((humidity_pct * 65535.0f) / 100.0f);
        t_ticks  = (uint16_t)(((temperature_c + 45.0f) * 65535.0f) / 175.0f);
    }

    uint8_t cmd[8];
    cmd[0] = (SGP40_CMD_MEASURE_RAW >> 8) & 0xFF;
    cmd[1] = SGP40_CMD_MEASURE_RAW & 0xFF;
    cmd[2] = (rh_ticks >> 8) & 0xFF;
    cmd[3] = rh_ticks & 0xFF;
    cmd[4] = sensirion_crc8(&cmd[2], 2);
    cmd[5] = (t_ticks >> 8) & 0xFF;
    cmd[6] = t_ticks & 0xFF;
    cmd[7] = sensirion_crc8(&cmd[5], 2);

    if (i2c_master_transmit(s_sgp40_dev, cmd, sizeof(cmd), 1000) != ESP_OK) return out;
    vTaskDelay(pdMS_TO_TICKS(35));  // datasheet: 30 ms measurement time

    uint8_t resp[3];
    if (i2c_master_receive(s_sgp40_dev, resp, sizeof(resp), 1000) != ESP_OK) return out;
    if (sensirion_crc8(resp, 2) != resp[2]) {
        ESP_LOGW(TAG, "SGP40 CRC error");
        return out;
    }
    out.voc_raw   = ((uint16_t)resp[0] << 8) | resp[1];
    out.voc_index = sgp40_estimate_index(out.voc_raw);
    out.valid     = true;
    return out;
}

// ---------------------------------------------------------------------------
// DFRobot SEN0466 — factory-calibrated electrochemical CO sensor (I2C)
// Uses DFRobot's 9-byte command frame protocol; checksum is 2's complement
// of the sum of bytes 1-7.
// ---------------------------------------------------------------------------

static uint8_t sen0466_checksum(const uint8_t *frame)
{
    uint16_t sum = 0;
    for (int i = 1; i < 8; i++) sum += frame[i];
    return (uint8_t)((~sum + 1) & 0xFF);
}

static esp_err_t sen0466_send_cmd(uint8_t cmd_byte,
                                  uint8_t b3, uint8_t b4, uint8_t b5,
                                  uint8_t b6, uint8_t b7,
                                  uint8_t *resp_out)
{
    if (!s_co_dev) return ESP_ERR_INVALID_STATE;

    uint8_t frame[9] = { 0xFF, 0x01, cmd_byte, b3, b4, b5, b6, b7, 0 };
    frame[8] = sen0466_checksum(frame);

    if (i2c_master_transmit(s_co_dev, frame, 9, 1000) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t resp[9];
    if (i2c_master_receive(s_co_dev, resp, 9, 1000) != ESP_OK) return ESP_FAIL;
    if (resp[0] != 0xFF) return ESP_FAIL;
    if (sen0466_checksum(resp) != resp[8]) return ESP_FAIL;
    if (resp_out) memcpy(resp_out, resp, 9);
    return ESP_OK;
}

esp_err_t co_sen0466_set_active_mode(void)
{
    if (!s_co_dev) return ESP_ERR_INVALID_STATE;
    // 0x78: set mode; 0x03 = active (continuous), 0x04 = passive.
    // Matches Borealis's working driver.
    esp_err_t ret = sen0466_send_cmd(0x78, 0x03, 0x00, 0x00, 0x00, 0x00, NULL);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SEN0466 set to active acquisition mode");
    } else {
        ESP_LOGW(TAG, "SEN0466 set_active_mode failed");
    }
    return ret;
}

static float sen0466_read_temp_c(void)
{
    uint8_t resp[9];
    if (sen0466_send_cmd(0x87, 0, 0, 0, 0, 0, resp) != ESP_OK) return -273.15f;
    uint16_t adc = ((uint16_t)resp[2] << 8) | resp[3];
    // Beta thermistor formula lifted from DFRobot_MultiGasSensor.
    float Vpd3 = 3.0f * (float)adc / 1024.0f;
    if (Vpd3 >= 3.0f) return -273.15f;
    float Rth  = Vpd3 * 10000.0f / (3.0f - Vpd3);
    float Tk   = 1.0f / (1.0f / (273.15f + 25.0f) + 1.0f / 3380.13f * logf(Rth / 10000.0f));
    return Tk - 273.15f;
}

static float sen0466_read_cell_voltage(void)
{
    uint8_t resp[9];
    if (sen0466_send_cmd(0x91, 0, 0, 0, 0, 0, resp) != ESP_OK) return -1.0f;
    uint16_t adc = ((uint16_t)resp[2] << 8) | resp[3];
    return (float)adc * 3.0f / 1024.0f * 2.0f;
}

co_data_t co_sen0466_read(void)
{
    co_data_t out = { .valid = false };
    if (!s_co_dev) return out;

    // Diagnostic every 15 s so we can watch the electrochemical cell across
    // exposures. Live cell = ~0.05-0.6 V; sits stable in clean air, rises
    // when CO is present. Dead-zero voltage = the CO cell isn't polarized,
    // sensor is DOA.
    static int diag_counter = 0;
    if ((diag_counter++ % 5) == 0) {
        float v = sen0466_read_cell_voltage();
        float t = sen0466_read_temp_c();
        ESP_LOGI(TAG, "SEN0466 diagnostics: cell voltage=%.3f V, board temp=%.1f °C", v, t);
    }

    uint8_t resp[9];
    // 0x86: read concentration. See byte-layout comment below the log call.
    if (sen0466_send_cmd(0x86, 0x00, 0x00, 0x00, 0x00, 0x00, resp) != ESP_OK) {
        return out;
    }
    ESP_LOGI(TAG, "SEN0466 raw resp: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             resp[0], resp[1], resp[2], resp[3], resp[4],
             resp[5], resp[6], resp[7], resp[8]);
    // Response layout: [FF][86][conc_hi][conc_lo][gas_type=0x04 for CO]
    //                  [decimal_places][reserved][reserved][checksum].
    // Verified live on bench (candle exposure): raw bytes moved 0x02→0x32
    // as CO rose, byte 4 stayed a constant 0x04 (gas-type ID), byte 5 was
    // the varying decimal-places field.
    uint16_t raw = ((uint16_t)resp[2] << 8) | resp[3];
    uint8_t  decimal_places = resp[5];
    float divisor = 1.0f;
    for (int i = 0; i < decimal_places; i++) divisor *= 10.0f;
    out.ppm = (float)raw / divisor;
    out.valid = true;
    return out;
}

#endif  // SWITCHBACK_VARIANT_SCD41
