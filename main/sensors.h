#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdbool.h>
#include <stdint.h>

// DFRobot Gravity SCD41 (Sensirion silicon) — real NDIR CO2 + T + RH.
// Fixed I2C address, no user-selectable jumper.
#define SCD41_I2C_ADDR   0x62

// DFRobot Gravity SGP40 (Sensirion silicon) — VOC raw signal.
// Fixed I2C address.
#define SGP40_I2C_ADDR   0x59

// DFRobot Gravity SEN0466 — factory-calibrated electrochemical CO sensor.
// Fixed I2C address (no jumper).
#define SEN0466_I2C_ADDR 0x74

typedef struct {
    uint16_t co2_ppm;       // 0-40000 ppm (NDIR)
    float    temperature_c; // °C
    float    humidity;      // %RH (0-100)
    bool     valid;
} scd41_data_t;

typedef struct {
    uint16_t voc_raw;    // raw SGP40 signal (~30000 clean-air baseline)
    uint16_t voc_index;  // 1-500 air-quality index (in-line EMA baseline)
    bool     valid;
} sgp40_data_t;

typedef struct {
    float ppm;   // CO concentration in ppm
    bool  valid;
} co_data_t;

/**
 * Attach the SCD41 to an existing I2C master bus (the same bus that hosts
 * the TCA9554 relay expander). Safe to call before scd41_start_periodic.
 * Returns ESP_ERR_INVALID_ARG if bus is NULL, or whatever the driver reports.
 */
esp_err_t sensors_init(i2c_master_bus_handle_t bus);

/**
 * Kick the SCD41 into periodic-measurement mode (fresh sample every 5 s).
 * Call once after sensors_init.
 */
esp_err_t scd41_start_periodic(void);

/**
 * Read the most recent measurement if one is ready. valid=false means "no
 * new data yet" — polling every 5 s guarantees a fresh sample per call once
 * the module warms up (~5 s after start_periodic).
 */
scd41_data_t scd41_read(void);

/**
 * Trigger an SGP40 measurement with humidity/temperature compensation and
 * return the raw VOC signal plus a running-baseline VOC index. Pass values
 * from the SCD41; pass humidity < 0 to fall back to the default 50 %RH /
 * 25 °C compensation reference.
 */
sgp40_data_t sgp40_measure(float humidity_pct, float temperature_c);

/**
 * Put the SEN0466 CO sensor into active (continuous) acquisition mode.
 * Call once after sensors_init. In active mode the sensor updates its
 * internal reading every ~1 s; each call to co_sen0466_read returns the
 * latest value.
 */
esp_err_t co_sen0466_set_active_mode(void);

/**
 * Read the latest CO concentration in ppm. valid=false means the sensor
 * didn't answer (offline, wire issue, or checksum mismatch).
 */
co_data_t co_sen0466_read(void);
