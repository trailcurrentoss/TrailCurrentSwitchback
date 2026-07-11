#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdbool.h>
#include <stdint.h>

esp_err_t relay_init(void);
esp_err_t relay_set(uint8_t channel, bool state);
esp_err_t relay_toggle(uint8_t channel);
esp_err_t relay_set_all(bool state);
uint8_t   relay_get_states(void);

// Shared I2C master bus (TCA9554 relay expander lives here). Returns NULL
// before relay_init(). Used by the picket_gnss variant's SCD41 attachment
// so both peripherals hang off a single bus.
i2c_master_bus_handle_t relay_get_i2c_bus(void);
