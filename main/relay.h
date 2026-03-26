#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

esp_err_t relay_init(void);
esp_err_t relay_set(uint8_t channel, bool state);
esp_err_t relay_toggle(uint8_t channel);
esp_err_t relay_set_all(bool state);
uint8_t   relay_get_states(void);
