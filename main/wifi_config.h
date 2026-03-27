#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

esp_err_t wifi_config_init(void);
bool wifi_config_load(char *ssid, size_t ssid_len, char *password, size_t pass_len);
bool wifi_config_has_credentials(void);
const char *wifi_config_get_hostname(void);
bool wifi_connect(void);
void wifi_disconnect(void);
void wifi_config_handle_can(const uint8_t *data, uint8_t length);
void wifi_config_check_timeout(void);
