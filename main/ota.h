#pragma once

#include <stdint.h>
#include <stdbool.h>

// OTA update window duration (3 minutes)
#define OTA_TIMEOUT_MS 180000

/**
 * Initialize OTA subsystem.
 * Must be called after wifi_config_init().
 */
void ota_init(void);

/**
 * Handle a CAN OTA trigger message (ID 0x00).
 * Compares MAC bytes in data[0..2] against this device's MAC.
 * If matched, enters OTA mode: connects WiFi, starts HTTP server,
 * waits for firmware upload or timeout, then disconnects and returns.
 */
void ota_handle_trigger(const uint8_t *data, uint8_t len);

/**
 * Check whether OTA is currently in progress.
 * Used by discovery to enforce mutual exclusion.
 */
bool ota_is_running(void);
