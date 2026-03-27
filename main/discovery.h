#pragma once

#include <stdint.h>

// Discovery window duration (3 minutes)
#define DISCOVERY_TIMEOUT_MS 180000

/**
 * Initialize discovery subsystem -- load configured flag from NVS.
 * Must be called after wifi_config_init().
 */
void discovery_init(void);

/**
 * Handle a CAN discovery trigger (ID 0x02, broadcast).
 * If this module is unconfigured and has WiFi credentials,
 * joins WiFi, advertises via mDNS with module metadata,
 * and waits for Headwaters to confirm registration.
 */
void discovery_handle_trigger(void);

