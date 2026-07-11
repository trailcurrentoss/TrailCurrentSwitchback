#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    float    latitude;    // decimal degrees, negative = south
    float    longitude;   // decimal degrees, negative = west
    double   altitude;    // meters (from GGA)
    uint8_t  satellites;  // from GGA
    double   speed;       // knots (from RMC)
    double   course;      // degrees true (from RMC)
    uint8_t  gnss_mode;   // GGA fix quality (0=none, 1=GPS, 2=DGPS, etc.)
} gnss_data_t;

// Called from the gnss RX task after a full NMEA snapshot updates the
// internal cache. The callback runs on the gnss task and MUST NOT block
// (the CAN handler's publish is a spinlock — safe).
typedef void (*gnss_update_cb_t)(const gnss_data_t *data);

/**
 * Configure a UART for NMEA input and spawn the RX/parser task.
 * The DFRobot Gravity GNSS module streams NMEA at 9600 baud in UART mode.
 * on_update may be NULL if the caller only wants to poll via gnss_read().
 */
esp_err_t gnss_init(int uart_num, int tx_pin, int rx_pin, int baud_rate,
                    gnss_update_cb_t on_update);

/**
 * Snapshot the latest parsed fix into *data. Safe to call from any task.
 */
esp_err_t gnss_read(gnss_data_t *data);
