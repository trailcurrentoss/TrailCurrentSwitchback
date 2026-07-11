#pragma once

#include "esp_err.h"

#ifdef SWITCHBACK_VARIANT_GNSS
#include "gnss.h"
#endif
#ifdef SWITCHBACK_VARIANT_SCD41
#include "sensors.h"
#endif

esp_err_t can_handler_init(void);
void can_handler_task(void *arg);

#ifdef SWITCHBACK_VARIANT_GNSS
// Called by the GNSS poll task after each successful read. Snapshots the
// fields into a spinlock-protected buffer that can_handler_task drains at
// its own TX cadence, so torn reads across minute/second rollovers don't
// leak onto the CAN bus.
void can_handler_publish_gnss(const gnss_data_t *data);
#endif

#ifdef SWITCHBACK_VARIANT_SCD41
// Called by the SCD41 poll task after each successful measurement. Same
// snapshot pattern as the GNSS publisher.
void can_handler_publish_env(const scd41_data_t *data);

// Called after each SGP40 measurement. Updates only the VOC bytes of the
// environmental snapshot — the 0x1F frame is still gated by SCD41 validity.
void can_handler_publish_voc(const sgp40_data_t *data);

// Called after each SEN0466 CO measurement. Updates the Safety frame (0x20).
void can_handler_publish_co(const co_data_t *data);
#endif
