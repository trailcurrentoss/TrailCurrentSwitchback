#pragma once

#include "esp_err.h"

#ifdef SWITCHBACK_VARIANT_GNSS
#include "gnss.h"
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
