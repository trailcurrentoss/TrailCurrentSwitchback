#pragma once

#include "esp_err.h"

esp_err_t can_handler_init(void);
void can_handler_task(void *arg);
