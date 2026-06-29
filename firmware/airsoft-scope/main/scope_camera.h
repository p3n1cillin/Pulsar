#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t scope_camera_init(void);
bool scope_camera_ready(void);
bool scope_camera_grab_rgb565(uint16_t **pixels, int *width, int *height);
void scope_camera_release(void);
