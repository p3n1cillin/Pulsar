#pragma once

#include "scope_types.h"

typedef void (*sim_radar_cb_t)(const airsoft_radar_frame_t *frame, void *context);

esp_err_t sim_radar_start(sim_radar_cb_t callback, void *context);
