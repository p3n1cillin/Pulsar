#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "scope_types.h"

typedef void (*ld2450_frame_cb_t)(const airsoft_radar_frame_t *frame, void *context);

typedef struct {
    ld2450_frame_cb_t callback;
    void *context;
} ld2450_config_t;

// Start the LD2450 UART reader. The sensor streams target reports on its own,
// so no mode command is required. Targets are delivered via config->callback.
esp_err_t ld2450_init(const ld2450_config_t *config);
