#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "scope_controls_logic.h"

typedef struct {
    scope_settings_t settings;
    bool battery_valid;
    uint8_t battery_percent;
    uint16_t battery_mv;
} scope_controls_status_t;

// Load persisted settings from NVS, apply the saved backlight level, and start
// polling the housing buttons / battery monitor when they are enabled in
// menuconfig. Call after scope_ui_init (the backlight must exist).
esp_err_t scope_controls_init(void);

// Thread-safe snapshot of the current operator settings and battery state.
void scope_controls_get(scope_controls_status_t *out);
