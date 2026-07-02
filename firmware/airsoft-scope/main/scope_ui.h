#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "scope_types.h"

esp_err_t scope_ui_init(int width, int height);
void scope_ui_composite(
    const airsoft_overlay_t *overlay,
    uint16_t *live_frame,
    int live_width,
    int live_height);
// Render the radar-only scope view (reticle + contacts) to the display.
// Used when no camera frame is available.
void scope_ui_render_scope(const airsoft_overlay_t *overlay);

// Lock and return the last framebuffer sent to the panel (RGB565). The caller
// must call scope_ui_panel_snapshot_release() after it finishes reading.
bool scope_ui_panel_snapshot_acquire(const uint16_t **buf, int *w, int *h);
void scope_ui_panel_snapshot_release(void);

void scope_ui_serial_preview(const airsoft_overlay_t *overlay);
