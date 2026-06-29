#pragma once

#include "scope_types.h"

void radar_overlay_project(
    const airsoft_radar_frame_t *frame,
    int display_width,
    int display_height,
    float camera_hfov_deg,
    float radar_hfov_deg,
    float max_range_m,
    airsoft_overlay_t *overlay);
