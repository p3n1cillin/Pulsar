#pragma once

#include "scope_types.h"

// Project tracks into screen space using the pinhole model in camera_model.h.
// x follows bearing; y follows range (near = bottom) when range is known, or
// the camera's image row for bearing-only tracks.
void radar_overlay_project(
    const airsoft_track_frame_t *frame,
    int display_width,
    int display_height,
    float camera_hfov_deg,
    float max_range_m,
    airsoft_overlay_t *overlay);

// Keep only live radar-backed contacts (drop camera-only and coasting tracks).
// Used by the operator's "declutter" view in busy scenes.
void radar_overlay_declutter(airsoft_overlay_t *overlay);
