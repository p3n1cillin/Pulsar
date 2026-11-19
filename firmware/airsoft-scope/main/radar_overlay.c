#include <math.h>
#include <string.h>

#include "camera_model.h"
#include "radar_overlay.h"

static float clampf(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void radar_overlay_project(
    const airsoft_track_frame_t *frame,
    int display_width,
    int display_height,
    float camera_hfov_deg,
    float max_range_m,
    airsoft_overlay_t *overlay) {
    if (!overlay) {
        return;
    }
    memset(overlay, 0, sizeof(*overlay));

    if (!frame || display_width <= 0 || display_height <= 0 ||
        camera_hfov_deg <= 0.0f || max_range_m <= 0.0f) {
        return;
    }

    for (uint8_t i = 0; i < frame->count && overlay->count < AIRSOFT_MAX_TARGETS; ++i) {
        const airsoft_track_t *track = &frame->tracks[i];
        if (!track->valid) {
            continue;
        }

        airsoft_overlay_blip_t *blip = &overlay->blips[overlay->count++];
        float projected_x = 0.0f;
        const bool projectable = camera_model_bearing_to_x(
            track->bearing_deg, display_width, camera_hfov_deg, &projected_x);
        blip->offscreen =
            !projectable || projected_x < 0.0f || projected_x >= (float)display_width;
        blip->x = (int16_t)lroundf(clampf(projected_x, 7.0f, (float)display_width - 8.0f));
        if (track->range_valid) {
            blip->y = (int16_t)lroundf(
                (float)display_height *
                (1.0f - clampf(track->range_m / max_range_m, 0.0f, 1.0f)));
        } else {
            blip->y = (int16_t)lroundf(clampf(
                (float)track->image_y, 7.0f, (float)display_height - 8.0f));
        }
        blip->box_width = track->image_width;
        blip->box_height = track->image_height;
        blip->bearing_deg = track->bearing_deg;
        blip->range_m = track->range_m;
        blip->confidence = track->confidence;
        blip->track_id = track->id;
        blip->source_flags = track->source_flags;
        blip->range_valid = track->range_valid;
        blip->valid = true;
    }
}

void radar_overlay_declutter(airsoft_overlay_t *overlay) {
    if (!overlay) {
        return;
    }
    uint8_t kept = 0;
    for (uint8_t i = 0; i < overlay->count; ++i) {
        const airsoft_overlay_blip_t *blip = &overlay->blips[i];
        const bool radar_backed = (blip->source_flags & AIRSOFT_TRACK_SOURCE_RADAR) != 0;
        const bool predicted = (blip->source_flags & AIRSOFT_TRACK_SOURCE_PREDICTED) != 0;
        if (blip->valid && radar_backed && !predicted) {
            overlay->blips[kept++] = *blip;
        }
    }
    for (uint8_t i = kept; i < overlay->count; ++i) {
        memset(&overlay->blips[i], 0, sizeof(overlay->blips[i]));
    }
    overlay->count = kept;
}
