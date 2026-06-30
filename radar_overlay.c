#include <math.h>

#include "radar_overlay.h"
#include "sdkconfig.h"

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
    overlay->count = 0;

    if (!frame || display_width <= 0 || display_height <= 0 ||
        camera_hfov_deg <= 0.0f || max_range_m <= 0.0f) {
        return;
    }

    const float half_camera = camera_hfov_deg * 0.5f;
    const float pixels_per_deg = (float)display_width / camera_hfov_deg;

    for (uint8_t i = 0; i < frame->count && overlay->count < AIRSOFT_MAX_TARGETS; ++i) {
        const airsoft_track_t *track = &frame->tracks[i];
        if (!track->valid) {
            continue;
        }

        airsoft_overlay_blip_t *blip = &overlay->blips[overlay->count++];
        const float projected_x = (track->bearing_deg + half_camera) * pixels_per_deg;
        blip->offscreen = projected_x < 0.0f || projected_x >= (float)display_width;
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
