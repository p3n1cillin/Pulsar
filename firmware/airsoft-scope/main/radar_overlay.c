#include <math.h>

#include "radar_overlay.h"
#include "sdkconfig.h"

static float clampf(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void radar_overlay_project(
    const airsoft_radar_frame_t *frame,
    int display_width,
    int display_height,
    float camera_hfov_deg,
    float radar_hfov_deg,
    float max_range_m,
    airsoft_overlay_t *overlay) {
    overlay->count = 0;

    if (!frame) {
        return;
    }

    const float min_range_m = (float)CONFIG_AIRSOFT_MIN_RANGE_M / 100.0f;
    const float half_camera = camera_hfov_deg * 0.5f;
    const float half_radar = radar_hfov_deg * 0.5f;
    const float pixels_per_deg = (float)display_width / camera_hfov_deg;

    for (uint8_t i = 0; i < frame->count && overlay->count < AIRSOFT_MAX_TARGETS; ++i) {
        if (!frame->targets[i].valid) {
            continue;
        }

        const float x = frame->targets[i].x_m;
        const float y = frame->targets[i].y_m;
        const float range = hypotf(x, y);
        if (range < min_range_m || range > max_range_m) {
            continue;
        }

        const float bearing = atan2f(x, y) * (180.0f / (float)M_PI);
        if (fabsf(bearing) > half_radar) {
            continue;
        }

        airsoft_overlay_blip_t *blip = &overlay->blips[overlay->count++];
        blip->bearing_deg = bearing;
        blip->range_m = range;
        blip->x = (int16_t)lroundf((bearing + half_camera) * pixels_per_deg);
        blip->y = (int16_t)lroundf((float)display_height * (1.0f - clampf(range / max_range_m, 0.0f, 1.0f)));
        blip->valid = true;
    }
}
