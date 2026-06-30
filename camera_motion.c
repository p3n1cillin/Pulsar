#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"

#include "camera_motion.h"

static uint8_t rgb565_luma(uint16_t pixel) {
    const unsigned r = ((pixel >> 11) & 0x1F) * 255U / 31U;
    const unsigned g = ((pixel >> 5) & 0x3F) * 255U / 63U;
    const unsigned b = (pixel & 0x1F) * 255U / 31U;
    return (uint8_t)((77U * r + 150U * g + 29U * b) >> 8);
}

static int clamp_int(int value, int low, int high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float clamp_float(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void sample_luma(
    const uint16_t *pixels,
    int width,
    int height,
    uint8_t *out) {
    const int half_step_x = width / (CAMERA_MOTION_GRID_WIDTH * 4);
    const int half_step_y = height / (CAMERA_MOTION_GRID_HEIGHT * 4);

    for (int gy = 0; gy < CAMERA_MOTION_GRID_HEIGHT; ++gy) {
        const int cy = ((2 * gy + 1) * height) / (2 * CAMERA_MOTION_GRID_HEIGHT);
        for (int gx = 0; gx < CAMERA_MOTION_GRID_WIDTH; ++gx) {
            const int cx = ((2 * gx + 1) * width) / (2 * CAMERA_MOTION_GRID_WIDTH);
            const int x0 = clamp_int(cx - half_step_x, 0, width - 1);
            const int x1 = clamp_int(cx + half_step_x, 0, width - 1);
            const int y0 = clamp_int(cy - half_step_y, 0, height - 1);
            const int y1 = clamp_int(cy + half_step_y, 0, height - 1);
            const unsigned sum =
                rgb565_luma(pixels[(size_t)y0 * (size_t)width + (size_t)x0]) +
                rgb565_luma(pixels[(size_t)y0 * (size_t)width + (size_t)x1]) +
                rgb565_luma(pixels[(size_t)y1 * (size_t)width + (size_t)x0]) +
                rgb565_luma(pixels[(size_t)y1 * (size_t)width + (size_t)x1]);
            out[gy * CAMERA_MOTION_GRID_WIDTH + gx] = (uint8_t)(sum / 4U);
        }
    }
}

static void insert_detection(
    airsoft_camera_frame_t *out,
    const airsoft_camera_detection_t *detection) {
    uint8_t slot = out->count;
    if (slot < AIRSOFT_MAX_CAMERA_DETECTIONS) {
        ++out->count;
    } else if (detection->confidence <= out->detections[slot - 1].confidence) {
        return;
    } else {
        slot = AIRSOFT_MAX_CAMERA_DETECTIONS - 1;
    }

    while (slot > 0 &&
           detection->confidence > out->detections[slot - 1].confidence) {
        if (slot < AIRSOFT_MAX_CAMERA_DETECTIONS) {
            out->detections[slot] = out->detections[slot - 1];
        }
        --slot;
    }
    out->detections[slot] = *detection;
}

void camera_motion_init(camera_motion_state_t *state) {
    if (state) {
        memset(state, 0, sizeof(*state));
    }
}

void camera_motion_process(
    camera_motion_state_t *state,
    const uint16_t *pixels,
    int width,
    int height,
    float camera_hfov_deg,
    uint64_t now_us,
    airsoft_camera_frame_t *out) {
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->updated_us = now_us;

#if !CONFIG_AIRSOFT_CAMERA_MOTION_TRACKING
    (void)state;
    (void)pixels;
    (void)width;
    (void)height;
    (void)camera_hfov_deg;
    return;
#else
    if (!state || !pixels || width < CAMERA_MOTION_GRID_WIDTH ||
        height < CAMERA_MOTION_GRID_HEIGHT) {
        return;
    }

    sample_luma(pixels, width, height, state->current_luma);
    if (!state->initialized) {
        memcpy(state->previous_luma, state->current_luma, CAMERA_MOTION_GRID_CELLS);
        state->initialized = true;
        return;
    }

    // Remove the average brightness change before thresholding. This handles
    // most camera auto-exposure steps without mistaking them for a player.
    int delta_sum = 0;
    for (int i = 0; i < CAMERA_MOTION_GRID_CELLS; ++i) {
        delta_sum += (int)state->current_luma[i] - (int)state->previous_luma[i];
    }
    const int brightness_delta = delta_sum / CAMERA_MOTION_GRID_CELLS;

    int active_cells = 0;
    for (int i = 0; i < CAMERA_MOTION_GRID_CELLS; ++i) {
        const int compensated =
            ((int)state->current_luma[i] - (int)state->previous_luma[i]) - brightness_delta;
        const bool active = abs(compensated) >= CONFIG_AIRSOFT_CAMERA_MOTION_THRESHOLD;
        state->motion_mask[i] = active ? 1 : 0;
        active_cells += active ? 1 : 0;
    }
    memcpy(state->previous_luma, state->current_luma, CAMERA_MOTION_GRID_CELLS);

    const int active_percent = (active_cells * 100) / CAMERA_MOTION_GRID_CELLS;
    if (active_percent >= CONFIG_AIRSOFT_CAMERA_GLOBAL_MOTION_PERCENT) {
        out->global_motion = true;
        return;
    }

    // A one-cell dilation joins the two moving edges of the same person and
    // makes connected components much less sensitive to clothing texture.
    memset(state->expanded_mask, 0, CAMERA_MOTION_GRID_CELLS);
    for (int gy = 0; gy < CAMERA_MOTION_GRID_HEIGHT; ++gy) {
        for (int gx = 0; gx < CAMERA_MOTION_GRID_WIDTH; ++gx) {
            bool near_motion = false;
            for (int dy = -1; dy <= 1 && !near_motion; ++dy) {
                const int ny = gy + dy;
                if (ny < 0 || ny >= CAMERA_MOTION_GRID_HEIGHT) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = gx + dx;
                    if (nx < 0 || nx >= CAMERA_MOTION_GRID_WIDTH) continue;
                    if (state->motion_mask[ny * CAMERA_MOTION_GRID_WIDTH + nx]) {
                        near_motion = true;
                        break;
                    }
                }
            }
            state->expanded_mask[gy * CAMERA_MOTION_GRID_WIDTH + gx] = near_motion ? 1 : 0;
        }
    }

    memset(state->visited, 0, CAMERA_MOTION_GRID_CELLS);
    static const int neighbor_x[4] = {-1, 1, 0, 0};
    static const int neighbor_y[4] = {0, 0, -1, 1};

    for (int start = 0; start < CAMERA_MOTION_GRID_CELLS; ++start) {
        if (!state->expanded_mask[start] || state->visited[start]) {
            continue;
        }

        int head = 0;
        int tail = 0;
        state->queue[tail++] = (uint16_t)start;
        state->visited[start] = 1;
        int min_x = CAMERA_MOTION_GRID_WIDTH;
        int max_x = 0;
        int min_y = CAMERA_MOTION_GRID_HEIGHT;
        int max_y = 0;
        int raw_points = 0;
        int sum_x = 0;
        int sum_y = 0;

        while (head < tail) {
            const int cell = state->queue[head++];
            const int x = cell % CAMERA_MOTION_GRID_WIDTH;
            const int y = cell / CAMERA_MOTION_GRID_WIDTH;
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;
            if (state->motion_mask[cell]) {
                ++raw_points;
                sum_x += x;
                sum_y += y;
            }

            for (int n = 0; n < 4; ++n) {
                const int nx = x + neighbor_x[n];
                const int ny = y + neighbor_y[n];
                if (nx < 0 || nx >= CAMERA_MOTION_GRID_WIDTH ||
                    ny < 0 || ny >= CAMERA_MOTION_GRID_HEIGHT) {
                    continue;
                }
                const int next = ny * CAMERA_MOTION_GRID_WIDTH + nx;
                if (state->expanded_mask[next] && !state->visited[next]) {
                    state->visited[next] = 1;
                    state->queue[tail++] = (uint16_t)next;
                }
            }
        }

        const int blob_width = max_x - min_x + 1;
        const int blob_height = max_y - min_y + 1;
        if (raw_points < CONFIG_AIRSOFT_CAMERA_MIN_MOTION_CELLS ||
            blob_width < 2 || blob_height < 2) {
            continue;
        }

        const float grid_center_x = raw_points > 0
            ? (float)sum_x / (float)raw_points + 0.5f
            : ((float)min_x + (float)max_x + 1.0f) * 0.5f;
        const float grid_center_y = raw_points > 0
            ? (float)sum_y / (float)raw_points + 0.5f
            : ((float)min_y + (float)max_y + 1.0f) * 0.5f;
        airsoft_camera_detection_t detection = {
            .bearing_deg =
                (grid_center_x / (float)CAMERA_MOTION_GRID_WIDTH - 0.5f) * camera_hfov_deg,
            .confidence = 0.35f + 0.65f * clamp_float(
                (float)raw_points /
                    (float)(CONFIG_AIRSOFT_CAMERA_MIN_MOTION_CELLS * 6),
                0.0f,
                1.0f),
            .center_x = (int16_t)lroundf(
                grid_center_x * (float)width / (float)CAMERA_MOTION_GRID_WIDTH),
            .center_y = (int16_t)lroundf(
                grid_center_y * (float)height / (float)CAMERA_MOTION_GRID_HEIGHT),
            .width = (int16_t)((blob_width * width) / CAMERA_MOTION_GRID_WIDTH),
            .height = (int16_t)((blob_height * height) / CAMERA_MOTION_GRID_HEIGHT),
            .valid = true,
        };
        insert_detection(out, &detection);
    }
#endif
}

