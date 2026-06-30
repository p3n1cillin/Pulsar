#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "scope_types.h"

// QVGA is reduced to this small grid before differencing. Keeping the detector
// fixed-size makes its CPU and memory cost predictable on the ESP32-S3.
#define CAMERA_MOTION_GRID_WIDTH 40
#define CAMERA_MOTION_GRID_HEIGHT 30
#define CAMERA_MOTION_GRID_CELLS (CAMERA_MOTION_GRID_WIDTH * CAMERA_MOTION_GRID_HEIGHT)

typedef struct {
    uint8_t previous_luma[CAMERA_MOTION_GRID_CELLS];
    uint8_t current_luma[CAMERA_MOTION_GRID_CELLS];
    uint8_t motion_mask[CAMERA_MOTION_GRID_CELLS];
    uint8_t expanded_mask[CAMERA_MOTION_GRID_CELLS];
    uint8_t visited[CAMERA_MOTION_GRID_CELLS];
    uint16_t queue[CAMERA_MOTION_GRID_CELLS];
    bool initialized;
} camera_motion_state_t;

void camera_motion_init(camera_motion_state_t *state);

// Find moving visual regions and turn them into bearing-only observations.
// Whole-frame changes (scope movement or auto-exposure jumps) are flagged and
// suppressed so they do not create a screen full of false player tracks.
void camera_motion_process(
    camera_motion_state_t *state,
    const uint16_t *pixels,
    int width,
    int height,
    float camera_hfov_deg,
    uint64_t now_us,
    airsoft_camera_frame_t *out);

