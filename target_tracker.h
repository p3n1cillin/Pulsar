#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "scope_types.h"

typedef struct {
    float radar_position_gate_m;
    float camera_bearing_gate_deg;
    float radar_half_fov_deg;
    float min_range_m;
    float max_range_m;
    float radar_boresight_deg;
    uint32_t radar_hold_ms;
    uint32_t camera_hold_ms;
    uint32_t coast_ms;
} target_tracker_config_t;

typedef struct {
    bool active;
    bool range_valid;
    uint8_t id;
    uint16_t radar_hits;
    uint16_t camera_hits;
    float x_m;
    float y_m;
    float velocity_x_mps;
    float velocity_y_mps;
    float bearing_deg;
    float bearing_velocity_dps;
    float confidence;
    int16_t image_x;
    int16_t image_y;
    int16_t image_width;
    int16_t image_height;
    uint64_t created_us;
    uint64_t last_predict_us;
    uint64_t last_seen_us;
    uint64_t radar_seen_us;
    uint64_t camera_seen_us;
} target_tracker_slot_t;

typedef struct {
    target_tracker_config_t config;
    target_tracker_slot_t slots[AIRSOFT_MAX_TRACKS];
    uint8_t next_id;
} target_tracker_t;

void target_tracker_init(
    target_tracker_t *tracker,
    const target_tracker_config_t *config);

// Each non-NULL input is treated as one new sensor observation. Call with NULL
// for a sensor that has not produced a fresh frame since the previous update.
void target_tracker_update(
    target_tracker_t *tracker,
    const airsoft_radar_frame_t *radar,
    const airsoft_camera_frame_t *camera,
    uint64_t now_us,
    airsoft_track_frame_t *out);

