#pragma once

#include <stdbool.h>
#include <stdint.h>

#define AIRSOFT_MAX_TARGETS 5

typedef struct {
    float x_m;
    float y_m;
    bool valid;
} airsoft_target_t;

typedef struct {
    airsoft_target_t targets[AIRSOFT_MAX_TARGETS];
    uint8_t count;
    bool region_present[3];
    bool has_regions;
    uint64_t updated_us;
} airsoft_radar_frame_t;

typedef struct {
    int16_t x;
    int16_t y;
    float range_m;
    float bearing_deg;
    bool valid;
} airsoft_overlay_blip_t;

typedef struct {
    airsoft_overlay_blip_t blips[AIRSOFT_MAX_TARGETS];
    uint8_t count;
} airsoft_overlay_t;
