#pragma once

#include <stdbool.h>
#include <stdint.h>

#define AIRSOFT_MAX_TARGETS 5
#define AIRSOFT_MAX_TRACKS 5
#define AIRSOFT_MAX_CAMERA_DETECTIONS 5

typedef enum {
    AIRSOFT_TRACK_SOURCE_NONE = 0,
    AIRSOFT_TRACK_SOURCE_RADAR = 1 << 0,
    AIRSOFT_TRACK_SOURCE_CAMERA = 1 << 1,
    AIRSOFT_TRACK_SOURCE_PREDICTED = 1 << 2,
} airsoft_track_source_t;

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

// A lightweight visual-motion observation. The camera supplies a reliable
// bearing and image-space location, but no range; range is acquired when a
// radar observation is associated with the same persistent track.
typedef struct {
    float bearing_deg;
    float confidence;
    int16_t center_x;
    int16_t center_y;
    int16_t width;
    int16_t height;
    bool valid;
} airsoft_camera_detection_t;

typedef struct {
    airsoft_camera_detection_t detections[AIRSOFT_MAX_CAMERA_DETECTIONS];
    uint8_t count;
    bool global_motion;
    uint64_t updated_us;
} airsoft_camera_frame_t;

typedef struct {
    uint8_t id;
    uint8_t source_flags;
    float x_m;
    float y_m;
    float velocity_x_mps;
    float velocity_y_mps;
    float range_m;
    float bearing_deg;
    float confidence;
    int16_t image_x;
    int16_t image_y;
    int16_t image_width;
    int16_t image_height;
    uint32_t age_ms;
    bool range_valid;
    bool valid;
} airsoft_track_t;

typedef struct {
    airsoft_track_t tracks[AIRSOFT_MAX_TRACKS];
    uint8_t count;
    uint64_t updated_us;
} airsoft_track_frame_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t box_width;
    int16_t box_height;
    float range_m;
    float bearing_deg;
    float confidence;
    uint8_t track_id;
    uint8_t source_flags;
    bool range_valid;
    bool offscreen;
    bool valid;
} airsoft_overlay_blip_t;

typedef struct {
    airsoft_overlay_blip_t blips[AIRSOFT_MAX_TARGETS];
    uint8_t count;
} airsoft_overlay_t;
