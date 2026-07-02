#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "radar_overlay.h"
#include "target_tracker.h"

#define ASSERT_TRUE(condition) assert_true((condition), #condition, __LINE__)
#define ASSERT_NEAR(actual, expected, tolerance) \
    assert_near((actual), (expected), (tolerance), #actual, __LINE__)

static void assert_true(bool condition, const char *expression, int line) {
    if (!condition) {
        fprintf(stderr, "line %d: assertion failed: %s\n", line, expression);
        exit(EXIT_FAILURE);
    }
}

static void assert_near(
    float actual,
    float expected,
    float tolerance,
    const char *expression,
    int line) {
    if (fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "line %d: %s was %.4f, expected %.4f +/- %.4f\n",
                line, expression, actual, expected, tolerance);
        exit(EXIT_FAILURE);
    }
}

static target_tracker_config_t test_config(void) {
    const target_tracker_config_t config = {
        .radar_position_gate_m = 1.2f,
        .camera_bearing_gate_deg = 12.0f,
        .radar_half_fov_deg = 45.0f,
        .min_range_m = 1.2f,
        .max_range_m = 8.0f,
        .radar_boresight_deg = 0.0f,
        .radar_hold_ms = 500,
        .camera_hold_ms = 400,
        .coast_ms = 1600,
    };
    return config;
}

static airsoft_radar_frame_t radar_frame(float x_m, float y_m, uint64_t now_us) {
    airsoft_radar_frame_t frame = {0};
    frame.count = 1;
    frame.updated_us = now_us;
    frame.targets[0] = (airsoft_target_t){.x_m = x_m, .y_m = y_m, .valid = true};
    return frame;
}

static airsoft_camera_frame_t camera_frame(float bearing_deg, uint64_t now_us) {
    airsoft_camera_frame_t frame = {0};
    frame.count = 1;
    frame.updated_us = now_us;
    frame.detections[0] = (airsoft_camera_detection_t){
        .bearing_deg = bearing_deg,
        .confidence = 0.8f,
        .center_x = 160,
        .center_y = 100,
        .width = 30,
        .height = 60,
        .valid = true,
    };
    return frame;
}

static void test_radar_tracking_and_coasting(void) {
    target_tracker_t tracker;
    const target_tracker_config_t config = test_config();
    target_tracker_init(&tracker, &config);

    airsoft_track_frame_t output;
    airsoft_radar_frame_t radar = radar_frame(0.0f, 4.0f, 1000000);
    target_tracker_update(&tracker, &radar, NULL, 1000000, &output);
    ASSERT_TRUE(output.count == 1);
    const uint8_t id = output.tracks[0].id;
    ASSERT_TRUE(id != 0);
    ASSERT_TRUE(output.tracks[0].source_flags == AIRSOFT_TRACK_SOURCE_RADAR);

    radar = radar_frame(0.2f, 4.0f, 1100000);
    target_tracker_update(&tracker, &radar, NULL, 1100000, &output);
    ASSERT_TRUE(output.count == 1 && output.tracks[0].id == id);
    ASSERT_TRUE(output.tracks[0].velocity_x_mps > 0.0f);

    target_tracker_update(&tracker, NULL, NULL, 1700000, &output);
    ASSERT_TRUE(output.count == 1);
    ASSERT_TRUE(output.tracks[0].source_flags == AIRSOFT_TRACK_SOURCE_PREDICTED);

    target_tracker_update(&tracker, NULL, NULL, 2800000, &output);
    ASSERT_TRUE(output.count == 0);
}

static void test_camera_confirmation_and_count_clamp(void) {
    target_tracker_t tracker;
    const target_tracker_config_t config = test_config();
    target_tracker_init(&tracker, &config);

    airsoft_track_frame_t output;
    airsoft_camera_frame_t camera = camera_frame(5.0f, 1000000);
    camera.count = UINT8_MAX;
    target_tracker_update(&tracker, NULL, &camera, 1000000, &output);
    ASSERT_TRUE(output.count == 0);

    camera.count = 1;
    target_tracker_update(&tracker, NULL, &camera, 1100000, &output);
    ASSERT_TRUE(output.count == 1);
    ASSERT_TRUE(!output.tracks[0].range_valid);
    ASSERT_TRUE(output.tracks[0].source_flags == AIRSOFT_TRACK_SOURCE_CAMERA);
}

static void test_sensor_fusion_and_filtering(void) {
    target_tracker_t tracker;
    const target_tracker_config_t config = test_config();
    target_tracker_init(&tracker, &config);

    airsoft_track_frame_t output;
    airsoft_radar_frame_t radar = radar_frame(0.0f, 4.0f, 1000000);
    airsoft_camera_frame_t camera = camera_frame(1.0f, 1000000);
    target_tracker_update(&tracker, &radar, &camera, 1000000, &output);
    ASSERT_TRUE(output.count == 1);
    ASSERT_TRUE((output.tracks[0].source_flags & AIRSOFT_TRACK_SOURCE_RADAR) != 0);
    ASSERT_TRUE((output.tracks[0].source_flags & AIRSOFT_TRACK_SOURCE_CAMERA) != 0);

    target_tracker_init(&tracker, &config);
    radar = radar_frame(0.0f, 0.5f, 2000000);
    target_tracker_update(&tracker, &radar, NULL, 2000000, &output);
    ASSERT_TRUE(output.count == 0);
    radar = radar_frame(5.0f, 1.0f, 2100000);
    target_tracker_update(&tracker, &radar, NULL, 2100000, &output);
    ASSERT_TRUE(output.count == 0);
}

static void test_track_ids_do_not_collide_after_wrap(void) {
    target_tracker_t tracker;
    const target_tracker_config_t config = test_config();
    target_tracker_init(&tracker, &config);
    tracker.next_id = 1;
    tracker.slots[0] = (target_tracker_slot_t){
        .active = true,
        .range_valid = true,
        .id = 1,
        .x_m = -5.0f,
        .y_m = 4.0f,
        .created_us = 1000000,
        .last_predict_us = 1000000,
        .last_seen_us = 1000000,
    };

    airsoft_track_frame_t output;
    const airsoft_radar_frame_t radar = radar_frame(0.0f, 4.0f, 1000000);
    target_tracker_update(&tracker, &radar, NULL, 1000000, &output);
    ASSERT_TRUE(output.count == 1);
    ASSERT_TRUE(output.tracks[0].id == 2);
}

static void test_overlay_projection_and_reset(void) {
    airsoft_track_frame_t tracks = {0};
    tracks.count = 1;
    tracks.tracks[0] = (airsoft_track_t){
        .id = 7,
        .source_flags = AIRSOFT_TRACK_SOURCE_RADAR,
        .range_m = 4.0f,
        .bearing_deg = 0.0f,
        .confidence = 0.9f,
        .range_valid = true,
        .valid = true,
    };
    airsoft_overlay_t overlay;
    memset(&overlay, 0xA5, sizeof(overlay));
    radar_overlay_project(&tracks, 320, 240, 66.0f, 8.0f, &overlay);
    ASSERT_TRUE(overlay.count == 1);
    ASSERT_TRUE(overlay.blips[0].track_id == 7);
    ASSERT_NEAR((float)overlay.blips[0].x, 160.0f, 1.0f);
    ASSERT_NEAR((float)overlay.blips[0].y, 120.0f, 1.0f);
    ASSERT_TRUE(!overlay.blips[0].offscreen);
    ASSERT_TRUE(!overlay.blips[1].valid);

    tracks.tracks[0].bearing_deg = 50.0f;
    radar_overlay_project(&tracks, 320, 240, 66.0f, 8.0f, &overlay);
    ASSERT_TRUE(overlay.blips[0].offscreen);
    radar_overlay_project(&tracks, 320, 240, 66.0f, 8.0f, NULL);
}

int main(void) {
    test_radar_tracking_and_coasting();
    test_camera_confirmation_and_count_clamp();
    test_sensor_fusion_and_filtering();
    test_track_ids_do_not_collide_after_wrap();
    test_overlay_projection_and_reset();
    puts("tracking tests passed");
    return EXIT_SUCCESS;
}
