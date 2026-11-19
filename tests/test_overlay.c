#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "camera_model.h"
#include "overlay_draw.h"
#include "radar_overlay.h"

#define ASSERT_TRUE(condition) assert_true((condition), #condition, __LINE__)
#define ASSERT_NEAR(actual, expected, tolerance) \
    assert_near((actual), (expected), (tolerance), #actual, __LINE__)

#define W 320
#define H 240

static void assert_true(bool condition, const char *expression, int line) {
    if (!condition) {
        fprintf(stderr, "line %d: assertion failed: %s\n", line, expression);
        exit(EXIT_FAILURE);
    }
}

static void assert_near(
    float actual, float expected, float tolerance, const char *expression, int line) {
    if (fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "line %d: %s was %.4f, expected %.4f +/- %.4f\n",
                line, expression, actual, expected, tolerance);
        exit(EXIT_FAILURE);
    }
}

static airsoft_track_frame_t one_track(float bearing_deg, uint8_t flags) {
    airsoft_track_frame_t tracks = {0};
    tracks.count = 1;
    tracks.tracks[0] = (airsoft_track_t){
        .id = 3,
        .source_flags = flags,
        .range_m = 4.0f,
        .bearing_deg = bearing_deg,
        .confidence = 0.8f,
        .range_valid = true,
        .valid = true,
    };
    return tracks;
}

static void test_camera_model_round_trip(void) {
    for (float bearing = -32.0f; bearing <= 32.0f; bearing += 4.0f) {
        float x = 0.0f;
        ASSERT_TRUE(camera_model_bearing_to_x(bearing, W, 66.0f, &x));
        ASSERT_NEAR(camera_model_x_to_bearing(x, W, 66.0f), bearing, 0.01f);
    }
    // Half the HFOV lands exactly on the frame edge.
    float edge = 0.0f;
    camera_model_bearing_to_x(33.0f, W, 66.0f, &edge);
    ASSERT_NEAR(edge, (float)W, 0.01f);
    float behind = 0.0f;
    ASSERT_TRUE(!camera_model_bearing_to_x(95.0f, W, 66.0f, &behind));
    ASSERT_TRUE(behind > (float)W);
    ASSERT_TRUE(!camera_model_bearing_to_x(-120.0f, W, 66.0f, &behind));
    ASSERT_TRUE(behind < 0.0f);
}

static void test_off_axis_projection_is_rectilinear(void) {
    airsoft_overlay_t overlay;
    airsoft_track_frame_t tracks = one_track(25.0f, AIRSOFT_TRACK_SOURCE_RADAR);
    radar_overlay_project(&tracks, W, H, 66.0f, 8.0f, &overlay);
    // Pinhole: 160 + f * tan(25 deg), f = 160 / tan(33 deg) -> ~274.9 px.
    // The old linear mapping put this at ~281 px.
    ASSERT_NEAR((float)overlay.blips[0].x, 274.9f, 1.0f);
    ASSERT_TRUE(!overlay.blips[0].offscreen);

    tracks = one_track(-25.0f, AIRSOFT_TRACK_SOURCE_RADAR);
    radar_overlay_project(&tracks, W, H, 66.0f, 8.0f, &overlay);
    ASSERT_NEAR((float)overlay.blips[0].x, 45.1f, 1.0f);

    // Beyond 90 deg must not wrap around through tan() onto the screen.
    tracks = one_track(-100.0f, AIRSOFT_TRACK_SOURCE_RADAR);
    radar_overlay_project(&tracks, W, H, 66.0f, 8.0f, &overlay);
    ASSERT_TRUE(overlay.blips[0].offscreen);
    ASSERT_TRUE(overlay.blips[0].x < W / 2);
}

static void test_declutter_keeps_live_radar_contacts(void) {
    airsoft_track_frame_t tracks = {0};
    const uint8_t flags[4] = {
        AIRSOFT_TRACK_SOURCE_CAMERA,
        AIRSOFT_TRACK_SOURCE_RADAR,
        AIRSOFT_TRACK_SOURCE_RADAR | AIRSOFT_TRACK_SOURCE_PREDICTED,
        AIRSOFT_TRACK_SOURCE_RADAR | AIRSOFT_TRACK_SOURCE_CAMERA,
    };
    tracks.count = 4;
    for (int i = 0; i < 4; ++i) {
        const airsoft_track_frame_t single = one_track(0.0f, flags[i]);
        tracks.tracks[i] = single.tracks[0];
        tracks.tracks[i].id = (uint8_t)(i + 1);
    }
    airsoft_overlay_t overlay;
    radar_overlay_project(&tracks, W, H, 66.0f, 8.0f, &overlay);
    ASSERT_TRUE(overlay.count == 4);
    radar_overlay_declutter(&overlay);
    ASSERT_TRUE(overlay.count == 2);
    ASSERT_TRUE(overlay.blips[0].track_id == 2);
    ASSERT_TRUE(overlay.blips[1].track_id == 4);
    ASSERT_TRUE(!overlay.blips[2].valid && !overlay.blips[3].valid);
    radar_overlay_declutter(NULL);
}

static int count_changed(const uint16_t *fb, int n, uint16_t background) {
    int changed = 0;
    for (int i = 0; i < n; ++i) {
        changed += fb[i] != background;
    }
    return changed;
}

static void test_hud_draws_and_stays_in_bounds(void) {
    // Guard bands before/after the framebuffer catch any out-of-bounds write.
    enum { GUARD = 512 };
    uint16_t *mem = malloc(sizeof(uint16_t) * (W * H + 2 * GUARD));
    ASSERT_TRUE(mem != NULL);
    for (int i = 0; i < W * H + 2 * GUARD; ++i) mem[i] = 0x1234;
    uint16_t *fb = mem + GUARD;

    scope_hud_t hud = {0};
    overlay_draw_hud(fb, W, H, &hud);
    ASSERT_TRUE(count_changed(fb, W * H, 0x1234) == 0);  // nothing to show

    hud = (scope_hud_t){
        .radar_lost = true,
        .blink_phase = true,
        .calibrating = true,
        .boresight_deg = -2.5f,
        .declutter = true,
        .battery_valid = true,
        .battery_percent = 64,
    };
    overlay_draw_hud(fb, W, H, &hud);
    ASSERT_TRUE(count_changed(fb, W * H, 0x1234) > 500);
    // Radar-lost banner occupies the top centre.
    ASSERT_TRUE(fb[10 * W + W / 2] != 0x1234);

    // Contacts at extreme positions on a tiny frame must clip, not overflow.
    airsoft_overlay_t overlay = {0};
    overlay.count = 2;
    overlay.blips[0] = (airsoft_overlay_blip_t){
        .x = 0, .y = 0, .range_m = 1.0f, .range_valid = true, .valid = true,
        .source_flags = AIRSOFT_TRACK_SOURCE_RADAR | AIRSOFT_TRACK_SOURCE_CAMERA,
        .box_width = 400, .box_height = 400, .track_id = 9};
    overlay.blips[1] = (airsoft_overlay_blip_t){
        .x = 30, .y = 20, .offscreen = true, .valid = true,
        .source_flags = AIRSOFT_TRACK_SOURCE_RADAR, .track_id = 1};
    overlay_draw_contacts(fb, 32, 24, &overlay, 8.0f);
    overlay_draw_hud(fb, 32, 24, &hud);

    for (int i = 0; i < GUARD; ++i) {
        ASSERT_TRUE(mem[i] == 0x1234);
        ASSERT_TRUE(mem[GUARD + W * H + i] == 0x1234);
    }
    free(mem);
}

static void test_text_width(void) {
    ASSERT_TRUE(overlay_text_width("", 2) == 0);
    ASSERT_TRUE(overlay_text_width("A", 1) == 3);
    ASSERT_TRUE(overlay_text_width("NO RADAR", 2) == 8 * 8 - 2);
    ASSERT_TRUE(overlay_text_width(NULL, 1) == 0);
}

int main(void) {
    test_camera_model_round_trip();
    test_off_axis_projection_is_rectilinear();
    test_declutter_keeps_live_radar_contacts();
    test_hud_draws_and_stays_in_bounds();
    test_text_width();
    puts("overlay tests passed");
    return EXIT_SUCCESS;
}
