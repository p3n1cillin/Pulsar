#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "camera_motion.h"

#define ASSERT_TRUE(condition) assert_true((condition), #condition, __LINE__)
#define ASSERT_NEAR(actual, expected, tolerance) \
    assert_near((actual), (expected), (tolerance), #actual, __LINE__)

#define WIDTH 320   // QVGA: 8 px per grid cell in both axes
#define HEIGHT 240

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

static uint16_t *make_frame(uint16_t color) {
    uint16_t *pixels = malloc((size_t)WIDTH * (size_t)HEIGHT * sizeof(uint16_t));
    if (!pixels) {
        fprintf(stderr, "out of memory\n");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < WIDTH * HEIGHT; ++i) {
        pixels[i] = color;
    }
    return pixels;
}

// Fill grid-cell-aligned rectangles; cell (gx, gy) covers pixels
// [8*gx, 8*gx+7] x [8*gy, 8*gy+7].
static void fill_cells(
    uint16_t *pixels,
    int gx0,
    int gy0,
    int gx1_inclusive,
    int gy1_inclusive,
    uint16_t color) {
    for (int y = gy0 * 8; y < (gy1_inclusive + 1) * 8; ++y) {
        for (int x = gx0 * 8; x < (gx1_inclusive + 1) * 8; ++x) {
            pixels[y * WIDTH + x] = color;
        }
    }
}

static void run_two_frames(
    camera_motion_state_t *state,
    uint16_t *first,
    uint16_t *second,
    airsoft_camera_frame_t *out) {
    camera_motion_init(state);
    camera_motion_process(state, first, WIDTH, HEIGHT, 66.0f, 1000000, out);
    camera_motion_process(state, second, WIDTH, HEIGHT, 66.0f, 1100000, out);
}

static void test_static_scene_yields_nothing(void) {
    camera_motion_state_t state;
    airsoft_camera_frame_t out;
    uint16_t *frame = make_frame(0x0000);
    run_two_frames(&state, frame, frame, &out);
    ASSERT_TRUE(out.count == 0);
    ASSERT_TRUE(!out.global_motion);
    free(frame);
}

static void test_moving_square_is_detected_centred(void) {
    camera_motion_state_t state;
    airsoft_camera_frame_t out;
    uint16_t *dark = make_frame(0x0000);
    uint16_t *with_player = make_frame(0x0000);
    // 4x4 cells centred on the grid: raw-point mean lands on grid centre.
    fill_cells(with_player, 18, 13, 21, 16, 0xFFFF);

    run_two_frames(&state, dark, with_player, &out);
    ASSERT_TRUE(out.count == 1);
    ASSERT_TRUE(out.detections[0].valid);
    ASSERT_NEAR(out.detections[0].bearing_deg, 0.0f, 5.0f);
    ASSERT_TRUE(out.detections[0].confidence >= 0.35f &&
                out.detections[0].confidence <= 1.0f);
    ASSERT_TRUE(!out.global_motion);

    // Image-space box should map back to the filled area plus the one-cell
    // dilation on each side: 4 raw cells + 2 = 6 cells * 8 px = 48 px.
    ASSERT_TRUE(out.detections[0].center_x > (int16_t)(18 * 8) &&
                out.detections[0].center_x < (int16_t)(22 * 8));
    ASSERT_TRUE(out.detections[0].width >= 40 && out.detections[0].width <= 56);

    free(dark);
    free(with_player);
}

static void test_half_frame_change_flags_global_motion(void) {
    camera_motion_state_t state;
    airsoft_camera_frame_t out;
    uint16_t *black = make_frame(0x0000);
    uint16_t *half_white = make_frame(0x0000);
    fill_cells(half_white, 0, 0, 19, 29, 0xFFFF);  // left half bright

    run_two_frames(&state, black, half_white, &out);
    ASSERT_TRUE(out.global_motion);
    ASSERT_TRUE(out.count == 0);  // suppressed so it cannot spawn fake tracks

    free(black);
    free(half_white);
}

static void test_degenerate_inputs_are_rejected(void) {
    camera_motion_state_t state;
    airsoft_camera_frame_t out;
    uint16_t *frame = make_frame(0x0000);

    camera_motion_init(&state);
    memset(&out, 0xAA, sizeof(out));  // poison: process must zero or skip
    camera_motion_process(&state, NULL, WIDTH, HEIGHT, 66.0f, 1000000, &out);
    if (memcmp(&out, &(airsoft_camera_frame_t){0}, sizeof(out)) != 0 &&
        out.count != 0) {
        fprintf(stderr, "NULL pixels must not produce detections\n");
        exit(EXIT_FAILURE);
    }

    camera_motion_process(
        &state, frame, CAMERA_MOTION_GRID_WIDTH - 1, HEIGHT, 66.0f, 1100000, &out);
    ASSERT_TRUE(out.count == 0);  // narrower than the grid: rejected

    free(frame);
}

int main(void) {
    test_static_scene_yields_nothing();
    test_moving_square_is_detected_centred();
    test_half_frame_change_flags_global_motion();
    test_degenerate_inputs_are_rejected();
    puts("camera motion tests passed");
    return EXIT_SUCCESS;
}

