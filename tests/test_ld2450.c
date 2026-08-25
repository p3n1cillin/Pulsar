#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ld2450_parser.h"

#define ASSERT_TRUE(condition) assert_true((condition), #condition, __LINE__)
#define ASSERT_NEAR(actual, expected, tolerance) \
    assert_near((actual), (expected), (tolerance), #actual, __LINE__)

static int callback_count;
static airsoft_radar_frame_t last_frame;

static void capture_frame(const airsoft_radar_frame_t *frame, void *context) {
    (void)context;
    ++callback_count;
    last_frame = *frame;
}

// LD2450 sign-magnitude encoding: bit15 set = positive, else negative.
// Zero encodes as all-zero bits (an empty target slot on the wire).
static uint16_t encode_mm(int32_t mm) {
    const uint32_t magnitude = mm < 0 ? (uint32_t)(-mm) : (uint32_t)mm;
    if (magnitude == 0) {
        return 0;
    }
    if (mm > 0) {
        return (uint16_t)(0x8000u | magnitude);
    }
    return (uint16_t)magnitude;
}

static void put_target(uint8_t *t, int32_t x_mm, int32_t y_mm) {
    const uint16_t x = encode_mm(x_mm);
    const uint16_t y = encode_mm(y_mm);
    t[0] = (uint8_t)(x & 0xFF);
    t[1] = (uint8_t)(x >> 8);
    t[2] = (uint8_t)(y & 0xFF);
    t[3] = (uint8_t)(y >> 8);
    t[4] = 0;  // speed (cm/s), unused
    t[5] = 0;
    t[6] = 0;  // distance resolution, unused
    t[7] = 0;
}

static size_t make_frame(uint8_t *out, const int32_t xs[3], const int32_t ys[3]) {
    out[0] = 0xAA;
    out[1] = 0xFF;
    out[2] = 0x03;
    out[3] = 0x00;
    for (int i = 0; i < 3; ++i) {
        const int32_t x = xs ? xs[i] : 0;
        const int32_t y = ys ? ys[i] : 0;
        put_target(out + 4 + i * 8, x, y);
    }
    out[28] = 0x55;
    out[29] = 0xCC;
    return 30;
}

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

static ld2450_parser_t fresh_parser(void) {
    ld2450_parser_t parser;
    ld2450_parser_init(&parser, &capture_frame, NULL);
    callback_count = 0;
    memset(&last_frame, 0, sizeof(last_frame));
    return parser;
}

static void test_single_frame_decode_and_signs(void) {
    ld2450_parser_t parser = fresh_parser();
    uint8_t frame[30];
    // bit15 set => positive; clear => negative.
    const int32_t xs[3] = {1200, -750, 0};
    const int32_t ys[3] = {-4000, 2300, 0};

    ld2450_parser_feed(&parser, frame, (size_t)make_frame(frame, xs, ys));
    ASSERT_TRUE(callback_count == 1);
    ASSERT_TRUE(last_frame.count == 2);  // third slot empty
    ASSERT_NEAR(last_frame.targets[0].x_m, 1.2f, 0.0001f);
    ASSERT_NEAR(last_frame.targets[0].y_m, -4.0f, 0.0001f);
    ASSERT_NEAR(last_frame.targets[1].x_m, -0.75f, 0.0001f);
    ASSERT_NEAR(last_frame.targets[1].y_m, 2.3f, 0.0001f);
    ASSERT_TRUE(last_frame.updated_us == 0);  // stamped by the firmware layer
}

static void test_all_empty_slots_still_reports(void) {
    ld2450_parser_t parser = fresh_parser();
    uint8_t frame[30];
    (void)make_frame(frame, NULL, NULL);

    ld2450_parser_feed(&parser, frame, 30);
    ASSERT_TRUE(callback_count == 1);
    ASSERT_TRUE(last_frame.count == 0);
}

static void test_garbage_between_frames_is_discarded(void) {
    ld2450_parser_t parser = fresh_parser();
    uint8_t junk[64];
    for (size_t i = 0; i < sizeof(junk); ++i) {
        junk[i] = (uint8_t)(i * 37 + 5);  // no AA FF 03 00 sequence
    }
    uint8_t frame[30];
    const int32_t xs[3] = {500, 0, 0};
    const int32_t ys[3] = {2000, 0, 0};

    ld2450_parser_feed(&parser, junk, sizeof(junk));
    ASSERT_TRUE(callback_count == 0);

    ld2450_parser_feed(&parser, frame, (size_t)make_frame(frame, xs, ys));
    ASSERT_TRUE(callback_count == 1);
    ASSERT_TRUE(last_frame.count == 1);
    ASSERT_NEAR(last_frame.targets[0].x_m, 0.5f, 0.0001f);
    ASSERT_NEAR(last_frame.targets[0].y_m, 2.0f, 0.0001f);
}

static void test_split_feed_decodes_once(void) {
    ld2450_parser_t parser = fresh_parser();
    uint8_t frame[30];
    const int32_t xs[3] = {-1000, 0, 0};
    const int32_t ys[3] = {3000, 0, 0};
    (void)make_frame(frame, xs, ys);

    ld2450_parser_feed(&parser, frame, 11);      // header + partial target
    ASSERT_TRUE(callback_count == 0);
    ld2450_parser_feed(&parser, frame + 11, 9);  // rest of the target bytes
    ASSERT_TRUE(callback_count == 0);
    ld2450_parser_feed(&parser, frame + 20, 10); // tail targets + footer
    ASSERT_TRUE(callback_count == 1);
    ASSERT_TRUE(last_frame.count == 1);
    ASSERT_NEAR(last_frame.targets[0].x_m, -1.0f, 0.0001f);
    ASSERT_NEAR(last_frame.targets[0].y_m, 3.0f, 0.0001f);
}

static void test_false_header_resyncs(void) {
    ld2450_parser_t parser = fresh_parser();

    // Valid header followed by a wrong footer: must be skipped, not emitted.
    uint8_t bad[30] = {0};
    bad[0] = 0xAA;
    bad[1] = 0xFF;
    bad[2] = 0x03;
    bad[3] = 0x00;
    bad[28] = 0xDE;
    bad[29] = 0xAD;

    uint8_t good[30];
    const int32_t xs[3] = {250, 0, 0};
    const int32_t ys[3] = {1000, 0, 0};
    (void)make_frame(good, xs, ys);

    ld2450_parser_feed(&parser, bad, 30);
    ASSERT_TRUE(callback_count == 0);
    ld2450_parser_feed(&parser, good, 30);
    ASSERT_TRUE(callback_count == 1);
    ASSERT_TRUE(last_frame.count == 1);
    ASSERT_NEAR(last_frame.targets[0].x_m, 0.25f, 0.0001f);
}

static void test_junk_flood_cannot_overflow(void) {
    ld2450_parser_t parser = fresh_parser();
    uint8_t junk[512];
    memset(junk, 0xCC, sizeof(junk));
    ld2450_parser_feed(&parser, junk, sizeof(junk));
    ASSERT_TRUE(callback_count == 0);
    ASSERT_TRUE(parser.len < LD2450_FRAME_LEN);  // never retains a full bogus frame

    // The parser must still decode a valid frame after the flood.
    uint8_t frame[30];
    const int32_t xs[3] = {100, 0, 0};
    const int32_t ys[3] = {400, 0, 0};
    (void)make_frame(frame, xs, ys);
    ld2450_parser_feed(&parser, frame, 30);
    ASSERT_TRUE(callback_count == 1);
    ASSERT_NEAR(last_frame.targets[0].x_m, 0.1f, 0.0001f);
    ASSERT_NEAR(last_frame.targets[0].y_m, 0.4f, 0.0001f);
}

int main(void) {
    test_single_frame_decode_and_signs();
    test_all_empty_slots_still_reports();
    test_garbage_between_frames_is_discarded();
    test_split_feed_decodes_once();
    test_false_header_resyncs();
    test_junk_flood_cannot_overflow();
    puts("ld2450 parser tests passed");
    return EXIT_SUCCESS;
}

