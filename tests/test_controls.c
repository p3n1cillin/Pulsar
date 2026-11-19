#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "battery_gauge.h"
#include "button_input.h"
#include "scope_controls_logic.h"

#define ASSERT_TRUE(condition) assert_true((condition), #condition, __LINE__)
#define ASSERT_EQ(actual, expected) assert_eq((long)(actual), (long)(expected), #actual, __LINE__)

static void assert_true(bool condition, const char *expression, int line) {
    if (!condition) {
        fprintf(stderr, "line %d: assertion failed: %s\n", line, expression);
        exit(EXIT_FAILURE);
    }
}

static void assert_eq(long actual, long expected, const char *expression, int line) {
    if (actual != expected) {
        fprintf(stderr, "line %d: %s was %ld, expected %ld\n", line, expression, actual, expected);
        exit(EXIT_FAILURE);
    }
}

static const button_timing_t timing = {.debounce_ms = 30, .long_press_ms = 900};

// Hold a raw level from start_ms to end_ms (exclusive) at 10 ms polls and
// return the events seen.
static int feed(button_state_t *b, bool pressed, uint32_t start_ms, uint32_t end_ms,
                button_event_t *events, int max_events) {
    int count = 0;
    const uint32_t steps = (uint32_t)(end_ms - start_ms) / 10U;  // wrap-safe
    for (uint32_t i = 0; i < steps; ++i) {
        const button_event_t e = button_update(b, &timing, pressed, start_ms + i * 10U);
        if (e != BUTTON_EVENT_NONE && count < max_events) {
            events[count++] = e;
        }
    }
    return count;
}

static void test_short_press(void) {
    button_state_t b;
    button_event_t ev[4];
    button_init(&b, false, 0);
    ASSERT_EQ(feed(&b, true, 0, 200, ev, 4), 0);
    ASSERT_EQ(feed(&b, false, 200, 300, ev, 4), 1);
    ASSERT_EQ(ev[0], BUTTON_EVENT_SHORT);
}

static void test_long_press_fires_once_while_held(void) {
    button_state_t b;
    button_event_t ev[4];
    button_init(&b, false, 0);
    ASSERT_EQ(feed(&b, true, 0, 2500, ev, 4), 1);
    ASSERT_EQ(ev[0], BUTTON_EVENT_LONG);
    // Releasing after a long press must not also emit a short press.
    ASSERT_EQ(feed(&b, false, 2500, 2700, ev, 4), 0);
}

static void test_bounce_is_ignored(void) {
    button_state_t b;
    button_init(&b, false, 0);
    // Contact chatter faster than the debounce window never settles.
    for (uint32_t t = 0; t < 400; t += 10) {
        ASSERT_EQ(button_update(&b, &timing, (t / 10) % 2 == 0, t), BUTTON_EVENT_NONE);
    }
    ASSERT_TRUE(!b.stable_pressed);
}

static void test_held_through_boot_is_suppressed(void) {
    button_state_t b;
    button_event_t ev[4];
    button_init(&b, true, 0);
    ASSERT_EQ(feed(&b, true, 0, 3000, ev, 4), 0);
    ASSERT_EQ(feed(&b, false, 3000, 3200, ev, 4), 0);
    // The next real press works normally.
    ASSERT_EQ(feed(&b, true, 3200, 3400, ev, 4), 0);
    ASSERT_EQ(feed(&b, false, 3400, 3500, ev, 4), 1);
    ASSERT_EQ(ev[0], BUTTON_EVENT_SHORT);
}

static void test_millisecond_wraparound(void) {
    button_state_t b;
    button_event_t ev[4];
    const uint32_t start = UINT32_MAX - 100;
    button_init(&b, false, start);
    ASSERT_EQ(feed(&b, true, start, start + 150, ev, 4), 0);  // wraps past zero
    ASSERT_EQ(feed(&b, false, start + 150, start + 250, ev, 4), 1);
    ASSERT_EQ(ev[0], BUTTON_EVENT_SHORT);
}

static void test_brightness_and_declutter(void) {
    scope_settings_t s = {.brightness_index = SCOPE_BRIGHTNESS_LEVELS - 1};
    uint8_t changes = scope_controls_handle(&s, SCOPE_BUTTON_MODE, BUTTON_EVENT_SHORT, 0);
    ASSERT_EQ(s.brightness_index, 0);  // wraps from brightest to dimmest
    ASSERT_TRUE(changes & SCOPE_CHANGE_BRIGHTNESS);
    ASSERT_TRUE(changes & SCOPE_CHANGE_PERSIST);
    ASSERT_TRUE(scope_brightness_percent(0) < scope_brightness_percent(1));
    ASSERT_EQ(scope_brightness_percent(200), 100);

    changes = scope_controls_handle(&s, SCOPE_BUTTON_ADJUST, BUTTON_EVENT_SHORT, 0);
    ASSERT_TRUE(s.declutter);
    ASSERT_TRUE(changes & SCOPE_CHANGE_VIEW);
    ASSERT_EQ(scope_controls_handle(&s, SCOPE_BUTTON_ADJUST, BUTTON_EVENT_NONE, 0),
              SCOPE_CHANGE_NONE);
}

static void test_boresight_calibration_flow(void) {
    scope_settings_t s = {.brightness_index = 2, .boresight_decideg = 0};
    scope_controls_handle(&s, SCOPE_BUTTON_MODE, BUTTON_EVENT_LONG, 20);
    ASSERT_TRUE(s.calibrating);

    // In calibration the buttons nudge the offset instead of brightness.
    uint8_t changes = scope_controls_handle(&s, SCOPE_BUTTON_ADJUST, BUTTON_EVENT_SHORT, 20);
    ASSERT_EQ(s.boresight_decideg, SCOPE_BORESIGHT_STEP_DECIDEG);
    ASSERT_TRUE(changes & SCOPE_CHANGE_BORESIGHT);
    ASSERT_TRUE(!(changes & SCOPE_CHANGE_PERSIST));  // not saved until confirmed
    scope_controls_handle(&s, SCOPE_BUTTON_MODE, BUTTON_EVENT_SHORT, 20);
    scope_controls_handle(&s, SCOPE_BUTTON_MODE, BUTTON_EVENT_SHORT, 20);
    ASSERT_EQ(s.boresight_decideg, -SCOPE_BORESIGHT_STEP_DECIDEG);
    ASSERT_EQ(s.brightness_index, 2);

    // Clamped to the menuconfig range.
    for (int i = 0; i < 200; ++i) {
        scope_controls_handle(&s, SCOPE_BUTTON_ADJUST, BUTTON_EVENT_SHORT, 20);
    }
    ASSERT_EQ(s.boresight_decideg, SCOPE_BORESIGHT_LIMIT_DECIDEG);

    // Long ADJUST resets to the default, long MODE saves and exits.
    scope_controls_handle(&s, SCOPE_BUTTON_ADJUST, BUTTON_EVENT_LONG, 20);
    ASSERT_EQ(s.boresight_decideg, 20);
    changes = scope_controls_handle(&s, SCOPE_BUTTON_MODE, BUTTON_EVENT_LONG, 20);
    ASSERT_TRUE(!s.calibrating);
    ASSERT_TRUE(changes & SCOPE_CHANGE_PERSIST);
}

static void test_settings_sanitize(void) {
    scope_settings_t s = {.brightness_index = 99, .boresight_decideg = -32000, .calibrating = true};
    scope_settings_sanitize(&s);
    ASSERT_EQ(s.brightness_index, SCOPE_BRIGHTNESS_LEVELS - 1);
    ASSERT_EQ(s.boresight_decideg, -SCOPE_BORESIGHT_LIMIT_DECIDEG);
    ASSERT_TRUE(!s.calibrating);  // never boot into calibration
}

static void test_battery_curve(void) {
    ASSERT_EQ(battery_percent_from_mv(2900), 0);
    ASSERT_EQ(battery_percent_from_mv(3300), 0);
    ASSERT_EQ(battery_percent_from_mv(4180), 100);
    ASSERT_EQ(battery_percent_from_mv(4300), 100);
    ASSERT_EQ(battery_percent_from_mv(3800), 50);
    ASSERT_TRUE(battery_percent_from_mv(3650) > 10 && battery_percent_from_mv(3650) < 25);
    // Monotonic across the whole range.
    uint8_t previous = 0;
    for (uint32_t mv = 3000; mv <= 4300; mv += 5) {
        const uint8_t pct = battery_percent_from_mv(mv);
        ASSERT_TRUE(pct >= previous);
        previous = pct;
    }
}

static void test_battery_filter(void) {
    battery_gauge_t g = {0};
    ASSERT_EQ(battery_gauge_update(&g, 3900), 3900);  // first sample primes
    // A single 300 mV load dip moves the filtered value only a little.
    const uint32_t dipped = battery_gauge_update(&g, 3600);
    ASSERT_TRUE(dipped < 3900 && dipped > 3840);
    for (int i = 0; i < 100; ++i) {
        battery_gauge_update(&g, 3700);
    }
    ASSERT_TRUE(fabs((double)battery_gauge_update(&g, 3700) - 3700.0) <= 1.0);
}

int main(void) {
    test_short_press();
    test_long_press_fires_once_while_held();
    test_bounce_is_ignored();
    test_held_through_boot_is_suppressed();
    test_millisecond_wraparound();
    test_brightness_and_declutter();
    test_boresight_calibration_flow();
    test_settings_sanitize();
    test_battery_curve();
    test_battery_filter();
    puts("controls tests passed");
    return EXIT_SUCCESS;
}
