#pragma once

// Pure debounce + short/long press classifier for a single push button.
// No ESP-IDF dependencies, so it is unit-tested on the host.

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SHORT,  // released before the long-press threshold
    BUTTON_EVENT_LONG,   // fired once while still held at the threshold
} button_event_t;

typedef struct {
    uint32_t debounce_ms;    // raw level must be stable this long to count
    uint32_t long_press_ms;  // held this long -> LONG instead of SHORT
} button_timing_t;

typedef struct {
    bool raw_last;
    bool stable_pressed;
    bool long_fired;
    uint32_t raw_changed_ms;
    uint32_t pressed_at_ms;
} button_state_t;

void button_init(button_state_t *button, bool raw_pressed, uint32_t now_ms);

// Feed the current raw level (true = pressed) at a regular poll interval.
// Returns at most one event per call. A press that started before
// button_init (e.g. held through boot) never produces an event.
button_event_t button_update(
    button_state_t *button,
    const button_timing_t *timing,
    bool raw_pressed,
    uint32_t now_ms);
