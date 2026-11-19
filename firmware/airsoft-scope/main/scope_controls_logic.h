#pragma once

// Operator control state machine for the two housing buttons. Pure logic
// (no ESP-IDF), host-tested; scope_controls.c wires it to GPIO, NVS and the
// backlight.
//
//   Normal mode
//     MODE   short  cycle display brightness
//     MODE   long   enter boresight calibration
//     ADJUST short  toggle declutter (radar-backed contacts only)
//   Calibration mode (boresight line + offset shown on screen)
//     MODE   short  nudge radar bearing left  (-0.5 deg)
//     ADJUST short  nudge radar bearing right (+0.5 deg)
//     ADJUST long   reset to the menuconfig default
//     MODE   long   save to NVS and leave calibration

#include <stdbool.h>
#include <stdint.h>

#include "button_input.h"

#define SCOPE_BRIGHTNESS_LEVELS 4
#define SCOPE_BORESIGHT_STEP_DECIDEG 5
#define SCOPE_BORESIGHT_LIMIT_DECIDEG 300

typedef enum {
    SCOPE_BUTTON_MODE = 0,
    SCOPE_BUTTON_ADJUST = 1,
} scope_button_t;

typedef enum {
    SCOPE_CHANGE_NONE = 0,
    SCOPE_CHANGE_BRIGHTNESS = 1 << 0,
    SCOPE_CHANGE_BORESIGHT = 1 << 1,
    SCOPE_CHANGE_VIEW = 1 << 2,
    SCOPE_CHANGE_PERSIST = 1 << 3,  // settings should be written to NVS
} scope_change_t;

typedef struct {
    uint8_t brightness_index;    // 0 .. SCOPE_BRIGHTNESS_LEVELS-1
    int16_t boresight_decideg;   // radar-to-camera offset, tenths of a degree
    bool declutter;
    bool calibrating;
} scope_settings_t;

// Backlight duty for each brightness index, dimmest first.
uint8_t scope_brightness_percent(uint8_t index);

// Clamp loaded/default values into range (NVS data may be stale or corrupt).
void scope_settings_sanitize(scope_settings_t *settings);

// Apply one button event. Returns a scope_change_t bit mask.
uint8_t scope_controls_handle(
    scope_settings_t *settings,
    scope_button_t button,
    button_event_t event,
    int16_t default_boresight_decideg);
