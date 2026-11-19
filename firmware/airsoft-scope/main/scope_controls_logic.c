#include "scope_controls_logic.h"

#include <stddef.h>

static const uint8_t brightness_table[SCOPE_BRIGHTNESS_LEVELS] = {15, 40, 70, 100};

uint8_t scope_brightness_percent(uint8_t index) {
    if (index >= SCOPE_BRIGHTNESS_LEVELS) {
        index = SCOPE_BRIGHTNESS_LEVELS - 1;
    }
    return brightness_table[index];
}

static int16_t clamp_boresight(int value) {
    if (value > SCOPE_BORESIGHT_LIMIT_DECIDEG) return SCOPE_BORESIGHT_LIMIT_DECIDEG;
    if (value < -SCOPE_BORESIGHT_LIMIT_DECIDEG) return -SCOPE_BORESIGHT_LIMIT_DECIDEG;
    return (int16_t)value;
}

void scope_settings_sanitize(scope_settings_t *settings) {
    if (!settings) {
        return;
    }
    if (settings->brightness_index >= SCOPE_BRIGHTNESS_LEVELS) {
        settings->brightness_index = SCOPE_BRIGHTNESS_LEVELS - 1;
    }
    settings->boresight_decideg = clamp_boresight(settings->boresight_decideg);
    settings->calibrating = false;
}

static uint8_t handle_normal(scope_settings_t *s, scope_button_t button, button_event_t event) {
    if (button == SCOPE_BUTTON_MODE && event == BUTTON_EVENT_SHORT) {
        s->brightness_index = (uint8_t)((s->brightness_index + 1) % SCOPE_BRIGHTNESS_LEVELS);
        return SCOPE_CHANGE_BRIGHTNESS | SCOPE_CHANGE_PERSIST;
    }
    if (button == SCOPE_BUTTON_MODE && event == BUTTON_EVENT_LONG) {
        s->calibrating = true;
        return SCOPE_CHANGE_VIEW;
    }
    if (button == SCOPE_BUTTON_ADJUST && event == BUTTON_EVENT_SHORT) {
        s->declutter = !s->declutter;
        return SCOPE_CHANGE_VIEW | SCOPE_CHANGE_PERSIST;
    }
    return SCOPE_CHANGE_NONE;
}

static uint8_t handle_calibration(
    scope_settings_t *s,
    scope_button_t button,
    button_event_t event,
    int16_t default_boresight_decideg) {
    const int16_t before = s->boresight_decideg;
    if (event == BUTTON_EVENT_SHORT) {
        const int step = button == SCOPE_BUTTON_MODE
            ? -SCOPE_BORESIGHT_STEP_DECIDEG
            : SCOPE_BORESIGHT_STEP_DECIDEG;
        s->boresight_decideg = clamp_boresight(s->boresight_decideg + step);
    } else if (event == BUTTON_EVENT_LONG && button == SCOPE_BUTTON_ADJUST) {
        s->boresight_decideg = clamp_boresight(default_boresight_decideg);
    } else if (event == BUTTON_EVENT_LONG && button == SCOPE_BUTTON_MODE) {
        s->calibrating = false;
        return SCOPE_CHANGE_VIEW | SCOPE_CHANGE_PERSIST;
    }
    return s->boresight_decideg != before ? SCOPE_CHANGE_BORESIGHT : SCOPE_CHANGE_NONE;
}

uint8_t scope_controls_handle(
    scope_settings_t *settings,
    scope_button_t button,
    button_event_t event,
    int16_t default_boresight_decideg) {
    if (!settings || event == BUTTON_EVENT_NONE) {
        return SCOPE_CHANGE_NONE;
    }
    return settings->calibrating
        ? handle_calibration(settings, button, event, default_boresight_decideg)
        : handle_normal(settings, button, event);
}
