#include "button_input.h"

#include <string.h>

void button_init(button_state_t *button, bool raw_pressed, uint32_t now_ms) {
    if (!button) {
        return;
    }
    memset(button, 0, sizeof(*button));
    button->raw_last = raw_pressed;
    button->stable_pressed = raw_pressed;
    button->raw_changed_ms = now_ms;
    button->pressed_at_ms = now_ms;
    // A button held through boot is treated as already handled so a stuck or
    // shorted switch cannot spam actions.
    button->long_fired = raw_pressed;
}

button_event_t button_update(
    button_state_t *button,
    const button_timing_t *timing,
    bool raw_pressed,
    uint32_t now_ms) {
    if (!button || !timing) {
        return BUTTON_EVENT_NONE;
    }

    if (raw_pressed != button->raw_last) {
        button->raw_last = raw_pressed;
        button->raw_changed_ms = now_ms;
    }

    // uint32_t subtraction is wrap-safe for the ~49 day millisecond rollover.
    const bool settled = (uint32_t)(now_ms - button->raw_changed_ms) >= timing->debounce_ms;
    if (settled && raw_pressed != button->stable_pressed) {
        button->stable_pressed = raw_pressed;
        if (raw_pressed) {
            button->pressed_at_ms = now_ms;
            button->long_fired = false;
            return BUTTON_EVENT_NONE;
        }
        const bool was_long = button->long_fired;
        button->long_fired = false;
        return was_long ? BUTTON_EVENT_NONE : BUTTON_EVENT_SHORT;
    }

    if (button->stable_pressed && !button->long_fired &&
        (uint32_t)(now_ms - button->pressed_at_ms) >= timing->long_press_ms) {
        button->long_fired = true;
        return BUTTON_EVENT_LONG;
    }
    return BUTTON_EVENT_NONE;
}
