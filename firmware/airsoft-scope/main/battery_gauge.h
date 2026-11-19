#pragma once

// Single-cell Li-ion state-of-charge estimate from a resting voltage. Pure
// logic for host tests; the ADC side lives in scope_controls.c.

#include <stdbool.h>
#include <stdint.h>

#define BATTERY_LOW_PERCENT 15

typedef struct {
    float filtered_mv;
    bool primed;
} battery_gauge_t;

// Piecewise-linear open-circuit-voltage curve for a typical 18650 cell.
uint8_t battery_percent_from_mv(uint32_t cell_mv);

// Exponential smoothing so load spikes (backlight, radar TX) do not make the
// gauge jump. Returns the filtered cell voltage in mV.
uint32_t battery_gauge_update(battery_gauge_t *gauge, uint32_t sample_mv);
