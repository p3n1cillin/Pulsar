#include "battery_gauge.h"

#include <stddef.h>

typedef struct {
    uint16_t mv;
    uint8_t percent;
} ocv_point_t;

// Conservative under light load: the device draws ~300-450 mA, so the cell
// sits slightly below its rest voltage while running.
static const ocv_point_t ocv_curve[] = {
    {3300, 0},  {3500, 5},  {3600, 10}, {3700, 25}, {3750, 38}, {3800, 50},
    {3850, 60}, {3900, 68}, {3950, 75}, {4000, 82}, {4100, 92}, {4180, 100},
};

uint8_t battery_percent_from_mv(uint32_t cell_mv) {
    const size_t points = sizeof(ocv_curve) / sizeof(ocv_curve[0]);
    if (cell_mv <= ocv_curve[0].mv) {
        return 0;
    }
    if (cell_mv >= ocv_curve[points - 1].mv) {
        return 100;
    }
    for (size_t i = 1; i < points; ++i) {
        if (cell_mv <= ocv_curve[i].mv) {
            const ocv_point_t lo = ocv_curve[i - 1];
            const ocv_point_t hi = ocv_curve[i];
            const uint32_t span_mv = (uint32_t)(hi.mv - lo.mv);
            const uint32_t span_pct = (uint32_t)(hi.percent - lo.percent);
            const uint32_t offset = cell_mv - lo.mv;
            return (uint8_t)(lo.percent + (offset * span_pct + span_mv / 2) / span_mv);
        }
    }
    return 100;
}

uint32_t battery_gauge_update(battery_gauge_t *gauge, uint32_t sample_mv) {
    if (!gauge) {
        return sample_mv;
    }
    if (!gauge->primed) {
        gauge->filtered_mv = (float)sample_mv;
        gauge->primed = true;
    } else {
        gauge->filtered_mv += 0.15f * ((float)sample_mv - gauge->filtered_mv);
    }
    return (uint32_t)(gauge->filtered_mv + 0.5f);
}
