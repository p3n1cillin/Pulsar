#pragma once

#include <stdint.h>

#include "scope_types.h"

void overlay_draw_contacts(
    uint16_t *framebuffer,
    int width,
    int height,
    const airsoft_overlay_t *overlay,
    float max_range_m);
