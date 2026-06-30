#include <math.h>

#include "overlay_draw.h"

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    // ST7789 over SPI expects big-endian 16-bit pixels; byte-swap so the
    // little-endian framebuffer is read back in the right order.
    const uint16_t v = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)((v >> 8) | (v << 8));
}

static bool in_bounds(int x, int y, int width, int height) {
    return x >= 0 && y >= 0 && x < width && y < height;
}

static void put_pixel(uint16_t *fb, int width, int height, int x, int y, uint16_t color) {
    if (in_bounds(x, y, width, height)) {
        fb[(size_t)y * (size_t)width + (size_t)x] = color;
    }
}

static void draw_disk(uint16_t *fb, int width, int height, int cx, int cy, int radius, uint16_t color) {
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            if ((x * x + y * y) <= radius * radius) {
                put_pixel(fb, width, height, cx + x, cy + y, color);
            }
        }
    }
}

static void draw_ring(uint16_t *fb, int width, int height, int cx, int cy, int radius, uint16_t color) {
    for (int angle = 0; angle < 360; ++angle) {
        const float rad = (float)angle * (float)M_PI / 180.0f;
        const int x = cx + (int)lroundf(cosf(rad) * (float)radius);
        const int y = cy + (int)lroundf(sinf(rad) * (float)radius);
        put_pixel(fb, width, height, x, y, color);
        put_pixel(fb, width, height, x + 1, y, color);
    }
}

static void draw_dashed_ring(
    uint16_t *fb,
    int width,
    int height,
    int cx,
    int cy,
    int radius,
    uint16_t color) {
    for (int angle = 0; angle < 360; angle += 3) {
        if ((angle / 18) % 2 != 0) continue;
        const float rad = (float)angle * (float)M_PI / 180.0f;
        const int x = cx + (int)lroundf(cosf(rad) * (float)radius);
        const int y = cy + (int)lroundf(sinf(rad) * (float)radius);
        put_pixel(fb, width, height, x, y, color);
    }
}

static void draw_box_corners(
    uint16_t *fb,
    int width,
    int height,
    int cx,
    int cy,
    int box_width,
    int box_height,
    uint16_t color) {
    const int half_w = box_width > 10 ? box_width / 2 : 8;
    const int half_h = box_height > 10 ? box_height / 2 : 12;
    const int arm = 5;
    for (int i = 0; i < arm; ++i) {
        put_pixel(fb, width, height, cx - half_w + i, cy - half_h, color);
        put_pixel(fb, width, height, cx - half_w, cy - half_h + i, color);
        put_pixel(fb, width, height, cx + half_w - i, cy - half_h, color);
        put_pixel(fb, width, height, cx + half_w, cy - half_h + i, color);
        put_pixel(fb, width, height, cx - half_w + i, cy + half_h, color);
        put_pixel(fb, width, height, cx - half_w, cy + half_h - i, color);
        put_pixel(fb, width, height, cx + half_w - i, cy + half_h, color);
        put_pixel(fb, width, height, cx + half_w, cy + half_h - i, color);
    }
}

static void draw_digit(
    uint16_t *fb,
    int width,
    int height,
    int x,
    int y,
    uint8_t digit,
    uint16_t color) {
    static const uint8_t glyphs[10][5] = {
        {0x7, 0x5, 0x5, 0x5, 0x7}, {0x2, 0x6, 0x2, 0x2, 0x7},
        {0x7, 0x1, 0x7, 0x4, 0x7}, {0x7, 0x1, 0x7, 0x1, 0x7},
        {0x5, 0x5, 0x7, 0x1, 0x1}, {0x7, 0x4, 0x7, 0x1, 0x7},
        {0x7, 0x4, 0x7, 0x5, 0x7}, {0x7, 0x1, 0x1, 0x1, 0x1},
        {0x7, 0x5, 0x7, 0x5, 0x7}, {0x7, 0x5, 0x7, 0x1, 0x7},
    };
    digit %= 10;
    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < 3; ++col) {
            if (glyphs[digit][row] & (1U << (2 - col))) {
                put_pixel(fb, width, height, x + col, y + row, color);
            }
        }
    }
}

static void draw_edge_chevron(
    uint16_t *fb,
    int width,
    int height,
    int x,
    int y,
    uint16_t color) {
    const int direction = x < width / 2 ? 1 : -1;
    for (int i = 0; i < 7; ++i) {
        put_pixel(fb, width, height, x + direction * i, y - i, color);
        put_pixel(fb, width, height, x + direction * i, y + i, color);
    }
}

static void draw_crosshair(
    uint16_t *fb,
    int width,
    int height,
    int cx,
    int cy,
    int arm,
    uint16_t color) {
    for (int i = -arm; i <= arm; ++i) {
        put_pixel(fb, width, height, cx + i, cy, color);
        put_pixel(fb, width, height, cx, cy + i, color);
    }
}

static int marker_radius(float range_m, float max_range_m, bool range_valid) {
    if (!range_valid) {
        return 9;
    }
    const float normalized = range_m / max_range_m;
    const float scaled = 14.0f - normalized * 8.0f;
    if (scaled < 6.0f) {
        return 6;
    }
    if (scaled > 14.0f) {
        return 14;
    }
    return (int)lroundf(scaled);
}

void overlay_draw_contacts(
    uint16_t *framebuffer,
    int width,
    int height,
    const airsoft_overlay_t *overlay,
    float max_range_m) {
    if (!framebuffer || !overlay) {
        return;
    }

    const uint16_t radar_outer = rgb565(255, 120, 0);
    const uint16_t radar_inner = rgb565(255, 220, 64);
    const uint16_t camera_color = rgb565(30, 220, 255);
    const uint16_t fused_outer = rgb565(255, 48, 32);
    const uint16_t predicted_color = rgb565(155, 125, 70);
    const uint16_t contact_core = rgb565(255, 255, 255);

    for (uint8_t i = 0; i < overlay->count; ++i) {
        if (!overlay->blips[i].valid) {
            continue;
        }

        const int cx = overlay->blips[i].x;
        const int cy = overlay->blips[i].y;
        const int radius = marker_radius(
            overlay->blips[i].range_m, max_range_m, overlay->blips[i].range_valid);
        const bool radar =
            (overlay->blips[i].source_flags & AIRSOFT_TRACK_SOURCE_RADAR) != 0;
        const bool camera =
            (overlay->blips[i].source_flags & AIRSOFT_TRACK_SOURCE_CAMERA) != 0;
        const bool predicted =
            (overlay->blips[i].source_flags & AIRSOFT_TRACK_SOURCE_PREDICTED) != 0;

        if (overlay->blips[i].offscreen) {
            const uint16_t color = predicted ? predicted_color : radar_outer;
            draw_edge_chevron(framebuffer, width, height, cx, cy, color);
            draw_digit(framebuffer, width, height, cx < width / 2 ? cx + 9 : cx - 12,
                       cy - 2, overlay->blips[i].track_id, color);
            continue;
        }

        if (predicted) {
            draw_dashed_ring(
                framebuffer, width, height, cx, cy, radius + 3, predicted_color);
        } else if (radar && camera) {
            draw_ring(framebuffer, width, height, cx, cy, radius + 4, fused_outer);
            draw_ring(framebuffer, width, height, cx, cy, radius + 2, fused_outer);
            draw_disk(framebuffer, width, height, cx, cy, radius - 2, radar_inner);
        } else if (radar) {
            draw_ring(framebuffer, width, height, cx, cy, radius + 4, radar_outer);
            draw_ring(framebuffer, width, height, cx, cy, radius + 2, radar_outer);
            draw_disk(framebuffer, width, height, cx, cy, radius - 2, radar_inner);
        } else if (camera) {
            draw_ring(framebuffer, width, height, cx, cy, radius, camera_color);
        }

        if (camera && !predicted) {
            draw_box_corners(
                framebuffer, width, height, cx, cy,
                overlay->blips[i].box_width, overlay->blips[i].box_height,
                camera_color);
        }
        if (!predicted) {
            const uint16_t cross_color = radar && camera ? fused_outer
                : (radar ? radar_outer : camera_color);
            draw_disk(framebuffer, width, height, cx, cy, 2, contact_core);
            draw_crosshair(framebuffer, width, height, cx, cy, radius + 6, cross_color);
        }
        draw_digit(framebuffer, width, height, cx + radius + 7, cy - 2,
                   overlay->blips[i].track_id,
                   predicted ? predicted_color : contact_core);
    }
}
