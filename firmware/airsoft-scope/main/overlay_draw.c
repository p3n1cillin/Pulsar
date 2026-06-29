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

static int marker_radius(float range_m, float max_range_m) {
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

    const uint16_t contact_outer = rgb565(255, 120, 0);
    const uint16_t contact_inner = rgb565(255, 220, 64);
    const uint16_t contact_core = rgb565(255, 255, 255);

    for (uint8_t i = 0; i < overlay->count; ++i) {
        if (!overlay->blips[i].valid) {
            continue;
        }

        const int cx = overlay->blips[i].x;
        const int cy = overlay->blips[i].y;
        const int radius = marker_radius(overlay->blips[i].range_m, max_range_m);

        draw_ring(framebuffer, width, height, cx, cy, radius + 4, contact_outer);
        draw_ring(framebuffer, width, height, cx, cy, radius + 2, contact_outer);
        draw_disk(framebuffer, width, height, cx, cy, radius - 2, contact_inner);
        draw_disk(framebuffer, width, height, cx, cy, 2, contact_core);
        draw_crosshair(framebuffer, width, height, cx, cy, radius + 6, contact_outer);
    }
}
