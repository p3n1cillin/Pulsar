#include <math.h>
#include <stdio.h>
#include <string.h>

#include "overlay_draw.h"

#define OVERLAY_PI 3.14159265358979323846f

uint16_t overlay_rgb565(uint8_t r, uint8_t g, uint8_t b) {
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
        const float rad = (float)angle * OVERLAY_PI / 180.0f;
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
        const float rad = (float)angle * OVERLAY_PI / 180.0f;
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

// 3x5 font, one row per byte (bit 2 = left column). Digits, A-Z and a few
// symbols are enough for track IDs and the status HUD.
static const char font_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ+-.%:/";
static const uint8_t font_glyphs[][5] = {
    {7, 5, 5, 5, 7}, {2, 6, 2, 2, 7}, {7, 1, 7, 4, 7}, {7, 1, 7, 1, 7},  // 0-3
    {5, 5, 7, 1, 1}, {7, 4, 7, 1, 7}, {7, 4, 7, 5, 7}, {7, 1, 1, 1, 1},  // 4-7
    {7, 5, 7, 5, 7}, {7, 5, 7, 1, 7},                                    // 8-9
    {2, 5, 7, 5, 5}, {6, 5, 6, 5, 6}, {3, 4, 4, 4, 3}, {6, 5, 5, 5, 6},  // A-D
    {7, 4, 6, 4, 7}, {7, 4, 6, 4, 4}, {3, 4, 5, 5, 3}, {5, 5, 7, 5, 5},  // E-H
    {7, 2, 2, 2, 7}, {1, 1, 1, 5, 2}, {5, 5, 6, 5, 5}, {4, 4, 4, 4, 7},  // I-L
    {5, 7, 7, 5, 5}, {6, 5, 5, 5, 5}, {2, 5, 5, 5, 2}, {6, 5, 6, 4, 4},  // M-P
    {2, 5, 5, 6, 3}, {6, 5, 6, 5, 5}, {3, 4, 2, 1, 6}, {7, 2, 2, 2, 2},  // Q-T
    {5, 5, 5, 5, 7}, {5, 5, 5, 5, 2}, {5, 5, 7, 7, 5}, {5, 5, 2, 5, 5},  // U-X
    {5, 5, 2, 2, 2}, {7, 1, 2, 4, 7},                                    // Y-Z
    {0, 2, 7, 2, 0}, {0, 0, 7, 0, 0}, {0, 0, 0, 0, 2}, {5, 1, 2, 4, 5},  // + - . %
    {0, 2, 0, 2, 0}, {1, 1, 2, 4, 4},                                    // : /
};

static const uint8_t *glyph_for(char c) {
    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 'a' + 'A');
    }
    const char *hit = c ? strchr(font_chars, c) : NULL;
    return hit ? font_glyphs[hit - font_chars] : NULL;
}

static void fill_rect(
    uint16_t *fb, int width, int height, int x, int y, int w, int h, uint16_t color) {
    for (int yy = y; yy < y + h; ++yy) {
        for (int xx = x; xx < x + w; ++xx) {
            put_pixel(fb, width, height, xx, yy, color);
        }
    }
}

int overlay_text_width(const char *text, int scale) {
    if (!text || scale < 1) {
        return 0;
    }
    const int n = (int)strlen(text);
    return n > 0 ? n * 4 * scale - scale : 0;
}

int overlay_draw_text(
    uint16_t *fb,
    int width,
    int height,
    int x,
    int y,
    const char *text,
    int scale,
    uint16_t color) {
    if (!fb || !text || scale < 1) {
        return 0;
    }
    int cursor = x;
    for (const char *c = text; *c; ++c) {
        const uint8_t *glyph = glyph_for(*c);
        if (glyph) {
            for (int row = 0; row < 5; ++row) {
                for (int col = 0; col < 3; ++col) {
                    if (glyph[row] & (1U << (2 - col))) {
                        fill_rect(fb, width, height, cursor + col * scale, y + row * scale,
                                  scale, scale, color);
                    }
                }
            }
        }
        cursor += 4 * scale;
    }
    return overlay_text_width(text, scale);
}

static void draw_digit(
    uint16_t *fb,
    int width,
    int height,
    int x,
    int y,
    uint8_t digit,
    uint16_t color) {
    const char text[2] = {(char)('0' + digit % 10), '\0'};
    overlay_draw_text(fb, width, height, x, y, text, 1, color);
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

    const uint16_t radar_outer = overlay_rgb565(255, 120, 0);
    const uint16_t radar_inner = overlay_rgb565(255, 220, 64);
    const uint16_t camera_color = overlay_rgb565(30, 220, 255);
    const uint16_t fused_outer = overlay_rgb565(255, 48, 32);
    const uint16_t predicted_color = overlay_rgb565(155, 125, 70);
    const uint16_t contact_core = overlay_rgb565(255, 255, 255);

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

// Text with a 1 px dark outline so it stays readable over a bright camera feed.
static void draw_text_outlined(
    uint16_t *fb, int width, int height, int x, int y, const char *text, int scale,
    uint16_t color) {
    const uint16_t shadow = overlay_rgb565(0, 0, 0);
    static const int offsets[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    for (int i = 0; i < 4; ++i) {
        overlay_draw_text(fb, width, height, x + offsets[i][0], y + offsets[i][1], text, scale,
                          shadow);
    }
    overlay_draw_text(fb, width, height, x, y, text, scale, color);
}

static void draw_battery(
    uint16_t *fb, int width, int height, uint8_t percent, bool blink_phase) {
    if (percent > 100) {
        percent = 100;
    }
    const bool low = percent <= 15;
    const uint16_t outline = low ? overlay_rgb565(255, 40, 30) : overlay_rgb565(220, 220, 220);
    const uint16_t fill = low ? overlay_rgb565(255, 40, 30)
        : (percent <= 40 ? overlay_rgb565(255, 190, 0) : overlay_rgb565(60, 220, 90));
    const int x = 4;
    const int y = 4;
    const int body_w = 22;
    const int body_h = 10;
    if (low && !blink_phase) {
        return;  // flash the gauge when nearly empty
    }
    for (int i = 0; i < body_w; ++i) {
        put_pixel(fb, width, height, x + i, y, outline);
        put_pixel(fb, width, height, x + i, y + body_h - 1, outline);
    }
    for (int j = 0; j < body_h; ++j) {
        put_pixel(fb, width, height, x, y + j, outline);
        put_pixel(fb, width, height, x + body_w - 1, y + j, outline);
    }
    fill_rect(fb, width, height, x + body_w, y + 3, 2, body_h - 6, outline);
    const int level = ((body_w - 4) * (int)percent + 50) / 100;
    fill_rect(fb, width, height, x + 2, y + 2, level, body_h - 4, fill);

    char label[8];
    snprintf(label, sizeof(label), "%u%%", (unsigned)percent);
    draw_text_outlined(fb, width, height, x + body_w + 5, y + 2, label, 1, outline);
}

static void draw_calibration(
    uint16_t *fb, int width, int height, float boresight_deg) {
    const uint16_t line = overlay_rgb565(80, 255, 120);
    // Dashed boresight reference: centre of the image = camera optical axis.
    for (int y = 22; y < height - 30; ++y) {
        if ((y / 4) % 2 == 0) {
            put_pixel(fb, width, height, width / 2, y, line);
        }
    }
    if (boresight_deg > 99.0f) boresight_deg = 99.0f;
    if (boresight_deg < -99.0f) boresight_deg = -99.0f;
    char label[24];
    snprintf(label, sizeof(label), "CAL %+.1f", (double)boresight_deg);
    const int scale = 2;
    const int text_w = overlay_text_width(label, scale);
    draw_text_outlined(fb, width, height, (width - text_w) / 2, height - 26, label, scale, line);
    const char *hint = "A-  B+  HOLD A SAVE";
    draw_text_outlined(fb, width, height, (width - overlay_text_width(hint, 1)) / 2,
                       height - 10, hint, 1, line);
}

void overlay_draw_hud(
    uint16_t *framebuffer,
    int width,
    int height,
    const scope_hud_t *hud) {
    if (!framebuffer || !hud || width <= 0 || height <= 0) {
        return;
    }

    if (hud->battery_valid) {
        draw_battery(framebuffer, width, height, hud->battery_percent, hud->blink_phase);
    }

    if (hud->declutter) {
        const char *tag = "DCL";
        draw_text_outlined(framebuffer, width, height,
                           width - overlay_text_width(tag, 1) - 5, 6, tag, 1,
                           overlay_rgb565(255, 120, 0));
    }

    if (hud->radar_lost) {
        // A silent radar must never look like "no one there".
        const uint16_t banner = hud->blink_phase ? overlay_rgb565(200, 20, 20) : overlay_rgb565(110, 0, 0);
        const char *text = "NO RADAR";
        const int scale = 2;
        const int text_w = overlay_text_width(text, scale);
        const int box_w = text_w + 12;
        const int box_x = (width - box_w) / 2;
        fill_rect(framebuffer, width, height, box_x, 3, box_w, 16, banner);
        overlay_draw_text(framebuffer, width, height, box_x + 6, 6, text, scale,
                          overlay_rgb565(255, 255, 255));
    }

    if (hud->calibrating) {
        draw_calibration(framebuffer, width, height, hud->boresight_deg);
    }
}
