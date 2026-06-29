#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "overlay_draw.h"
#include "scope_ui.h"
#include "st7789.h"

static const char *TAG = "scope_ui";

static int ui_width;   // logical render size (landscape), e.g. 320
static int ui_height;  // logical render size (landscape), e.g. 240
static uint16_t *ui_fb;   // landscape render buffer (ui_width x ui_height)
static uint16_t *rot_fb;  // panel buffer for portrait rotations (240 x 320), else NULL

// Last buffer sent to the panel, for the debug web stream.
static const uint16_t *s_present_buf;
static int s_present_w;
static int s_present_h;

bool scope_ui_panel_snapshot(const uint16_t **buf, int *w, int *h) {
    if (!s_present_buf || !buf || !w || !h) {
        return false;
    }
    *buf = s_present_buf;
    *w = s_present_w;
    *h = s_present_h;
    return true;
}

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    // ST7789 over SPI expects big-endian 16-bit pixels; byte-swap so the
    // little-endian framebuffer is read back in the right order.
    const uint16_t v = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)((v >> 8) | (v << 8));
}

static void fb_fill(uint16_t *fb, int w, int h, uint16_t color) {
    const size_t n = (size_t)w * (size_t)h;
    for (size_t i = 0; i < n; ++i) {
        fb[i] = color;
    }
}

static void fb_hline(uint16_t *fb, int w, int h, int y, uint16_t color) {
    if (y < 0 || y >= h) {
        return;
    }
    uint16_t *row = fb + (size_t)y * (size_t)w;
    for (int x = 0; x < w; ++x) {
        row[x] = color;
    }
}

static void fb_vline(uint16_t *fb, int w, int h, int x, uint16_t color) {
    if (x < 0 || x >= w) {
        return;
    }
    for (int y = 0; y < h; ++y) {
        fb[(size_t)y * (size_t)w + (size_t)x] = color;
    }
}

// Dark scope background with a boresight line and range gridlines.
// Matches radar_overlay_project: bottom-center is the operator, the vertical
// centre is boresight, and near the top is maximum range.
static void draw_reticle(uint16_t *fb, int w, int h) {
    const uint16_t bg = rgb565(2, 6, 10);
    const uint16_t grid = rgb565(0, 60, 40);
    const uint16_t boresight = rgb565(0, 110, 70);

    fb_fill(fb, w, h, bg);
    fb_hline(fb, w, h, h / 4, grid);
    fb_hline(fb, w, h, h / 2, grid);
    fb_hline(fb, w, h, (3 * h) / 4, grid);
    fb_vline(fb, w, h, w / 2, boresight);
}

// Send the finished landscape image to the panel, rotating in software for
// portrait orientations (panel is narrower than the render buffer).
static void present(const uint16_t *land) {
    if (!st7789_ready()) {
        return;
    }
    if (!rot_fb) {
        st7789_blit(land);  // landscape: panel matches the render buffer
        s_present_buf = land;
        s_present_w = ui_width;
        s_present_h = ui_height;
        return;
    }

    // Portrait: transpose the ui_width x ui_height landscape image into the
    // 240x320 panel buffer.
    const int dw = st7789_panel_width();
    const int dh = st7789_panel_height();
    for (int dy = 0; dy < dh; ++dy) {
        for (int dx = 0; dx < dw; ++dx) {
#if CONFIG_AIRSOFT_DISPLAY_ROTATION_270
            const int sx = (ui_width - 1) - dy;
            const int sy = (ui_height - 1) - dx;
#else  // CONFIG_AIRSOFT_DISPLAY_ROTATION_90
            const int sx = dy;
            const int sy = (ui_height - 1) - dx;
#endif
            rot_fb[(size_t)dy * (size_t)dw + (size_t)dx] =
                land[(size_t)sy * (size_t)ui_width + (size_t)sx];
        }
    }
    st7789_blit(rot_fb);
    s_present_buf = rot_fb;
    s_present_w = dw;
    s_present_h = dh;
}

void scope_ui_init(int width, int height) {
    ui_width = width;
    ui_height = height;

    if (st7789_init() != ESP_OK) {
        ESP_LOGE(TAG, "display init failed; falling back to serial preview only");
    }

    const size_t land_bytes = (size_t)width * (size_t)height * sizeof(uint16_t);
    const bool portrait = st7789_ready() && st7789_panel_width() != width;

    if (portrait) {
        // Render buffer is CPU-only (transpose source); panel buffer needs DMA.
        ui_fb = heap_caps_malloc(land_bytes, MALLOC_CAP_SPIRAM);
        if (!ui_fb) {
            ui_fb = heap_caps_malloc(land_bytes, MALLOC_CAP_DEFAULT);
        }
        const size_t panel_bytes =
            (size_t)st7789_panel_width() * (size_t)st7789_panel_height() * sizeof(uint16_t);
        rot_fb = heap_caps_malloc(panel_bytes, MALLOC_CAP_DMA);
        if (!rot_fb) {
            ESP_LOGE(TAG, "no DMA memory for rotated panel buffer");
        }
    } else {
        // Landscape: ui_fb is blitted directly, so it must be DMA-capable.
        ui_fb = heap_caps_malloc(land_bytes, MALLOC_CAP_DMA);
        if (!ui_fb) {
            ui_fb = heap_caps_malloc(land_bytes, MALLOC_CAP_SPIRAM);
        }
        rot_fb = NULL;
    }
    if (!ui_fb) {
        ESP_LOGE(TAG, "no memory for %dx%d framebuffer", width, height);
    }

    ESP_LOGI(TAG, "overlay compositor ready for %dx%d (%s)", width, height,
             portrait ? "portrait" : "landscape");
}

void scope_ui_composite(
    const airsoft_overlay_t *overlay,
    const airsoft_radar_frame_t *frame,
    uint16_t *live_frame,
    int live_width,
    int live_height) {
    if (!overlay || !live_frame || !ui_fb) {
        return;
    }

    if (live_width == ui_width && live_height == ui_height) {
        // Copy the camera frame (PSRAM) into the render buffer, draw blips, present.
        memcpy(ui_fb, live_frame, (size_t)ui_width * (size_t)ui_height * sizeof(uint16_t));
        overlay_draw_contacts(
            ui_fb, ui_width, ui_height, overlay, (float)CONFIG_AIRSOFT_MAX_RANGE_M);
        present(ui_fb);
    }

    (void)frame;
}

void scope_ui_render_scope(const airsoft_overlay_t *overlay) {
    if (!overlay || !ui_fb) {
        return;
    }

    draw_reticle(ui_fb, ui_width, ui_height);
    overlay_draw_contacts(
        ui_fb, ui_width, ui_height, overlay, (float)CONFIG_AIRSOFT_MAX_RANGE_M);
    present(ui_fb);
}

void scope_ui_serial_preview(const airsoft_overlay_t *overlay) {
#if CONFIG_AIRSOFT_SERIAL_PREVIEW
    if (!overlay) {
        return;
    }

    char row[96];
    const int rows = 12;
    const int cols = 32;
    ESP_LOGI(TAG, "---- contact overlay (%dx%d) ----", ui_width, ui_height);

    for (int y = 0; y < rows; ++y) {
        memset(row, ' ', (size_t)cols);
        row[cols] = '\0';
        for (uint8_t i = 0; i < overlay->count; ++i) {
            if (!overlay->blips[i].valid) {
                continue;
            }
            const int px = (overlay->blips[i].x * cols) / ui_width;
            const int py = (overlay->blips[i].y * rows) / ui_height;
            if (py == y && px >= 0 && px < cols) {
                row[px] = i == 0 ? '@' : '+';
            }
        }
        ESP_LOGI(TAG, "|%s|", row);
    }

    for (uint8_t i = 0; i < overlay->count; ++i) {
        if (!overlay->blips[i].valid) {
            continue;
        }
        ESP_LOGI(
            TAG,
            "contact %u: %.1fm @ %.0f deg -> screen (%d,%d) [behind cover]",
            (unsigned)(i + 1),
            overlay->blips[i].range_m,
            overlay->blips[i].bearing_deg,
            overlay->blips[i].x,
            overlay->blips[i].y);
    }
#else
    (void)overlay;
#endif
}
