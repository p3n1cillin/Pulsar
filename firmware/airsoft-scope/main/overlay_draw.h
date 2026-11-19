#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "scope_types.h"

// Status drawn on top of the contacts. Pure data so the renderer stays
// host-testable; main.c fills it from the controls / radar link state.
typedef struct {
    bool radar_lost;          // live mode: no LD2450 frame within the timeout
    bool blink_phase;         // toggles ~2 Hz for attention cues
    bool calibrating;         // boresight calibration mode
    float boresight_deg;      // current radar-to-camera offset
    bool declutter;           // radar-backed contacts only
    bool battery_valid;
    uint8_t battery_percent;
} scope_hud_t;

void overlay_draw_contacts(
    uint16_t *framebuffer,
    int width,
    int height,
    const airsoft_overlay_t *overlay,
    float max_range_m);

void overlay_draw_hud(
    uint16_t *framebuffer,
    int width,
    int height,
    const scope_hud_t *hud);

// Draw upper-case text with the built-in 3x5 font, scaled by `scale`.
// Unsupported characters render as blanks. Returns the drawn width in pixels.
int overlay_draw_text(
    uint16_t *framebuffer,
    int width,
    int height,
    int x,
    int y,
    const char *text,
    int scale,
    uint16_t color);

// Width in pixels that overlay_draw_text would use.
int overlay_text_width(const char *text, int scale);

// RGB565 in the panel's wire (big-endian) byte order.
uint16_t overlay_rgb565(uint8_t r, uint8_t g, uint8_t b);
