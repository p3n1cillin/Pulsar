#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

// Initialise the ST7789V panel on the SPI pins defined in pins.h. The panel
// orientation is taken from CONFIG_AIRSOFT_DISPLAY_ROTATION.
esp_err_t st7789_init(void);

// Push a full panel-sized RGB565 framebuffer (st7789_panel_width x
// st7789_panel_height) to the display.
esp_err_t st7789_blit(const uint16_t *framebuffer);

// True once st7789_init has succeeded.
bool st7789_ready(void);

// Panel dimensions after rotation: 320x240 for 0/180, 240x320 for 90/270.
int st7789_panel_width(void);
int st7789_panel_height(void);
