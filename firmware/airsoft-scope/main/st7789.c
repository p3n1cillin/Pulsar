#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "pins.h"
#include "st7789.h"

static const char *TAG = "st7789";

// SPI2 (FSPI). 20 MHz is reliable over jumper wires; raise once on a PCB.
#define ST7789_SPI_HOST SPI2_HOST
#define ST7789_PIXEL_CLOCK_HZ (20 * 1000 * 1000)

// Native panel resolution (portrait).
#define ST7789_NATIVE_W 240
#define ST7789_NATIVE_H 320
#define ST7789_MAX_PX (ST7789_NATIVE_W * ST7789_NATIVE_H)

static esp_lcd_panel_handle_t s_panel;
static int s_width;
static int s_height;
static bool s_ready;

bool st7789_ready(void) {
    return s_ready;
}

int st7789_panel_width(void) {
    return s_width;
}

int st7789_panel_height(void) {
    return s_height;
}

esp_err_t st7789_init(void) {
    // Backlight on (steady). BL can also be tied to 3V3 in hardware.
    const gpio_config_t bk_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << AIRSOFT_PIN_LCD_BL,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&bk_cfg), TAG, "backlight gpio");
    gpio_set_level(AIRSOFT_PIN_LCD_BL, 1);

    const spi_bus_config_t bus_cfg = {
        .sclk_io_num = AIRSOFT_PIN_LCD_SCLK,
        .mosi_io_num = AIRSOFT_PIN_LCD_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ST7789_MAX_PX * (int)sizeof(uint16_t) + 16,
    };
    ESP_RETURN_ON_ERROR(
        spi_bus_initialize(ST7789_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO), TAG, "spi bus init");

    esp_lcd_panel_io_handle_t io = NULL;
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = AIRSOFT_PIN_LCD_DC,
        .cs_gpio_num = AIRSOFT_PIN_LCD_CS,
        .pclk_hz = ST7789_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)ST7789_SPI_HOST, &io_cfg, &io),
        TAG,
        "panel io");

    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = AIRSOFT_PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel), TAG, "new st7789");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_panel, true), TAG, "invert");

    // 0/180 -> landscape via swap_xy; 90/270 are produced as a native-portrait
    // 240x320 image rotated in software by scope_ui, so swap_xy stays off.
#if CONFIG_AIRSOFT_DISPLAY_ROTATION_180
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(s_panel, true), TAG, "swap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_panel, false, true), TAG, "mirror");
    s_width = ST7789_NATIVE_H;
    s_height = ST7789_NATIVE_W;
#elif CONFIG_AIRSOFT_DISPLAY_ROTATION_90
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(s_panel, false), TAG, "swap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_panel, false, false), TAG, "mirror");
    s_width = ST7789_NATIVE_W;
    s_height = ST7789_NATIVE_H;
#elif CONFIG_AIRSOFT_DISPLAY_ROTATION_270
    // Rotation is done in software (scope_ui transpose); keep the panel neutral
    // so the debug stream matches the glass.
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(s_panel, false), TAG, "swap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_panel, false, false), TAG, "mirror");
    s_width = ST7789_NATIVE_W;
    s_height = ST7789_NATIVE_H;
#else  // CONFIG_AIRSOFT_DISPLAY_ROTATION_0
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(s_panel, true), TAG, "swap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_panel, true, false), TAG, "mirror");
    s_width = ST7789_NATIVE_H;
    s_height = ST7789_NATIVE_W;
#endif

    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_panel, 0, 0), TAG, "gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "disp on");

    s_ready = true;
    ESP_LOGI(TAG, "ST7789V ready (%dx%d, SPI2 %d MHz)", s_width, s_height, ST7789_PIXEL_CLOCK_HZ / 1000000);
    return ESP_OK;
}

esp_err_t st7789_blit(const uint16_t *framebuffer) {
    if (!s_ready || !framebuffer) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_lcd_panel_draw_bitmap(s_panel, 0, 0, s_width, s_height, framebuffer);
}
