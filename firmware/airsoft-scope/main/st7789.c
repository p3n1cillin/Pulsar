#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
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

// Backlight PWM. esp32-camera owns LEDC timer 0 / channel 0 for XCLK, so the
// backlight uses timer 1 / channel 1 to avoid retuning the camera clock.
#define BL_LEDC_MODE LEDC_LOW_SPEED_MODE
#define BL_LEDC_TIMER LEDC_TIMER_1
#define BL_LEDC_CHANNEL LEDC_CHANNEL_1
#define BL_LEDC_RESOLUTION LEDC_TIMER_10_BIT
#define BL_LEDC_MAX_DUTY ((1U << 10) - 1U)
#define BL_LEDC_FREQ_HZ 20000  // above audible range; no coil whine

static esp_lcd_panel_handle_t s_panel;
static bool s_backlight_pwm;
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
    // Backlight starts fully on; scope_controls applies the saved level.
    const ledc_timer_config_t bl_timer = {
        .speed_mode = BL_LEDC_MODE,
        .duty_resolution = BL_LEDC_RESOLUTION,
        .timer_num = BL_LEDC_TIMER,
        .freq_hz = BL_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t bl_channel = {
        .gpio_num = AIRSOFT_PIN_LCD_BL,
        .speed_mode = BL_LEDC_MODE,
        .channel = BL_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BL_LEDC_TIMER,
        .duty = BL_LEDC_MAX_DUTY,
        .hpoint = 0,
    };
    s_backlight_pwm = ledc_timer_config(&bl_timer) == ESP_OK &&
        ledc_channel_config(&bl_channel) == ESP_OK;
    if (!s_backlight_pwm) {
        // Fall back to a steady backlight so the display is still usable.
        ESP_LOGW(TAG, "backlight PWM unavailable; using fixed full brightness");
        const gpio_config_t bk_cfg = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << AIRSOFT_PIN_LCD_BL,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&bk_cfg), TAG, "backlight gpio");
        gpio_set_level(AIRSOFT_PIN_LCD_BL, 1);
    }

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

void st7789_set_backlight(uint8_t percent) {
    if (!s_backlight_pwm) {
        return;
    }
    if (percent > 100) {
        percent = 100;
    }
    const uint32_t duty = (BL_LEDC_MAX_DUTY * (uint32_t)percent + 50U) / 100U;
    ledc_set_duty(BL_LEDC_MODE, BL_LEDC_CHANNEL, duty);
    ledc_update_duty(BL_LEDC_MODE, BL_LEDC_CHANNEL);
}
