// Housing buttons, persisted operator settings, and the optional battery
// monitor. Decisions live in scope_controls_logic.c / button_input.c /
// battery_gauge.c (host-tested); this file only owns the hardware and NVS.
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#if CONFIG_AIRSOFT_BATTERY_SENSE
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#endif

#include "battery_gauge.h"
#include "button_input.h"
#include "scope_controls.h"
#include "st7789.h"

static const char *TAG = "scope_controls";

#define SETTINGS_NAMESPACE "pulsar"
#define KEY_BRIGHTNESS "bright"
#define KEY_BORESIGHT "bore"
#define KEY_DECLUTTER "declut"
#define KEY_BORESIGHT_DEFAULT "bore_def"

#define CONTROLS_POLL_MS 10
#define BATTERY_PERIOD_MS 1000

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static scope_controls_status_t s_status;

static int16_t default_boresight_decideg(void) {
    return (int16_t)(CONFIG_AIRSOFT_RADAR_BORESIGHT_OFFSET_DEG * 10);
}

static void settings_load(scope_settings_t *settings) {
    settings->brightness_index = SCOPE_BRIGHTNESS_LEVELS - 1;
    settings->boresight_decideg = default_boresight_decideg();
    settings->declutter = false;
    settings->calibrating = false;

#if !CONFIG_AIRSOFT_CONTROLS
    // Without buttons nothing can change the stored values, so menuconfig is
    // the only source of truth; stale NVS settings must not shadow it.
    return;
#endif
    nvs_handle_t handle;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return;  // first boot: nothing stored yet
    }
    uint8_t u8 = 0;
    int16_t i16 = 0;
    int16_t stored_default = 0;
    if (nvs_get_u8(handle, KEY_BRIGHTNESS, &u8) == ESP_OK) {
        settings->brightness_index = u8;
    }
    // A field calibration is relative to the menuconfig default it started
    // from. If the firmware's default has changed since, drop the old one.
    if (nvs_get_i16(handle, KEY_BORESIGHT, &i16) == ESP_OK &&
        nvs_get_i16(handle, KEY_BORESIGHT_DEFAULT, &stored_default) == ESP_OK &&
        stored_default == default_boresight_decideg()) {
        settings->boresight_decideg = i16;
    }
    if (nvs_get_u8(handle, KEY_DECLUTTER, &u8) == ESP_OK) {
        settings->declutter = u8 != 0;
    }
    nvs_close(handle);
    scope_settings_sanitize(settings);
}

#if CONFIG_AIRSOFT_CONTROLS
static void settings_save(const scope_settings_t *settings) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, KEY_BRIGHTNESS, settings->brightness_index);
        if (err == ESP_OK) err = nvs_set_i16(handle, KEY_BORESIGHT, settings->boresight_decideg);
        if (err == ESP_OK) {
            err = nvs_set_i16(handle, KEY_BORESIGHT_DEFAULT, default_boresight_decideg());
        }
        if (err == ESP_OK) err = nvs_set_u8(handle, KEY_DECLUTTER, settings->declutter ? 1 : 0);
        if (err == ESP_OK) err = nvs_commit(handle);
        nvs_close(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "settings not saved: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "saved: brightness %u%%, boresight %+.1f deg, declutter %s",
                 (unsigned)scope_brightness_percent(settings->brightness_index),
                 (double)settings->boresight_decideg / 10.0,
                 settings->declutter ? "on" : "off");
    }
}
#endif

static esp_err_t nvs_ready(void) {
    esp_err_t err = nvs_flash_init();  // idempotent; web_stream calls it too
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        err = nvs_flash_init();
    }
    return err;
}

#if CONFIG_AIRSOFT_BATTERY_SENSE
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_adc_cali;
static adc_channel_t s_adc_channel;
static battery_gauge_t s_gauge;
static bool s_battery_ok;

static esp_err_t battery_init(void) {
    adc_unit_t unit;
    ESP_RETURN_ON_ERROR(
        adc_oneshot_io_to_channel(CONFIG_AIRSOFT_BATTERY_ADC_GPIO, &unit, &s_adc_channel),
        TAG, "battery GPIO is not an ADC pin");
    ESP_RETURN_ON_FALSE(unit == ADC_UNIT_1, ESP_ERR_INVALID_ARG, TAG,
                        "battery sense needs an ADC1 pin (ADC2 conflicts with Wi-Fi)");
    const adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_1};
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG, "adc unit");
    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    const esp_err_t chan_err = adc_oneshot_config_channel(s_adc, s_adc_channel, &chan_cfg);
    if (chan_err != ESP_OK) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
        ESP_LOGE(TAG, "adc chan: %s", esp_err_to_name(chan_err));
        return chan_err;
    }
    const adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = s_adc_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) != ESP_OK) {
        s_adc_cali = NULL;
        ESP_LOGW(TAG, "ADC calibration unavailable; battery reading is approximate");
    }
    return ESP_OK;
}

static void battery_sample(void) {
    if (!s_battery_ok) {
        return;
    }
    int raw = 0;
    int pin_mv = 0;
    if (adc_oneshot_read(s_adc, s_adc_channel, &raw) != ESP_OK) {
        return;
    }
    if (!s_adc_cali || adc_cali_raw_to_voltage(s_adc_cali, raw, &pin_mv) != ESP_OK) {
        pin_mv = raw * 3100 / 4095;  // uncalibrated estimate at 12 dB
    }
    const uint32_t cell_mv =
        ((uint32_t)pin_mv * (uint32_t)CONFIG_AIRSOFT_BATTERY_DIVIDER_X100) / 100U;
    const uint32_t filtered = battery_gauge_update(&s_gauge, cell_mv);
    const uint8_t percent = battery_percent_from_mv(filtered);
    taskENTER_CRITICAL(&s_lock);
    s_status.battery_valid = true;
    s_status.battery_mv = (uint16_t)filtered;
    s_status.battery_percent = percent;
    taskEXIT_CRITICAL(&s_lock);
}
#endif

#if CONFIG_AIRSOFT_CONTROLS
static const gpio_num_t s_button_pins[2] = {
    (gpio_num_t)CONFIG_AIRSOFT_BUTTON_MODE_GPIO,
    (gpio_num_t)CONFIG_AIRSOFT_BUTTON_ADJUST_GPIO,
};
static button_state_t s_buttons[2];
static bool s_buttons_ok;

static esp_err_t buttons_init(uint32_t now_ms) {
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << CONFIG_AIRSOFT_BUTTON_MODE_GPIO) |
                        (1ULL << CONFIG_AIRSOFT_BUTTON_ADJUST_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // buttons short to GND
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "button gpio");
    for (int i = 0; i < 2; ++i) {
        button_init(&s_buttons[i], gpio_get_level(s_button_pins[i]) == 0, now_ms);
    }
    return ESP_OK;
}

static void buttons_poll(uint32_t now_ms) {
    static const button_timing_t timing = {
        .debounce_ms = 30,
        .long_press_ms = CONFIG_AIRSOFT_BUTTON_LONG_PRESS_MS,
    };
    for (int i = 0; i < 2; ++i) {
        const button_event_t event =
            button_update(&s_buttons[i], &timing, gpio_get_level(s_button_pins[i]) == 0, now_ms);
        if (event == BUTTON_EVENT_NONE) {
            continue;
        }
        taskENTER_CRITICAL(&s_lock);
        const uint8_t changes = scope_controls_handle(
            &s_status.settings, (scope_button_t)i, event, default_boresight_decideg());
        const scope_settings_t settings = s_status.settings;
        taskEXIT_CRITICAL(&s_lock);

        if (changes & SCOPE_CHANGE_BRIGHTNESS) {
            st7789_set_backlight(scope_brightness_percent(settings.brightness_index));
        }
        if (changes & SCOPE_CHANGE_BORESIGHT) {
            ESP_LOGI(TAG, "boresight %+.1f deg", (double)settings.boresight_decideg / 10.0);
        }
        if ((changes & SCOPE_CHANGE_VIEW) && settings.calibrating) {
            ESP_LOGI(TAG, "calibration: align a walking teammate's blip with them, hold MODE to save");
        }
        if (changes & SCOPE_CHANGE_PERSIST) {
            settings_save(&settings);
        }
    }
}
#endif

#if CONFIG_AIRSOFT_CONTROLS || CONFIG_AIRSOFT_BATTERY_SENSE
static void controls_task(void *arg) {
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_battery_ms = 0;
    while (true) {
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
#if CONFIG_AIRSOFT_CONTROLS
        if (s_buttons_ok) {
            buttons_poll(now_ms);
        }
#endif
#if CONFIG_AIRSOFT_BATTERY_SENSE
        if (last_battery_ms == 0 || (uint32_t)(now_ms - last_battery_ms) >= BATTERY_PERIOD_MS) {
            battery_sample();
            last_battery_ms = now_ms;
        }
#else
        (void)last_battery_ms;
#endif
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROLS_POLL_MS));
    }
}
#endif

esp_err_t scope_controls_init(void) {
    memset(&s_status, 0, sizeof(s_status));
    const esp_err_t nvs_err = nvs_ready();
    if (nvs_err != ESP_OK) {
        ESP_LOGW(TAG, "NVS unavailable (%s); settings will not persist", esp_err_to_name(nvs_err));
    }
    scope_settings_t settings;
    settings_load(&settings);
    s_status.settings = settings;
    st7789_set_backlight(scope_brightness_percent(settings.brightness_index));
    ESP_LOGI(TAG, "brightness %u%%, boresight %+.1f deg, declutter %s",
             (unsigned)scope_brightness_percent(settings.brightness_index),
             (double)settings.boresight_decideg / 10.0,
             settings.declutter ? "on" : "off");

#if CONFIG_AIRSOFT_CONTROLS || CONFIG_AIRSOFT_BATTERY_SENSE
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
#if CONFIG_AIRSOFT_CONTROLS
    // A bad button GPIO must not take the battery monitor down with it.
    s_buttons_ok = buttons_init(now_ms) == ESP_OK;
    if (s_buttons_ok) {
        ESP_LOGI(TAG, "buttons: MODE=GPIO%d ADJUST=GPIO%d (active low)",
                 CONFIG_AIRSOFT_BUTTON_MODE_GPIO, CONFIG_AIRSOFT_BUTTON_ADJUST_GPIO);
    } else {
        ESP_LOGW(TAG, "buttons disabled: check the button GPIOs in menuconfig");
    }
#else
    (void)now_ms;
#endif
#if CONFIG_AIRSOFT_BATTERY_SENSE
    s_battery_ok = battery_init() == ESP_OK;
    if (!s_battery_ok) {
        ESP_LOGW(TAG, "battery monitor disabled");
    }
    battery_sample();
#endif
    if (xTaskCreate(controls_task, "controls", 4096, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
#endif
    return ESP_OK;
}

void scope_controls_get(scope_controls_status_t *out) {
    if (!out) {
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    *out = s_status;
    taskEXIT_CRITICAL(&s_lock);
}
