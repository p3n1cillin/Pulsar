#include "esp_camera.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "scope_camera.h"

static const char *TAG = "scope_camera";
static bool camera_ready;
static camera_fb_t *held_frame;

#if CONFIG_AIRSOFT_CAMERA_BOARD_S3_CAM

static camera_config_t s3_cam_config(void) {
    camera_config_t config = {
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = 15,
        .pin_sccb_sda = 4,
        .pin_sccb_scl = 5,
        .pin_d7 = 16,
        .pin_d6 = 17,
        .pin_d5 = 18,
        .pin_d4 = 12,
        .pin_d3 = 10,
        .pin_d2 = 8,
        .pin_d1 = 9,
        .pin_d0 = 11,
        .pin_vsync = 6,
        .pin_href = 7,
        .pin_pclk = 13,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    return config;
}

#endif

esp_err_t scope_camera_init(void) {
    camera_ready = false;
    held_frame = NULL;

#if CONFIG_AIRSOFT_CAMERA_BOARD_S3_CAM
    const camera_config_t config = s3_cam_config();
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed: %s", esp_err_to_name(err));
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        sensor->set_vflip(sensor, 1);
        sensor->set_hmirror(sensor, 1);
    }

    camera_ready = true;
    ESP_LOGI(TAG, "OV2640 live feed ready (QVGA RGB565, PSRAM)");
    return ESP_OK;
#else
    ESP_LOGI(TAG, "camera disabled — enable ESP32-S3-CAM in menuconfig when board arrives");
    return ESP_OK;
#endif
}

bool scope_camera_ready(void) {
    return camera_ready;
}

bool scope_camera_grab_rgb565(uint16_t **pixels, int *width, int *height) {
    if (!camera_ready || !pixels || !width || !height) {
        return false;
    }

    if (held_frame) {
        esp_camera_fb_return(held_frame);
        held_frame = NULL;
    }

    held_frame = esp_camera_fb_get();
    if (!held_frame || held_frame->format != PIXFORMAT_RGB565) {
        if (held_frame) {
            esp_camera_fb_return(held_frame);
            held_frame = NULL;
        }
        return false;
    }

    *pixels = (uint16_t *)held_frame->buf;
    *width = (int)held_frame->width;
    *height = (int)held_frame->height;
    return true;
}

void scope_camera_release(void) {
    if (held_frame) {
        esp_camera_fb_return(held_frame);
        held_frame = NULL;
    }
}
