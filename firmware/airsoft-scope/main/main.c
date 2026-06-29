#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "ld2450.h"
#include "radar_overlay.h"
#include "scope_camera.h"
#include "scope_types.h"
#include "scope_ui.h"
#include "sim_radar.h"
#include "web_stream.h"

static const char *TAG = "airsoft_scope";

static airsoft_radar_frame_t latest_frame;
static SemaphoreHandle_t radar_lock;

static void on_radar_frame(const airsoft_radar_frame_t *frame, void *context) {
    (void)context;
    if (!frame || !radar_lock) {
        return;
    }
    if (xSemaphoreTake(radar_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        latest_frame = *frame;
        xSemaphoreGive(radar_lock);
    }
}

static void copy_latest_frame(airsoft_radar_frame_t *dest) {
    if (!dest || !radar_lock) {
        return;
    }
    if (xSemaphoreTake(radar_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
        *dest = latest_frame;
        xSemaphoreGive(radar_lock);
    }
}

static void render_task(void *arg) {
    (void)arg;
    const TickType_t frame_delay = scope_camera_ready() ? pdMS_TO_TICKS(100) : pdMS_TO_TICKS(250);

    while (true) {
        airsoft_radar_frame_t frame = {0};
        airsoft_overlay_t overlay = {0};
        copy_latest_frame(&frame);

        int width = CONFIG_AIRSOFT_DISPLAY_WIDTH;
        int height = CONFIG_AIRSOFT_DISPLAY_HEIGHT;
        uint16_t *live_pixels = NULL;

        if (scope_camera_grab_rgb565(&live_pixels, &width, &height)) {
            radar_overlay_project(
                &frame,
                width,
                height,
                (float)CONFIG_AIRSOFT_CAMERA_HFOV_DEG,
                (float)CONFIG_AIRSOFT_RADAR_HFOV_DEG,
                (float)CONFIG_AIRSOFT_MAX_RANGE_M,
                &overlay);
            scope_ui_composite(&overlay, &frame, live_pixels, width, height);
            scope_camera_release();
        } else {
            radar_overlay_project(
                &frame,
                CONFIG_AIRSOFT_DISPLAY_WIDTH,
                CONFIG_AIRSOFT_DISPLAY_HEIGHT,
                (float)CONFIG_AIRSOFT_CAMERA_HFOV_DEG,
                (float)CONFIG_AIRSOFT_RADAR_HFOV_DEG,
                (float)CONFIG_AIRSOFT_MAX_RANGE_M,
                &overlay);
            scope_ui_render_scope(&overlay);
        }

        scope_ui_serial_preview(&overlay);
        vTaskDelay(frame_delay);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Airsoft scope: radar contacts overlaid on live feed");

    radar_lock = xSemaphoreCreateMutex();
    scope_ui_init(CONFIG_AIRSOFT_DISPLAY_WIDTH, CONFIG_AIRSOFT_DISPLAY_HEIGHT);
    if (scope_camera_init() != ESP_OK) {
        ESP_LOGW(TAG, "camera unavailable; using radar-only scope view");
    }

#if CONFIG_AIRSOFT_SCOPE_MODE_SIMULATION
    ESP_LOGI(TAG, "Radar source: simulation (swap to live LD2450 when wired)");
    ESP_ERROR_CHECK(sim_radar_start(on_radar_frame, NULL));
#else
    ESP_LOGI(TAG, "Radar source: LD2450 UART");
    const ld2450_config_t radar_cfg = {
        .callback = on_radar_frame,
        .context = NULL,
    };
    ESP_ERROR_CHECK(ld2450_init(&radar_cfg));
#endif

    web_stream_start();  // no-op unless the Wi-Fi debug stream is enabled

    xTaskCreate(render_task, "composite", 8192, NULL, 5, NULL);
    ESP_LOGI(
        TAG,
        "Pipeline: LD2450 -> ESP32 -> overlay -> %s",
        scope_camera_ready() ? "live camera frame" : "serial preview until S3-CAM enabled");
}
