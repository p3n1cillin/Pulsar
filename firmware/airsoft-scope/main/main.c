#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "camera_motion.h"
#include "ld2450.h"
#include "radar_overlay.h"
#include "scope_camera.h"
#include "scope_types.h"
#include "scope_ui.h"
#include "sim_radar.h"
#include "target_tracker.h"
#include "web_stream.h"

static const char *TAG = "airsoft_scope";

static airsoft_radar_frame_t latest_frame;
static SemaphoreHandle_t radar_lock;
static target_tracker_t player_tracker;
static camera_motion_state_t camera_motion;
static uint64_t last_radar_processed_us;

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
        airsoft_camera_frame_t camera_frame = {0};
        airsoft_track_frame_t tracks = {0};
        airsoft_overlay_t overlay = {0};
        copy_latest_frame(&frame);
        const uint64_t now_us = (uint64_t)esp_timer_get_time();
        const bool radar_is_new =
            frame.updated_us != 0 && frame.updated_us != last_radar_processed_us;
        if (radar_is_new) {
            last_radar_processed_us = frame.updated_us;
        }

        int width = CONFIG_AIRSOFT_DISPLAY_WIDTH;
        int height = CONFIG_AIRSOFT_DISPLAY_HEIGHT;
        uint16_t *live_pixels = NULL;

        if (scope_camera_grab_rgb565(&live_pixels, &width, &height)) {
            camera_motion_process(
                &camera_motion,
                live_pixels,
                width,
                height,
                (float)CONFIG_AIRSOFT_CAMERA_HFOV_DEG,
                now_us,
                &camera_frame);
            target_tracker_update(
                &player_tracker,
                radar_is_new ? &frame : NULL,
                &camera_frame,
                now_us,
                &tracks);
            radar_overlay_project(
                &tracks,
                width,
                height,
                (float)CONFIG_AIRSOFT_CAMERA_HFOV_DEG,
                (float)CONFIG_AIRSOFT_MAX_RANGE_M,
                &overlay);
            scope_ui_composite(&overlay, live_pixels, width, height);
            scope_camera_release();
        } else {
            target_tracker_update(
                &player_tracker,
                radar_is_new ? &frame : NULL,
                NULL,
                now_us,
                &tracks);
            radar_overlay_project(
                &tracks,
                CONFIG_AIRSOFT_DISPLAY_WIDTH,
                CONFIG_AIRSOFT_DISPLAY_HEIGHT,
                (float)CONFIG_AIRSOFT_CAMERA_HFOV_DEG,
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
    camera_motion_init(&camera_motion);
    const target_tracker_config_t tracker_config = {
        .radar_position_gate_m = (float)CONFIG_AIRSOFT_TRACK_RADAR_GATE_CM / 100.0f,
        .camera_bearing_gate_deg = (float)CONFIG_AIRSOFT_TRACK_CAMERA_GATE_DEG,
        .radar_half_fov_deg = (float)CONFIG_AIRSOFT_RADAR_HFOV_DEG * 0.5f,
        .min_range_m = (float)CONFIG_AIRSOFT_MIN_RANGE_M / 100.0f,
        .max_range_m = (float)CONFIG_AIRSOFT_MAX_RANGE_M,
        .radar_boresight_deg = (float)CONFIG_AIRSOFT_RADAR_BORESIGHT_OFFSET_DEG,
        .radar_hold_ms = CONFIG_AIRSOFT_TRACK_RADAR_HOLD_MS,
        .camera_hold_ms = CONFIG_AIRSOFT_TRACK_CAMERA_HOLD_MS,
        .coast_ms = CONFIG_AIRSOFT_TRACK_COAST_MS,
    };
    target_tracker_init(&player_tracker, &tracker_config);

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
        "Pipeline: camera motion + LD2450 -> persistent player tracks -> %s",
        scope_camera_ready() ? "live camera frame" : "serial preview until S3-CAM enabled");
}
