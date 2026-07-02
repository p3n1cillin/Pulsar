#include <math.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sim_radar.h"

static sim_radar_cb_t sim_callback;
static void *sim_context;

static void sim_task(void *arg) {
    float phase = 0.0f;
    while (true) {
        airsoft_radar_frame_t frame = {0};
        frame.updated_us = (uint64_t)esp_timer_get_time();
        frame.count = 2;

        frame.targets[0].x_m = sinf(phase) * 2.5f;
        frame.targets[0].y_m = 4.0f + cosf(phase * 0.7f) * 0.8f;
        frame.targets[0].valid = true;

        frame.targets[1].x_m = -1.2f + sinf(phase * 1.3f) * 0.4f;
        frame.targets[1].y_m = 6.5f;
        frame.targets[1].valid = true;

        if (sim_callback) {
            sim_callback(&frame, sim_context);
        }

        phase += 0.18f;
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

esp_err_t sim_radar_start(sim_radar_cb_t callback, void *context) {
    if (!callback) {
        return ESP_ERR_INVALID_ARG;
    }
    sim_callback = callback;
    sim_context = context;
    if (xTaskCreate(sim_task, "sim_radar", 3072, NULL, 4, NULL) != pdPASS) {
        sim_callback = NULL;
        sim_context = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
