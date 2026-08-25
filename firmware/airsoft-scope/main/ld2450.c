// LD2450 UART reader. Frame decoding lives in ld2450_parser.c so it can be
// unit-tested on the host; this file owns the UART driver and receive timing.
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ld2450.h"
#include "ld2450_parser.h"
#include "pins.h"

static const char *TAG = "ld2450";

#define LD2450_UART_BUF_SIZE 2048

static ld2450_parser_t s_parser;
static ld2450_frame_cb_t frame_callback;
static void *callback_context;

// The pure parser delivers decoded frames without a timestamp; stamp the
// receive time here before forwarding to the application callback.
static void stamp_and_forward(const airsoft_radar_frame_t *parsed, void *context) {
    (void)context;
    airsoft_radar_frame_t frame = *parsed;
    frame.updated_us = (uint64_t)esp_timer_get_time();
    frame.has_regions = false;
    if (frame_callback) {
        frame_callback(&frame, callback_context);
    }
}

static void uart_reader_task(void *arg) {
    (void)arg;
    uint8_t chunk[128];
    while (true) {
        const int read = uart_read_bytes(
            CONFIG_AIRSOFT_LD2450_UART_NUM, chunk, sizeof(chunk), pdMS_TO_TICKS(50));
        if (read > 0) {
            ld2450_parser_feed(&s_parser, chunk, (size_t)read);
        }
    }
}

esp_err_t ld2450_init(const ld2450_config_t *config) {
    ESP_RETURN_ON_FALSE(config && config->callback, ESP_ERR_INVALID_ARG, TAG, "callback required");
    frame_callback = config->callback;
    callback_context = config->context;
    ld2450_parser_init(&s_parser, &stamp_and_forward, NULL);

    const uart_config_t uart_cfg = {
        .baud_rate = CONFIG_AIRSOFT_LD2450_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(
        uart_driver_install(CONFIG_AIRSOFT_LD2450_UART_NUM, LD2450_UART_BUF_SIZE, 0, 0, NULL, 0),
        TAG,
        "uart install failed");
    ESP_RETURN_ON_ERROR(
        uart_param_config(CONFIG_AIRSOFT_LD2450_UART_NUM, &uart_cfg), TAG, "uart config failed");
    ESP_RETURN_ON_ERROR(
        uart_set_pin(
            CONFIG_AIRSOFT_LD2450_UART_NUM,
            AIRSOFT_PIN_LD2450_TX,
            AIRSOFT_PIN_LD2450_RX,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE),
        TAG,
        "uart pins failed");

    if (xTaskCreate(uart_reader_task, "ld2450_rx", 4096, NULL, 5, NULL) != pdPASS) {
        uart_driver_delete(CONFIG_AIRSOFT_LD2450_UART_NUM);
        frame_callback = NULL;
        callback_context = NULL;
        ESP_LOGE(TAG, "unable to create UART reader task");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(
        TAG,
        "LD2450 UART%d @ %d baud on TX=%d RX=%d",
        CONFIG_AIRSOFT_LD2450_UART_NUM,
        CONFIG_AIRSOFT_LD2450_BAUD,
        AIRSOFT_PIN_LD2450_TX,
        AIRSOFT_PIN_LD2450_RX);
    return ESP_OK;
}
