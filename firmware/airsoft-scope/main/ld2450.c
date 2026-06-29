#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ld2450.h"
#include "pins.h"

static const char *TAG = "ld2450";

#define LD2450_UART_BUF_SIZE 2048
#define LD2450_PARSER_BUF_SIZE 256
#define LD2450_FRAME_LEN 30  // AA FF 03 00 | 3x8 target bytes | 55 CC
#define LD2450_TARGETS 3

static const uint8_t LD2450_HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t LD2450_FOOTER[2] = {0x55, 0xCC};

static ld2450_frame_cb_t frame_callback;
static void *callback_context;
static uint8_t parser_buf[LD2450_PARSER_BUF_SIZE];
static size_t parser_len;

// LD2450 encodes X/Y/speed as sign-magnitude where bit15 is the sign flag
// (1 = positive, 0 = negative). Returns the signed value (mm, or cm/s).
static int16_t ld2450_signed(uint16_t raw) {
    const int16_t magnitude = (int16_t)(raw & 0x7FFF);
    return (raw & 0x8000) ? magnitude : (int16_t)(-magnitude);
}

static void emit_frame(const uint8_t *targets) {
    airsoft_radar_frame_t frame = {0};
    frame.updated_us = (uint64_t)esp_timer_get_time();
    frame.has_regions = false;

    uint8_t count = 0;
    for (int i = 0; i < LD2450_TARGETS && count < AIRSOFT_MAX_TARGETS; ++i) {
        const uint8_t *t = targets + i * 8;
        const uint16_t x_raw = (uint16_t)(t[0] | (t[1] << 8));
        const uint16_t y_raw = (uint16_t)(t[2] | (t[3] << 8));
        // bytes 4-5 speed (cm/s), bytes 6-7 distance resolution -- unused for now.
        if (x_raw == 0 && y_raw == 0) {
            continue;  // empty target slot
        }
        frame.targets[count].x_m = (float)ld2450_signed(x_raw) / 1000.0f;
        frame.targets[count].y_m = (float)ld2450_signed(y_raw) / 1000.0f;
        frame.targets[count].valid = true;
        ++count;
    }
    frame.count = count;

    if (frame_callback) {
        frame_callback(&frame, callback_context);
    }
}

static bool header_at(size_t offset) {
    return parser_buf[offset] == LD2450_HEADER[0] &&
           parser_buf[offset + 1] == LD2450_HEADER[1] &&
           parser_buf[offset + 2] == LD2450_HEADER[2] &&
           parser_buf[offset + 3] == LD2450_HEADER[3];
}

static void shift_parser(size_t count) {
    if (count >= parser_len) {
        parser_len = 0;
        return;
    }
    memmove(parser_buf, parser_buf + count, parser_len - count);
    parser_len -= count;
}

static void consume_bytes(const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        if (parser_len >= LD2450_PARSER_BUF_SIZE) {
            shift_parser(1);
        }
        parser_buf[parser_len++] = data[i];

        while (parser_len >= LD2450_FRAME_LEN) {
            size_t start = 0;
            bool found = false;
            for (; start + 4 <= parser_len; ++start) {
                if (header_at(start)) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                // No header yet; keep only a possible partial header tail.
                if (parser_len > 3) {
                    shift_parser(parser_len - 3);
                }
                break;
            }
            if (start > 0) {
                shift_parser(start);
            }
            if (parser_len < LD2450_FRAME_LEN) {
                break;  // wait for the rest of the frame
            }
            if (parser_buf[LD2450_FRAME_LEN - 2] != LD2450_FOOTER[0] ||
                parser_buf[LD2450_FRAME_LEN - 1] != LD2450_FOOTER[1]) {
                shift_parser(4);  // false header, skip past it
                continue;
            }
            emit_frame(parser_buf + 4);
            shift_parser(LD2450_FRAME_LEN);
        }
    }
}

static void uart_reader_task(void *arg) {
    (void)arg;
    uint8_t chunk[128];
    while (true) {
        const int read = uart_read_bytes(
            CONFIG_AIRSOFT_LD2450_UART_NUM, chunk, sizeof(chunk), pdMS_TO_TICKS(50));
        if (read > 0) {
            consume_bytes(chunk, (size_t)read);
        }
    }
}

esp_err_t ld2450_init(const ld2450_config_t *config) {
    frame_callback = config ? config->callback : NULL;
    callback_context = config ? config->context : NULL;
    parser_len = 0;

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

    xTaskCreate(uart_reader_task, "ld2450_rx", 4096, NULL, 5, NULL);
    ESP_LOGI(
        TAG,
        "LD2450 UART%d @ %d baud on TX=%d RX=%d",
        CONFIG_AIRSOFT_LD2450_UART_NUM,
        CONFIG_AIRSOFT_LD2450_BAUD,
        AIRSOFT_PIN_LD2450_TX,
        AIRSOFT_PIN_LD2450_RX);
    return ESP_OK;
}
