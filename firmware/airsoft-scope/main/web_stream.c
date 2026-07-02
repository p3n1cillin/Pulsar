#include "web_stream.h"

#include "sdkconfig.h"

#if CONFIG_AIRSOFT_DEBUG_WIFI_STREAM

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "img_converters.h"
#include "nvs_flash.h"

#include "scope_ui.h"

static const char *TAG = "web_stream";

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_events;
static httpd_handle_t s_httpd;

static bool request_authorized(httpd_req_t *req) {
    static const char prefix[] = "Bearer ";
    const size_t token_len = strlen(CONFIG_AIRSOFT_STREAM_TOKEN);
    const size_t expected_len = sizeof(prefix) - 1 + token_len;
    const size_t header_len = httpd_req_get_hdr_value_len(req, "Authorization");
    char value[128];
    if (token_len == 0 || header_len != expected_len || header_len >= sizeof(value) ||
        httpd_req_get_hdr_value_str(req, "Authorization", value, sizeof(value)) != ESP_OK) {
        return false;
    }

    unsigned difference = 0;
    for (size_t i = 0; i < expected_len; ++i) {
        const char expected = i < sizeof(prefix) - 1
            ? prefix[i]
            : CONFIG_AIRSOFT_STREAM_TOKEN[i - (sizeof(prefix) - 1)];
        difference |= (unsigned char)value[i] ^ (unsigned char)expected;
    }
    return difference == 0;
}

static esp_err_t send_unauthorized(httpd_req_t *req) {
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Bearer");
    return httpd_resp_sendstr(req, "Authorization required");
}

static void wifi_event_handler(
    void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "wifi disconnected, retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "stream ready:  http://" IPSTR "/jpg   (and /stream)", IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

// Convert the current panel frame to JPEG into *out (caller frees with free()).
static bool grab_jpeg(uint8_t **out, size_t *out_len) {
    const uint16_t *buf = NULL;
    int w = 0;
    int h = 0;
    if (!scope_ui_panel_snapshot_acquire(&buf, &w, &h)) {
        return false;
    }
    const bool converted = fmt2jpg(
        (uint8_t *)buf, (size_t)w * (size_t)h * 2, w, h, PIXFORMAT_RGB565, 80, out, out_len);
    scope_ui_panel_snapshot_release();
    return converted;
}

static esp_err_t jpg_handler(httpd_req_t *req) {
    if (!request_authorized(req)) {
        return send_unauthorized(req);
    }
    uint8_t *jpg = NULL;
    size_t jpg_len = 0;
    if (!grab_jpeg(&jpg, &jpg_len)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const esp_err_t err = httpd_resp_send(req, (const char *)jpg, jpg_len);
    free(jpg);
    return err;
}

#define MJPEG_BOUNDARY "fieldnoteframe"

static esp_err_t stream_handler(httpd_req_t *req) {
    if (!request_authorized(req)) {
        return send_unauthorized(req);
    }
    httpd_resp_set_type(
        req, "multipart/x-mixed-replace;boundary=" MJPEG_BOUNDARY);
    char part[96];
    while (true) {
        uint8_t *jpg = NULL;
        size_t jpg_len = 0;
        if (!grab_jpeg(&jpg, &jpg_len)) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        const int n = snprintf(
            part,
            sizeof(part),
            "\r\n--" MJPEG_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            (unsigned)jpg_len);
        esp_err_t err = httpd_resp_send_chunk(req, part, n);
        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(req, (const char *)jpg, jpg_len);
        }
        free(jpg);
        if (err != ESP_OK) {
            break;  // client disconnected
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // ~10 fps
    }
    return ESP_OK;
}

esp_err_t web_stream_start(void) {
    ESP_RETURN_ON_FALSE(CONFIG_AIRSOFT_WIFI_SSID[0] != '\0', ESP_ERR_INVALID_STATE, TAG,
                        "Wi-Fi SSID is empty");
    ESP_RETURN_ON_FALSE(CONFIG_AIRSOFT_STREAM_TOKEN[0] != '\0', ESP_ERR_INVALID_STATE, TAG,
                        "debug stream token is empty");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "NVS init failed");

    s_wifi_events = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_wifi_events, ESP_ERR_NO_MEM, TAG, "event group allocation failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "network init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop init failed");
    ESP_RETURN_ON_FALSE(esp_netif_create_default_wifi_sta(), ESP_ERR_NO_MEM, TAG,
                        "station interface allocation failed");

    const wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "Wi-Fi init failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL), TAG,
        "Wi-Fi event registration failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL), TAG,
        "IP event registration failed");

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, CONFIG_AIRSOFT_WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, CONFIG_AIRSOFT_WIFI_PASS, sizeof(wifi_cfg.sta.password) - 1);
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "Wi-Fi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg), TAG,
                        "Wi-Fi config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");

    ESP_LOGI(TAG, "connecting to SSID '%s' ...", CONFIG_AIRSOFT_WIFI_SSID);
    const EventBits_t connected = xEventGroupWaitBits(
        s_wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
    ESP_RETURN_ON_FALSE((connected & WIFI_CONNECTED_BIT) != 0, ESP_ERR_TIMEOUT, TAG,
                        "Wi-Fi connection timed out");

    httpd_config_t http_cfg = HTTPD_DEFAULT_CONFIG();
    http_cfg.stack_size = 8192;
    if (httpd_start(&s_httpd, &http_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd start failed");
        return ESP_FAIL;
    }
    const httpd_uri_t jpg_uri = {.uri = "/jpg", .method = HTTP_GET, .handler = jpg_handler};
    const httpd_uri_t stream_uri = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler};
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &jpg_uri), TAG,
                        "JPEG route registration failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &stream_uri), TAG,
                        "stream route registration failed");
    return ESP_OK;
}

#else  // CONFIG_AIRSOFT_DEBUG_WIFI_STREAM

esp_err_t web_stream_start(void) {
    return ESP_OK;  // disabled
}

#endif
