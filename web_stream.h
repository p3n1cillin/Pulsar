#pragma once

#include "esp_err.h"

// Start the debug Wi-Fi camera stream (no-op unless
// CONFIG_AIRSOFT_DEBUG_WIFI_STREAM is enabled). Joins the configured 2.4 GHz
// network and serves the current display frame as JPEG at GET /jpg, plus an
// MJPEG stream at GET /stream.
esp_err_t web_stream_start(void);
