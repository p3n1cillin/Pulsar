idf_component_register(
    SRCS
        "main.c"
        "ld2450.c"
        "radar_overlay.c"
        "scope_ui.c"
        "scope_camera.c"
        "overlay_draw.c"
        "sim_radar.c"
        "st7789.c"
        "web_stream.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_lcd esp32-camera esp_wifi esp_http_server nvs_flash esp_netif esp_event
)
