# Requirements are listed unconditionally. ESP-IDF expands component
# requirements before it loads sdkconfig, so an if(CONFIG_...) around them
# never fires and the camera and Wi-Fi stream builds fail on missing headers.
# The code that uses these components is guarded by its own #if CONFIG_...,
# and esp32-camera is already an unconditional dependency in idf_component.yml.
set(airsoft_private_requires
    driver esp_timer esp_lcd
    esp32-camera
    esp_wifi esp_http_server nvs_flash esp_netif esp_event)

idf_component_register(
    SRCS
        "main.c"
        "camera_motion.c"
        "ld2450.c"
        "ld2450_parser.c"
        "radar_overlay.c"
        "scope_ui.c"
        "scope_camera.c"
        "overlay_draw.c"
        "sim_radar.c"
        "st7789.c"
        "target_tracker.c"
        "web_stream.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES ${airsoft_private_requires}
)
