set(airsoft_private_requires driver esp_timer esp_lcd)

if(CONFIG_AIRSOFT_CAMERA_BOARD_S3_CAM)
    list(APPEND airsoft_private_requires esp32-camera)
endif()

if(CONFIG_AIRSOFT_DEBUG_WIFI_STREAM)
    list(APPEND airsoft_private_requires
        esp_wifi esp_http_server nvs_flash esp_netif esp_event esp32-camera)
endif()

idf_component_register(
    SRCS
        "main.c"
        "camera_motion.c"
        "ld2450.c"
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
