cmake_minimum_required(VERSION 3.16)
project(pulsar_host_tests C)

enable_testing()

set(FW_MAIN ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/airsoft-scope/main)

# Host-testable firmware units are pure C with no ESP-IDF dependencies.
# camera_motion.c includes the generated sdkconfig.h, so the stub include dir
# comes first and mirrors the production values.
function(pulsar_test name)
    add_executable(${name} ${ARGN})
    target_include_directories(${name} PRIVATE stub ${FW_MAIN})
    if(MSVC)
        target_compile_options(${name} PRIVATE /W4 /WX)
    else()
        target_compile_options(${name} PRIVATE -Wall -Wextra -Werror -pedantic)
        target_link_libraries(${name} PRIVATE m)
    endif()
    add_test(NAME ${name} COMMAND ${name})
endfunction()

pulsar_test(tracking_tests
    test_tracking.c
    ${FW_MAIN}/target_tracker.c
    ${FW_MAIN}/radar_overlay.c
)

pulsar_test(ld2450_tests
    test_ld2450.c
    ${FW_MAIN}/ld2450_parser.c
)

pulsar_test(camera_motion_tests
    test_camera_motion.c
    ${FW_MAIN}/camera_motion.c
)

pulsar_test(overlay_tests
    test_overlay.c
    ${FW_MAIN}/radar_overlay.c
    ${FW_MAIN}/overlay_draw.c
)

pulsar_test(controls_tests
    test_controls.c
    ${FW_MAIN}/button_input.c
    ${FW_MAIN}/scope_controls_logic.c
    ${FW_MAIN}/battery_gauge.c
)
