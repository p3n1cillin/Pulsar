cmake_minimum_required(VERSION 3.16)
project(pulsar_host_tests C)

enable_testing()

add_executable(tracking_tests
    test_tracking.c
    ../firmware/airsoft-scope/main/target_tracker.c
    ../firmware/airsoft-scope/main/radar_overlay.c
)
target_include_directories(tracking_tests PRIVATE ../firmware/airsoft-scope/main)

add_executable(ld2450_tests
    test_ld2450.c
    ../firmware/airsoft-scope/main/ld2450_parser.c
)
target_include_directories(ld2450_tests PRIVATE ../firmware/airsoft-scope/main)

if(MSVC)
    target_compile_options(tracking_tests PRIVATE /W4 /WX)
    target_compile_options(ld2450_tests PRIVATE /W4 /WX)
else()
    target_compile_options(tracking_tests PRIVATE -Wall -Wextra -Werror -pedantic)
    target_link_libraries(tracking_tests PRIVATE m)
    target_compile_options(ld2450_tests PRIVATE -Wall -Wextra -Werror -pedantic)
endif()

add_test(NAME tracking_tests COMMAND tracking_tests)
add_test(NAME ld2450_tests COMMAND ld2450_tests)
