#pragma once

// Host-test stand-in for the ESP-IDF generated sdkconfig.h. Values mirror
// firmware/airsoft-scope/sdkconfig so the tests exercise production settings.
#ifndef CONFIG_AIRSOFT_CAMERA_MOTION_TRACKING
#define CONFIG_AIRSOFT_CAMERA_MOTION_TRACKING 1
#endif

#ifndef CONFIG_AIRSOFT_CAMERA_MOTION_THRESHOLD
#define CONFIG_AIRSOFT_CAMERA_MOTION_THRESHOLD 18
#endif

#ifndef CONFIG_AIRSOFT_CAMERA_MIN_MOTION_CELLS
#define CONFIG_AIRSOFT_CAMERA_MIN_MOTION_CELLS 4
#endif

#ifndef CONFIG_AIRSOFT_CAMERA_GLOBAL_MOTION_PERCENT
#define CONFIG_AIRSOFT_CAMERA_GLOBAL_MOTION_PERCENT 45
#endif
