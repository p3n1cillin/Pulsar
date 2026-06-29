#pragma once

#include "sdkconfig.h"

// Pin map for the planned handheld housing. Adjust before PCB/harness build.
// UART0 (GPIO43/44 on DevKitC-1) stays on the USB bridge for logs.

#define AIRSOFT_PIN_LD2450_TX CONFIG_AIRSOFT_LD2450_TX_GPIO
#define AIRSOFT_PIN_LD2450_RX CONFIG_AIRSOFT_LD2450_RX_GPIO

// OV2640 DVP — fixed by the Freenove ESP32-S3-WROOM CAM PCB.
// These match scope_camera.c. Do not reassign for radar/display.
#define AIRSOFT_PIN_CAM_XCLK   15
#define AIRSOFT_PIN_CAM_PCLK   13
#define AIRSOFT_PIN_CAM_VSYNC  6
#define AIRSOFT_PIN_CAM_HREF   7
#define AIRSOFT_PIN_CAM_D0     11
#define AIRSOFT_PIN_CAM_D1     9
#define AIRSOFT_PIN_CAM_D2     8
#define AIRSOFT_PIN_CAM_D3     10
#define AIRSOFT_PIN_CAM_D4     12
#define AIRSOFT_PIN_CAM_D5     18
#define AIRSOFT_PIN_CAM_D6     17
#define AIRSOFT_PIN_CAM_D7     16
#define AIRSOFT_PIN_CAM_SIOD   4
#define AIRSOFT_PIN_CAM_SIOC   5

// ST7789V SPI display on the operator-facing back panel.
// Freenove S3-CAM safe pins: avoids camera GPIOs (4-18), octal-PSRAM
// GPIOs (33-37) and flash GPIOs (26-32). Reuses the freed microSD pins
// (38/39/40) -- do NOT insert a microSD card while these are wired.
#define AIRSOFT_PIN_LCD_MOSI   40
#define AIRSOFT_PIN_LCD_SCLK   39
#define AIRSOFT_PIN_LCD_CS     38
#define AIRSOFT_PIN_LCD_DC     42
#define AIRSOFT_PIN_LCD_RST    41
#define AIRSOFT_PIN_LCD_BL     2
