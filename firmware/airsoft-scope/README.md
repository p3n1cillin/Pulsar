# Airsoft radar scope firmware

ESP-IDF firmware for a handheld airsoft "wall sense" scope: **HLK-LD2450** mmWave
radar targets are mapped onto a **camera-aligned overlay** and shown on a rear
**ST7789V SPI display**. All processing stays on the ESP32-S3.

```text
HLK-LD2450 --UART--> ESP32-S3 --> X/Y (mm) -> bearing/range -> screen (x,y)
OV2640/OV3660 --DVP--> live RGB565 frame --> draw blips --> ST7789V
```

## Requirements

- Freenove ESP32-S3-WROOM CAM (ESP32-S3 N16R8, onboard OV2640/OV3660)
- ESP-IDF 5.4+
- USB data cable for flash + serial monitor

## Build and flash

```powershell
cd firmware/airsoft-scope
idf.py set-target esp32s3
idf.py build
idf.py -p COM7 flash monitor
```

## Configuration (`idf.py menuconfig` → *Airsoft scope configuration*)

| Setting | Default | Notes |
|---|---|---|
| Operating mode | Simulation | Switch to *Live* after wiring the LD2450 |
| LD2450 UART | UART1, GPIO1 TX / GPIO14 RX, 256000 baud | radar TX → ESP RX (GPIO14) |
| Camera board | ESP32-S3-CAM | composites blips onto the live feed |
| Display rotation | 0 / 90 / 180 / 270 | 0/180 landscape, 90/270 portrait |
| Camera HFOV | 66° | align blips to the image |
| Min range | 120 cm | suppress muzzle-adjacent clutter |
| Camera mirror / flip | on / on | match the camera orientation in the housing |
| Radar link timeout | 1000 ms | **NO RADAR** banner when the LD2450 goes silent |
| Housing buttons | GPIO 21 / 47 | brightness, declutter, boresight calibration (NVS) |
| Battery monitor | off | GPIO 3 ADC through a 100k/100k divider |

The optional debug stream also requires a non-empty bearer token. Requests to
`/jpg` and `/stream` must send `Authorization: Bearer TOKEN`; the server fails
closed when either the SSID or token is empty.

## Tests

From the repository root, run the native tracking tests with CMake and CTest.
Run `idf.py build` here for the full firmware build. CI performs both checks and
also compiles the feature-complete configuration from `sdkconfig.ci.defaults`.

## Wiring

See [`main/pins.h`](main/pins.h) for the full GPIO map. Display on SPI2
(MOSI 40, SCLK 39, CS 38, DC 42, RST 41, BL 2); radar on UART1 (TX 14←, RX 1→);
display VCC 3V3, radar VCC 5V, common ground. The display reuses the freed
microSD pins (38/39/40) — do not insert a card.

## Module layout

```text
main/
  main.c                  Boot, mode selection, render loop, radar watchdog
  ld2450.c / _parser.c    LD2450 UART reader + pure frame parser
  camera_motion.c         Grid motion detector -> bearing-only detections
  camera_model.h          Shared pinhole model (bearing <-> pixel)
  target_tracker.c        Radar/camera fusion into persistent tracks
  radar_overlay.c         Track -> screen projection, declutter
  overlay_draw.c          Contact markers, 3x5 font, status HUD
  scope_controls.c        Buttons, NVS settings, battery ADC
  scope_controls_logic.c  Control state machine (host-tested)
  button_input.c          Debounce + long press (host-tested)
  battery_gauge.c         Li-ion voltage -> percent (host-tested)
  scope_camera.c          OV2640/OV3660 DVP capture (RGB565, PSRAM)
  scope_ui.c              Framebuffer compositor + rotation + serial preview
  st7789.c                ST7789V SPI driver (esp_lcd) + PWM backlight
  web_stream.c            Optional authenticated MJPEG debug stream
  sim_radar.c             Synthetic targets for bench development
```

See [`../../docs/airsoft-scope.md`](../../docs/airsoft-scope.md) for the BOM,
mechanical layout, and sensor capability notes.
