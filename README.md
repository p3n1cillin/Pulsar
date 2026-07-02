# PULSAR — Airsoft radar scope

A handheld "wall sense" scope for airsoft: an **HLK-LD2450** 24 GHz mmWave radar
detects human-sized motion through light cover (drywall, plywood, foliage), and
its targets are projected onto a live camera feed shown on a rear display. The
operator sees the wall or doorway in front of them with **orange contact markers**
where the radar thinks a person is — even when the camera can't see them.

All processing stays on the ESP32-S3. The radar never talks to the display directly.

```text
HLK-LD2450  --UART-->  ESP32-S3  -->  parse X/Y (mm) -> bearing/range
                          |
OV2640/OV3660 --DVP-->  live RGB565 frame
                          |
                     draw contact markers on the frame
                          |
                     ST7789V SPI display  (rear, operator-facing)
```

## Hardware

| Part | Role | Notes |
|---|---|---|
| Freenove ESP32-S3-WROOM CAM | Compute + camera | ESP32-S3 N16R8 (8 MB octal PSRAM), onboard OV2640/OV3660 |
| HLK-LD2450 | 24 GHz mmWave radar | up to 3 targets, X/Y in mm, 256000 baud, 5 V supply |
| ST7789V SPI TFT | Rear operator display | 240×320 panel, driven 320×240 landscape (rotatable) |

The OV2640 camera is integrated on the Freenove board. The radar and display are
wired to the GPIOs below (see [`firmware/airsoft-scope/main/pins.h`](firmware/airsoft-scope/main/pins.h)).

### Wiring

**ST7789V display → ESP32-S3**

| Display | GPIO | | Display | GPIO |
|---|---|---|---|---|
| SCL (SCLK) | 39 | | DC | 42 |
| SDA (MOSI) | 40 | | CS | 38 |
| RES (RST) | 41 | | BLK | 2 |
| VCC | 3V3 | | GND | GND |

**HLK-LD2450 → ESP32-S3** (UART1; TX/RX cross over)

| Radar | GPIO | Notes |
|---|---|---|
| TX | 14 (ESP RX) | radar 3.3 V logic |
| RX | 1 (ESP TX) | config commands only |
| 5V | 5V | radar needs 5 V |
| GND | GND | common ground |

> The display reuses the freed microSD pins (38/39/40) — **do not insert a microSD card**.
> Camera GPIOs (4–18), octal-PSRAM GPIOs (33–37), and flash GPIOs (26–32) are reserved.

## Build and flash

Requires ESP-IDF 5.4+.

```powershell
cd firmware/airsoft-scope
idf.py set-target esp32s3
idf.py build
idf.py -p COM7 flash monitor
```

## Configuration (`idf.py menuconfig` → *Airsoft scope configuration*)

| Setting | Default | Notes |
|---|---|---|
| Operating mode | **Simulation** | Switch to *Live* after wiring the LD2450 |
| LD2450 UART | UART1, GPIO1 TX / GPIO14 RX | 256000 baud |
| Camera board | **ESP32-S3-CAM** | Composites blips onto the live OV2640/OV3660 feed |
| Display rotation | 0/90/180/270 | Match the mounted orientation (90/270 = portrait) |
| Camera HFOV | 66° | Calibrate to align blips with the image |
| Min range | 120 cm | Ignores close self-clutter near the muzzle |

In **Simulation** mode the firmware generates moving synthetic targets and renders
the radar-only scope view (reticle + blips) plus an ASCII preview on the serial monitor.

The optional Wi-Fi debug stream is disabled by default. When enabled, configure
the SSID, password, and a non-empty bearer token in `menuconfig`; `/jpg` and
`/stream` reject unauthenticated requests. For example:

```powershell
curl.exe -H "Authorization: Bearer YOUR_TOKEN" http://DEVICE_IP/jpg -o frame.jpg
```

## Verification

The portable tracking and projection tests run on the host, while the firmware
build checks the complete ESP-IDF integration:

```powershell
cmake -S tests -B build/tests
cmake --build build/tests
ctest --test-dir build/tests --output-on-failure

cd firmware/airsoft-scope
idf.py build
```

GitHub Actions runs the host tests and builds both the default offline profile
and a feature-complete camera, live-radar, and authenticated-stream profile.

## Firmware layout

```text
firmware/airsoft-scope/main/
  main.c            Boot, mode selection, render loop
  ld2450.c          LD2450 UART frame parser -> target X/Y
  radar_overlay.c   Radar X/Y -> screen bearing/range projection
  overlay_draw.c    Orange contact markers (rings + crosshair)
  scope_camera.c    OV2640/OV3660 DVP capture (RGB565, PSRAM)
  scope_ui.c        Framebuffer compositor, rotation, serial preview
  st7789.c          ST7789V SPI display driver (esp_lcd)
  sim_radar.c       Synthetic targets for bench development
  pins.h            GPIO map for the Freenove S3-CAM build
```

See [`docs/airsoft-scope.md`](docs/airsoft-scope.md) for the BOM, mechanical layout,
sensor capabilities/limits, and field-tuning notes.

## Current boundary

Contact markers are conservative motion hints for airsoft use, not a validated
sensing system. The LD2450 reports coarse angle/range blobs, not silhouettes, and
material/posture affect detection. This is a tactical hinting tool, not SAR.
