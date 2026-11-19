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
| 2× 7 mm buttons, rocker switch, 18650 + charger/boost | Controls and power | see [`hardware/housing`](hardware/housing/) |

A printable, support-free housing with a Picatinny quick-detach clamp and an
M-LOK→Picatinny adapter (e.g. for a Tokyo Marui MWS) lives in
[`hardware/housing/`](hardware/housing/), with an exploded assembly diagram,
wiring diagram, and Bambu print settings.

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

**Controls** (active-low to GND, internal pull-ups): button A / MODE on GPIO 21,
button B / ADJUST on GPIO 47. Optional battery sense on GPIO 3 (ADC1) through a
100k/100k divider.

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
| Camera mirror / flip | on / on | Match the camera orientation in your housing |
| Radar link timeout | 1000 ms | Live mode shows **NO RADAR** when the LD2450 goes silent |
| Housing buttons | on, GPIO 21 / 47 | Brightness, declutter, boresight calibration |
| Battery monitor | off | GPIO 3 + divider; adds a battery gauge to the HUD |

### Operator controls

| Button | Short press | Long press |
|---|---|---|
| A (MODE) | Cycle brightness 15/40/70/100% | Enter boresight calibration / save and exit |
| B (ADJUST) | Declutter: show radar-backed contacts only | In calibration: reset to the menuconfig default |

In calibration a dashed boresight line and `CAL +x.x` readout appear; A and B
move the radar bearing by ∓0.5°. Brightness, declutter, and the boresight
offset persist in NVS across power cycles.

In **Simulation** mode the firmware generates moving synthetic targets and renders
the radar-only scope view (reticle + blips) plus an ASCII preview on the serial monitor.

The optional Wi-Fi debug stream is disabled by default. When enabled, configure
the SSID, password, and a non-empty bearer token in `menuconfig`; `/jpg` and
`/stream` reject unauthenticated requests. For example:

```powershell
curl.exe -H "Authorization: Bearer YOUR_TOKEN" http://DEVICE_IP/jpg -o frame.jpg
```

## Verification

The portable tracking, projection, and radar-frame-parser tests run on the host,
while the firmware build checks the complete ESP-IDF integration:

```powershell
cmake -S tests -B build/tests
cmake --build build/tests --config Debug   # omit --config on single-config generators
ctest --test-dir build/tests -C Debug --output-on-failure

cd firmware/airsoft-scope
idf.py build
```

`tracking_tests` covers target association/fusion/coasting;
`overlay_tests` covers the pinhole projection, declutter filter, and HUD
rendering bounds; `controls_tests` covers button debounce/long-press, the
brightness/declutter/calibration state machine, and the battery gauge; `ld2450_tests` covers the pure LD2450 byte-stream parser in
`firmware/airsoft-scope/main/ld2450_parser.c` (frame decode + sign handling,
split feeds, garbage resync, overflow safety); `camera_motion_tests` covers the
grid motion detector (`camera_motion.c`, via `tests/stub/sdkconfig.h`) including
static-scene suppression, blob detection, global-motion rejection,
big-endian RGB565 decoding, and rectilinear off-axis bearings.

The housing generator verifies its own fit (electronics keep-outs, insertion
paths, clamp clearance):

```powershell
cd hardware/housing
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt
.venv\Scripts\python pulsar_housing.py
```

GitHub Actions runs the host tests, the housing fit checks, and builds both the
default offline profile and a feature-complete camera, live-radar,
battery-monitor, and authenticated-stream profile.

## Firmware layout

```text
firmware/airsoft-scope/main/
  main.c            Boot, mode selection, render loop, radar-link watchdog
  ld2450.c          LD2450 UART reader (timestamps frames)
  ld2450_parser.c   Pure LD2450 byte-stream parser (host-tested)
  camera_motion.c   Grid motion detector -> bearing-only observations
  target_tracker.c  Radar/camera fusion into persistent tracks
  camera_model.h    Shared pinhole camera model (bearing <-> pixel)
  radar_overlay.c   Track bearing/range -> screen projection, declutter
  overlay_draw.c    Contact markers, 3x5 font, status HUD
  scope_controls.c  Buttons, NVS settings, battery ADC
  scope_controls_logic.c  Button/calibration state machine (host-tested)
  button_input.c    Debounce + short/long press (host-tested)
  battery_gauge.c   18650 voltage -> percent (host-tested)
  scope_camera.c    OV2640/OV3660 DVP capture (RGB565, PSRAM)
  scope_ui.c        Framebuffer compositor, rotation, serial preview
  web_stream.c      Optional authenticated MJPEG debug stream
  st7789.c          ST7789V SPI display driver (esp_lcd), PWM backlight
  sim_radar.c       Synthetic targets for bench development
  scope_types.h     Shared frame/track/detection types
  pins.h            GPIO map for the Freenove S3-CAM build
```

See [`docs/airsoft-scope.md`](docs/airsoft-scope.md) for the BOM, mechanical layout,
sensor capabilities/limits, and field-tuning notes.

## Current boundary

Contact markers are conservative motion hints for airsoft use, not a validated
sensing system. The LD2450 reports coarse angle/range blobs, not silhouettes, and
material/posture affect detection. This is a tactical hinting tool, not SAR.
