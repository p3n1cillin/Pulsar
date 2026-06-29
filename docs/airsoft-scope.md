# Airsoft radar scope

Handheld directional sensor for airsoft: mmWave detects human-sized motion through
light cover (drywall, plywood, foliage), projects target bearings onto a live
camera feed, and shows the composite on a rear display for the operator.

## Data pipeline (all on ESP32)

```text
LD2450 --UART--> ESP32 parser (X/Y in mm)
                      |
OV2640/OV3660 --DVP--> live RGB565 frame
                      |
                 bearing + range -> screen (x, y)
                      |
                 draw contact markers on the live feed
                      |
                 rear ST7789V display (or serial preview on bench)
```

The camera shows the wall or doorway you are looking at. Orange crosshair markers
show **where the radar sees a person behind** the cover — even when the camera
cannot see them.

## Concept layout

```text
 FRONT (field of view)                    BACK (operator)
+---------------------------+            +---------------------------+
|  [OV2640]    [LD2450]     |            |      ST7789V 240x320     |
|   camera      radar       |            |   live feed + blips       |
|   boresight   boresight   |            |                           |
+---------------------------+            +---------------------------+
              |                                      ^
              v                                      |
        +------------------+                         |
        |  ESP32-S3 N16R8  |-------------------------+
        |  PSRAM + flash   |
        +------------------+
              |
        18650 / USB-C PD
```

Keep the camera and radar apertures **parallel and close** (≤20 mm baseline), both
on the same horizontal plane. A 3D-printed shroud reduces side lobes and protects
the radar PCB. Use no metal-filled filament near the antenna.

## Bill of materials

| Part | Role | Notes |
|---|---|---|
| Freenove ESP32-S3-WROOM CAM | Compute, camera, UI | ESP32-S3 N16R8, 8 MB octal PSRAM, onboard OV2640/OV3660 |
| HLK-LD2450 | 24 GHz mmWave radar | up to 3 targets with X/Y + speed; 5 V; UART 256000 baud |
| ST7789V SPI TFT | Rear operator display | 240×320 panel, driven landscape or portrait |
| 5 V buck (≥1 A) | Radar + display backlight | separate from the ESP32 3.3 V rail |
| 18650 + holder or 2S pack | Field power | add a kill switch; pad for airsoft use |
| Tactile buttons | Brightness, cal, mute | optional |

## What the LD2450 can and cannot do

**Can (per Hi-Link spec, indoor human sensing):**

- Track up to **3 moving targets** with X/Y coordinates (mm) and speed
- Detect movement to roughly **6 m**; weaker on slow/static targets
- **±60° horizontal** detection field
- Penetrate common indoor materials (gypsum, thin wood) for "behind cover" hints

**Cannot / limitations for airsoft:**

- **Not imaging** — you get position blobs by angle/range, not silhouettes
- **Coarse** — fine for "someone behind that wall," not exact peek height
- **Material dependent** — metal, concrete, wet foliage attenuate heavily
- **Human-tuned** — small crouched players or very slow crawls may drop out; BBs won't register
- **Regulatory** — 24 GHz ISM; legal for hobby use in most regions but verify local RF rules

Set expectations: this is a **tactical hinting tool**, not military SAR.

## Recommended improvements over the base concept

### 1. Dual-mode blips
Render contacts differently when the camera also sees motion in that sector:
**solid orange** = radar only (likely occluded), **red ring** = radar + visible
motion (confirmed). Phase 1 ships radar-only blips; add a camera-ROI motion mask later.

### 2. One-time boresight calibration
Store horizontal bearing offset (and optional FOV scale) in NVS after aligning on a
walking teammate at known distances. Add a "cal mode" button.

### 3. Power and thermal
LD2450 + camera capture + backlight can exceed 1.5 W. Use a separate 5 V rail for
the radar, a bulk cap (470 µF) at the module, and vent slots in the print.

### 4. Display mirroring
The operator faces the screen while the sensors face the field — mirror the camera
horizontally so left/right matches intuitive shouldering. (The `Display rotation`
menuconfig option sets the panel orientation for the housing.)

### 5. Game-safe UX
Mute/privacy switch for future recording paths; a haptic/audio ping when a blip
enters the centre reticle; an outdoor brightness preset.

### 6. Housing print tips
Radar window in ABS/PETG (no metal-filled filament); flat polycarbonate camera lens
guard; cable service loop between the front sensor tray and rear display; indexing
pins so reassembly preserves boresight.

## Pin map (Freenove S3-CAM build)

| Function | GPIO | Notes |
|---|---|---|
| LD2450 TX → ESP RX | 14 | UART1 |
| LD2450 RX ← ESP TX | 1 | UART1 |
| LCD MOSI / SCLK / CS | 40 / 39 / 38 | SPI2; 38/39/40 are the freed microSD pins |
| LCD DC / RST / BL | 42 / 41 / 2 | |
| Camera DVP | fixed on PCB | GPIO 4–18, see `firmware/airsoft-scope/main/scope_camera.c` |

Reserved on this board: camera GPIO 4–18, octal PSRAM 33–37, flash 26–32. Do not
insert a microSD card while the display is wired.

## Bring-up sequence

1. **Simulation** — flash with mode = *Simulation*; verify the reticle + moving blips
   on the display and the ASCII preview on serial.
2. **LD2450** — wire UART1 (TX 14 / RX 1, 5 V), switch mode to *Live*; targets stream
   automatically at 256000 baud.
3. **Camera** — select the ESP32-S3-CAM board; blips composite onto the live feed.
4. **Field tune** — camera HFOV, range gate, boresight offset, and display rotation
   for the housing.

## Firmware entry point

[`firmware/airsoft-scope/`](../firmware/airsoft-scope/)

```powershell
cd firmware/airsoft-scope
idf.py set-target esp32s3
idf.py -p COM7 flash monitor
```
