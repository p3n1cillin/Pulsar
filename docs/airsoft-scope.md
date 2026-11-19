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

Keep the camera and radar apertures **parallel** and on the **same vertical axis**
(radar directly above the camera). The overlay maps radar *bearing* to image x,
so only horizontal offset between the sensors causes parallax; a vertical
offset costs nothing. The printed housing in
[`hardware/housing/`](../hardware/housing/) does this, puts the radar λ/2
(6.2 mm) behind a half-wave (3.78 mm) solid front wall, and keeps metal and
CF/GF-filled filament away from the antenna.

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

### 1. Dual-mode blips (done)
**Solid orange** = radar only (likely occluded), **red rings** = radar + visible
motion (confirmed), **cyan** = camera only, **dashed** = coasting prediction.

### 2. One-time boresight calibration (done)
Hold button A to enter calibration, nudge with A/B in 0.5° steps while a
teammate walks across the scene, hold A to save the offset to NVS.

### 3. Power and thermal (housing done)
LD2450 + camera capture + backlight can exceed 1.5 W. Use a separate 5 V rail for
the radar, a bulk cap (470 µF) at the module, and vent slots in the print (the
housing vents the ESP end). Optional battery gauge on GPIO 3.

### 4. Display mirroring
The operator faces the screen while the sensors face the field — mirror the camera
horizontally so left/right matches intuitive shouldering. (The `Display rotation`
menuconfig option sets the panel orientation for the housing.)

### 5. Game-safe UX (partly done)
Done: four-step PWM brightness, declutter view, and a **NO RADAR** warning when
the LD2450 link drops (silence never looks like "nobody there"). Still open: a
haptic/audio ping when a blip enters the centre reticle.

### 6. Housing (done)
See [`hardware/housing/`](../hardware/housing/): front sensor shell (radome,
polycarbonate lens guard, snap clips), rear shell (display, 18650, controls),
indexing lip so reassembly preserves boresight, Picatinny quick-detach clamp,
and an M-LOK→Picatinny section.

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
idf.py build
idf.py -p COM7 flash monitor
```
