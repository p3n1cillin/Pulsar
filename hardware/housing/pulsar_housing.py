#!/usr/bin/env python3
"""PULSAR airsoft radar scope -- parametric prototype housing.

Generates printable STL files for a two-shell enclosure plus two rail mounts
(Picatinny clamp and M-LOK plate, e.g. for a Tokyo Marui MWS handguard), and
checks every part against keep-out volumes for the electronics.

    pip install -r requirements.txt
    python pulsar_housing.py                # writes stl/*.stl + stl/manifest.json
    python pulsar_housing.py --set cam_stack=8.5 --set esp_l=62.0

Assembled coordinates (mm): X = right (operator's view), Y = boresight from
the front face (y=0) to the rear display face (y=depth), Z = up from the
bottom outer face (z=0). Every part is also exported in its print
orientation (largest flat face on the bed, no supports needed).

Layout (see README.md for the full diagram):
  * Front shell: radome front wall, camera aperture, snap clips that hold the
    LD2450 and the Freenove ESP32-S3-CAM board. The radar sits directly above
    the camera on the same vertical axis, so there is no horizontal parallax
    between the two sensors.
  * Rear shell: ST7789 display window + snap frame, 18650 holder bay, power
    switch, charger port, two buttons, vents, mount screw holes.
  * The rear shell's lip indexes into the front shell (boresight survives
    reassembly); four M3 countersunk screws through the front corner tubes
    clamp the shells together.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from manifold3d import CrossSection, JoinType, Manifold, OpType, set_circular_segments

C_MM_GHZ = 299.792458  # speed of light in mm * GHz


@dataclass
class Params:
    # --- general -------------------------------------------------------------
    wall: float = 2.4              # side/top/bottom/rear wall
    fit: float = 0.3               # general sliding clearance
    segments: int = 72             # circle resolution

    # --- radar window (radome) ------------------------------------------------
    radar_freq_ghz: float = 24.125
    radome_er: float = 2.7         # PETG/ASA/PLA at 24 GHz ~2.5-3.0; NOT CF/GF-filled
    radome_override: float = 0.0   # >0 forces a front-wall thickness

    # --- HLK-LD2450 -----------------------------------------------------------
    radar_l: float = 44.0
    radar_w: float = 15.0
    radar_t: float = 1.6
    radar_back_h: float = 3.2      # connector/components behind the PCB

    # --- Freenove ESP32-S3-WROOM CAM (MEASURE YOUR BOARD) ----------------------
    esp_l: float = 63.6
    esp_w: float = 28.0
    esp_t: float = 1.6
    cam_from_end: float = 14.0     # camera centre from the board end opposite USB
    cam_dz: float = 0.0            # camera centre offset from board centre line
    cam_stack: float = 7.5         # board front face -> lens front
    cam_module: float = 9.0        # camera module square size (keep-out)
    cam_pupil_r: float = 2.0
    cam_half_fov_diag: float = 42.0
    lens_guard_d: float = 14.0     # clear PC/acrylic disc glued inside the aperture
    lens_guard_t: float = 1.0
    usb_cut_y: float = 13.0        # ESP USB-C opening (covers either board face)
    usb_cut_z: float = 12.5
    header_zone: float = 26.0      # headers + Dupont plugs behind the board

    # --- ST7789V 2.0" module (MEASURE YOURS) ------------------------------------
    disp_pcb_l: float = 58.0
    disp_pcb_w: float = 35.0
    disp_pcb_t: float = 1.6
    disp_glass_t: float = 2.4
    disp_active_l: float = 40.8
    disp_active_w: float = 30.6
    disp_active_dx: float = 0.0    # active-area centre offset from PCB centre
    disp_active_dz: float = 0.0
    disp_window_margin: float = 0.8
    disp_pins_h: float = 10.0      # header pins / wires behind the display PCB

    # --- power / controls -------------------------------------------------------
    bat_l: float = 77.0            # 1x18650 holder
    bat_w: float = 21.0
    bat_h: float = 20.0
    switch_cut_y: float = 13.2     # KCD11-style mini rocker panel cut-out
    switch_cut_z: float = 8.8
    switch_body: float = 12.0
    charger_cut_y: float = 11.0    # charger/boost module USB-C opening
    charger_cut_z: float = 6.0
    button_hole: float = 7.2       # 7 mm momentary panel button (R13-507 style)
    button_body_h: float = 14.0

    # --- enclosure --------------------------------------------------------------
    inner_w: float = 79.0
    inner_h: float = 49.0
    front_depth: float = 26.0      # seam position
    depth: float = 80.0
    corner_r: float = 4.0
    lip_len: float = 4.0
    lip_t: float = 1.6
    pillar_r: float = 3.8
    m3_clear: float = 3.4
    m3_csk_d: float = 6.4
    m3_insert_d: float = 4.0       # M3 x 5.7 heat-set insert
    m3_insert_depth: float = 6.0
    mount_hole_dx: float = 9.0     # 4 mount screws at (+-dx, mount_y +- dy)
    mount_hole_dy: float = 18.0
    mount_y: float = 52.0          # mount centre along Y

    # --- clips --------------------------------------------------------------------
    clip_arm_t: float = 1.6
    clip_barb: float = 0.7

    # --- Picatinny (MIL-STD-1913) --------------------------------------------------
    rail_v_width: float = 21.2     # width across the V tips
    rail_v_depth: float = 2.75     # rail top -> V tip line
    mount_len: float = 56.0
    mount_w: float = 34.0
    plate_t: float = 7.0
    jaw_drop: float = 5.6
    m4_clear: float = 4.4
    m4_head_d: float = 7.6
    m4_head_h: float = 3.0
    m4_nut_af: float = 7.3
    m4_nut_h: float = 3.4
    bolt_z: float = -1.0           # cross-bolt centre (rests in a rail slot)

    # --- M-LOK -> Picatinny adapter --------------------------------------------------
    mlok_pitch: float = 40.0       # slot pitch (centre to centre)
    adapter_len: float = 60.0
    adapter_rail_h: float = 9.5    # handguard surface -> rail top
    adapter_base_w: float = 20.0
    adapter_slots: int = 3         # recoil slots at 10 mm pitch, centred
    adapter_cbore: float = 6.0     # M5 head recess below the rail top
    mlok_lug_w: float = 6.8        # slot is 7.0 wide
    mlok_lug_l: float = 14.0
    mlok_lug_h: float = 1.8
    m5_clear: float = 5.5
    m5_head_d: float = 10.2        # M5 button head (ISO 7380) + clearance

    # derived --------------------------------------------------------------------------
    @property
    def radome_t(self) -> float:
        if self.radome_override > 0:
            return self.radome_override
        lam0 = C_MM_GHZ / self.radar_freq_ghz
        return lam0 / (2.0 * math.sqrt(self.radome_er))   # half-wave window

    @property
    def radome_gap(self) -> float:
        return C_MM_GHZ / self.radar_freq_ghz / 2.0          # antenna -> radome, lambda0/2


# ---------------------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------------------

def box(x0, x1, y0, y1, z0, z1) -> Manifold:
    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))
    z0, z1 = sorted((z0, z1))
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def union(parts) -> Manifold:
    parts = [p for p in parts if p is not None and not p.is_empty()]
    if not parts:
        return Manifold()
    return Manifold.batch_boolean(parts, OpType.Add)


def poly(points) -> CrossSection:
    pts = list(points)
    area = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        area += x0 * y1 - x1 * y0
    if area < 0:
        pts.reverse()
    return CrossSection([pts])


def rrect(w, h, r, cx=0.0, cy=0.0) -> CrossSection:
    r = max(0.0, min(r, w / 2 - 0.01, h / 2 - 0.01))
    if r <= 0:
        return CrossSection.square((w, h), center=True).translate((cx, cy))
    core = CrossSection.square((w - 2 * r, h - 2 * r), center=True)
    return core.offset(r, JoinType.Round).translate((cx, cy))


def circle(r, cx=0.0, cy=0.0) -> CrossSection:
    return CrossSection.circle(r).translate((cx, cy))


def extrude_y(cs: CrossSection, y0: float, y1: float, scale_top=(1.0, 1.0)) -> Manifold:
    """Extrude a profile drawn in (x, z) along +Y from y0 to y1.

    scale_top scales the profile (about its local origin) at y0 relative to y1.
    """
    y0, y1 = sorted((y0, y1))
    m = Manifold.extrude(cs, y1 - y0, scale_top=scale_top)
    return m.rotate((90, 0, 0)).translate((0, y1, 0))  # (u,v,w) -> (u, y1-w, v)


_PERM_X = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)


def extrude_x(cs: CrossSection, x0: float, x1: float) -> Manifold:
    """Extrude a profile drawn in (y, z) along +X."""
    x0, x1 = sorted((x0, x1))
    return Manifold.extrude(cs, x1 - x0).transform(_PERM_X).translate((x0, 0, 0))


def extrude_z(cs: CrossSection, z0: float, z1: float) -> Manifold:
    z0, z1 = sorted((z0, z1))
    return Manifold.extrude(cs, z1 - z0).translate((0, 0, z0))


def cyl_y(x, z, r, y0, y1, r1=None) -> Manifold:
    y0, y1 = sorted((y0, y1))
    r1 = r if r1 is None else r1
    # radius r at y0, r1 at y1
    m = Manifold.cylinder(y1 - y0, r, r1)
    return m.rotate((-90, 0, 0)).translate((x, y0, z))  # (u,v,w) -> (u, w, -v)


def cyl_x(y, z, r, x0, x1) -> Manifold:
    x0, x1 = sorted((x0, x1))
    return Manifold.cylinder(x1 - x0, r).transform(_PERM_X).translate((x0, y, z))


def cyl_z(x, y, r, z0, z1, r1=None) -> Manifold:
    z0, z1 = sorted((z0, z1))
    r1 = r if r1 is None else r1
    return Manifold.cylinder(z1 - z0, r, r1).translate((x, y, z0))


def hexagon_x(y, z, af, x0, x1) -> Manifold:
    """Hex pocket along X, flats parallel to the XY plane (top/bottom)."""
    r = af / math.sqrt(3.0)
    pts = [(y + r * math.cos(math.radians(a)), z + r * math.sin(math.radians(a)))
           for a in range(0, 360, 60)]
    return extrude_x(poly(pts), x0, x1)


# ---------------------------------------------------------------------------------------
# layout
# ---------------------------------------------------------------------------------------

class Layout:
    """All derived positions in assembled coordinates."""

    def __init__(self, p: Params):
        self.p = p
        t = p.wall
        self.W = p.inner_w + 2 * t
        self.H = p.inner_h + 2 * t
        self.zf = t                          # interior floor
        self.zc = t + p.inner_h              # interior ceiling
        self.xi = p.inner_w / 2              # interior half width
        self.tf = p.radome_t                 # front wall
        self.Df = p.front_depth
        self.D = p.depth

        # ESP board: USB end at -X, camera near the +X end. Radar centred on the
        # camera's X so radar bearings need no parallax correction.
        reach = p.clip_arm_t + p.fit + 0.6   # clip arm beyond a PCB end
        self.bcx = -self.xi + reach + p.esp_l / 2
        self.cam_x = self.bcx + p.esp_l / 2 - p.cam_from_end
        self.rcx = self.cam_x
        radar_right = self.rcx + p.radar_l / 2 + reach
        if radar_right > self.xi + 1e-6:
            raise ValueError(
                f"radar clip at x={radar_right:.1f} exceeds interior {self.xi:.1f}; "
                "increase inner_w or reduce cam_from_end")
        self.bz0 = self.zf + 2.5
        self.bcz = self.bz0 + p.esp_w / 2
        self.bz1 = self.bz0 + p.esp_w
        self.cam_z = self.bcz + p.cam_dz
        self.rz1 = self.zc - 1.5
        self.rz0 = self.rz1 - p.radar_w
        self.rcz = (self.rz0 + self.rz1) / 2
        if self.rz0 - self.bz1 < 1.5:
            raise ValueError("radar and ESP board overlap vertically; increase inner_h")

        # Depth stack
        self.y_radar = self.tf + p.radome_gap                 # radar antenna face
        self.y_lens = self.tf + 0.35                          # lens just behind guard disc
        self.y_board = self.y_lens + p.cam_stack              # board front face
        if self.y_board + p.esp_t + 2.6 > self.Df:
            raise ValueError("ESP board does not fit in the front shell depth")

        # Rear shell
        self.y_disp_wall = self.D - t                          # rear wall inner face
        self.y_disp_pcb = self.y_disp_wall - p.disp_glass_t   # PCB front face
        self.disp_x = 0.0
        self.disp_z = self.H / 2
        self.y_bat0 = self.Df + p.header_zone - 12.0           # battery holder front
        self.y_bat1 = self.y_bat0 + p.bat_w + 0.6
        pins_front = self.y_disp_pcb - p.disp_pcb_t - p.disp_pins_h
        if self.y_bat1 + 1.6 > pins_front + 2.0:
            raise ValueError("battery bay collides with display pins; increase depth")
        self.y_ctrl = (self.y_bat0 + self.y_bat1) / 2          # switch/buttons/charger Y

        # Corner tubes (screws) and mount holes
        # Tube centres sit outside the interior corner so the tubes leave the
        # full interior width free for the 77 mm battery holder.
        o = 2.2
        self.pillars = [(sx * (self.xi + o), z)
                        for sx in (-1, 1) for z in (self.zf - o, self.zc + o)]
        self.mount_holes = [(sx * p.mount_hole_dx, p.mount_y + sy * p.mount_hole_dy)
                            for sx in (-1, 1) for sy in (-1, 1)]


# ---------------------------------------------------------------------------------------
# parts
# ---------------------------------------------------------------------------------------

def outer_profile(L: Layout) -> CrossSection:
    p = L.p
    body = rrect(L.W, L.H, p.corner_r, 0, L.H / 2)
    tubes = [circle(p.pillar_r, x, z) for x, z in L.pillars]
    return CrossSection.batch_boolean([body] + tubes, OpType.Add)


def cavity_profile(L: Layout) -> CrossSection:
    p = L.p
    cav = rrect(p.inner_w, p.inner_h, 1.0, 0, L.zf + p.inner_h / 2)
    return cav - CrossSection.batch_boolean(
        [circle(p.pillar_r, x, z) for x, z in L.pillars], OpType.Add)


def pcb_clip(edge_x, out_dir, y_wall, y_face, pcb_t, zc, wz, p: Params) -> Manifold:
    """Snap clip for a PCB end: spacer shoulder + flexing arm with a barb.

    edge_x: PCB end; out_dir: +1 if the PCB lies at x < edge_x.
    y_face: PCB front face (toward the front wall at y_wall).
    """
    u = out_dir
    z0, z1 = zc - wz / 2, zc + wz / 2
    shoulder = box(edge_x - 1.5 * u, edge_x, y_wall - 0.01, y_face, z0, z1)
    a0 = edge_x + p.fit * u
    a1 = a0 + p.clip_arm_t * u
    y_b0 = y_face + pcb_t + 0.15
    y_tip = y_b0 + 1.4
    arm = box(a0, a1, y_wall - 0.01, y_tip, z0, z1)
    root = a0 + 0.4 * u  # overlap the arm so barb and arm are one body
    barb_pts = [(root, y_b0), (a0 - p.clip_barb * u, y_b0),
                (a0 - p.clip_barb * u, y_b0 + 0.3), (a0, y_tip), (root, y_tip)]
    barb = extrude_z(poly(barb_pts), z0, z1)
    return union([shoulder, arm, barb])


def rest_pad(xc, wx, y_wall, y_back, z_top) -> Manifold:
    return box(xc - wx / 2, xc + wx / 2, y_wall - 0.01, y_back, z_top - 1.5, z_top)


def front_shell(L: Layout, with_clips=True) -> tuple[Manifold, Manifold]:
    p = L.p
    shell = extrude_y(outer_profile(L), 0, L.Df)
    shell -= extrude_y(cavity_profile(L), L.tf, L.Df + 1)

    clips = []
    if with_clips:
        # ESP board: USB end clips straddle the USB band; far end one wide clip.
        left = L.bcx - p.esp_l / 2
        right = L.bcx + p.esp_l / 2
        for dz in (-10.5, 10.5):
            clips.append(pcb_clip(left, -1, L.tf, L.y_board, p.esp_t, L.bcz + dz, 4.0, p))
        clips.append(pcb_clip(right, +1, L.tf, L.y_board, p.esp_t, L.bcz, 8.0, p))
        # LD2450
        rl = L.rcx - p.radar_l / 2
        rr = L.rcx + p.radar_l / 2
        clips.append(pcb_clip(rl, -1, L.tf, L.y_radar, p.radar_t, L.rcz, 6.0, p))
        clips.append(pcb_clip(rr, +1, L.tf, L.y_radar, p.radar_t, L.rcz, 6.0, p))
    clip_body = union(clips)

    pads = []
    for dx in (-20.0, 20.0):  # board bottom rests (rise from the floor)
        pads.append(box(L.bcx + dx - 3, L.bcx + dx + 3, L.tf - 0.01,
                        L.y_board + p.esp_t, L.zf - 0.01, L.bz0 - 0.2))
    for dx in (-15.0, 15.0):  # radar bottom rests
        pads.append(rest_pad(L.rcx + dx, 6.0, L.tf, L.y_radar + p.radar_t, L.rz0 - 0.2))
    shell = union([shell, clip_body] + pads)

    # Camera aperture: FOV cone + lens-guard counterbore on the inside face.
    tan_t = math.tan(math.radians(p.cam_half_fov_diag))
    y_cb = L.tf - p.lens_guard_t
    r_in = max(4.0, p.cam_pupil_r + (L.y_lens - y_cb) * tan_t)
    r_out = r_in + y_cb * tan_t
    if p.lens_guard_d / 2 < r_in + 1.5:
        raise ValueError("lens guard disc too small for the camera aperture")
    cuts = [
        cyl_y(L.cam_x, L.cam_z, r_out, -0.01, y_cb + 0.01, r_in),
        cyl_y(L.cam_x, L.cam_z, p.lens_guard_d / 2 + 0.2, y_cb, L.tf + 0.01),
    ]
    # ESP USB-C opening in the left wall
    y_mid = L.y_board + p.esp_t / 2
    cuts.append(extrude_x(rrect(p.usb_cut_y, p.usb_cut_z, 1.5, y_mid, L.bcz),
                          -L.W / 2 - 6, -L.xi + 0.01))
    # Corner tubes: clearance + countersink on the front face
    for x, z in L.pillars:
        cuts.append(cyl_y(x, z, p.m3_clear / 2, -0.01, L.Df + 0.01))
        cuts.append(cyl_y(x, z, p.m3_csk_d / 2, -0.01, (p.m3_csk_d - p.m3_clear) / 2,
                          p.m3_clear / 2))
    shell -= union(cuts)
    return shell, clip_body


def display_frame(L: Layout) -> tuple[Manifold, Manifold]:
    """Snap frame on the rear wall's inner face. Returns (frame, barbs)."""
    p = L.p
    arm_t = 1.2
    barb = 0.5
    hw = p.disp_pcb_l / 2 + p.fit
    hh = p.disp_pcb_w / 2 + p.fit
    yw = L.y_disp_wall
    y_back = L.y_disp_pcb - p.disp_pcb_t          # PCB back face
    y_b0 = y_back - 0.15                          # barb holding face
    y_tip = y_b0 - 1.4
    cx, cz = L.disp_x, L.disp_z

    outer = rrect(2 * (hw + arm_t), 2 * (hh + arm_t), 1.0, cx, cz)
    inner = CrossSection.square((2 * hw, 2 * hh), center=True).translate((cx, cz))
    frame = extrude_y(outer - inner, y_tip, yw + 0.01)

    # The glass rests directly on the rear wall; no standoffs (on most 2.0"
    # modules the glass spans the full PCB height).

    # Flex arms on the long (top/bottom) edges: slit the frame either side.
    barbs = []
    slit = 0.7
    arm_w = 8.0
    for ax in (-13.0, 13.0):
        for sz in (-1, 1):
            zi = cz + sz * hh             # inner face of the frame wall
            zo = zi + sz * arm_t
            for s in (-1, 1):
                xs = cx + ax + s * (arm_w / 2 + slit / 2)
                frame -= box(xs - slit / 2, xs + slit / 2, y_tip - 0.01, yw - 2.2,
                             min(zi, zo) - 0.01, max(zi, zo) + 0.01)
            root = zi + sz * 0.4
            pts = [(root, y_b0), (zi - sz * barb, y_b0),
                   (zi - sz * barb, y_b0 - 0.3), (zi, y_tip), (root, y_tip)]
            # profile in (z, y) -> build in (y, z) for extrude_x
            pts_yz = [(y, z) for z, y in pts]
            barbs.append(extrude_x(poly(pts_yz), cx + ax - arm_w / 2, cx + ax + arm_w / 2))
    barb_body = union(barbs)
    return union([frame, barb_body]), barb_body


def rear_shell(L: Layout) -> tuple[Manifold, Manifold]:
    p = L.p
    t = p.wall
    shell = extrude_y(outer_profile(L), L.Df, L.D)
    shell -= extrude_y(cavity_profile(L), L.Df - 1, L.D - t)

    # Indexing lip that slides into the front shell.
    lip_o = rrect(p.inner_w - 2 * 0.2, p.inner_h - 2 * 0.2, 1.0, 0, L.zf + p.inner_h / 2)
    lip_i = rrect(p.inner_w - 2 * (0.2 + p.lip_t), p.inner_h - 2 * (0.2 + p.lip_t), 0.5,
                  0, L.zf + p.inner_h / 2)
    lip = extrude_y(lip_o - lip_i, L.Df - p.lip_len, L.Df + 0.5)
    # Root ring bridges the 0.2 mm fit gap so the lip is fused to the walls.
    lip += extrude_y(rrect(p.inner_w + 1.0, p.inner_h + 1.0, 1.0, 0, L.zf + p.inner_h / 2)
                     - lip_i, L.Df, L.Df + 2.0)
    lip -= extrude_y(CrossSection.batch_boolean(
        [circle(p.pillar_r + 0.3, x, z) for x, z in L.pillars], OpType.Add),
        L.Df - p.lip_len - 1, L.Df + 1)

    frame, barbs = display_frame(L)

    # Battery bay: rear stop wall + low front lip (tilt the holder over it).
    bay = [
        box(-L.xi, L.xi, L.y_bat1, L.y_bat1 + 1.6, L.zf - 0.01, L.zf + 6.0),
        box(-L.xi, L.xi, L.y_bat0 - 1.6, L.y_bat0, L.zf - 0.01, L.zf + 1.2),
    ]
    # Charger shelf on the right wall (a fin along Y -> prints vertically).
    shelf_z = L.zf + p.bat_h + 6.0
    shelf = box(L.xi - 5.0, L.xi + 0.01, L.y_ctrl - 10, L.y_ctrl + 10, shelf_z - 1.6, shelf_z)
    shelf_gusset = extrude_y(poly([(L.xi + 0.01, shelf_z - 1.6), (L.xi - 5.0, shelf_z - 1.6),
                                   (L.xi + 0.01, shelf_z - 5.6)]),
                             L.y_ctrl - 10, L.y_ctrl + 10)
    # Button nut bosses under the top wall (flat seat for the panel nut).
    bosses = [cyl_z(bx, L.y_ctrl, 6.5, L.zc - 0.8, L.zc + 0.01) for bx in (-9.0, 9.0)]
    shell = union([shell, lip, frame] + bay + [shelf, shelf_gusset] + bosses)

    cuts = []
    # Display window, chamfered 45 deg outward.
    aw = p.disp_active_l + 2 * p.disp_window_margin
    ah = p.disp_active_w + 2 * p.disp_window_margin
    c = t
    win = rrect(aw + 2 * c, ah + 2 * c, 1.5 + c)
    win_m = extrude_y(win, L.D - t - 0.01, L.D + 0.01,
                      scale_top=(aw / (aw + 2 * c), ah / (ah + 2 * c)))
    # extrude_y puts the unscaled base at y1 (rear face) -> outer opening there.
    cuts.append(win_m.translate((L.disp_x + p.disp_active_dx, 0,
                                 L.disp_z + p.disp_active_dz)))
    # Power switch (left wall), charger USB-C (right wall)
    sw_z = L.zf + p.bat_h + 15.0
    cuts.append(extrude_x(rrect(p.switch_cut_y, p.switch_cut_z, 0.8, L.y_ctrl, sw_z),
                          -L.W / 2 - 6, -L.xi + 0.01))
    ch_z = shelf_z + 1.6 + 1.6
    cuts.append(extrude_x(rrect(p.charger_cut_y, p.charger_cut_z, 1.5, L.y_ctrl, ch_z),
                          L.xi - 0.01, L.W / 2 + 6))
    # Buttons in the top wall
    for bx in (-9.0, 9.0):
        cuts.append(cyl_z(bx, L.y_ctrl, p.button_hole / 2, L.zc - 1.0, L.H + 6))
    # Vents near the seam (ESP heat), both side walls
    for z in (L.zc - 12.0, L.zc - 8.0, L.zc - 4.0):
        for side in (-1, 1):
            x0 = side * (L.xi - 0.01)
            x1 = side * (L.W / 2 + 6)
            cuts.append(extrude_x(rrect(8.0, 2.0, 0.9, L.Df + 8.0, z), x0, x1))
    # Corner tubes: heat-set inserts from the seam face + screw clearance beyond
    for x, z in L.pillars:
        cuts.append(cyl_y(x, z, p.m3_insert_d / 2, L.Df - 0.01, L.Df + p.m3_insert_depth))
        cuts.append(cyl_y(x, z, 1.5, L.Df, L.Df + p.m3_insert_depth + 6))
    # Mount screws: countersunk from inside the floor
    for x, y in L.mount_holes:
        cuts.append(cyl_z(x, y, p.m3_clear / 2, -0.01, L.zf + 0.01))
        cuts.append(cyl_z(x, y, p.m3_clear / 2, L.zf - (p.m3_csk_d - p.m3_clear) / 2,
                          L.zf + 0.02, p.m3_csk_d / 2))
    shell -= union(cuts)
    return shell, barbs


def mount_inserts(L: Layout, z_top: float) -> Manifold:
    p = L.p
    return union([cyl_z(x, y, p.m3_insert_d / 2, z_top - 5.5, z_top + 0.01)
                  for x, y in L.mount_holes])


def picatinny_mount(L: Layout) -> tuple[Manifold, Manifold]:
    """Fixed clamp body (left jaw) + separate sliding right jaw, M4 cross bolt.

    Local rail top is z = -plate_t (mount top face meets the housing at z=0).
    """
    p = L.p
    zr = -p.plate_t                 # rail top
    y0 = p.mount_y - p.mount_len / 2
    y1 = p.mount_y + p.mount_len / 2
    hw = p.mount_w / 2
    tip = p.rail_v_width / 2 + p.fit         # jaw inner face at the V tips
    vd = p.rail_v_depth
    lip_in = 2.2
    zb = zr - p.jaw_drop

    def jaw_profile(sign):
        pts = [(sign * hw, zr), (sign * tip, zr), (sign * tip, zr - vd),
               (sign * (tip - lip_in), zr - vd - lip_in), (sign * (tip - lip_in), zb),
               (sign * hw, zb)]
        return poly(pts)

    pocket_top = zr + 3.3
    plate = CrossSection.square((2 * hw, p.plate_t), center=False).translate((-hw, zr))
    body_cs = CrossSection.batch_boolean([plate, jaw_profile(-1)], OpType.Add)
    body_cs -= CrossSection.square((hw - (tip - 0.7) + 0.01, pocket_top - zr + 0.01)) \
        .translate((tip - 0.7, zr - 0.01))
    body = extrude_y(body_cs, y0, y1)

    jaw_cs = CrossSection.batch_boolean([
        jaw_profile(+1),
        CrossSection.square((hw - tip, 3.0)).translate((tip, zr)),
    ], OpType.Add)
    jaw = extrude_y(jaw_cs, y0, y1)

    yb = p.mount_y
    bolt = cyl_x(yb, zr + p.bolt_z - 0.0, p.m4_clear / 2, -hw - 1, hw + 1)
    channel = box(-tip - 0.01, tip + 0.01, yb - 2.4, yb + 2.4, zr - 0.01, zr + 1.4)
    head = cyl_x(yb, zr + p.bolt_z, p.m4_head_d / 2, -hw - 1, -hw + p.m4_head_h)
    nut = hexagon_x(yb, zr + p.bolt_z, p.m4_nut_af, hw - p.m4_nut_h, hw + 1)
    body -= union([bolt, channel, head, mount_inserts(L, 0.0)])
    jaw -= union([bolt, nut])
    return body, jaw


def mlok_rail_adapter(L: Layout) -> Manifold:
    """Short M-LOK -> Picatinny rail section (stays on the handguard).

    The Picatinny clamp then fits either MWS handguard and stays quick-detach.
    Local frame: z=0 is the handguard surface, rail top at z=rail_h, rail axis
    along Y centred on y=0. Two M5 screws on the 40 mm M-LOK slot pitch.
    """
    p = L.p
    hl = p.adapter_len / 2
    top = p.adapter_rail_h
    tip_z = top - p.rail_v_depth
    neck_z = tip_z - p.rail_v_depth
    half_tip = p.rail_v_width / 2
    half_top = half_tip - p.rail_v_depth
    base_hw = p.adapter_base_w / 2
    base_t = neck_z - 0.6
    # 45 deg chamfer under the flange so the part prints rail-top-down.
    flange = base_hw - half_top
    if base_t - flange < 0.8:
        raise ValueError("adapter_base_w too wide for a 45 deg printable flange")
    prof = poly([(-base_hw, 0), (base_hw, 0), (base_hw, base_t - flange),
                 (half_top, base_t), (half_top, neck_z), (half_tip, tip_z), (half_top, top),
                 (-half_top, top), (-half_tip, tip_z), (-half_top, neck_z), (-half_top, base_t),
                 (-base_hw, base_t - flange)])
    body = extrude_y(prof, -hl, hl)
    lugs, cuts = [], []
    for sy in (-1, 1):
        y = sy * p.mlok_pitch / 2
        lugs.append(extrude_z(rrect(p.mlok_lug_w, p.mlok_lug_l, 1.5, 0, y),
                              -p.mlok_lug_h, 0.01))
        cuts.append(cyl_z(0, y, p.m5_clear / 2, -p.mlok_lug_h - 1, top + 1))
        cuts.append(cyl_z(0, y, p.m5_head_d / 2, top - p.adapter_cbore, top + 1))
    for k in range(-p.adapter_slots // 2 + 1, p.adapter_slots // 2 + 1):
        y = k * 10.0
        if abs(y) + 2.62 > p.mlok_pitch / 2 - p.m5_head_d / 2 - 1.0:
            raise ValueError("rail slot runs into an M5 counterbore")
        cuts.append(box(-half_tip - 1, half_tip + 1, y - 5.23 / 2, y + 5.23 / 2,
                        top - 3.0, top + 1))
    if p.mlok_pitch / 2 + p.mlok_lug_l / 2 > hl:
        raise ValueError("M-LOK lugs overrun the adapter; increase adapter_len")
    return union([body] + lugs) - union(cuts)


# ---------------------------------------------------------------------------------------
# keep-outs (electronics) for interference checks
# ---------------------------------------------------------------------------------------

def keepouts(L: Layout) -> dict[str, Manifold]:
    p = L.p
    k = {}
    k["ld2450_pcb"] = box(L.rcx - p.radar_l / 2, L.rcx + p.radar_l / 2, L.y_radar,
                          L.y_radar + p.radar_t, L.rz0, L.rz1)
    k["ld2450_back"] = box(L.rcx - p.radar_l / 2 + 2.5, L.rcx + p.radar_l / 2 - 2.5,
                           L.y_radar + p.radar_t, L.y_radar + p.radar_t + p.radar_back_h,
                           L.rz0 + 1, L.rz1 - 1)
    bx0, bx1 = L.bcx - p.esp_l / 2, L.bcx + p.esp_l / 2
    k["esp_pcb"] = box(bx0, bx1, L.y_board, L.y_board + p.esp_t, L.bz0, L.bz1)
    hc = p.cam_module / 2
    k["camera"] = box(L.cam_x - hc, L.cam_x + hc, L.y_lens, L.y_board, L.cam_z - hc,
                      L.cam_z + hc)
    k["esp_headers"] = union([
        box(bx0 + 2, bx1 - 2, L.y_board + p.esp_t, L.y_board + p.esp_t + p.header_zone - 1,
            z0, z1)
        for z0, z1 in ((L.bz0, L.bz0 + 2.6), (L.bz1 - 2.6, L.bz1))])
    hw, hh = p.disp_pcb_l / 2, p.disp_pcb_w / 2
    k["display_pcb"] = box(L.disp_x - hw, L.disp_x + hw, L.y_disp_pcb - p.disp_pcb_t,
                           L.y_disp_pcb, L.disp_z - hh, L.disp_z + hh)
    k["display_glass"] = box(L.disp_x - hw + 7, L.disp_x + hw - 2, L.y_disp_pcb,
                             L.y_disp_wall, L.disp_z - hh + 0.8, L.disp_z + hh - 0.8)
    k["display_pins"] = box(L.disp_x - hw + 0.5, L.disp_x - hw + 5,
                            L.y_disp_pcb - p.disp_pcb_t - p.disp_pins_h,
                            L.y_disp_pcb - p.disp_pcb_t, L.disp_z - 10, L.disp_z + 10)
    k["battery"] = box(-p.bat_l / 2, p.bat_l / 2, L.y_bat0 + 0.3, L.y_bat0 + 0.3 + p.bat_w,
                       L.zf, L.zf + p.bat_h)
    sw_z = L.zf + p.bat_h + 15.0
    k["switch"] = box(-L.xi, -L.xi + p.switch_body, L.y_ctrl - p.switch_cut_y / 2,
                      L.y_ctrl + p.switch_cut_y / 2, sw_z - p.switch_cut_z / 2,
                      sw_z + p.switch_cut_z / 2)
    shelf_z = L.zf + p.bat_h + 6.0
    k["charger"] = box(L.xi - 24, L.xi - 0.1, L.y_ctrl - 9, L.y_ctrl + 9, shelf_z + 0.05,
                       shelf_z + 5.0)
    k["buttons"] = union([cyl_z(bx, L.y_ctrl, 4.0, L.zc - 0.8 - p.button_body_h,
                                L.zc - 0.8) for bx in (-9.0, 9.0)])
    return k


def sweeps(L: Layout) -> dict[str, tuple[Manifold, str]]:
    """Insertion paths: (swept volume, which shell)."""
    p = L.p
    s = {}
    s["ld2450_insert"] = (box(L.rcx - p.radar_l / 2, L.rcx + p.radar_l / 2, L.y_radar,
                              L.Df + 2, L.rz0, L.rz1), "front")
    s["esp_insert"] = (box(L.bcx - p.esp_l / 2, L.bcx + p.esp_l / 2, L.y_board, L.Df + 2,
                           L.bz0, L.bz1), "front")
    hw, hh = p.disp_pcb_l / 2, p.disp_pcb_w / 2
    s["display_insert"] = (box(L.disp_x - hw, L.disp_x + hw, L.Df - 6,
                               L.y_disp_pcb, L.disp_z - hh, L.disp_z + hh), "rear")
    return s


# ---------------------------------------------------------------------------------------
# output
# ---------------------------------------------------------------------------------------

def write_stl(m: Manifold, path: Path, name: str) -> None:
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties, dtype=np.float32)[:, :3]
    f = np.asarray(mesh.tri_verts, dtype=np.int64)
    tri = v[f]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"] = n
    rec["v"] = tri
    header = f"PULSAR {name}".encode()[:80].ljust(80, b" ")
    with open(path, "wb") as fh:
        fh.write(header)
        fh.write(struct.pack("<I", len(f)))
        fh.write(rec.tobytes())


def to_bed(m: Manifold) -> Manifold:
    x0, y0, z0, x1, y1, _ = m.bounding_box()
    return m.translate((-(x0 + x1) / 2, -(y0 + y1) / 2, -z0))


def build(p: Params):
    set_circular_segments(p.segments)
    L = Layout(p)
    front, front_clips = front_shell(L)
    rear, rear_barbs = rear_shell(L)
    pic_body, pic_jaw = picatinny_mount(L)
    # Placed under the clamp: adapter rail top == clamp rail seat (z = -plate_t).
    mlok = mlok_rail_adapter(L).translate((0, p.mount_y, -p.plate_t - p.adapter_rail_h))
    return L, {
        "front": front, "front_clips": front_clips,
        "rear": rear, "rear_barbs": rear_barbs,
        "pic_body": pic_body, "pic_jaw": pic_jaw, "mlok": mlok,
    }


def check(L: Layout, parts) -> list[str]:
    problems = []
    printed = {k: parts[k] for k in ("front", "rear", "pic_body", "pic_jaw", "mlok")}
    for name, m in printed.items():
        if m.status().name != "NoError" or m.is_empty():
            problems.append(f"{name}: invalid manifold ({m.status().name})")
        n = len(m.decompose())
        if n != 1:
            problems.append(f"{name}: {n} disconnected bodies (expected 1)")
    housing = parts["front"] + parts["rear"]
    for name, ko in keepouts(L).items():
        vol = (housing ^ ko).volume()
        if vol > 0.05:
            problems.append(f"keep-out '{name}' intersects housing by {vol:.2f} mm^3")
    # Shells must not overlap each other (the lip sits inside the front shell).
    ov = (parts["front"] ^ parts["rear"]).volume()
    if ov > 0.05:
        problems.append(f"front/rear shells overlap by {ov:.2f} mm^3")
    # Insertion sweeps may only touch snap barbs (they flex out of the way).
    front_rigid = parts["front"] - parts["front_clips"]
    rear_rigid = parts["rear"] - parts["rear_barbs"]
    for name, (sw, which) in sweeps(L).items():
        target = front_rigid if which == "front" else rear_rigid
        vol = (target ^ sw).volume()
        if vol > 0.05:
            problems.append(f"insertion path '{name}' blocked ({vol:.2f} mm^3)")
    # Mounts: clamp parts must not overlap each other or the M-LOK rail section
    # they grip (a real rail is then a looser fit than the printed one).
    clamp = {"body": parts["pic_body"], "jaw": parts["pic_jaw"]}
    jaw_ov = (clamp["body"] ^ clamp["jaw"]).volume()
    if jaw_ov > 0.05:
        problems.append(f"Picatinny jaw overlaps body by {jaw_ov:.2f} mm^3")
    for name, m in clamp.items():
        ov = (m ^ parts["mlok"]).volume()
        if ov > 0.05:
            problems.append(f"Picatinny {name} collides with the M-LOK rail by {ov:.2f} mm^3")
    return problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", type=Path, default=Path(__file__).with_name("stl"))
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override a parameter, e.g. --set cam_stack=8.5")
    ap.add_argument("--no-check", action="store_true")
    args = ap.parse_args(argv)

    p = Params()
    for item in args.set:
        key, _, val = item.partition("=")
        if not hasattr(p, key):
            ap.error(f"unknown parameter {key!r}")
        setattr(p, key, type(getattr(p, key))(val))

    L, parts = build(p)
    problems = [] if args.no_check else check(L, parts)
    for msg in problems:
        print("CHECK FAILED:", msg, file=sys.stderr)

    args.out.mkdir(parents=True, exist_ok=True)
    # Print orientations (all support-free):
    exports = {
        # front face (radome) on the bed
        "pulsar_front_shell": parts["front"].rotate((90, 0, 0)),
        # rear face (display window) on the bed
        "pulsar_rear_shell": parts["rear"].rotate((-90, 0, 0)),
        # top face on the bed, jaws pointing up
        "pulsar_mount_picatinny": parts["pic_body"].rotate((0, 180, 0)),
        # lying on its outer face, lip up
        "pulsar_mount_picatinny_jaw": parts["pic_jaw"].rotate((0, 90, 0)),
        # rail top on the bed, M-LOK lugs up
        "pulsar_mlok_picatinny_adapter": parts["mlok"].rotate((0, 180, 0)),
    }
    manifest = {
        "generator": "pulsar_housing.py",
        "parameters": dataclasses.asdict(p),
        "derived": {
            "outer_mm": [round(L.W, 2), round(L.D, 2), round(L.H, 2)],
            "radome_thickness_mm": round(p.radome_t, 3),
            "radome_gap_mm": round(p.radome_gap, 3),
            "camera_xz_mm": [round(L.cam_x, 2), round(L.cam_z, 2)],
            "radar_xz_mm": [round(L.rcx, 2), round(L.rcz, 2)],
            "esp_board_front_face_y_mm": round(L.y_board, 2),
        },
        "checks_passed": not problems and not args.no_check,
        "parts": {},
    }
    for name, m in exports.items():
        m = to_bed(m)
        path = args.out / f"{name}.stl"
        write_stl(m, path, name)
        x0, y0, z0, x1, y1, z1 = m.bounding_box()
        manifest["parts"][name] = {
            "file": path.name,
            "triangles": m.num_tri(),
            "volume_cm3": round(m.volume() / 1000.0, 2),
            "bed_footprint_mm": [round(x1 - x0, 1), round(y1 - y0, 1), round(z1 - z0, 1)],
        }
        print(f"{name:32s} {x1 - x0:6.1f} x {y1 - y0:6.1f} x {z1 - z0:6.1f} mm  "
              f"{m.volume() / 1000:6.1f} cm3")
    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"radome {p.radome_t:.2f} mm, radar gap {p.radome_gap:.2f} mm, "
          f"outer {L.W:.1f} x {L.D:.1f} x {L.H:.1f} mm")
    if problems:
        return 1
    if not args.no_check:
        print("all fit checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
