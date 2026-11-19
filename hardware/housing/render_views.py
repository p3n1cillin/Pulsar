#!/usr/bin/env python3
"""Render assembly/fit diagrams for the PULSAR housing from the real geometry.

    python render_views.py            # writes img/*.png

Uses a small painter's-algorithm renderer (matplotlib 2D polygons) so printed
parts and electronics keep-outs from pulsar_housing.py share one depth sort.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
import pulsar_housing as ph  # noqa: E402

OUT = Path(__file__).with_name("img")

PRINT = {
    "front": "#4f6d7a",
    "rear": "#5b7f8c",
    "pic_body": "#3d4a52",
    "pic_jaw": "#55636b",
    "mlok": "#3d4a52",
}
ELEC = {
    "ld2450_pcb": ("#e07b00", "HLK-LD2450 radar"),
    "ld2450_back": ("#b86500", None),
    "esp_pcb": ("#2c7a3f", "ESP32-S3-CAM board"),
    "camera": ("#202020", "OV2640 camera"),
    "esp_headers": ("#8a8a8a", "headers / Dupont"),
    "display_pcb": ("#2a4fa0", "ST7789 display"),
    "display_glass": ("#0e1a33", None),
    "display_pins": ("#8a8a8a", None),
    "battery": ("#b0302a", "18650 holder"),
    "switch": ("#d9b300", "power switch"),
    "charger": ("#7a3fa0", "charger / 5 V boost"),
    "buttons": ("#d9b300", "buttons A/B"),
}


def mesh_arrays(m):
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties, dtype=float)[:, :3]
    f = np.asarray(mesh.tri_verts, dtype=np.int64)
    return v, f


class Scene:
    def __init__(self, azim=-50.0, elev=24.0):
        a, e = math.radians(azim), math.radians(elev)
        # camera looks along -d; screen axes r (right), u (up)
        self.d = np.array([math.cos(e) * math.sin(a), -math.cos(e) * math.cos(a), math.sin(e)])
        self.r = np.array([math.cos(a), math.sin(a), 0.0])
        self.u = np.cross(self.d, self.r)
        self.light = self.d * 0.6 + self.u * 0.6 + self.r * 0.3
        self.light /= np.linalg.norm(self.light)
        self.polys, self.depths, self.colors = [], [], []
        self.labels = []

    def add(self, m, color, alpha=1.0, offset=(0, 0, 0), edge=False):
        if m is None or m.is_empty():
            return
        # Subdivide big faces so the per-triangle depth sort stays correct.
        v, f = mesh_arrays(m.translate(offset).refine_to_length(4.0))
        tri = v[f]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        ln = np.linalg.norm(n, axis=1)
        keep = ln > 1e-9
        tri, n, ln = tri[keep], n[keep], ln[keep]
        n = n / ln[:, None]
        facing = n @ self.d > 0
        tri, n = tri[facing], n[facing]
        shade = 0.45 + 0.55 * np.clip(n @ self.light, 0, 1)
        base = np.array(matplotlib.colors.to_rgb(color))
        cols = np.clip(base[None, :] * shade[:, None] + 0.04, 0, 1)
        cols = np.concatenate([cols, np.full((len(cols), 1), alpha)], axis=1)
        p2 = np.stack([tri @ self.r, tri @ self.u], axis=-1)
        depth = (tri @ self.d).mean(axis=1)
        self.polys.append(p2)
        self.depths.append(depth)
        self.colors.append(cols)

    def label(self, point, text, dx=0, dy=0):
        p = np.asarray(point, dtype=float)
        self.labels.append(((p @ self.r, p @ self.u), text, dx, dy))

    def render(self, path, title, size=(11, 8), note=None):
        P = np.concatenate(self.polys)
        D = np.concatenate(self.depths)
        C = np.concatenate(self.colors)
        order = np.argsort(D)
        fig, ax = plt.subplots(figsize=size, dpi=130)
        ax.add_collection(PolyCollection(P[order], facecolors=C[order], edgecolors=C[order],
                                         linewidths=0.15))
        pts = P.reshape(-1, 2)
        pad = 8
        ax.set_xlim(pts[:, 0].min() - pad, pts[:, 0].max() + pad)
        ax.set_ylim(pts[:, 1].min() - pad, pts[:, 1].max() + pad)
        for (x, y), text, dx, dy in self.labels:
            ax.annotate(text, (x, y), xytext=(x + dx, y + dy), fontsize=8.5,
                        arrowprops=dict(arrowstyle="-", lw=0.6, color="#333"),
                        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#999", lw=0.5))
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(title, fontsize=12, loc="left")
        if note:
            fig.text(0.01, 0.01, note, fontsize=8, color="#444")
        fig.tight_layout()
        fig.savefig(path, facecolor="white")
        plt.close(fig)
        print("wrote", path)


def centre(m):
    x0, y0, z0, x1, y1, z1 = m.bounding_box()
    return np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])


def exploded(L, parts, ko):
    s = Scene(azim=-58, elev=20)
    gap_front = (0, -80, 0)
    gap_mount = (0, 0, -26)
    s.add(parts["front"], PRINT["front"], offset=gap_front)
    s.add(parts["rear"], PRINT["rear"])
    s.add(parts["pic_body"], PRINT["pic_body"], offset=gap_mount)
    s.add(parts["pic_jaw"], PRINT["pic_jaw"], offset=(14, 0, -26))
    s.add(parts["mlok"], "#6b5a3a", offset=(0, 0, -48))
    s.label(centre(parts["mlok"]) + (0, 0, -48), "M-LOK -> Picatinny section (M-LOK handguards)", 40, -10)
    front_elec = ("ld2450_pcb", "ld2450_back", "esp_pcb", "camera", "esp_headers")
    elec_off = {k: (0, -40, 0) for k in front_elec}
    for name, m in ko.items():
        s.add(m, ELEC[name][0], offset=elec_off.get(name, (0, 0, 0)))
    s.label(centre(ko["ld2450_pcb"]) + (0, -40, 4), "1  LD2450 snaps in (antenna to front)", -60, 18)
    s.label(centre(ko["esp_pcb"]) + (0, -40, -6), "2  ESP32-S3-CAM snaps in (camera to front)", -70, -22)
    s.label(centre(parts["front"]) + (0, -80, 20), "front shell (radome + camera window)", -40, 22)
    s.label(centre(ko["display_pcb"]), "3  display presses into rear frame", 40, 26)
    s.label(centre(ko["battery"]), "4  18650 holder", 46, -12)
    s.label(centre(parts["pic_body"]) + (0, 0, -26), "5  Picatinny quick-detach clamp", 44, -4)
    s.render(OUT / "exploded.png", "PULSAR housing - exploded assembly",
             note="Front shell slides over the rear shell lip; 4x M3x30 countersunk screws "
                  "through the corner tubes into heat-set inserts.")


def cutaway(L, parts, ko):
    # Section through the camera/radar axis, viewed from the +X side.
    s = Scene(azim=-90 + 0.01, elev=0.01)
    s.d = np.array([1.0, 0.0, 0.0])
    s.r = np.array([0.0, 1.0, 0.0])
    s.u = np.array([0.0, 0.0, 1.0])
    s.light = np.array([0.8, -0.3, 0.5])
    s.light /= np.linalg.norm(s.light)
    x_cut = L.cam_x
    for k in ("front", "rear", "pic_body"):
        m = parts[k].trim_by_plane((-1.0, 0.0, 0.0), -x_cut)  # keep x <= cam_x
        s.add(m, PRINT[k])
    for name, m in ko.items():
        mm = m.trim_by_plane((-1.0, 0.0, 0.0), -x_cut)
        if not mm.is_empty():
            s.add(mm, ELEC[name][0])
    s.label((x_cut, L.tf / 2, L.rcz), f"radome {L.p.radome_t:.2f} mm (half-wave)", -14, 14)
    s.label((x_cut, L.y_radar, L.rcz + 4), f"radar gap {L.p.radome_gap:.1f} mm (lambda/2)", 4, 22)
    s.label((x_cut, L.y_lens, L.cam_z), "camera + lens guard disc", -16, -22)
    s.label((x_cut, L.y_board, L.bcz - 8), "ESP32-S3-CAM", 16, -18)
    s.label((x_cut, L.Df, L.H - 2), "shell seam / indexing lip", 22, 10)
    s.label((x_cut, (L.y_bat0 + L.y_bat1) / 2, L.zf + 10), "18650", 8, -18)
    s.label((x_cut, L.y_disp_pcb, L.disp_z), "display", 14, 16)
    s.label((x_cut, L.p.mount_y, -8), "rail mount", 22, -6)
    s.render(OUT / "cutaway.png",
             "Side section through the camera/radar axis (front at left)",
             size=(11, 6.5),
             note="Radar is stacked directly above the camera on the same vertical axis: "
                  "zero horizontal parallax between radar bearings and the image.")


def front_view(L, parts, ko):
    s = Scene()
    s.d = np.array([0.0, -1.0, 0.0])
    s.r = np.array([-1.0, 0.0, 0.0])  # looking at the front: +X is on the viewer's left
    s.u = np.array([0.0, 0.0, 1.0])
    s.light = np.array([0.2, -1.0, 0.4])
    s.light /= np.linalg.norm(s.light)
    s.add(parts["front"], PRINT["front"], alpha=1.0)
    s.add(parts["pic_body"], PRINT["pic_body"])
    s.add(parts["pic_jaw"], PRINT["pic_jaw"])
    s.label((L.cam_x, 0, L.cam_z), "camera window", 30, -10)
    s.label((L.rcx, 0, L.rcz), "radar sits behind this (solid radome)", 20, 14)
    s.label((L.pillars[0][0], 0, L.pillars[0][1]), "M3 countersunk x4", -20, -12)
    s.render(OUT / "front.png", "Front face (radar + camera share one vertical axis)",
             size=(8, 6))


def front_inside(L, parts, ko):
    # Looking forward into the open front shell, as you do while fitting it.
    s = Scene()
    s.d = np.array([0.0, 1.0, 0.0])
    s.r = np.array([1.0, 0.0, 0.0])
    s.u = np.array([0.0, 0.0, 1.0])
    s.light = np.array([0.3, 1.0, 0.5])
    s.light /= np.linalg.norm(s.light)
    s.add(parts["front"], PRINT["front"])
    for name in ("ld2450_pcb", "ld2450_back", "esp_pcb", "esp_headers"):
        s.add(ko[name], ELEC[name][0], alpha=0.55)
    p = L.p
    s.label((L.rcx - p.radar_l / 2 - 1, L.Df, L.rcz), "radar snap clip", -26, 10)
    s.label((L.rcx + p.radar_l / 2 + 1, L.Df, L.rcz), "radar snap clip", 8, 14)
    s.label((L.rcx, L.Df, L.rz0 - 1), "radar rest pads", 26, -6)
    s.label((L.bcx - p.esp_l / 2 - 1, L.Df, L.bcz + 10.5), "board clips (straddle USB)", -30, 14)
    s.label((L.bcx + p.esp_l / 2 + 1, L.Df, L.bcz), "board clip", 10, -12)
    s.label((L.bcx, L.Df, L.bz0 - 1), "board rest pads", 0, -16)
    s.label((-L.W / 2, L.Df, L.bcz), "USB-C (flash/debug)", -30, -24)
    s.render(OUT / "front_inside.png",
             "Inside the front shell (viewed from the seam): radar above, ESP board below",
             size=(9, 6.5))


def main():
    OUT.mkdir(exist_ok=True)
    p = ph.Params(segments=40)
    L, parts = ph.build(p)
    ko = ph.keepouts(L)
    exploded(L, parts, ko)
    cutaway(L, parts, ko)
    front_view(L, parts, ko)
    front_inside(L, parts, ko)


if __name__ == "__main__":
    main()
