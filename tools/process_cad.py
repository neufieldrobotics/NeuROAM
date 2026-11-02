#!/usr/bin/env python3
"""Convert NeuROAM payload STEP to a web-friendly grouped GLB."""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

import cascadio
import numpy as np
import trimesh

# Logical groups for explode / legend. First matching rule wins.
RULES = [
    ("lidar", re.compile(r"LIDAR|CUSTOM_LID|BASE_PLATE|LIDAR_PLATE|LIDAR_CABLE|LIDAR_INTERFACE", re.I)),
    ("cameras", re.compile(r"SENSOR|M118FM08|PRINTED_8982|BRACKET", re.I)),
    ("imu", re.compile(r"^IMU$|SPOT_IMU|BALL_DAMPER", re.I)),
    ("gnss", re.compile(r"ZED|GNSS|QWIIC|TIME_SYNC|SMA|USBC|^BOARD$", re.I)),
    ("radio", re.compile(r"RADIO|WiFi Antenna|N_FEMALE|N_CONNECOR|ANTENNA|CONNECTOR|EXTENSION", re.I)),
    ("compute", re.compile(r"JETSON|HUB|ANKER|AT503|AT506|AT527|Lockwasher|SMHC|STD_toggle|Universal_board|BABY_BUCK|JST|3-PIN", re.I)),
    ("power", re.compile(r"BATTERY|CHARGER|Part[568]", re.I)),
    ("display", re.compile(r"LCD|7inch|HDMI", re.I)),
    ("enclosure", re.compile(r"ENCLOSURE|BOTTOM_PLATE|PIN|INSERT|LID_STOPPER|GNSS_RADIO_PLATE|RADIO_PLATE|JETSON_PLATE|HUB_PLATE|BATTERY_HOLDER", re.I)),
]

FASTENER = re.compile(
    r"socket head|hex nut|flat washer|Standoff|Locknut|94205A220",
    re.I,
)

COLORS = {
    "lidar": [0.18, 0.18, 0.20, 1.0],
    "cameras": [0.16, 0.47, 0.84, 1.0],
    "imu": [0.92, 0.41, 0.20, 1.0],
    "gnss": [0.11, 0.69, 0.48, 1.0],
    "radio": [0.93, 0.63, 0.00, 1.0],
    "compute": [0.29, 0.23, 0.65, 1.0],
    "power": [0.55, 0.55, 0.52, 1.0],
    "display": [0.35, 0.55, 0.75, 1.0],
    "enclosure": [0.78, 0.78, 0.74, 1.0],
    "fasteners": [0.45, 0.45, 0.42, 1.0],
}


def classify(name: str) -> str:
    if FASTENER.search(name):
        return "fasteners"
    for group, pat in RULES:
        if pat.search(name):
            return group
    return "enclosure"


def transform_mesh(mesh: trimesh.Trimesh, matrix: np.ndarray) -> trimesh.Trimesh:
    out = mesh.copy()
    out.apply_transform(matrix)
    return out


def simplify(mesh: trimesh.Trimesh, target_faces: int) -> trimesh.Trimesh:
    if len(mesh.faces) <= target_faces:
        return mesh
    try:
        simplified = mesh.simplify_quadric_decimation(face_count=target_faces)
        if simplified is not None and len(simplified.faces) > 0:
            return simplified
    except Exception:
        pass
    return mesh


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tol-linear", type=float, default=0.5)
    ap.add_argument("--max-faces-per-group", type=int, default=40000)
    args = ap.parse_args()

    raw = os.path.join(os.path.dirname(args.out), "_payload_raw.glb")
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    print(f"converting {args.step} -> {raw}", flush=True)
    cascadio.step_to_glb(
        args.step,
        raw,
        tol_linear=args.tol_linear,
        tol_angular=1.0,
        merge_primitives=True,
        use_parallel=True,
    )
    print(f"raw size {os.path.getsize(raw) / 1e6:.2f} MB", flush=True)

    scene = trimesh.load(raw, force="scene")
    groups: dict[str, list[trimesh.Trimesh]] = {k: [] for k in COLORS}

    # Apply graph transforms so meshes sit in world frame.
    for node_name in scene.graph.nodes_geometry:
        transform, geom_name = scene.graph[node_name]
        mesh = scene.geometry[geom_name]
        if not isinstance(mesh, trimesh.Trimesh):
            continue
        group = classify(geom_name)
        if group == "fasteners":
            continue  # omit fasteners for a lighter web model
        groups[group].append(transform_mesh(mesh, transform))

    out_scene = trimesh.Scene()
    meta = {"groups": {}, "centroids": {}}
    for group, meshes in groups.items():
        if not meshes or group == "fasteners":
            continue
        combined = trimesh.util.concatenate(meshes)
        target = args.max_faces_per_group
        if group in ("radio", "compute", "enclosure"):
            target = min(target, 25000)
        combined = simplify(combined, target)
        rgba = (np.array(COLORS[group]) * 255).astype(np.uint8)
        # Prefer vertex colors for reliable GLB export without scipy face→vertex.
        combined.visual.vertex_colors = np.tile(rgba, (len(combined.vertices), 1))
        out_scene.add_geometry(combined, geom_name=group, node_name=group)
        c = combined.centroid.tolist()
        meta["groups"][group] = {
            "faces": int(len(combined.faces)),
            "vertices": int(len(combined.vertices)),
            "color": COLORS[group],
        }
        meta["centroids"][group] = c
        print(f"  {group}: {len(combined.faces)} faces", flush=True)

    # Center at origin for the viewer.
    bounds = out_scene.bounds
    center = (bounds[0] + bounds[1]) / 2.0
    for name, geom in out_scene.geometry.items():
        geom.apply_translation(-center)
        meta["centroids"][name] = geom.centroid.tolist()

    out_scene.export(args.out)
    meta_path = os.path.splitext(args.out)[0] + ".json"
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"wrote {args.out} ({os.path.getsize(args.out) / 1e6:.2f} MB)", flush=True)
    print(f"wrote {meta_path}", flush=True)
    try:
        os.remove(raw)
    except OSError:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
