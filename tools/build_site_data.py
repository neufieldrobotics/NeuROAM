#!/usr/bin/env python3
"""Build compact JSON + compressed images for the NeuROAM project page."""
from __future__ import annotations

import csv
import json
import math
import os
import re
import shutil
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
BACKUP = Path("/media/galoaab/NeuROAM_backup")
RELEASE = BACKUP / "NeuROAM_2025_11_19"
PAPER_SCRIPTS = BACKUP / "paper_figure_scripts" / "paper_scripts"
COMM = BACKUP / "paper_figure_scripts" / "NeuROAM" / "data_post_processing" / "bag_analysis" / "comm"
OVERLEAF = Path("/tmp/neuroam-overleaf")

OUT_DATA = ROOT / "static" / "data"
OUT_IMG = ROOT / "static" / "images"
OUT_VIDEO = ROOT / "static" / "video"

PCOLOR = {
    0: "#2a78d6",
    1: "#eb6834",
    2: "#1baf7a",
    3: "#eda100",
    4: "#4a3aa7",
}
ROBOT = {0: "Spot", 1: "Scout", 2: "Hunter", 3: "Go2-W", 4: "Go2"}
LIDAR = {0: "OS1-128", 1: "OS1-32", 2: "OS1-128", 3: "OS1-32", 4: "OS1-32"}
# Paper folder names match paper numbering for the release drive.
RECORDER_OF = {0: 0, 1: 3, 2: 2, 3: 1, 4: 4}  # paper -> recorder (for note only)


def place(name: str) -> str:
    if "elevator" in name:
        return "Elevator"
    if "init" in name:
        return "Start area"
    if "neu_campus" in name or "_to_" in name:
        return "Outdoor campus"
    if "highbay" in name:
        return "High bay"
    if "isec" in name:
        return "ISEC"
    if "exp" in name or name.endswith("end"):
        return "EXP"
    return "Start area"


def load_georef():
    return json.loads((COMM / "georef.json").read_text())


def xy_to_ll(x, y, geo):
    R, t, K = geo["R"], geo["t"], geo["K"]
    mx = K * (R[0][0] * x + R[0][1] * y) + t[0]
    my = K * (R[1][0] * x + R[1][1] * y) + t[1]
    lon = mx / 20037508.34 * 180.0
    lat = math.degrees(math.atan(math.sinh(my / 20037508.34 * math.pi)))
    return lat, lon


def load_tum(path: Path):
    rows = []
    with path.open() as f:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            p = line.split()
            rows.append(
                (
                    float(p[0]),
                    float(p[1]),
                    float(p[2]),
                    float(p[3]),
                )
            )
    return rows


def downsample(rows, dt=1.0):
    if not rows:
        return []
    out = [rows[0]]
    last = rows[0][0]
    for r in rows[1:]:
        if r[0] - last >= dt:
            out.append(r)
            last = r[0]
    if out[-1][0] != rows[-1][0]:
        out.append(rows[-1])
    return out


def build_trajectories(geo):
    payloads = {}
    for p in range(5):
        folder = RELEASE / f"payload{p}" / "ground_truth" / "merged"
        files = sorted(folder.glob("*.tum"))
        points = []
        for fp in files:
            rows = downsample(load_tum(fp), dt=1.0)
            for t, x, y, z in rows:
                lat, lon = xy_to_ll(x, y, geo)
                points.append([round(t, 3), round(lat, 6), round(lon, 6), round(z, 3)])
        points.sort(key=lambda r: r[0])
        payloads[str(p)] = {
            "robot": ROBOT[p],
            "color": PCOLOR[p],
            "lidar": LIDAR[p],
            "points": points,
        }
        print(f"P{p} traj points {len(points)}")
    return payloads


def build_sequences():
    stats = json.loads((PAPER_SCRIPTS / "out" / "dataset_stats.json").read_text())
    payloads = {}
    t0 = None
    t1 = None
    for k, v in stats.items():
        p = int(k)
        seqs = []
        for s in v["sequences"]:
            start = s["start_ns"] / 1e9
            end = s["end_ns"] / 1e9
            t0 = start if t0 is None else min(t0, start)
            t1 = end if t1 is None else max(t1, end)
            seqs.append(
                {
                    "name": s["name"],
                    "start": round(start, 3),
                    "end": round(end, 3),
                    "duration_s": round(s["duration_s"], 2),
                    "length_m": round(s.get("gt_length_m") or 0, 2),
                    "z_range_m": round(s.get("gt_z_range_m") or 0, 2),
                    "place": place(s["name"]),
                    "elevator": bool(s.get("elevator")),
                }
            )
        payloads[str(p)] = {
            "robot": ROBOT[p],
            "color": PCOLOR[p],
            "lidar": LIDAR[p],
            "n_sequences": len(seqs),
            "duration_min": round(sum(s["duration_s"] for s in seqs) / 60.0, 1),
            "distance_km": round(sum(s["length_m"] for s in seqs) / 1000.0, 2),
            "sequences": seqs,
        }
    return {
        "t_start": round(t0, 3),
        "t_end": round(t1, 3),
        "payloads": payloads,
        "places": [
            {"id": "Start area", "color": "#d9d4c7"},
            {"id": "Outdoor campus", "color": "#9bc97f"},
            {"id": "ISEC", "color": "#7fb2e0"},
            {"id": "EXP", "color": "#e8a37a"},
            {"id": "High bay", "color": "#c3a6dc"},
            {"id": "Elevator", "color": "#4d4d4d"},
        ],
    }


def build_comm():
    meta = json.loads((COMM / "trace" / "comm_trace.json").read_text())
    links = []
    with (COMM / "trace" / "comm_trace_links.csv").open() as f:
        for row in csv.DictReader(f):
            links.append(
                {
                    "a": row["node_a"],
                    "b": row["node_b"],
                    "t0": float(row["t_start"]),
                    "t1": float(row["t_end"]),
                    "state": row["state"],
                    "rssi": float(row["rssi_dbm"]) if row["rssi_dbm"] else None,
                }
            )
    throughput = []
    with (COMM / "trace" / "comm_trace_throughput.csv").open() as f:
        for row in csv.DictReader(f):
            throughput.append(
                {
                    "t": float(row["t"]),
                    "client": row["client"],
                    "server": row["server"],
                    "ok": row["ok"].lower() in ("1", "true", "yes"),
                    "mbps": float(row["mbps"]) if row["mbps"] else None,
                }
            )
    return {
        "meta": {
            "t_start": meta["t_start"],
            "t_end": meta["t_end"],
            "grid_step_s": meta["grid_step_s"],
            "nodes": meta["nodes"],
            "states": meta["states"],
        },
        "links": links,
        "throughput": throughput,
    }


def parse_benchmark_summary(tex: str):
    """Parse tables/benchmark_summary.tex into structured rows."""
    rows = []
    current_input = None
    for line in tex.splitlines():
        line = line.strip()
        if not line or line.startswith("%") or line.startswith("\\") and "midrule" in line:
            continue
        # Input category lines start with a word then & Method
        m = re.match(
            r"^(Monocular|Stereo|Stereo--Inertial|LiDAR--Inertial)\s*&\s*([^&]+)&\s*"
            r"([^&]+)&\s*([^&]+)&\s*([^&]+)&\s*([^&]+)&\s*([^&]+)&\s*([^\\\\]+)",
            line,
        )
        if m:
            current_input = m.group(1).replace("--", "-")
            rows.append(
                {
                    "input": current_input,
                    "method": m.group(2).strip(),
                    "full": m.group(3).strip().rstrip("$").replace("$^\\ast$", "*").replace("$^{\\ast}$", "*"),
                    "partial": m.group(4).strip(),
                    "fail": m.group(5).strip(),
                    "ate": m.group(6).strip(),
                    "rpe_t": m.group(7).strip(),
                    "rpe_r": m.group(8).strip().rstrip("\\").strip(),
                }
            )
            continue
        m2 = re.match(
            r"^&\s*([^&]+)&\s*([^&]+)&\s*([^&]+)&\s*([^&]+)&\s*([^&]+)&\s*([^&]+)&\s*([^\\\\]+)",
            line,
        )
        if m2 and current_input:
            rows.append(
                {
                    "input": current_input,
                    "method": m2.group(1).strip(),
                    "full": m2.group(2).strip().replace("$^\\ast$", "*").replace("$^{\\ast}$", "*"),
                    "partial": m2.group(3).strip(),
                    "fail": m2.group(4).strip(),
                    "ate": m2.group(5).strip(),
                    "rpe_t": m2.group(6).strip(),
                    "rpe_r": m2.group(7).strip().rstrip("\\").strip(),
                }
            )
    return rows


def parse_benchmark_detail(tex: str):
    """Lightweight extraction of per-sequence ATE rows from benchmarking_all.tex."""
    methods = [
        "DROID-SLAM",
        "MASt3R-SLAM",
        "VGGT-SLAM 2.0",
        "DROID-SLAM",
        "ORB-SLAM3",
        "Basalt",
        "ORB-SLAM3",
        "VINS-Fusion",
        "LIO-SAM",
        "Vectr-SLAM",
    ]
    inputs = [
        "mono",
        "mono",
        "mono",
        "stereo",
        "stereo",
        "stereo-inertial",
        "stereo-inertial",
        "stereo-inertial",
        "lidar-inertial",
        "lidar-inertial",
    ]
    method_labels = [
        "DROID (M)",
        "MASt3R",
        "VGGT 2.0",
        "DROID (S)",
        "ORB3 (S)",
        "Basalt",
        "ORB3 (SI)",
        "VINS-Fusion",
        "LIO-SAM",
        "Vectr-SLAM",
    ]

    current_payload = None
    current_seq = None
    out = []
    # Capture payload headers like P0 (Spot)
    for line in tex.splitlines():
        pm = re.search(r"\\textbf\{P(\d)\}\s*\(([^)]+)\)", line)
        if pm:
            current_payload = int(pm.group(1))
        sm = re.search(r"\\multirow\{\d+\}\{\*\}\{([^}]+)\}", line)
        if sm and "ATE" not in line and "RPE" not in line and "Cov" not in line and "Full" not in sm.group(1):
            # sequence name cells
            name = sm.group(1).replace("\\_", "_").strip()
            if re.match(r"^\d", name) or name.startswith("0") or "_" in name:
                current_seq = name
        if "ATE" in line and current_payload is not None and current_seq:
            # strip latex coloring and take numeric cells after Metric
            cleaned = line
            cleaned = re.sub(r"\\textcolor\{[^}]+\}\{([^}]*)\}", r"\1", cleaned)
            cleaned = cleaned.replace("\\textbf{", "").replace("}", "")
            cleaned = cleaned.replace("\\ding{55}", "fail")
            cleaned = cleaned.replace("\\multirow{3}{*}{\\ding{55}}", "fail")
            cleaned = cleaned.replace("\\multirow{3}{*}{$\\dagger$}", "—")
            cleaned = cleaned.replace("$\\dagger$", "—")
            cleaned = cleaned.replace("--", "—")
            parts = [p.strip() for p in cleaned.split("&")]
            # find ATE row values: typically ends with values after 'ATE'
            vals = []
            collecting = False
            for p in parts:
                if "ATE" in p:
                    collecting = True
                    continue
                if collecting:
                    vals.append(p.rstrip("\\").strip())
            if len(vals) >= 10:
                vals = vals[:10]
                entry = {
                    "payload": current_payload,
                    "sequence": current_seq,
                    "metric": "ATE",
                    "values": {},
                }
                for lab, val in zip(method_labels, vals):
                    entry["values"][lab] = val
                out.append(entry)
    return {"methods": method_labels, "inputs": dict(zip(method_labels, inputs)), "rows": out}


def build_benchmark():
    summary_tex = (OVERLEAF / "tables" / "benchmark_summary.tex").read_text()
    detail_tex = (OVERLEAF / "tables" / "benchmarking_all.tex").read_text()
    summary = parse_benchmark_summary(summary_tex)
    # Manual fallback if regex fails
    if len(summary) < 5:
        summary = [
            {"input": "Monocular", "method": "DROID-SLAM", "full": "22", "partial": "0", "fail": "0", "ate": "10.81", "rpe_t": "0.810", "rpe_r": "5.03"},
            {"input": "Monocular", "method": "MASt3R-SLAM", "full": "1", "partial": "19", "fail": "1", "ate": "10.41", "rpe_t": "0.865", "rpe_r": "0.54"},
            {"input": "Monocular", "method": "VGGT-SLAM 2.0", "full": "5", "partial": "17", "fail": "0", "ate": "12.60", "rpe_t": "0.794", "rpe_r": "7.18"},
            {"input": "Stereo", "method": "DROID-SLAM", "full": "22", "partial": "0", "fail": "0", "ate": "7.13", "rpe_t": "0.200", "rpe_r": "4.66"},
            {"input": "Stereo", "method": "ORB-SLAM3", "full": "5", "partial": "17", "fail": "0", "ate": "2.97", "rpe_t": "0.214", "rpe_r": "1.24"},
            {"input": "Stereo-Inertial", "method": "Basalt", "full": "12", "partial": "0", "fail": "10", "ate": "1.34", "rpe_t": "0.087", "rpe_r": "0.59"},
            {"input": "Stereo-Inertial", "method": "ORB-SLAM3", "full": "6", "partial": "16", "fail": "0", "ate": "1.53", "rpe_t": "0.046", "rpe_r": "0.50"},
            {"input": "Stereo-Inertial", "method": "VINS-Fusion", "full": "16", "partial": "6", "fail": "0", "ate": "1.52", "rpe_t": "0.096", "rpe_r": "0.58"},
            {"input": "LiDAR-Inertial", "method": "Vectr-SLAM", "full": "17*", "partial": "—", "fail": "—", "ate": "0.13", "rpe_t": "0.058", "rpe_r": "1.11"},
        ]
    detail = parse_benchmark_detail(detail_tex)
    return {
        "summary": summary,
        "detail": detail,
        "note": "Medians over runs that cover the whole sequence (22 benchmark sequences). Partial-coverage numbers are not comparable.",
        "conditions": "Low light, little texture, glass, crowds, stairs, bridges, long outdoor routes, and elevator-exit starts are tested; no method is both complete and accurate on every condition.",
    }


def save_webp(src: Path, dst: Path, max_w: int = 1600, quality: int = 82):
    im = Image.open(src).convert("RGB")
    if im.width > max_w:
        h = int(im.height * max_w / im.width)
        im = im.resize((max_w, h), Image.LANCZOS)
    dst.parent.mkdir(parents=True, exist_ok=True)
    im.save(dst, "WEBP", quality=quality, method=6)
    print(f"image {dst.name} {dst.stat().st_size / 1024:.0f} KB")


def build_images():
    OUT_IMG.mkdir(parents=True, exist_ok=True)
    # Hero
    save_webp(OVERLEAF / "figures" / "intro" / "teaser_new.png", OUT_IMG / "teaser.webp", max_w=1920, quality=78)
    # Hardware
    for name in ("hw_payload.png", "hw_layers.png", "hw_sync_board.png", "PAYLOAD_LAYERS.jpg", "SYNC_BOARD_MINI.png"):
        src = OVERLEAF / "figures" / name
        if src.exists():
            save_webp(src, OUT_IMG / (Path(name).stem + ".webp"), max_w=1200, quality=85)
    # Dataset scenes
    scenes = []
    scene_dir = OVERLEAF / "figures" / "dataset"
    captions = {
        "basement": "Signal-denied basement",
        "dynamic": "Crowds during class change",
        "elev": "Elevator ride",
        "glass": "Glass and glare",
        "irreg_stair": "Irregular stairs",
        "lidar_degenerate": "LiDAR degeneracy",
        "perceptual_aliasing": "Perceptual aliasing",
        "reflective_floor": "Reflective floor",
        "transition": "Indoor–outdoor transition",
        "visual_degenerate": "Visual degeneracy",
    }
    for src in sorted(scene_dir.glob("*.png")):
        dst = OUT_IMG / "scenes" / (src.stem + ".webp")
        save_webp(src, dst, max_w=900, quality=80)
        scenes.append({"id": src.stem, "file": f"static/images/scenes/{src.stem}.webp", "caption": captions.get(src.stem, src.stem)})
    # Copy favicon if present
    fav = ROOT / "static" / "images" / "favicon.ico"
    return scenes


def copy_video():
    OUT_VIDEO.mkdir(parents=True, exist_ok=True)
    src = COMM / "plots" / "comm_video.mp4"
    dst = OUT_VIDEO / "comm_video.mp4"
    shutil.copy2(src, dst)
    print(f"video {dst.stat().st_size / 1e6:.2f} MB")


def write_json(name: str, obj):
    OUT_DATA.mkdir(parents=True, exist_ok=True)
    path = OUT_DATA / name
    with path.open("w") as f:
        json.dump(obj, f, separators=(",", ":"))
    print(f"json {name} {path.stat().st_size / 1024:.1f} KB")


def main():
    geo = load_georef()
    traj = build_trajectories(geo)
    write_json("trajectories.json", {"georef_note": "2D similarity from P2 GNSS; ~5 m median residual", "payloads": traj})

    seq = build_sequences()
    write_json("sequences.json", seq)

    platforms = []
    for p in range(5):
        info = seq["payloads"][str(p)]
        platforms.append(
            {
                "id": p,
                "label": f"P{p}",
                "robot": info["robot"],
                "color": info["color"],
                "lidar": info["lidar"],
                "sequences": info["n_sequences"],
                "duration_min": info["duration_min"],
                "distance_km": info["distance_km"],
                "scenes": ", ".join(
                    sorted({s["place"] for s in info["sequences"]})
                ),
            }
        )
    write_json(
        "platforms.json",
        {
            "platforms": platforms,
            "sensors": [
                {"name": "Stereo camera", "rate": "20 Hz", "detail": "2× FLIR Blackfly S BFS-U3-50S4C-C, Tamron 8 mm"},
                {"name": "LiDAR", "rate": "10 Hz", "detail": "Ouster OS1-128 (P0, P2) or OS1-32 (P1, P3, P4)"},
                {"name": "IMU", "rate": "200 Hz", "detail": "VectorNav VN-100"},
                {"name": "GNSS", "rate": "1 Hz", "detail": "u-blox ZED-F9T timing receiver"},
                {"name": "Radio", "rate": "0.2 Hz telemetry", "detail": "Doodle Labs RM-2450 mesh radio"},
            ],
            "totals": {"sequences": 67, "hours": 3.95, "km": 12.7, "floors": 14, "elevators": 28},
        },
    )

    write_json("comm.json", build_comm())
    write_json("benchmark.json", build_benchmark())

    scenes = build_images()
    write_json("scenes.json", {"scenes": scenes})
    copy_video()

    # Authors from paper
    authors = [
        "Aniket Gupta", "Zihao Dong", "Carlos Anzola Castellanos", "Xiangyu Bai",
        "Ajinkya Bhandare", "Srijan Dokania", "Bishoy Galoaa", "Dennis Giaya",
        "Jonah Jaffe", "Abhinav Reddy Kowkuntla", "Jasen Levoy", "Parth Mahajan",
        "Immanuel Ampomah Mensah", "Lillian Miles", "Rituraj Harish Navindgikar",
        "Gowtham Parasuraman", "Tapan Shashikant Patil", "Rohith Poola",
        "Nikolas Sanderson", "Samridhi Singh", "Andrew Snowdy", "Shriman Raghav Srinivasan",
        "Raj Harshit Srirangam", "Trishal Sai Srinivas Vengetela", "Zhexin Xu", "Mia Yim",
        "Michael Everett", "Sarah Ostadabbas", "Alan Papalia", "David M. Rosen", "Hanumant Singh",
    ]
    write_json(
        "meta.json",
        {
            "title": "NeuROAM",
            "subtitle": "A Dataset for Large-Scale, Heterogeneous, Multi-Robot SLAM in Dynamic Environments",
            "affiliation": "Institute for Experiential Robotics, Northeastern University",
            "authors": authors,
            "links": {
                "dataset": "https://huggingface.co/datasets/NEUFR/NeuROAM",
                "code": "https://github.com/neufieldrobotics/NeuROAM",
                "page": "https://neufieldrobotics.github.io/NeuROAM/",
            },
            "colors": {str(k): v for k, v in PCOLOR.items()},
        },
    )
    print("done")


if __name__ == "__main__":
    main()
