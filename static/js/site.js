import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const COLORS = {
  0: "#2a78d6",
  1: "#eb6834",
  2: "#1baf7a",
  3: "#eda100",
  4: "#4a3aa7",
};

const STATE_COLOR = {
  direct: "#2a78d6",
  mesh_route: "#1baf7a",
  no_link: "#c9c9c4",
  unknown: "#e8e8e4",
};

const CAD_COLORS = {
  lidar: "#2e2e33",
  cameras: "#2a78d6",
  imu: "#eb6834",
  gnss: "#1baf7a",
  radio: "#eda100",
  compute: "#4a3aa7",
  power: "#8c8c85",
  display: "#598cbf",
  enclosure: "#c7c7be",
};

const CAD_LABELS = {
  lidar: "Ouster OS1 LiDAR",
  cameras: "FLIR Blackfly S (stereo)",
  imu: "VectorNav VN-100",
  gnss: "u-blox ZED-F9T",
  radio: "Doodle Labs RM-2450",
  compute: "Jetson Orin Nano",
  power: "Power distribution",
  display: "Status display",
  enclosure: "Enclosure",
};

const CAD_EXPLODE = {
  lidar: { to: [0, 0.1, -0.36], at: [0, 0.45] },
  enclosure: { to: [0, -0.08, 0], at: [0, 0.45] },
  cameras: { to: [-0.2, 0, 0], at: [0.1, 0.55] },
  display: { to: [0.2, 0, 0], at: [0.1, 0.55] },
  power: { to: [0, 0.1, 0.24], at: [0.2, 0.65] },
  compute: { to: [0, 0.42, 0], at: [0.3, 0.8] },
  radio: { to: [0, 0.3, 0], at: [0.4, 0.88] },
  gnss: { to: [0, 0.2, 0], at: [0.5, 0.94] },
  imu: { to: [0, 0.1, 0], at: [0.55, 1] },
};

const ROBOT_PHOTOS = {
  0: { file: "static/images/robots/spot.webp?v=2", alt: "NeuROAM Spot with payload" },
  1: { file: "static/images/robots/scout.webp?v=2", alt: "AgileX Scout Mini" },
  2: { file: "static/images/robots/hunter.webp?v=2", alt: "NeuROAM Hunter SE with payload" },
  3: { file: "static/images/robots/go2w.webp?v=2", alt: "NeuROAM Go2-W with payload" },
  4: { file: "static/images/robots/go2.webp?v=2", alt: "Unitree Go2" },
};

const ROBOT_MODELS = {
  0: "static/models/robots/spot.glb",
  1: "static/models/robots/scout.glb",
  2: "static/models/robots/hunter.glb",
  3: "static/models/robots/go2w.glb",
  4: "static/models/robots/go2.glb",
};

const GT_COLOR = "#9a9a94";
const TRAIL_GAP_S = 60;

async function loadJSON(path) {
  const res = await fetch(path);
  if (!res.ok) throw new Error(`Failed to load ${path}`);
  return res.json();
}

function fmtClock(t) {
  const d = new Date(t * 1000);
  return (
    d.toLocaleTimeString("en-US", {
      timeZone: "America/New_York",
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      hour12: false,
    }) + " EST"
  );
}

function initTheme() {
  const root = document.documentElement;
  const btn = document.getElementById("themeToggle");
  const saved = localStorage.getItem("neuroam-theme");
  const preferDark = window.matchMedia("(prefers-color-scheme: dark)").matches;
  const theme = saved || (preferDark ? "dark" : "light");
  root.setAttribute("data-theme", theme);
  btn.textContent = theme === "dark" ? "Light" : "Dark";
  btn.addEventListener("click", () => {
    const next = root.getAttribute("data-theme") === "dark" ? "light" : "dark";
    root.setAttribute("data-theme", next);
    localStorage.setItem("neuroam-theme", next);
    btn.textContent = next === "dark" ? "Light" : "Dark";
    window.dispatchEvent(new CustomEvent("neuroam-theme", { detail: next }));
  });
}

function mountFleetViewer(host, url) {
  const scene = new THREE.Scene();
  const syncBg = () => {
    const dark = document.documentElement.getAttribute("data-theme") === "dark";
    scene.background = new THREE.Color(dark ? 0x1c1c1a : 0xf7f6f3);
  };
  syncBg();
  window.addEventListener("neuroam-theme", syncBg);
  const camera = new THREE.PerspectiveCamera(32, 1, 0.01, 20);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.75));
  host.appendChild(renderer.domElement);
  renderer.domElement.style.width = "100%";
  renderer.domElement.style.height = "100%";
  renderer.domElement.style.display = "block";
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.enablePan = false;
  controls.autoRotate = true;
  controls.autoRotateSpeed = 1.1;
  scene.add(new THREE.AmbientLight(0xffffff, 0.72));
  const key = new THREE.DirectionalLight(0xffffff, 1.05);
  key.position.set(2, 3, 2);
  scene.add(key);
  const root = new THREE.Group();
  scene.add(root);
  let spinning = true;
  controls.addEventListener("start", () => {
    spinning = false;
  });
  controls.addEventListener("end", () => {
    spinning = true;
  });

  new GLTFLoader().load(url, (gltf) => {
    // CAD is Z-up. Stand it up in Three's Y-up frame.
    const upright = new THREE.Group();
    upright.rotation.x = -Math.PI / 2;
    upright.add(gltf.scene);
    root.add(upright);
    upright.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(upright);
    const center = box.getCenter(new THREE.Vector3());
    upright.position.sub(center);
    const size = Math.max(box.getSize(new THREE.Vector3()).length(), 0.2);
    camera.near = size / 200;
    camera.far = size * 20;
    camera.position.set(size * 0.72, size * 0.38, size * 0.95);
    controls.target.set(0, size * 0.02, 0);
    controls.update();
    camera.updateProjectionMatrix();
  });

  const resize = () => {
    const w = Math.max(host.clientWidth, 1);
    const h = Math.max(host.clientHeight, 1);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  };
  resize();
  window.addEventListener("resize", resize);
  (function loop() {
    requestAnimationFrame(loop);
    controls.autoRotate = spinning;
    controls.update();
    renderer.render(scene, camera);
  })();
}

function initFleet(platforms) {
  const grid = document.getElementById("robotGrid");
  if (!grid) return;
  const fleet = platforms.platforms.map((p) => ({
    id: p.id,
    name: p.robot,
    file: `${ROBOT_MODELS[p.id]}?v=7`,
    detail: `${p.sequences} sequences · ${p.distance_km} km`,
  }));
  grid.innerHTML = fleet
    .map(
      (r) => `<figure class="fleet-card" style="--accent:${COLORS[r.id]}">
        <div class="fleet-view" id="fleetView${r.id}"></div>
        <figcaption>
          <b style="color:${COLORS[r.id]}">P${r.id} ${r.name}</b>
          <span>${r.detail}</span>
        </figcaption>
      </figure>`
    )
    .join("");
  for (const r of fleet) {
    if (r.file) mountFleetViewer(document.getElementById(`fleetView${r.id}`), r.file);
  }
}

function renderHero(meta, platforms) {
  const totals = platforms.totals;
  const stats = [
    [totals.sequences, "sequences"],
    [`${totals.hours} h`, "duration"],
    [`${totals.km} km`, "distance"],
    [totals.floors, "floors"],
    [totals.elevators, "elevator rides"],
  ];
  document.getElementById("heroStats").innerHTML = stats
    .map(([v, l]) => `<div class="stat"><b>${v}</b><span>${l}</span></div>`)
    .join("");
  const aff = document.getElementById("affiliation");
  if (aff) aff.textContent = meta.affiliation || "";
}

function initHeroParallax() {
  const hero = document.getElementById("hero");
  if (!hero) return;
  const reduce = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  if (reduce) return;

  let pointerX = 0;
  let pointerY = 0;
  let hasPointer = false;
  let currX = 0;
  let currY = 0;
  let start = performance.now();
  let visible = true;

  const tick = (now) => {
    if (!visible) {
      requestAnimationFrame(tick);
      return;
    }
    const t = (now - start) / 1000;
    const idleX = Math.sin(t * 0.35) * 0.28 + Math.sin(t * 0.11) * 0.12;
    const idleY = Math.cos(t * 0.27) * 0.22 + Math.cos(t * 0.09) * 0.1;
    const targetX = hasPointer ? pointerX * 0.85 + idleX * 0.25 : idleX;
    const targetY = hasPointer ? pointerY * 0.85 + idleY * 0.25 : idleY;
    currX += (targetX - currX) * 0.06;
    currY += (targetY - currY) * 0.06;
    hero.style.setProperty("--hero-x", currX.toFixed(4));
    hero.style.setProperty("--hero-y", currY.toFixed(4));
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);

  hero.addEventListener(
    "pointermove",
    (e) => {
      const r = hero.getBoundingClientRect();
      hasPointer = true;
      pointerX = ((e.clientX - r.left) / r.width) * 2 - 1;
      pointerY = ((e.clientY - r.top) / r.height) * 2 - 1;
    },
    { passive: true }
  );

  hero.addEventListener(
    "pointerleave",
    () => {
      hasPointer = false;
    },
    { passive: true }
  );

  const onScroll = () => {
    const r = hero.getBoundingClientRect();
    visible = r.bottom > 0 && r.top < window.innerHeight;
    if (!visible) return;
    const progress = 1 - Math.min(1, Math.max(0, r.bottom / (r.height + window.innerHeight * 0.35)));
    hero.style.setProperty("--hero-scroll", progress.toFixed(4));
  };
  window.addEventListener("scroll", onScroll, { passive: true });
  onScroll();
}

function renderPlatforms(platforms) {
  const swatches = document.getElementById("swatches");
  const rows = document.getElementById("platformRows");
  const sensors = document.getElementById("sensorNote");
  if (swatches) {
    swatches.innerHTML = platforms.platforms
      .map(
        (p) =>
          `<span class="swatch"><i style="background:${p.color}"></i>P${p.id} ${p.robot}</span>`
      )
      .join("");
  }
  if (rows) {
    rows.innerHTML = platforms.platforms
      .map(
        (p) => `<tr>
        <td class="pid" style="color:${p.color}">P${p.id}</td>
        <td>${p.robot}</td>
        <td>${p.sequences}</td>
        <td>${p.duration_min} min</td>
        <td>${p.distance_km} km</td>
        <td>${p.scenes || "—"}</td>
      </tr>`
      )
      .join("");
  }
  if (sensors) {
    sensors.innerHTML = platforms.sensors
      .map((s) => `<span><b>${s.name}</b> ${s.rate}</span>`)
      .join("");
  }
}

function initRobotGallery(platforms) {
  const grid = document.getElementById("robotGrid");
  if (!grid) return;
  grid.innerHTML = platforms.platforms
    .map((p) => {
      const photo = ROBOT_PHOTOS[p.id];
      const delay = (p.id * 1.7).toFixed(1);
      return `<article class="robot-card" style="--accent:${p.color}; --kb-delay:${delay}s">
        <div class="rv">
          <img class="robot-photo" src="${photo.file}" alt="${photo.alt}" loading="lazy" width="1000" height="750" />
          <span class="rv-glow" aria-hidden="true"></span>
        </div>
        <div class="meta">
          <strong style="color:${p.color}">P${p.id} ${p.robot}</strong>
          <span>${p.sequences} sequences · ${p.distance_km} km</span>
        </div>
      </article>`;
    })
    .join("");
}

function interpolatePose(points, t) {
  if (!points.length) return null;
  if (t <= points[0][0]) return points[0];
  if (t >= points[points.length - 1][0]) return points[points.length - 1];
  let lo = 0;
  let hi = points.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (points[mid][0] <= t) lo = mid;
    else hi = mid;
  }
  const a = points[lo];
  const b = points[hi];
  if (b[0] - a[0] > TRAIL_GAP_S) return a;
  const u = (t - a[0]) / Math.max(1e-6, b[0] - a[0]);
  return [t, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u, a[3] + (b[3] - a[3]) * u];
}

function placeAt(sequences, t) {
  for (const s of sequences) {
    if (t >= s.start && t <= s.end) return s.place;
  }
  return "—";
}

function linksAt(comm, t, node) {
  const heard = [];
  for (const link of comm.links) {
    if (t < link.t0 || t > link.t1) continue;
    if (link.a !== node && link.b !== node) continue;
    if (link.state === "no_link" || link.state === "unknown") continue;
    const other = link.a === node ? link.b : link.a;
    heard.push(`${other} (${link.state === "direct" ? "direct" : "mesh"})`);
  }
  return heard.length ? heard.join(", ") : "none";
}

function pickClip(clips, t, sequences) {
  // Prefer a clip whose payload is active and place matches.
  const active = [];
  for (let p = 0; p < 5; p++) {
    const place = placeAt(sequences.payloads[String(p)].sequences, t);
    if (place !== "—") active.push({ p, place });
  }
  if (!active.length) return clips.clips[0];
  for (const a of active) {
    const hit = clips.clips.find((c) => c.payload === a.p && c.place === a.place);
    if (hit) return hit;
  }
  const byPayload = clips.clips.find((c) => c.payload === active[0].p);
  return byPayload || clips.clips[0];
}

function linkStateAt(comm, tNow, a, b) {
  for (const l of comm.links) {
    if (tNow < l.t0 || tNow > l.t1) continue;
    if ((l.a === a && l.b === b) || (l.a === b && l.b === a)) return l;
  }
  return null;
}

function drawLinkGraph(comm, tNow) {
  const host = document.getElementById("linkBands");
  if (!host) return;

  const robots = ["P0", "P1", "P2", "P3", "P4"];
  const W = 520;
  const H = 360;
  const cx = 250;
  const cy = 188;
  const R = 118;
  const pos = { BS: { x: cx, y: cy } };
  robots.forEach((id, i) => {
    const ang = -Math.PI / 2 + (i * 2 * Math.PI) / robots.length;
    pos[id] = { x: cx + Math.cos(ang) * R, y: cy + Math.sin(ang) * R };
  });

  const pairs = [];
  for (let i = 0; i < robots.length; i++) {
    for (let j = i + 1; j < robots.length; j++) pairs.push([robots[i], robots[j]]);
    pairs.push([robots[i], "BS"]);
  }

  let edges = "";
  let directN = 0;
  let meshN = 0;
  for (const [a, b] of pairs) {
    const hit = linkStateAt(comm, tNow, a, b);
    const state = hit?.state || "unknown";
    if (state === "unknown" || state === "no_link") {
      const pa = pos[a];
      const pb = pos[b];
      edges += `<line class="link-edge link-idle" x1="${pa.x}" y1="${pa.y}" x2="${pb.x}" y2="${pb.y}" />`;
      continue;
    }
    const pa = pos[a];
    const pb = pos[b];
    const cls = state === "direct" ? "link-direct" : "link-mesh";
    if (state === "direct") directN += 1;
    else meshN += 1;
    const midX = (pa.x + pb.x) / 2;
    const midY = (pa.y + pb.y) / 2;
    // slight arc so crossing edges read clearer
    const dx = pb.x - pa.x;
    const dy = pb.y - pa.y;
    const nx = -dy * 0.08;
    const ny = dx * 0.08;
    const qx = midX + nx;
    const qy = midY + ny;
    edges += `<path class="link-edge ${cls}" d="M ${pa.x} ${pa.y} Q ${qx} ${qy} ${pb.x} ${pb.y}" />`;
  }

  const nodeSvg = Object.entries(pos)
    .map(([id, p]) => {
      const isBS = id === "BS";
      const color = isBS ? "#8a8a85" : COLORS[Number(id.slice(1))];
      const robot = isBS ? "Base" : comm.meta.nodes[id]?.robot || id;
      const r = isBS ? 16 : 18;
      const labelY = isBS ? p.y + 32 : p.y + (p.y < cy ? -28 : 30);
      return `<g class="link-node" data-id="${id}">
        <circle cx="${p.x}" cy="${p.y}" r="${r + 5}" class="link-node-halo" style="stroke:${color}" />
        <circle cx="${p.x}" cy="${p.y}" r="${r}" fill="${color}" />
        <text x="${p.x}" y="${p.y + 4}" text-anchor="middle" class="link-node-id">${id}</text>
        <text x="${p.x}" y="${labelY}" text-anchor="middle" class="link-node-label">${robot}</text>
      </g>`;
    })
    .join("");

  host.innerHTML = `
    <div class="link-graph-head">
      <strong>Radio topology</strong>
      <span>${directN} direct · ${meshN} mesh</span>
    </div>
    <svg class="link-graph" viewBox="0 0 ${W} ${H}" role="img" aria-label="Circular radio link graph">
      <circle cx="${cx}" cy="${cy}" r="${R}" class="link-orbit" />
      ${edges}
      ${nodeSvg}
    </svg>
    <div class="link-legend">
      <span><i class="lg-direct"></i>direct</span>
      <span><i class="lg-mesh"></i>mesh route</span>
      <span><i class="lg-idle"></i>no link</span>
    </div>`;
}

function convexHull(points) {
  const uniq = [];
  const sorted = points.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  for (const p of sorted) {
    const last = uniq[uniq.length - 1];
    if (!last || last[0] !== p[0] || last[1] !== p[1]) uniq.push(p);
  }
  if (uniq.length < 3) return uniq;
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [];
  for (const p of uniq) {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) {
      lower.pop();
    }
    lower.push(p);
  }
  const upper = [];
  for (let i = uniq.length - 1; i >= 0; i--) {
    const p = uniq[i];
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) {
      upper.pop();
    }
    upper.push(p);
  }
  lower.pop();
  upper.pop();
  return lower.concat(upper);
}

function campusRing(traj) {
  const pts = [];
  for (const payload of Object.values(traj.payloads)) {
    for (const p of payload.points) pts.push([p[2], p[1]]);
  }
  const hull = convexHull(pts);
  if (hull.length < 3) return null;
  let cx = 0;
  let cy = 0;
  for (const p of hull) {
    cx += p[0];
    cy += p[1];
  }
  cx /= hull.length;
  cy /= hull.length;
  const ring = hull.map((p) => [cx + (p[0] - cx) * 1.08, cy + (p[1] - cy) * 1.08]);
  ring.push(ring[0]);
  return ring;
}

function splitTrail(points, tNow) {
  const segs = [];
  let cur = [];
  let prevT = null;
  for (const p of points) {
    if (tNow != null && p[0] > tNow) break;
    if (prevT != null && p[0] - prevT > TRAIL_GAP_S) {
      if (cur.length > 1) segs.push(cur);
      cur = [];
    }
    cur.push([p[2], p[1]]);
    prevT = p[0];
  }
  if (cur.length > 1) segs.push(cur);
  return segs;
}

function initPlayback(traj, sequences, comm) {
  const tStart = Math.min(sequences.t_start, comm.meta.t_start);
  const tEnd = Math.max(sequences.t_end, comm.meta.t_end);
  let t = tStart;
  let playing = false;
  let raf = null;
  let lastTs = null;

  const map = new maplibregl.Map({
    container: "map",
    style: getComputedStyle(document.documentElement).getPropertyValue("--map-style").trim(),
    center: [-71.0885, 42.3395],
    zoom: 15.4,
    attributionControl: true,
  });
  map.addControl(new maplibregl.NavigationControl({ showCompass: false }), "top-right");

  const campus = campusRing(traj);
  const campusBounds = new maplibregl.LngLatBounds();
  if (campus) {
    for (const coord of campus) campusBounds.extend(coord);
  }

  function frameCampus() {
    if (!campusBounds.isEmpty()) {
      map.fitBounds(campusBounds, { padding: 42, duration: 0, maxZoom: 17 });
    }
  }

  const markers = {};
  const panels = document.getElementById("robotPanels");
  panels.innerHTML = "";

  for (const [pid, payload] of Object.entries(traj.payloads)) {
    const el = document.createElement("div");
    el.className = "status-chip";
    el.style.setProperty("--accent", payload.color);
    el.innerHTML = `<strong style="color:${payload.color}">P${pid} ${payload.robot}</strong><span data-meta>—</span>`;
    panels.appendChild(el);
    const mEl = document.createElement("div");
    mEl.style.cssText = `width:12px;height:12px;border-radius:50%;background:${payload.color};border:2px solid #fff;box-shadow:0 0 0 1px rgba(0,0,0,.2)`;
    markers[pid] = {
      marker: new maplibregl.Marker({ element: mEl }).setLngLat([-71.0885, 42.3395]).addTo(map),
      panel: el,
      points: payload.points,
      color: payload.color,
    };
  }

  function ensureLayers() {
    if (campus && !map.getSource("campus")) {
      const dark = document.documentElement.getAttribute("data-theme") === "dark";
      map.addSource("campus", {
        type: "geojson",
        data: { type: "Feature", geometry: { type: "Polygon", coordinates: [campus] } },
      });
      map.addLayer({
        id: "campus-fill",
        type: "fill",
        source: "campus",
        paint: {
          "fill-color": dark ? "#d0d0ca" : "#7a7a74",
          "fill-opacity": dark ? 0.14 : 0.18,
        },
      });
      map.addLayer({
        id: "campus-line",
        type: "line",
        source: "campus",
        paint: {
          "line-color": dark ? "#f2f2ec" : "#3c3c38",
          "line-width": 2,
          "line-opacity": 0.9,
        },
      });
    }
    for (const [pid, payload] of Object.entries(traj.payloads)) {
      const gtId = `gt-${pid}`;
      const liveId = `trail-${pid}`;
      if (!map.getSource(gtId)) {
        map.addSource(gtId, {
          type: "geojson",
          data: {
            type: "Feature",
            geometry: { type: "MultiLineString", coordinates: splitTrail(payload.points) },
          },
        });
        map.addLayer({
          id: gtId,
          type: "line",
          source: gtId,
          paint: { "line-color": GT_COLOR, "line-width": 2.2, "line-opacity": 0.45 },
        });
      }
      if (!map.getSource(liveId)) {
        map.addSource(liveId, {
          type: "geojson",
          data: { type: "Feature", geometry: { type: "MultiLineString", coordinates: [] } },
        });
        map.addLayer({
          id: liveId,
          type: "line",
          source: liveId,
          paint: { "line-color": payload.color, "line-width": 3, "line-opacity": 0.92 },
        });
      }
    }
  }

  function update(tNow) {
    t = tNow;
    document.getElementById("clockLabel").textContent = fmtClock(tNow);
    const frac = (tNow - tStart) / (tEnd - tStart);
    document.getElementById("timeSlider").value = String(Math.round(frac * 1000));
    if (map.isStyleLoaded()) {
      ensureLayers();
      for (const [pid, state] of Object.entries(markers)) {
        const pose = interpolatePose(state.points, tNow);
        const seqs = sequences.payloads[pid].sequences;
        if (pose) {
          state.marker.setLngLat([pose[2], pose[1]]);
          state.panel.querySelector("[data-meta]").textContent =
            `${placeAt(seqs, tNow)} · ${pose[3].toFixed(1)} m`;
        } else {
          state.panel.querySelector("[data-meta]").textContent = "outside span";
        }
        const src = map.getSource(`trail-${pid}`);
        if (src) {
          src.setData({
            type: "Feature",
            geometry: { type: "MultiLineString", coordinates: splitTrail(state.points, tNow) },
          });
        }
      }
    }
    drawLinkGraph(comm, tNow);
    window.dispatchEvent(new CustomEvent("neuroam-time", { detail: tNow }));
  }

  map.on("load", () => {
    update(tStart);
    frameCampus();
  });
  document.getElementById("mapReset")?.addEventListener("click", () => frameCampus());
  document.getElementById("timeSlider").addEventListener("input", (e) => {
    const u = Number(e.target.value) / 1000;
    update(tStart + u * (tEnd - tStart));
  });

  const playBtn = document.getElementById("playBtn");
  playBtn.addEventListener("click", () => {
    playing = !playing;
    playBtn.textContent = playing ? "Pause" : "Play";
    lastTs = null;
    if (playing) {
      const loop = (ts) => {
        if (!playing) return;
        if (lastTs != null) {
          const speed = Number(document.getElementById("speedSelect").value);
          t += ((ts - lastTs) / 1000) * speed;
          if (t > tEnd) t = tStart;
          update(t);
        }
        lastTs = ts;
        raf = requestAnimationFrame(loop);
      };
      raf = requestAnimationFrame(loop);
    } else if (raf) {
      cancelAnimationFrame(raf);
    }
  });

  window.addEventListener("neuroam-theme", () => {
    const style = getComputedStyle(document.documentElement).getPropertyValue("--map-style").trim();
    map.setStyle(style);
    map.once("style.load", () => update(t));
  });

  return { tStart, tEnd };
}

function initGantt(sequences) {
  const canvas = document.getElementById("gantt");
  const ctx = canvas.getContext("2d");
  const placeColor = Object.fromEntries(sequences.places.map((p) => [p.id, p.color]));
  let currentT = sequences.t_start;

  function color() {
    const styles = getComputedStyle(document.documentElement);
    return {
      ink: styles.getPropertyValue("--ink").trim(),
      ink2: styles.getPropertyValue("--ink2").trim(),
      panel: styles.getPropertyValue("--panel").trim(),
    };
  }

  function draw(tNow = currentT) {
    currentT = tNow;
    const c = color();
    const W = canvas.width;
    const H = canvas.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = c.panel;
    ctx.fillRect(0, 0, W, H);
    const left = 90;
    const top = 28;
    const rowH = 56;
    const t0 = sequences.t_start;
    const t1 = sequences.t_end;
    const span = t1 - t0;
    let isec = 0;
    let exp = 0;
    for (let p = 0; p < 5; p++) {
      const y = top + p * rowH;
      const payload = sequences.payloads[String(p)];
      ctx.fillStyle = COLORS[p];
      ctx.font = "600 13px 'IBM Plex Sans', sans-serif";
      ctx.fillText(`P${p} ${payload.robot}`, 12, y + 28);
      for (const s of payload.sequences) {
        const x = left + ((s.start - t0) / span) * (W - left - 16);
        const w = Math.max(2, ((s.end - s.start) / span) * (W - left - 16));
        ctx.fillStyle = placeColor[s.place] || "#ccc";
        ctx.fillRect(x, y + 10, w, 28);
        if (tNow >= s.start && tNow <= s.end) {
          if (s.place === "ISEC") isec += 1;
          if (s.place === "EXP") exp += 1;
        }
        const h = Math.min(10, (s.z_range_m / 30) * 10);
        ctx.fillStyle = COLORS[p];
        ctx.globalAlpha = 0.55;
        ctx.fillRect(x, y + 40, w, Math.max(1, h));
        ctx.globalAlpha = 1;
      }
    }
    const xNow = left + ((tNow - t0) / span) * (W - left - 16);
    ctx.strokeStyle = c.ink;
    ctx.beginPath();
    ctx.moveTo(xNow, 8);
    ctx.lineTo(xNow, H - 24);
    ctx.stroke();
    ctx.fillStyle = c.ink2;
    ctx.font = "12px 'IBM Plex Sans', sans-serif";
    ctx.fillText(`At playhead: ${isec} in ISEC · ${exp} in EXP`, left, H - 8);
    let lx = left;
    for (const p of sequences.places) {
      ctx.fillStyle = p.color;
      ctx.fillRect(lx, 8, 10, 10);
      ctx.fillStyle = c.ink2;
      ctx.fillText(p.id, lx + 14, 17);
      lx += ctx.measureText(p.id).width + 28;
    }
    document.getElementById("ganttNote").textContent =
      `Playhead ${fmtClock(tNow)} · ${isec} robot${isec === 1 ? "" : "s"} in ISEC, ${exp} in EXP.`;
  }

  draw(sequences.t_start);
  window.addEventListener("neuroam-time", (e) => draw(e.detail));
  window.addEventListener("neuroam-theme", () => draw(currentT));
}

function initCAD() {
  const host = document.getElementById("cad");
  document.getElementById("cadLegend").innerHTML = Object.entries(CAD_COLORS)
    .map(([k, v]) => `<span><i style="background:${v}"></i>${CAD_LABELS[k] || k}</span>`)
    .join("");

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
  camera.position.set(0.7, 0.45, 0.85);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.localClippingEnabled = true;
  host.appendChild(renderer.domElement);
  renderer.domElement.style.width = "100%";
  renderer.domElement.style.height = "100%";
  renderer.domElement.style.display = "block";

  const syncBg = () => {
    const dark = document.documentElement.getAttribute("data-theme") === "dark";
    const hex = dark ? 0x242421 : 0xf0efeb;
    scene.background = new THREE.Color(hex);
    renderer.setClearColor(hex, 1);
  };
  syncBg();
  window.addEventListener("neuroam-theme", syncBg);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const key = new THREE.DirectionalLight(0xffffff, 1.1);
  key.position.set(2.5, 4, 2);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xb8c4ff, 0.5);
  fill.position.set(-2, 1.5, -1);
  scene.add(fill);

  const clipPlane = new THREE.Plane(new THREE.Vector3(-1, 0, 0), 10);
  const clipHelper = new THREE.Mesh(
    new THREE.PlaneGeometry(1, 1),
    new THREE.MeshBasicMaterial({
      color: 0x2a78d6,
      transparent: true,
      opacity: 0.12,
      side: THREE.DoubleSide,
      depthWrite: false,
    })
  );
  clipHelper.visible = false;
  scene.add(clipHelper);

  const parts = []; // one entry per CAD group mesh: { obj, home, box, to, at, offset }
  let modelSize = 1;
  let baseRadius = 1;
  const explodeInput = document.getElementById("explode");
  const clipInput = document.getElementById("clip");
  let explodeTarget = Number(explodeInput.value) / 100;
  let explodeNow = 0;
  let clipU = Number(clipInput.value) / 100;
  const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const root = new THREE.Group();
  scene.add(root);

  const layout = { box: new THREE.Box3(), center: new THREE.Vector3(), radius: 1 };
  const _box = new THREE.Box3();
  const _v = new THREE.Vector3();
  const _c = new THREE.Vector3();
  const _prevCenter = new THREE.Vector3();
  const easeInOut = (x) => (x < 0.5 ? 4 * x * x * x : 1 - Math.pow(-2 * x + 2, 3) / 2);


  const leaderMat = new THREE.LineDashedMaterial({
    color: 0x8c8c85,
    dashSize: 0.012,
    gapSize: 0.008,
    transparent: true,
    opacity: 0,
    depthWrite: false,
    clippingPlanes: [clipPlane],
  });
  const leaders = new THREE.LineSegments(new THREE.BufferGeometry(), leaderMat);
  leaders.visible = false;
  scene.add(leaders);
  const syncLeaders = () => {
    const dark = document.documentElement.getAttribute("data-theme") === "dark";
    leaderMat.color.set(dark ? 0xa3a39b : 0x6f6f69);
  };
  syncLeaders();
  window.addEventListener("neuroam-theme", syncLeaders);

  function applyExplode(t) {
    layout.box.makeEmpty();
    for (const p of parts) {
      const [a, b] = p.at;
      const k = easeInOut(THREE.MathUtils.clamp((t - a) / (b - a), 0, 1));
      p.offset.copy(p.to).multiplyScalar(k * modelSize);
      p.obj.position.copy(p.home).add(p.offset);
      layout.box.union(_box.copy(p.box).translate(p.offset));
    }
    layout.box.getCenter(layout.center);
    let r = 0;
    for (const p of parts) {
      _box.copy(p.box).translate(p.offset);
      for (let i = 0; i < 8; i++) {
        _v.set(
          i & 1 ? _box.max.x : _box.min.x,
          i & 2 ? _box.max.y : _box.min.y,
          i & 4 ? _box.max.z : _box.min.z
        );
        r = Math.max(r, _v.distanceTo(layout.center));
      }
    }
    layout.radius = Math.max(r, 1e-3);
  }


  const frameScale = (r = layout.radius) => 1 + (r / baseRadius - 1) * 0.8;

  function reframe(prevCenter, prevRadius) {
    _v.subVectors(camera.position, controls.target).multiplyScalar(frameScale() / frameScale(prevRadius));
    controls.target.add(_c.subVectors(layout.center, prevCenter));
    camera.position.copy(controls.target).add(_v);
  }

  function updateClip() {
    if (!parts.length) return;
    // 0 = no clip (plane beyond +X), 1 = cut through past center toward -X
    const b = layout.box;
    const sx = b.max.x - b.min.x;
    const hi = b.max.x + sx * 0.05 + 0.02;
    const lo = b.min.x - sx * 0.05 - 0.02;
    clipPlane.constant = hi + clipU * (lo - hi);
    clipHelper.position.set(clipPlane.constant, layout.center.y, layout.center.z);
    clipHelper.scale.set((b.max.z - b.min.z) * 1.4, (b.max.y - b.min.y) * 1.4, 1);
    clipHelper.visible = clipU > 0.01;
  }

  function updateLeaders(t) {
    const pos = leaders.geometry.attributes.position;
    if (!pos) return;
    let i = 0;
    for (const p of parts) {
      if (!p.leader) continue;
      p.box.getCenter(_v);
      pos.setXYZ(i++, _v.x, _v.y, _v.z);
      _v.add(p.offset);
      pos.setXYZ(i++, _v.x, _v.y, _v.z);
    }
    pos.needsUpdate = true;
    leaders.geometry.computeBoundingSphere();
    leaders.computeLineDistances();
    leaderMat.opacity = 0.75 * THREE.MathUtils.smoothstep(t, 0.03, 0.3);
    leaders.visible = leaderMat.opacity > 0.01;
  }

  function resize() {
    const w = Math.max(host.clientWidth, 1);
    const h = Math.max(host.clientHeight, 1);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  }
  resize();
  window.addEventListener("resize", resize);

  new GLTFLoader().load(
    "static/models/payload.glb",
    (gltf) => {
      root.clear();
      root.add(gltf.scene);
      gltf.scene.updateMatrixWorld(true);
      const box = new THREE.Box3().setFromObject(gltf.scene);
      const sizeVec = box.getSize(new THREE.Vector3());
      const size = Math.max(sizeVec.length(), 0.2);
      const center = box.getCenter(new THREE.Vector3());
      gltf.scene.position.sub(center);
      gltf.scene.updateMatrixWorld(true);
      modelSize = size;
      clipHelper.rotation.y = Math.PI / 2;
      parts.length = 0;

      gltf.scene.traverse((obj) => {
        if (!obj.isMesh) return;
        const name = (obj.name || "enclosure").toLowerCase();
        const gkey = Object.keys(CAD_COLORS).find((k) => name === k) || "enclosure";
        if (obj.geometry && !obj.geometry.attributes.normal) obj.geometry.computeVertexNormals();
        obj.material = new THREE.MeshStandardMaterial({
          color: new THREE.Color(CAD_COLORS[gkey]),
          metalness: 0.1,
          roughness: 0.48,
          side: THREE.DoubleSide,
          clippingPlanes: [clipPlane],
          clipShadows: true,
        });
        const plan = CAD_EXPLODE[gkey] || { to: [0, 0, 0], at: [0, 1] };
        parts.push({
          obj,
          home: obj.position.clone(),
          box: new THREE.Box3().setFromObject(obj),
          to: new THREE.Vector3(...plan.to),
          at: plan.at,
          offset: new THREE.Vector3(),
          leader: gkey !== "enclosure" && plan.to.some((v) => v !== 0),
        });
      });

      const nLeaders = parts.filter((p) => p.leader).length;
      leaders.geometry.setAttribute(
        "position",
        new THREE.BufferAttribute(new Float32Array(nLeaders * 2 * 3), 3)
      );
      applyExplode(0);
      baseRadius = layout.radius;
      explodeNow = 0;
      updateClip();
      updateLeaders(0);

      const dist = size * 1.35;
      camera.near = size / 200;
      camera.far = size * 40;
      camera.position.set(dist * 0.85, dist * 0.55, dist * 0.95);
      camera.updateProjectionMatrix();
      controls.target.set(0, 0, 0);
      controls.update();
      resize();
    },
    undefined,
    (err) => {
      host.insertAdjacentHTML(
        "beforeend",
        `<p class="note" style="position:absolute;left:1rem;bottom:1rem;color:#c44">CAD failed to load: ${err.message}</p>`
      );
    }
  );

  clipInput.addEventListener("input", (e) => {
    clipU = Number(e.target.value) / 100;
    updateClip();
  });

  explodeInput.addEventListener("input", (e) => {
    explodeTarget = Number(e.target.value) / 100;
  });

  function setView(name) {
    const dist = Math.max(modelSize * 1.35, 0.5) * frameScale();
    const views = {
      iso: new THREE.Vector3(0.85, 0.55, 0.95),
      front: new THREE.Vector3(0, 0.25, 1),
      side: new THREE.Vector3(1, 0.2, 0),
      top: new THREE.Vector3(0.05, 1, 0.05),
    };
    controls.target.copy(layout.center);
    camera.position.copy(views[name].normalize().multiplyScalar(dist)).add(layout.center);
    controls.update();
  }
  document.querySelectorAll(".cad-ui [data-view]").forEach((btn) => {
    btn.addEventListener("click", () => setView(btn.dataset.view));
  });

  let last = performance.now();
  (function animate(now = performance.now()) {
    requestAnimationFrame(animate);
    const dt = Math.min(Math.max((now - last) / 1000, 0), 0.1);
    last = now;
    if (parts.length && explodeNow !== explodeTarget) {
      const prevCenter = _prevCenter.copy(layout.center);
      const prevRadius = layout.radius;
      explodeNow += (explodeTarget - explodeNow) * (reduceMotion ? 1 : 1 - Math.exp(-dt * 7));
      if (Math.abs(explodeTarget - explodeNow) < 5e-4) explodeNow = explodeTarget;
      applyExplode(explodeNow);
      reframe(prevCenter, prevRadius);
      updateClip();
      updateLeaders(explodeNow);
    }
    controls.update();
    renderer.render(scene, camera);
  })();
}

async function initCloud() {
  const host = document.getElementById("cloudView");
  const status = document.getElementById("cloudStatus");
  if (!host) return;

  const scene = new THREE.Scene();
  const syncBg = () => {
    const dark = document.documentElement.getAttribute("data-theme") === "dark";
    scene.background = new THREE.Color(dark ? 0x141413 : 0xf4f3ef);
    scene.fog = new THREE.Fog(scene.background, 80, 420);
  };
  syncBg();
  window.addEventListener("neuroam-theme", syncBg);

  const camera = new THREE.PerspectiveCamera(50, 1, 0.2, 2000);
  const renderer = new THREE.WebGLRenderer({ antialias: false, powerPreference: "high-performance" });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  host.appendChild(renderer.domElement);
  renderer.domElement.style.width = "100%";
  renderer.domElement.style.height = "100%";
  renderer.domElement.style.display = "block";
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.rotateSpeed = 0.55;
  controls.zoomSpeed = 0.8;

  function resize() {
    const w = Math.max(host.clientWidth, 1);
    const h = Math.max(host.clientHeight, 1);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  }
  resize();
  window.addEventListener("resize", resize);
  (function loop() {
    requestAnimationFrame(loop);
    controls.update();
    renderer.render(scene, camera);
  })();

  try {
    const meta = await loadJSON("static/cloud/campus_meta.json");
    if (status) status.textContent = `Loading ${meta.count.toLocaleString()} points…`;
    const [xyzBuf, rgbBuf] = await Promise.all([
      fetch("static/cloud/campus_xyz.bin").then((r) => {
        if (!r.ok) throw new Error("xyz");
        return r.arrayBuffer();
      }),
      fetch("static/cloud/campus_rgb.bin").then((r) => {
        if (!r.ok) throw new Error("rgb");
        return r.arrayBuffer();
      }),
    ]);
    const xyz = new Float32Array(xyzBuf);
    const rgb = new Uint8Array(rgbBuf);
    // Source map is Z-up. Three.js is Y-up.
    for (let i = 0; i < xyz.length; i += 3) {
      const y = xyz[i + 1];
      xyz[i + 1] = xyz[i + 2];
      xyz[i + 2] = -y;
    }
    const colors = new Float32Array(rgb.length);
    for (let i = 0; i < rgb.length; i++) colors[i] = rgb[i] / 255;

    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(xyz, 3));
    geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    geo.computeBoundingBox();
    geo.computeBoundingSphere();
    const mat = new THREE.PointsMaterial({
      size: 2.1,
      vertexColors: true,
      sizeAttenuation: false,
    });
    scene.add(new THREE.Points(geo, mat));

    const box = geo.boundingBox;
    const center = box.getCenter(new THREE.Vector3());
    const span = box.getSize(new THREE.Vector3()).length();
    camera.near = Math.max(0.2, span / 5000);
    camera.far = span * 4;
    camera.position.set(center.x + span * 0.22, center.y + span * 0.28, center.z + span * 0.38);
    controls.target.copy(center);
    controls.update();
    camera.updateProjectionMatrix();
    scene.fog = new THREE.Fog(scene.background, span * 0.45, span * 1.25);

    const millions = (meta.source_points / 1e6).toFixed(0);
    const note = document.getElementById("cloudNote");
    if (note) {
      note.textContent = `${meta.count.toLocaleString()} points on screen, voxel ${meta.voxel_m} m, from ${millions} million in the reference map. Height is colour.`;
    }
    if (status) status.hidden = true;
  } catch (err) {
    console.error(err);
    if (status) status.textContent = "Point cloud failed to load.";
  }
}

function initMap3D(traj3d) {
  const host = document.getElementById("map3d");
  if (!host) return;
  const scene = new THREE.Scene();
  const syncBg = () => {
    const dark = document.documentElement.getAttribute("data-theme") === "dark";
    scene.background = new THREE.Color(dark ? 0x1c1c1a : 0xf6f5f2);
  };
  syncBg();
  window.addEventListener("neuroam-theme", syncBg);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 8000);
  const home = new THREE.Vector3(80, 70, 90);
  camera.position.copy(home);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  host.appendChild(renderer.domElement);
  renderer.domElement.style.width = "100%";
  renderer.domElement.style.height = "100%";
  renderer.domElement.style.display = "block";
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  scene.add(new THREE.AmbientLight(0xffffff, 0.85));
  const grid = new THREE.GridHelper(900, 45, 0x8a8a85, 0x5c5c58);
  scene.add(grid);

  let cx = 0;
  let cy = 0;
  let n = 0;
  for (const payload of Object.values(traj3d.payloads)) {
    for (const p of payload.points) {
      cx += p[1];
      cy += p[2];
      n += 1;
    }
  }
  cx /= Math.max(n, 1);
  cy /= Math.max(n, 1);

  let reach = 40;
  for (const payload of Object.values(traj3d.payloads)) {
    for (const p of payload.points) {
      const x = p[1] - cx;
      const y = p[3] * 1.5;
      const z = p[2] - cy;
      reach = Math.max(reach, Math.hypot(x, y, z));
    }
  }
  home.set(reach * 0.85, reach * 0.62, reach * 1.05);
  camera.position.copy(home);
  camera.near = Math.max(0.5, reach / 500);
  camera.far = reach * 8;
  camera.updateProjectionMatrix();
  controls.target.set(0, 0, 0);
  controls.update();

  function resetView() {
    camera.position.copy(home);
    controls.target.set(0, 0, 0);
    controls.update();
  }
  document.getElementById("map3dReset")?.addEventListener("click", resetView);

  const lineGroups = {};
  const markers = {};
  for (const [pid, payload] of Object.entries(traj3d.payloads)) {
    const runs = [];
    let run = [];
    let prevT = null;
    for (const p of payload.points) {
      if (prevT != null && p[0] - prevT > TRAIL_GAP_S && run.length) {
        runs.push(run);
        run = [];
      }
      run.push(p);
      prevT = p[0];
    }
    if (run.length) runs.push(run);

    const liveLines = runs.map((segment) => {
      const positions = [];
      for (const p of segment) positions.push(p[1] - cx, p[3] * 1.5, p[2] - cy);
      const gtGeo = new THREE.BufferGeometry();
      gtGeo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
      scene.add(
        new THREE.Line(
          gtGeo,
          new THREE.LineBasicMaterial({ color: GT_COLOR, transparent: true, opacity: 0.35 })
        )
      );
      const liveGeo = new THREE.BufferGeometry();
      liveGeo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
      liveGeo.setDrawRange(0, 0);
      const liveLine = new THREE.Line(
        liveGeo,
        new THREE.LineBasicMaterial({ color: payload.color })
      );
      scene.add(liveLine);
      return { points: segment, liveLine };
    });

    lineGroups[pid] = { points: payload.points, liveLines };
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(1.2, 16, 16),
      new THREE.MeshStandardMaterial({ color: payload.color })
    );
    scene.add(m);
    markers[pid] = m;
  }

  function setTime(tNow) {
    for (const [pid, state] of Object.entries(lineGroups)) {
      const pose = interpolatePose(state.points, tNow);
      if (!pose) {
        markers[pid].visible = false;
        continue;
      }
      markers[pid].visible = true;
      markers[pid].position.set(pose[1] - cx, pose[3] * 1.5, pose[2] - cy);
      for (const run of state.liveLines) {
        let count = 0;
        for (const p of run.points) {
          if (p[0] > tNow) break;
          count += 1;
        }
        run.liveLine.geometry.setDrawRange(0, count);
      }
    }
  }

  window.addEventListener("neuroam-time", (e) => setTime(e.detail));

  function resize() {
    const w = Math.max(host.clientWidth, 1);
    const h = Math.max(host.clientHeight, 1);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  }
  resize();
  window.addEventListener("resize", resize);

  (function loop() {
    requestAnimationFrame(loop);
    controls.update();
    renderer.render(scene, camera);
  })();
}

function initScenes(scenes, clips) {
  const video = document.getElementById("sceneVideo");
  const caption = document.getElementById("sceneCaption");
  const thumbs = document.getElementById("sceneThumbs");
  let idx = 0;

  function show(i) {
    idx = (i + clips.clips.length) % clips.clips.length;
    const c = clips.clips[idx];
    video.src = c.file;
    caption.textContent = c.title;
    video.play().catch(() => {});
    thumbs.querySelectorAll(".scene-thumb").forEach((el, j) => {
      el.classList.toggle("active", j === idx);
    });
  }

  thumbs.innerHTML = clips.clips
    .map(
      (c, i) => `<button type="button" class="scene-thumb" data-i="${i}">
        <video muted playsinline preload="metadata" src="${c.file}"></video>
        <span style="color:${COLORS[c.payload]}">${c.title}</span>
      </button>`
    )
    .join("");
  thumbs.addEventListener("click", (e) => {
    const btn = e.target.closest(".scene-thumb");
    if (!btn) return;
    show(Number(btn.dataset.i));
  });
  video.addEventListener("ended", () => show(idx + 1));
  // Auto-advance every 9s even if looped
  setInterval(() => {
    if (document.visibilityState === "visible") show(idx + 1);
  }, 9000);
  show(0);

  const hardNav = document.getElementById("hardNav");
  const hardImage = document.getElementById("hardImage");
  const hardCaption = document.getElementById("hardCaption");
  if (hardNav && hardImage && scenes.scenes?.length) {
    let hardIdx = 0;
    function showHard(i) {
      hardIdx = (i + scenes.scenes.length) % scenes.scenes.length;
      const s = scenes.scenes[hardIdx];
      hardImage.src = s.file;
      hardImage.alt = s.caption;
      hardCaption.textContent = s.caption;
      hardNav.querySelectorAll("button").forEach((btn, j) => {
        btn.classList.toggle("active", j === hardIdx);
      });
    }
    hardNav.innerHTML = scenes.scenes
      .map(
        (s, i) =>
          `<button type="button" role="tab" data-i="${i}" aria-selected="${i === 0}">${s.caption}</button>`
      )
      .join("");
    hardNav.addEventListener("click", (e) => {
      const btn = e.target.closest("button");
      if (!btn) return;
      showHard(Number(btn.dataset.i));
    });
    showHard(0);
  }
}

async function main() {
  initTheme();
  initHeroParallax();
  const [meta, platforms, traj, sequences, comm, scenes, clips, traj3d] = await Promise.all([
    loadJSON("static/data/meta.json"),
    loadJSON("static/data/platforms.json"),
    loadJSON("static/data/trajectories.json"),
    loadJSON("static/data/sequences.json"),
    loadJSON("static/data/comm.json"),
    loadJSON("static/data/scenes.json"),
    loadJSON("static/data/clips.json"),
    loadJSON("static/data/trajectories_3d.json"),
  ]);

  const steps = [
    ["hero", () => renderHero(meta, platforms)],
    ["platforms", () => renderPlatforms(platforms)],
    ["fleet", () => initFleet(platforms)],
    ["playback", () => initPlayback(traj, sequences, comm)],
    ["map3d", () => initMap3D(traj3d)],
    ["cloud", () => initCloud()],
    ["cad", () => initCAD()],
    ["scenes", () => initScenes(scenes, clips)],
  ];
  const failures = [];
  for (const [name, run] of steps) {
    try {
      run();
    } catch (err) {
      console.error(`NeuROAM init failed at ${name}:`, err);
      failures.push(`${name}: ${err.message}`);
    }
  }
  if (failures.length) {
    document.body.insertAdjacentHTML(
      "afterbegin",
      `<p style="margin:1rem;color:#c44">Some sections failed to load (${failures.join("; ")}). Hard-refresh the page (Ctrl/Cmd+Shift+R).</p>`
    );
  }
}

main().catch((err) => {
  console.error(err);
  document.body.insertAdjacentHTML(
    "afterbegin",
    `<p style="margin:1rem;color:#c44">Failed to initialize page: ${err.message}. Try a hard refresh (Ctrl/Cmd+Shift+R).</p>`
  );
});
