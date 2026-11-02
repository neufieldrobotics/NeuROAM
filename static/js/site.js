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

const ROBOT_PHOTOS = {
  0: { file: "static/images/robots/spot.webp", alt: "Boston Dynamics Spot" },
  1: { file: "static/images/robots/scout.webp", alt: "AgileX Scout Mini" },
  2: { file: "static/images/robots/hunter.webp", alt: "AgileX Hunter SE" },
  3: { file: "static/images/robots/go2w.webp", alt: "Unitree Go2-W" },
  4: { file: "static/images/robots/go2.webp", alt: "Unitree Go2" },
};

const ROBOT_MODELS = {
  0: "static/models/robots/spot.glb",
  1: "static/models/robots/scout.glb",
  2: "static/models/robots/hunter.glb",
  3: "static/models/robots/go2w.glb",
  4: "static/models/robots/go2.glb",
};

const GT_COLOR = "#9a9a94";

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
  document.getElementById("swatches").innerHTML = platforms.platforms
    .map(
      (p) =>
        `<span class="swatch"><i style="background:${p.color}"></i>P${p.id} ${p.robot}</span>`
    )
    .join("");
  document.getElementById("platformRows").innerHTML = platforms.platforms
    .map(
      (p) => `<tr>
        <td class="pid" style="color:${p.color}">P${p.id}</td>
        <td>${p.robot}</td>
        <td>${p.sequences}</td>
        <td>${p.duration_min} min</td>
        <td>${p.distance_km} km</td>
        <td>${p.scenes}</td>
      </tr>`
    )
    .join("");
  document.getElementById("sensorNote").innerHTML = platforms.sensors
    .map((s) => `<span><b>${s.name}</b> ${s.rate}</span>`)
    .join("");
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

function fullTrailCoords(points) {
  return points.map((p) => [p[2], p[1]]);
}

function liveTrailCoords(points, tNow) {
  const coords = [];
  for (const p of points) {
    if (p[0] > tNow) break;
    coords.push([p[2], p[1]]);
  }
  return coords;
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
    for (const [pid, payload] of Object.entries(traj.payloads)) {
      const gtId = `gt-${pid}`;
      const liveId = `trail-${pid}`;
      if (!map.getSource(gtId)) {
        map.addSource(gtId, {
          type: "geojson",
          data: {
            type: "Feature",
            geometry: { type: "LineString", coordinates: fullTrailCoords(payload.points) },
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
          data: { type: "Feature", geometry: { type: "LineString", coordinates: [] } },
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
            geometry: { type: "LineString", coordinates: liveTrailCoords(state.points, tNow) },
          });
        }
      }
    }
    drawLinkGraph(comm, tNow);
    window.dispatchEvent(new CustomEvent("neuroam-time", { detail: tNow }));
  }

  map.on("load", () => update(tStart));
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

  const groupRoots = {};
  const centroids = {};
  let modelSize = 1;
  let clipMin = -0.3;
  let clipMax = 0.3;
  const root = new THREE.Group();
  scene.add(root);

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
      modelSize = size;
      clipMax = sizeVec.x * 0.55 + 0.02;
      clipMin = -sizeVec.x * 0.55 - 0.02;
      clipPlane.set(new THREE.Vector3(-1, 0, 0), clipMax);
      clipHelper.scale.set(sizeVec.z * 1.4, sizeVec.y * 1.4, 1);
      clipHelper.rotation.y = Math.PI / 2;

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
        const c = new THREE.Box3().setFromObject(obj).getCenter(new THREE.Vector3());
        centroids[gkey] = c;
        if (!groupRoots[gkey]) groupRoots[gkey] = [];
        groupRoots[gkey].push(obj);
        obj.userData.home = obj.position.clone();
        obj.userData.key = gkey;
      });

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

  document.getElementById("clip").addEventListener("input", (e) => {
    const u = Number(e.target.value) / 100;
    // 0 = no clip (plane beyond +X), 1 = cut through past center toward -X
    clipPlane.constant = clipMax + u * (clipMin - clipMax);
    clipHelper.position.set(clipPlane.constant, 0, 0);
    clipHelper.visible = u > 0.01;
  });

  document.getElementById("explode").addEventListener("input", (e) => {
    const amount = Number(e.target.value) / 100;
    const explodeDist = modelSize * 0.5;
    for (const obj of Object.values(groupRoots).flat()) {
      const c = centroids[obj.userData.key] || new THREE.Vector3(0, 1, 0);
      const dir = c.clone();
      if (dir.lengthSq() < 1e-8) dir.set(0, 1, 0);
      else dir.normalize();
      obj.position.copy(obj.userData.home).addScaledVector(dir, amount * explodeDist);
    }
  });

  function setView(name) {
    const dist = Math.max(modelSize * 1.35, 0.5);
    const views = {
      iso: new THREE.Vector3(0.85, 0.55, 0.95),
      front: new THREE.Vector3(0, 0.25, 1),
      side: new THREE.Vector3(1, 0.2, 0),
      top: new THREE.Vector3(0.05, 1, 0.05),
    };
    camera.position.copy(views[name].normalize().multiplyScalar(dist));
    controls.target.set(0, 0, 0);
    controls.update();
  }
  document.querySelectorAll(".cad-ui [data-view]").forEach((btn) => {
    btn.addEventListener("click", () => setView(btn.dataset.view));
  });

  (function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  })();
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

  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 5000);
  camera.position.set(80, 70, 90);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  host.appendChild(renderer.domElement);
  renderer.domElement.style.width = "100%";
  renderer.domElement.style.height = "100%";
  renderer.domElement.style.display = "block";
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  scene.add(new THREE.AmbientLight(0xffffff, 0.85));
  const grid = new THREE.GridHelper(200, 40, 0x888888, 0x444444);
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

  const lineGroups = {};
  const markers = {};
  for (const [pid, payload] of Object.entries(traj3d.payloads)) {
    const positions = [];
    for (const p of payload.points) {
      positions.push(p[1] - cx, p[3] * 1.5, p[2] - cy);
    }
    const gtGeo = new THREE.BufferGeometry();
    gtGeo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    const gtLine = new THREE.Line(
      gtGeo,
      new THREE.LineBasicMaterial({ color: GT_COLOR, transparent: true, opacity: 0.35 })
    );
    scene.add(gtLine);

    const liveGeo = new THREE.BufferGeometry();
    liveGeo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    liveGeo.setDrawRange(0, 2);
    const liveLine = new THREE.Line(
      liveGeo,
      new THREE.LineBasicMaterial({ color: payload.color })
    );
    scene.add(liveLine);

    lineGroups[pid] = { points: payload.points, liveLine };
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
      let count = 0;
      for (const p of state.points) {
        if (p[0] > tNow) break;
        count += 1;
      }
      state.liveLine.geometry.setDrawRange(0, Math.max(count, 2));
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

  renderHero(meta, platforms);
  renderPlatforms(platforms);
  initRobotGallery(platforms);
  initPlayback(traj, sequences, comm);
  initMap3D(traj3d);
  initCAD();
  initScenes(scenes, clips);
}

main().catch((err) => {
  console.error(err);
  document.body.insertAdjacentHTML(
    "afterbegin",
    `<p style="margin:1rem;color:#c44">Failed to initialize page: ${err.message}</p>`
  );
});
