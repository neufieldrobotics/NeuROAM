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

async function loadJSON(path) {
  const res = await fetch(path);
  if (!res.ok) throw new Error(`Failed to load ${path}`);
  return res.json();
}

function fmtClock(t) {
  const d = new Date(t * 1000);
  return d.toLocaleTimeString("en-US", {
    timeZone: "America/New_York",
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  }) + " EST";
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
  const authors = meta.authors.join(", ");
  document.getElementById("authors").innerHTML =
    `<div>${authors}</div><div class="affil">${meta.affiliation}</div>`;
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
        <td>${p.lidar}</td>
        <td>${p.sequences}</td>
        <td>${p.duration_min} min</td>
        <td>${p.distance_km} km</td>
      </tr>`
    )
    .join("");
  document.getElementById("sensorNote").textContent = platforms.sensors
    .map((s) => `${s.name} ${s.rate}`)
    .join(" · ");
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
  return [
    t,
    a[1] + (b[1] - a[1]) * u,
    a[2] + (b[2] - a[2]) * u,
    a[3] + (b[3] - a[3]) * u,
  ];
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
    el.className = "robot-panel";
    el.style.borderLeftColor = payload.color;
    el.innerHTML = `<strong style="color:${payload.color}">P${pid} ${payload.robot}</strong>
      <div class="meta" data-meta></div>
      <div class="links" data-links></div>`;
    panels.appendChild(el);

    const mEl = document.createElement("div");
    mEl.style.cssText = `width:14px;height:14px;border-radius:50%;background:${payload.color};border:2px solid #fff;box-shadow:0 0 0 1px rgba(0,0,0,.25)`;
    markers[pid] = {
      marker: new maplibregl.Marker({ element: mEl }).setLngLat([-71.0885, 42.3395]).addTo(map),
      panel: el,
      points: payload.points,
    };
  }

  function ensureLayers() {
    for (const [pid, payload] of Object.entries(traj.payloads)) {
      const id = `trail-${pid}`;
      if (map.getSource(id)) continue;
      map.addSource(id, {
        type: "geojson",
        data: { type: "Feature", geometry: { type: "LineString", coordinates: [] } },
      });
      map.addLayer({
        id,
        type: "line",
        source: id,
        paint: {
          "line-color": payload.color,
          "line-width": 2.5,
          "line-opacity": 0.85,
        },
      });
    }
  }

  function trailCoords(points, tNow) {
    const coords = [];
    for (const p of points) {
      if (p[0] > tNow) break;
      if (tNow - p[0] > 180) continue; // last 3 minutes
      coords.push([p[2], p[1]]);
    }
    return coords;
  }

  function update(tNow) {
    t = tNow;
    document.getElementById("clockLabel").textContent = fmtClock(tNow);
    const frac = (tNow - tStart) / (tEnd - tStart);
    document.getElementById("timeSlider").value = String(Math.round(frac * 1000));
    if (!map.isStyleLoaded()) return;
    ensureLayers();
    for (const [pid, state] of Object.entries(markers)) {
      const pose = interpolatePose(state.points, tNow);
      const seqs = sequences.payloads[pid].sequences;
      if (pose) {
        state.marker.setLngLat([pose[2], pose[1]]);
        state.panel.querySelector("[data-meta]").textContent =
          `${placeAt(seqs, tNow)} · z ${pose[3].toFixed(1)} m`;
      } else {
        state.panel.querySelector("[data-meta]").textContent = "outside released span";
      }
      state.panel.querySelector("[data-links]").textContent =
        `hears: ${linksAt(comm, tNow, `P${pid}`)}`;
      const src = map.getSource(`trail-${pid}`);
      if (src) {
        src.setData({
          type: "Feature",
          geometry: { type: "LineString", coordinates: trailCoords(state.points, tNow) },
        });
      }
    }
    drawLinkBands(comm, tStart, tEnd, tNow);
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
    } else if (raf) cancelAnimationFrame(raf);
  });

  window.addEventListener("neuroam-theme", () => {
    const style = getComputedStyle(document.documentElement).getPropertyValue("--map-style").trim();
    map.setStyle(style);
    map.once("style.load", () => update(t));
  });

  return { tStart, tEnd, update };
}

function drawLinkBands(comm, tStart, tEnd, tNow) {
  const host = document.getElementById("linkBands");
  const pairs = [];
  const seen = new Set();
  for (const l of comm.links) {
    if (l.a === "BS" || l.b === "BS") continue;
    const key = [l.a, l.b].sort().join("-");
    if (!seen.has(key)) {
      seen.add(key);
      pairs.push(key);
    }
  }
  pairs.sort();
  const W = 900;
  const rowH = 16;
  const left = 70;
  const H = 24 + pairs.length * rowH;
  let svg = `<svg viewBox="0 0 ${W} ${H}" xmlns="http://www.w3.org/2000/svg">`;
  pairs.forEach((pair, i) => {
    const y = 18 + i * rowH;
    const [a, b] = pair.split("-");
    svg += `<text x="0" y="${y + 10}" font-size="11" fill="currentColor" font-family="IBM Plex Sans, sans-serif">${a}–${b}</text>`;
    for (const l of comm.links) {
      const key = [l.a, l.b].sort().join("-");
      if (key !== pair) continue;
      const x0 = left + ((l.t0 - tStart) / (tEnd - tStart)) * (W - left);
      const x1 = left + ((l.t1 - tStart) / (tEnd - tStart)) * (W - left);
      svg += `<rect x="${x0}" y="${y}" width="${Math.max(1, x1 - x0)}" height="11" fill="${STATE_COLOR[l.state] || "#ccc"}" opacity="${l.state === "unknown" ? 0.45 : 0.9}"/>`;
    }
  });
  const xNow = left + ((tNow - tStart) / (tEnd - tStart)) * (W - left);
  svg += `<line x1="${xNow}" y1="0" x2="${xNow}" y2="${H}" stroke="currentColor" stroke-width="1.2"/>`;
  svg += `</svg>`;
  host.innerHTML = svg;
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
      rule: styles.getPropertyValue("--rule").trim(),
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
      }
      // height ribbon using z_range as a simple bar under each sequence
      for (const s of payload.sequences) {
        const x = left + ((s.start - t0) / span) * (W - left - 16);
        const w = Math.max(2, ((s.end - s.start) / span) * (W - left - 16));
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

    // place legend
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
  return { draw };
}

function initCAD() {
  const host = document.getElementById("cad");
  const legend = document.getElementById("cadLegend");
  legend.innerHTML = Object.entries(CAD_COLORS)
    .map(([k, v]) => `<span><i style="background:${v}"></i>${k}</span>`)
    .join("");

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
  camera.position.set(0.7, 0.45, 0.85);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, preserveDrawingBuffer: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
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
  controls.target.set(0, 0, 0);

  scene.add(new THREE.AmbientLight(0xffffff, 0.55));
  const key = new THREE.DirectionalLight(0xffffff, 1.15);
  key.position.set(2.5, 4, 2);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xb8c4ff, 0.55);
  fill.position.set(-2, 1.5, -1);
  scene.add(fill);

  const clipPlane = new THREE.Plane(new THREE.Vector3(-1, 0, 0), 10);
  renderer.localClippingEnabled = true;

  const groupRoots = {};
  const centroids = {};
  let modelSize = 1;
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

      gltf.scene.traverse((obj) => {
        if (!obj.isMesh) return;
        const name = (obj.name || "enclosure").toLowerCase();
        const gkey = Object.keys(CAD_COLORS).find((k) => name === k) || "enclosure";
        if (obj.geometry && !obj.geometry.attributes.normal) {
          obj.geometry.computeVertexNormals();
        }
        obj.material = new THREE.MeshStandardMaterial({
          color: new THREE.Color(CAD_COLORS[gkey]),
          metalness: 0.1,
          roughness: 0.48,
          side: THREE.DoubleSide,
          clippingPlanes: [clipPlane],
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
      controls.minDistance = size * 0.4;
      controls.maxDistance = size * 10;
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
    clipPlane.constant = 10 - u * 10.3;
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


function renderScenes(scenes) {
  document.getElementById("scenesGrid").innerHTML = scenes.scenes
    .map(
      (s) => `<figure class="scene"><img src="${s.file}" alt="${s.caption}" loading="lazy"><figcaption>${s.caption}</figcaption></figure>`
    )
    .join("");
}

function renderBenchmark(bench) {
  document.getElementById("benchSummary").innerHTML = bench.summary
    .map(
      (r) => `<tr>
        <td>${r.input}</td>
        <td>${r.method}</td>
        <td>${r.full}</td>
        <td>${r.partial}</td>
        <td>${r.fail}</td>
        <td>${r.ate}</td>
        <td>${r.rpe_t}</td>
        <td>${r.rpe_r}</td>
      </tr>`
    )
    .join("");
  document.getElementById("benchNote").textContent = `${bench.note} ${bench.conditions}`;

  const methods = bench.detail.methods || [];
  document.getElementById("benchDetailHead").innerHTML =
    `<tr><th>P</th><th>Sequence</th>${methods.map((m) => `<th>${m}</th>`).join("")}</tr>`;

  const filters = document.getElementById("payloadFilters");
  const allBtn = `<button type="button" data-p="all" class="active">All</button>`;
  filters.innerHTML =
    allBtn +
    [0, 1, 2, 3, 4]
      .map((p) => `<button type="button" data-p="${p}" style="color:${COLORS[p]}">P${p}</button>`)
      .join("");

  function paint(filter) {
    const rows = bench.detail.rows.filter((r) => filter === "all" || r.payload === Number(filter));
    document.getElementById("benchDetail").innerHTML = rows
      .map((r) => {
        const cells = methods
          .map((m) => {
            const v = r.values[m] || "—";
            const muted = /fail|—/.test(v) || Number(v) > 5;
            return `<td class="${muted ? "muted-cell" : ""}">${v}</td>`;
          })
          .join("");
        return `<tr><td class="pid" style="color:${COLORS[r.payload]}">P${r.payload}</td><td>${r.sequence}</td>${cells}</tr>`;
      })
      .join("");
  }
  paint("all");
  filters.addEventListener("click", (e) => {
    const btn = e.target.closest("button");
    if (!btn) return;
    filters.querySelectorAll("button").forEach((b) => b.classList.remove("active"));
    btn.classList.add("active");
    paint(btn.dataset.p);
  });
}

async function main() {
  initTheme();
  const [meta, platforms, traj, sequences, comm, scenes, bench] = await Promise.all([
    loadJSON("static/data/meta.json"),
    loadJSON("static/data/platforms.json"),
    loadJSON("static/data/trajectories.json"),
    loadJSON("static/data/sequences.json"),
    loadJSON("static/data/comm.json"),
    loadJSON("static/data/scenes.json"),
    loadJSON("static/data/benchmark.json"),
  ]);

  renderHero(meta, platforms);
  renderPlatforms(platforms);
  initPlayback(traj, sequences, comm);
  initGantt(sequences);
  initCAD();
  renderScenes(scenes);
  renderBenchmark(bench);
}

main().catch((err) => {
  console.error(err);
  document.body.insertAdjacentHTML(
    "afterbegin",
    `<p style="margin:1rem;color:#c44">Failed to initialize page: ${err.message}</p>`
  );
});
