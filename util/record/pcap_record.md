Great plan—pcap is the most bullet‑proof way to capture Ouster. Here’s the clean workflow that labs use:

# A) Record packets (+ metadata!) on the robot

## Option 1 — Use tcpdump/dumpcap (fast, minimal deps)

1. Find your NIC (e.g., `eth0`) and sensor IP (e.g., `10.5.5.87`).
2. Capture LiDAR **and** IMU UDP ports (defaults: 7502, 7503) to rotating pcap files:

```bash
sudo dumpcap -i eth0 -s 0 \
  -f "udp and host 10.5.5.87 and (port 7502 or port 7503)" \
  -b filesize:2048 -b files:50 \
  -w /data/ouster_%F_%H%M%S.pcap
```

(Default ports: LiDAR 7502, IMU 7503; TCP control is 7501. Confirm/override in your sensor config.) ([data.ouster.io][1], [static.ouster.dev][2])

3. Save the **metadata JSON** from the sensor at the same time (needed to decode pcaps later):

```bash
curl -s http://10.5.5.87/api/v1/sensor/metadata -o /data/ouster_metadata.json
```

(HTTP API endpoint for full metadata; keep it alongside the pcap.) ([data.ouster.io][3])

## Option 2 — Use the Ouster SDK to record (auto‑grabs metadata)

```bash
python3 -m ouster.sdk.examples.client os-<serial-or-hostname> record-pcap
```

This records LiDAR+IMU to a pcap **and** writes the matching metadata JSON for you. ([static.ouster.dev][4])

---

# B) Reconstruct points later

## Path 1 — With the Ouster SDK (Python)

The SDK reads `(pcap + metadata.json)` and yields `LidarScan`s (ranges/intensities/etc.), which you can convert to XYZ and write PCD/NPZ or publish. See the pcap examples/CLI:

```bash
python -m ouster.sdk.examples.pcap -h
```

…and “one scan” visualization helpers. ([static.ouster.dev][5])
Docs emphasize: recorded pcaps **must** be accompanied by the metadata JSON you saved. ([static.ouster.dev][6])

## Path 2 — Replay into ROS 2

Use the Ouster ROS driver’s replay support to publish `/ouster/points` from your pcap/metadata (handy if your downstream stack expects ROS topics). Many forks/branches include a `replay_pcap` launch that takes `pcap` + `metadata` and republishes point clouds:

* Example `replay_pcap.launch.xml` (ROS 2): shows parameters for `pcap_path` and `metadata_path`. ([GitLab][7])
* The official `ouster_ros` package documents replay functionality and changes in recent releases. ([ROS Documentation][8], [ROS Index][9])

(If you happen to have a **ROS bag** instead of pcap, there’s a `replay.launch` path that takes `bag_file` + `metadata` to regenerate topics.) ([static.ouster.dev][10])

## Path 3 — Ouster Studio (GUI)

Drag‑and‑drop `(pcap, json)` to visualize and export; useful for quick checks or sharing. ([data.ouster.io][11], [ouster.com][12])

---

# C) Small but important tips

* **Keep pcap + JSON together** (same folder, same session). Tools auto‑pair them. ([static.ouster.dev][6])
* If you changed modes (e.g., 1024×10 ↔ 2048×10), grab **new metadata**—it encodes intrinsics and operating mode. ([data.ouster.io][1])
* On the Jetson, write to **NVMe**, use `noatime`, and rotate files (`-b` options above) to avoid long fsync stalls.
* If you only need packets for lossless regen later, consider **not** building live point clouds during the run—just capture the pcap and reconstruct offline (saves CPU).

If you want, I can drop you a tiny Python script that: loads `(pcap+json)` via the SDK, converts each `LidarScan` to XYZ, and exports per‑scan `.pcd` files—or republishes `/ouster/points` in ROS 2 for downstream tools.

[1]: https://data.ouster.io/downloads/tcp-http-api-manual/api-manual-v2.3.0.pdf "Ouster Sensor API Guide"
[2]: https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html?utm_source=chatgpt.com "Sensor Data — Ouster Sensor Docs documentation"
[3]: https://data.ouster.io/downloads/tcp-http-api-manual/api-manual-v3.1.0.pdf?utm_source=chatgpt.com "api-manual-v3.1.0.pdf"
[4]: https://static.ouster.dev/sdk-docs/python/examples/record-stream.html?utm_source=chatgpt.com "Recording, Streaming, and Conversion - Ouster SDK"
[5]: https://static.ouster.dev/sdk-docs/_modules/ouster/sdk/examples/pcap.html?utm_source=chatgpt.com "Source code for ouster.sdk.examples.pcap"
[6]: https://static.ouster.dev/sdk-docs/python/examples/basics-sensor.html?utm_source=chatgpt.com "Working with an Ouster sensor - Ouster SDK"
[7]: https://git.wur.nl/parobotics/husky/sensors/ouster/-/blob/main/ouster-ros/launch/replay_pcap.launch.xml?utm_source=chatgpt.com "ouster-ros/launch/replay_pcap.launch.xml · main - Husky"
[8]: https://docs.ros.org/en/rolling/p/ouster_ros/__CHANGELOG.html?utm_source=chatgpt.com "CHANGELOG — ouster_ros 0.11.1 documentation"
[9]: https://index.ros.org/p/ouster_ros/?utm_source=chatgpt.com "ROS Package: ouster_ros"
[10]: https://static.ouster.dev/sdk-docs/0.5.1/ros/index.html?utm_source=chatgpt.com "ROS Guide — Ouster Sensor SDK 0.5.1 documentation"
[11]: https://data.ouster.io/downloads/software-user-manual/Ouster-Studio-2.0.4.pdf?utm_source=chatgpt.com "Ouster Studio Quick Start Guide"
[12]: https://ouster.com/insights/blog/new-ouster-studio-blog?utm_source=chatgpt.com "New Ouster Studio for desktop and web: A more seamless ..."
