# Ouster PCAP Capture → Merge → ROS 2 Replay

This guide walks through capturing Ouster UDP packets to rotating PCAP files, moving them into a workspace, merging them into a single PCAP, and replaying them with `ouster_ros`. **All command snippets below are exactly as provided.**

---

## 1) Capture packets with clean, rotating filenames

Use `dumpcap` to capture UDP traffic from your Ouster sensor interface into rolling 2 GB files (up to 50 files). The filename pattern will be like `ouster_00001_YYYYmmddHHMMSS.pcap`.

```bash
# cleaner filenames like ouster_00001_YYYYmmddHHMMSS.pcap
sudo dumpcap -i enP8p1s0 -s 0   -f 'udp and host 169.254.31.201'   -b filesize:2048 -b files:50   -w /tmp/ouster.pcap
```

**Notes**
- Replace `enP8p1s0` with your actual NIC name if different (check via `ip -br a` or `ifconfig`).
- `169.254.31.201` should match your Ouster sensor’s IP.
- `-s 0` captures full packets (no truncation).
- The `-b` options create a ring buffer so captures don’t grow without bound.

---

## 2) Move captures into your home directory

Create a workspace, move the new captures from `/tmp`, fix ownership, and confirm the files.

```bash
# Move the new captures into your home
mkdir -p ~/ouster_caps
sudo mv /tmp/ouster_*.pcap ~/ouster_caps/
sudo chown "$USER:$USER" ~/ouster_caps/ouster_*.pcap
ls -lh ~/ouster_caps | head
```

---

## 3) Merge and replay

Install the needed tooling (for `mergecap`), merge all PCAPs into one, then replay via `ouster_ros` with your metadata JSON.

```bash
# play 
sudo apt-get update && sudo apt-get install -y wireshark-common
cd ~/NeuROAM/ouster_caps
mergecap -w all_ouster_merged.pcap ouster_*.pcap
# then:
ros2 launch ouster_ros replay_pcap.launch.xml   pcap_file:=$HOME/NeuROAM/ouster_caps/all_ouster_merged.pcap   metadata:=$HOME/NeuROAM/ouster_caps/ouster_metadata.json
```

**Notes**
- Ensure `ouster_ros` is installed and `replay_pcap.launch.xml` is available in your ROS 2 environment.
- Confirm `ouster_metadata.json` matches the sensor configuration used during capture (model, UDP profiles, lidar mode, etc.).
