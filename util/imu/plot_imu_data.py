#!/usr/bin/env python3
# pip install rosbags matplotlib

import argparse
from rosbags.highlevel import AnyReader
try:
    import matplotlib.pyplot as plt
except ImportError as e:
    # install instructions: export LD_PRELOAD="$CONDA_PREFIX/lib/libstdc++.so.6"
    install_msg = "If seeing error about 'libstdc++.so.6', try running:\n" \
                  "export LD_PRELOAD=\"$CONDA_PREFIX/lib/libstdc++.so.6\""
    raise ImportError(f"{e} \n\n{install_msg}")

# import path
from pathlib import Path

def main():
    p = argparse.ArgumentParser(description="Plot IMU accelerations and angular velocities from a ROS2 MCAP bag.")
    p.add_argument("bag", help="Path to the bag file or directory containing the .mcap")
    p.add_argument("--topic", default="/vectornav/imu", help="IMU topic (default: /vectornav/imu)")
    args = p.parse_args()

    t, ax_x, ax_y, ax_z = [], [], [], []
    w_x, w_y, w_z = [], [], []

    # print out all topics in the bag
    bag_paths = Path(args.bag)
    print(f"Reading bag from: {bag_paths}")

    with AnyReader([bag_paths]) as reader:
        conns = [c for c in reader.connections if c.topic == args.topic]
        if not conns:
            all_topics = [c.topic for c in reader.connections]
            raise SystemExit(f"No connections found for topic '{args.topic}'. Available topics: {all_topics}")

        t0 = None
        for conn, ts, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)

            if t0 is None:
                t0 = ts
            t.append((ts - t0) * 1e-9)  # ns → s

            ax = msg.linear_acceleration
            ax_x.append(ax.x)
            ax_y.append(ax.y)
            ax_z.append(ax.z)

            w = msg.angular_velocity
            w_x.append(w.x)
            w_y.append(w.y)
            w_z.append(w.z)

    fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex='col')

    # Top-left: Linear acceleration (lines)
    axs[0, 0].plot(t, ax_x, label="x")
    axs[0, 0].plot(t, ax_y, label="y")
    axs[0, 0].plot(t, ax_z, label="z")
    axs[0, 0].set_ylabel("Accel [m/s²]")
    axs[0, 0].set_title("Linear Acceleration (Lines)")
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    # Top-right: Angular velocity (lines)
    axs[0, 1].plot(t, w_x, label="x")
    axs[0, 1].plot(t, w_y, label="y")
    axs[0, 1].plot(t, w_z, label="z")
    axs[0, 1].set_ylabel("Ang Vel [rad/s]")
    axs[0, 1].set_title("Angular Velocity (Lines)")
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    # Bottom-left: Linear acceleration (scatter)
    axs[1, 0].scatter(t, ax_x, s=6, label="x")
    axs[1, 0].scatter(t, ax_y, s=6, label="y")
    axs[1, 0].scatter(t, ax_z, s=6, label="z")
    axs[1, 0].set_xlabel("Time [s]")
    axs[1, 0].set_ylabel("Accel [m/s²]")
    axs[1, 0].set_title("Linear Acceleration (Scatter)")
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    # Bottom-right: Angular velocity (scatter)
    axs[1, 1].scatter(t, w_x, s=6, label="x")
    axs[1, 1].scatter(t, w_y, s=6, label="y")
    axs[1, 1].scatter(t, w_z, s=6, label="z")
    axs[1, 1].set_xlabel("Time [s]")
    axs[1, 1].set_ylabel("Ang Vel [rad/s]")
    axs[1, 1].set_title("Angular Velocity (Scatter)")
    axs[1, 1].legend()
    axs[1, 1].grid(True)

    fig.suptitle(f"IMU Data from {args.topic}")
    plt.tight_layout()
    plt.show()

    # dump the imu data into a .pkl file in /tmp
    import pickle

    imu_data = {
        "time": t,
        "acceleration_x": ax_x,
        "acceleration_y": ax_y,
        "acceleration_z": ax_z,
        "angular_velocity_x": w_x,
        "angular_velocity_y": w_y,
        "angular_velocity_z": w_z
    }

    fname = "/tmp/imu_data.pkl"
    with open(fname, "wb") as f:
        pickle.dump(imu_data, f)
    print(f"IMU data saved to {fname}")


if __name__ == "__main__":
    main()
