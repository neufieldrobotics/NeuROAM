#!/usr/bin/env python3
# pip install rosbags matplotlib

import argparse
from rosbags.highlevel import AnyReader
import matplotlib.pyplot as plt

def main():
    p = argparse.ArgumentParser(description="Plot IMU accelerations and angular velocities from a ROS2 MCAP bag.")
    p.add_argument("bag", help="Path to the bag file or directory containing the .mcap")
    p.add_argument("--topic", default="/vectornav/imu", help="IMU topic (default: /vectornav/imu)")
    args = p.parse_args()

    t, ax_x, ax_y, ax_z = [], [], [], []
    w_x, w_y, w_z = [], [], []

    with AnyReader([args.bag]) as reader:
        conns = [c for c in reader.connections if c.topic == args.topic]
        if not conns:
            raise SystemExit(f"Topic {args.topic!r} not found in {args.bag}")

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

if __name__ == "__main__":
    main()
