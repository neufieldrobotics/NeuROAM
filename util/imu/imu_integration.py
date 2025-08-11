#!/usr/bin/env python3
import argparse
import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R

G = 9.80665  # m/s^2

def load_imu(pkl_path):
    with open(pkl_path, "rb") as f:
        d = pickle.load(f)
    t = np.asarray(d["time"], dtype=float)
    ax = np.asarray(d["acceleration_x"], dtype=float)
    ay = np.asarray(d["acceleration_y"], dtype=float)
    az = np.asarray(d["acceleration_z"], dtype=float)
    wx = np.asarray(d["angular_velocity_x"], dtype=float)
    wy = np.asarray(d["angular_velocity_y"], dtype=float)
    wz = np.asarray(d["angular_velocity_z"], dtype=float)

    # Basic sanity checks
    n = len(t)
    for arr, name in [(ax,"ax"),(ay,"ay"),(az,"az"),(wx,"wx"),(wy,"wy"),(wz,"wz")]:
        if len(arr) != n:
            raise ValueError(f"Length mismatch: {name} has {len(arr)} but time has {n}")
    # Ensure strictly increasing time
    order = np.argsort(t)
    t = t[order]
    ax, ay, az, wx, wy, wz = ax[order], ay[order], az[order], wx[order], wy[order], wz[order]
    return t, np.vstack([ax, ay, az]).T, np.vstack([wx, wy, wz]).T

def finite_differences(t, dt_max=None):
    dt = np.diff(t, prepend=t[0])
    dt[0] = np.median(dt[1:]) if len(dt) > 1 else 0.01
    # Guard against time jumps
    if dt_max is not None:
        dt = np.clip(dt, 0.0, dt_max)
    return dt

def detect_initial_static(a, w, t, max_duration=5.0,
                          gyro_thr=0.02,  # rad/s
                          acc_mag_thr=0.15):  # | |a|-g | < thr
    """
    Return slice [0:k) considered stationary at the start for bias estimation.
    Auto-stops when thresholds are exceeded or time exceeds max_duration.
    """
    gmag = np.linalg.norm(a[:500], axis=1).mean() if len(a) > 500 else G
    k = 0
    max_k = np.searchsorted(t, t[0] + max_duration, side="right")
    for i in range(min(max_k, len(t))):
        if (np.linalg.norm(w[i]) < gyro_thr) and (abs(np.linalg.norm(a[i]) - gmag) < acc_mag_thr):
            k = i + 1
        else:
            break
    # Require at least ~0.3 s for stable bias estimate if possible
    if k < 3 and len(t) > 1:
        k = max(0, np.searchsorted(t, t[0] + 0.3, side="right"))
    return k

def estimate_biases(a_static, w_static):
    """
    Accelerometers measure specific force f = a - g.
    In static, average accel ~ -R^T g in body. We only estimate gyro bias directly.
    For accel, we estimate a small bias by matching magnitude to g after attitude init.
    Here we just return gyro bias; accel bias is refined online.
    """
    gyro_bias = w_static.mean(axis=0) if len(w_static) else np.zeros(3)
    return np.zeros(3), gyro_bias  # accel_bias placeholder, gyro_bias

def init_attitude_from_accel(a_mean):
    """
    Get initial rotation that aligns measured accel direction with gravity.
    If a measures specific force f ≈ -R^T g_b, then g_b_est ≈ -a/|a|.
    We want R such that R^T g_w ≈ -a_hat  -> equivalently R maps body z to -a_hat in world.
    We'll construct world-frame such that z points up (0,0,1).
    """
    a_hat = a_mean / (np.linalg.norm(a_mean) + 1e-12)
    g_b_est = -a_hat
    # Build a rotation that maps body z-axis to g_b_est
    z_b = np.array([0.0, 0.0, 1.0])
    v = np.cross(z_b, g_b_est)
    s = np.linalg.norm(v)
    c = np.dot(z_b, g_b_est)
    if s < 1e-8:  # already aligned or opposite
        if c > 0:
            R0 = np.eye(3)
        else:
            # 180 deg around any axis perpendicular to z
            R0 = R.from_rotvec(np.pi * np.array([1, 0, 0])).as_matrix()
    else:
        vx = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])
        R0 = (np.eye(3) + vx + vx @ vx * ((1 - c) / (s**2)))
    # R_body_to_world starts as R0 (gravity up)
    return R.from_matrix(R0)

def mahony_step(Rbw, omega_meas, acc_meas, dt, kp=2.0, ki=0.05, bias=None, integ=None):
    """
    Mahony-like complementary update.
    Rbw: scipy Rotation (body->world)
    omega_meas: rad/s in body
    acc_meas: m/s^2 in body (specific force)
    Returns updated rotation, bias, integ.
    """
    if bias is None: bias = np.zeros(3)
    if integ is None: integ = np.zeros(3)

    # Expected gravity in body from current attitude
    g_w = np.array([0, 0, G])
    g_b_est = (Rbw.inv().apply(g_w))
    # Normalize both to compare directions
    a_norm = np.linalg.norm(acc_meas)
    if a_norm > 1e-6:
        a_hat = acc_meas / a_norm
        g_hat = g_b_est / (np.linalg.norm(g_b_est) + 1e-12)
        # Error (body frame)
        err = np.cross(a_hat, g_hat)  # drives g_hat -> a_hat
    else:
        err = np.zeros(3)

    # PI correction to gyro bias + corrected omega
    integ = integ + ki * err * dt
    omega_corr = omega_meas - bias + kp * err + integ

    # Integrate attitude with corrected rates
    dR = R.from_rotvec(omega_corr * dt)
    Rbw_new = Rbw * dR

    # Update bias (slow integral term already accumulating in integ;
    # we keep explicit bias for clarity)
    bias_new = bias  # bias folded into 'integ' term; keep as provided
    return Rbw_new, bias_new, integ

def zupt_detect(a, w, Rbw, gyro_thr=0.025, acc_mag_thr=0.15):
    """
    Simple stationarity test: small angular rate AND accel magnitude ~ g.
    """
    still = (np.linalg.norm(w) < gyro_thr) and (abs(np.linalg.norm(a) - G) < acc_mag_thr)
    return still

def integrate_odometry(t, a, w, dt_max=None, kp=2.0, ki=0.05,
                       zupt_vel_reset=True):
    dt = finite_differences(t, dt_max=dt_max)

    # Initial static window for biases + attitude
    k0 = detect_initial_static(a, w, t)
    a0_mean = a[:max(k0, 1)].mean(axis=0)
    _, gyro_bias = estimate_biases(a[:k0], w[:k0])

    # Initialize attitude
    Rbw = init_attitude_from_accel(a0_mean)
    integ = np.zeros(3)

    # State
    n = len(t)
    vel = np.zeros((n, 3))
    pos = np.zeros((n, 3))
    att_quat_wxyz = np.zeros((n, 4))

    g_w = np.array([0, 0, G])

    for i in range(n):
        dti = dt[i]

        # Orientation update (Mahony)
        Rbw, _, integ = mahony_step(Rbw, w[i] - gyro_bias, a[i], dti, kp=kp, ki=ki, integ=integ)
        att_quat_wxyz[i] = Rbw.as_quat()[[3,0,1,2]]  # scipy gives xyzw; store wxyz

        # Specific force (body) -> world linear acceleration
        # a_world = R * a_body + g_w  (since a_body = a - R^T g_w)
        a_world = Rbw.apply(a[i]) + g_w

        # Integrate velocity/position
        vel[i] = vel[i-1] + a_world * dti if i > 0 else a_world * dti
        pos[i] = pos[i-1] + vel[i] * dti if i > 0 else vel[i] * dti

        # Zero-velocity update if stationary
        if zupt_vel_reset and zupt_detect(a[i], w[i], Rbw):
            vel[i] = np.zeros(3)
            # Optional: pull recent velocity history toward zero to reduce drift
            if i > 0:
                pos[i] = pos[i-1]  # hold position during stance

    return pos, vel, att_quat_wxyz

def main():
    ap = argparse.ArgumentParser(description="Inertial Odometry (strapdown + Mahony + ZUPT)")
    ap.add_argument("pkl", type=str, help="Path to IMU data pickle")
    ap.add_argument("--dt-max", type=float, default=0.05, help="Clamp unusually large dt (s)")
    ap.add_argument("--kp", type=float, default=2.0, help="Mahony proportional gain")
    ap.add_argument("--ki", type=float, default=0.05, help="Mahony integral gain")
    ap.add_argument("--no-zupt", action="store_true", help="Disable zero-velocity updates")
    ap.add_argument("--save", type=str, default="", help="Optional path to save results as pickle")
    ap.add_argument("--plot", action="store_true", help="Plot results (requires matplotlib)")
    args = ap.parse_args()

    t, a, w = load_imu(args.pkl)

    pos, vel, quat_wxyz = integrate_odometry(
        t, a, w, dt_max=args.dt_max, kp=args.kp, ki=args.ki, zupt_vel_reset=(not args.no_zupt)
    )

    out = {
        "time": t,
        "position_m": pos,
        "velocity_mps": vel,
        "attitude_quat_wxyz": quat_wxyz,
    }

    if args.save:
        with open(args.save, "wb") as f:
            pickle.dump(out, f)
        print(f"Saved odometry to {args.save}")

    # Optional plotting (kept separate to avoid import issues)
    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError as e:
            # install instructions: export LD_PRELOAD="$CONDA_PREFIX/lib/libstdc++.so.6"
            install_msg = "If seeing error about 'libstdc++.so.6', try running:\n" \
                        "export LD_PRELOAD=\"$CONDA_PREFIX/lib/libstdc++.so.6\""
            raise ImportError(f"{e} \n\n{install_msg}")

        # Convert quat (wxyz) to RPY in degrees
        r_obj = R.from_quat(quat_wxyz[:, [1, 2, 3, 0]])  # xyzw
        rpy_deg = r_obj.as_euler("xyz", degrees=True)  # roll, pitch, yaw

        # Compute RPY rates by finite diff
        dt = np.diff(t, prepend=t[0])
        rpy_rates = np.vstack([
            np.gradient(rpy_deg[:, 0], t),
            np.gradient(rpy_deg[:, 1], t),
            np.gradient(rpy_deg[:, 2], t)
        ]).T

        fig, axs = plt.subplots(2, 2, figsize=(12, 8))

        # --- Position ---
        axs[0, 0].plot(t, pos[:, 0], label="x")
        axs[0, 0].plot(t, pos[:, 1], label="y")
        axs[0, 0].plot(t, pos[:, 2], label="z")
        axs[0, 0].set_title("Position (m)")
        axs[0, 0].legend()
        axs[0, 0].grid(True)

        # --- Velocity ---
        axs[1, 0].plot(t, vel[:, 0], label="vx")
        axs[1, 0].plot(t, vel[:, 1], label="vy")
        axs[1, 0].plot(t, vel[:, 2], label="vz")
        axs[1, 0].set_title("Velocity (m/s)")
        axs[1, 0].legend()
        axs[1, 0].grid(True)

        # --- Orientation (Roll, Pitch, Yaw) ---
        axs[0, 1].plot(t, rpy_deg[:, 0], label="Roll")
        axs[0, 1].plot(t, rpy_deg[:, 1], label="Pitch")
        axs[0, 1].plot(t, rpy_deg[:, 2], label="Yaw")
        axs[0, 1].set_title("Orientation (deg)")
        axs[0, 1].legend()
        axs[0, 1].grid(True)

        # --- Orientation rates ---
        axs[1, 1].plot(t, rpy_rates[:, 0], label="Roll rate")
        axs[1, 1].plot(t, rpy_rates[:, 1], label="Pitch rate")
        axs[1, 1].plot(t, rpy_rates[:, 2], label="Yaw rate")
        axs[1, 1].set_title("Orientation rates (deg/s)")
        axs[1, 1].legend()
        axs[1, 1].grid(True)

        fig.tight_layout()
        plt.show()

if __name__ == "__main__":
    main()
