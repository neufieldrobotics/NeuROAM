#!/usr/bin/env python3
import argparse
import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R
try:
    import matplotlib.pyplot as plt
except ImportError as e:
    # install instructions: export LD_PRELOAD="$CONDA_PREFIX/lib/libstdc++.so.6"
    install_msg = "If seeing error about 'libstdc++.so.6', try running:\n" \
                "export LD_PRELOAD=\"$CONDA_PREFIX/lib/libstdc++.so.6\""
    raise ImportError(f"{e} \n\n{install_msg}")



# ---------- Helpers ----------
G = 9.80665
e3 = np.array([0.0, 0.0, 1.0])

def skew(v):
    """Return the 3x3 skew-symmetric matrix [v]_x such that [v]_x @ x = v \cross x."""
    x, y, z = v
    return np.array([[0, -z, y],
                     [z, 0, -x],
                     [-y, x, 0]])

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
    n = len(t)
    for arr, name in [(ax,"ax"),(ay,"ay"),(az,"az"),(wx,"wx"),(wy,"wy"),(wz,"wz")]:
        if len(arr) != n:
            raise ValueError(f"Length mismatch: {name} has {len(arr)} but time has {n}")
    order = np.argsort(t)
    t = t[order]
    a = np.vstack([ax, ay, az]).T[order]
    w = np.vstack([wx, wy, wz]).T[order]
    return t, a, w

def finite_dt(t, dt_max=None):
    """
    Compute sample intervals dt from timestamps `t` with a robust first element.

    - Uses np.diff with prepend to keep shape, then replaces dt[0] with the
      median of subsequent intervals (or 0.01s if only one sample).
    - Optionally clamps dt to [0, dt_max] to avoid numerical explosions when a
      logging glitch produces huge time jumps.

    Parameters
    ----------
    t : (N,) array_like
        Monotonic timestamps (seconds).
    dt_max : float or None
        If provided, clamp each dt <= dt_max.

    Returns
    -------
    dt : (N,) ndarray
        Per-sample time step (seconds).
    """
    dt = np.diff(t, prepend=t[0])
    dt[0] = np.median(dt[1:]) if len(dt) > 1 else 0.01
    if dt_max is not None:
        dt = np.clip(dt, 0.0, dt_max)
    return dt

def detect_initial_static(a, w, t, max_duration=5.0, gyro_thr=0.02, acc_mag_thr=0.15):
    """
    Find an initial stationary window [0:k) for bias/attitude initialization.

    Heuristic: walk forward from t[0] until either
      - angular rate exceeds `gyro_thr` [rad/s], or
      - |‖a‖ - g| exceeds `acc_mag_thr` [m/s^2], or
      - `max_duration` seconds elapse.

    If the detected window is too short (< ~0.3s), extend to at least that
    duration if data allows (helps stabilize averages).

    Parameters
    ----------
    a : (N,3) ndarray
        Accelerometer specific force measurements in **body** frame [m/s^2].
    w : (N,3) ndarray
        Gyro measurements in **body** frame [rad/s].
    t : (N,) ndarray
        Timestamps [s].
    max_duration : float
        Maximum time span to search for initial static interval (seconds).
    gyro_thr : float
        Threshold on ‖ω‖ for "still".
    acc_mag_thr : float
        Threshold on |‖a‖ - g| for "still".

    Returns
    -------
    k : int
        Count of initial samples deemed stationary.
    """
    k = 0
    max_k = np.searchsorted(t, t[0] + max_duration, side="right")
    for i in range(min(max_k, len(t))):
        below_gyro_thr = np.linalg.norm(w[i]) < gyro_thr
        below_acc_thr = abs(np.linalg.norm(a[i]) - G) < acc_mag_thr
        if below_gyro_thr and below_acc_thr:
            k += 1
        else:
            break
    if k < 3 and len(t) > 1:
        k = max(0, np.searchsorted(t, t[0] + 0.3, side="right"))
    return k

def zupt_gate(a, w, gyro_thr=0.03, acc_mag_thr=0.25):
    """
    Zero-velocity update (ZUPT) gate.

    Returns True when the IMU is *very likely* stationary:
      - small angular rate, and
      - accelerometer magnitude close to g.

    Tighter than gravity gate because we will *measure* velocity=0 when this passes.
    """
    return (np.linalg.norm(w) < gyro_thr) and (abs(np.linalg.norm(a) - G) < acc_mag_thr)

def gravity_gate(a, w, gyro_thr=0.3, acc_mag_thr=0.8):
    """
    Gravity-direction aiding gate.

    Returns True when we trust the accelerometer **direction** to reflect gravity,
    i.e., when linear accelerations are small (‖a‖≈g) and angular rate is modest.
    This gate is intentionally looser than ZUPT because here we only use the
    *direction* (unit vector) as an attitude observation, not a hard zero-velocity.
    """
    # Looser gate than ZUPT for attitude aiding
    return (np.linalg.norm(w) < gyro_thr) and (abs(np.linalg.norm(a) - G) < acc_mag_thr)

# ---------- ESKF ----------
def eskf_inertial_odometry(
    t: np.ndarray,
    a_meas: np.ndarray,
    w_meas: np.ndarray,
    dt_max: float = 0.05,
    # Process noise (tune for your IMU)
    sigma_g: float = 0.01,     # gyro white noise       [rad/s/√Hz]
    sigma_a: float = 0.08,     # accel white noise      [m/s^2/√Hz]
    sigma_bg: float = 5e-5,    # gyro bias RW           [rad/s/√Hz]
    sigma_ba: float = 5e-4,    # accel bias RW          [m/s^2/√Hz]
    # Measurement noise
    sigma_grav: float = 0.03,  # gravity direction noise (≈rad equiv; unit-vector residual)
    sigma_zupt: float = 0.03,  # ZUPT velocity noise    [m/s]
    use_gravity_when_dynamic: bool = False
) -> dict:
    """
    Error-State Kalman Filter (ESKF) for strapdown inertial odometry with:
      - Nominal propagation: position p, velocity v, attitude q, gyro bias b_g, accel bias b_a
      - Gravity-direction updates (accelerometer unit vector) → attitude correction
      - Zero-velocity updates (ZUPT) when stationary → velocity correction (and cross-coupled states)
      - Biases estimated online

    Frames & measurement model
    --------------------------
    - Body frame (b): IMU measurements are in body.
    - World frame (w): z axis points "up"; gravity vector is g_w = [0,0,G]^T.
    - Accelerometer reports **specific force** f_b ≈ a_b - R_bw^T g_w.
    - We propagate a_w = R_bw f_b + g_w  (undo gravity to get linear acceleration in world).
    - Gyro reports angular rate ω_b; we integrate dq/dt = 0.5 Ω(ω_b - b_g) q (here via rotvec).

    Error-state vector (15x1)
    -------------------------
      δx = [δp, δv, δθ, δb_g, δb_a]
    with covariance P (15x15). Attitude error is represented by a small rotation δθ in the
    right-multiplicative sense: q ← q ⊗ Exp(δθ).

    Noise parameters
    ----------------
    The {sigma_*} values are 1/√Hz spectral densities; we discretize as Qd ≈ G Qc Gᵀ dt.

    Returns
    -------
    dict with:
      - "time": (N,)
      - "position_m": (N,3)
      - "velocity_mps": (N,3)
      - "attitude_quat_wxyz": (N,4)  quaternion (w,x,y,z)
      - "rpy_deg": (N,3)   roll, pitch, yaw [deg]
      - "rpy_rate_dps": (N,3) finite-difference rates [deg/s]
    """
    n = len(t)
    dt = finite_dt(t, dt_max=dt_max)

    # --- Initialization from an initial static window ---
    k0 = detect_initial_static(a_meas, w_meas, t)
    a0 = a_meas[:max(k0, 1)].mean(axis=0)  # avg accel during initial stillness
    w0 = w_meas[:max(k0, 1)].mean(axis=0)  # avg gyro during initial stillness

    # Initial attitude from gravity: align body z to -a_hat (gravity "down" in body)
    a_hat = a0 / (np.linalg.norm(a0) + 1e-12)
    g_b_est = -a_hat  # unit vector pointing along gravity in body frame
    z_b = np.array([0.0, 0.0, 1.0])
    v = np.cross(z_b, g_b_est); s = np.linalg.norm(v); c = z_b @ g_b_est
    if s < 1e-8:
        # Already aligned or anti-aligned
        Rbw = (np.eye(3) if c > 0 else R.from_rotvec(np.pi * np.array([1, 0, 0])).as_matrix())
    else:
        vx = skew(v)
        Rbw = (np.eye(3) + vx + vx @ vx * ((1 - c) / (s**2)))
    q = R.from_matrix(Rbw)  # body->world rotation

    # Bias init:
    bg = w0.copy()       # gyro bias from initial still average
    ba = np.zeros(3)     # accel bias: start at 0; corrected by filter

    # --- Storage for outputs ---
    pos = np.zeros((n, 3))
    vel = np.zeros((n, 3))
    quat_wxyz = np.zeros((n, 4))
    zupt_flags = np.zeros(n, dtype=bool)
    grav_flags = np.zeros(n, dtype=bool)

    # --- Error-state covariance (15x15) ---
    # Diagonal is a reasonable starting point; tune if you know your uncertainties.
    P = np.diag([
        1e-4, 1e-4, 1e-4,   # δp
        1e-2, 1e-2, 1e-2,   # δv
        1e-2, 1e-2, 1e-2,   # δθ
        1e-4, 1e-4, 1e-4,   # δb_g
        1e-3, 1e-3, 1e-3    # δb_a
    ])
    I15 = np.eye(15)

    # --- Process noise discretization ---
    def process_Q(Rbw: np.ndarray, dt_i: float, f_b: np.ndarray, w_b: np.ndarray) -> np.ndarray:
        """
        Build discrete process noise Qd for δx over dt_i.

        We model white noise for gyro (n_g) and accel (n_a), and random-walks for
        biases (n_bg, n_ba). The mapping (Gmat) injects these into the error dynamics:

          δv̇ ≈ -R_bw n_a,    δθ̇ ≈ -n_g,    δḃ_g = n_bg,    δḃ_a = n_ba

        which yields the blocks set below. See standard ESKF derivations.
        """
        Gmat = np.zeros((15, 12))
        # δv gets accel white noise via world rotation
        Gmat[3:6, 3:6] = -Rbw
        # δθ gets gyro white noise
        Gmat[6:9, 0:3] = -np.eye(3)
        # bias random walks
        Gmat[9:12, 6:9] = np.eye(3)     # gyro bias RW
        Gmat[12:15, 9:12] = np.eye(3)   # accel bias RW

        Qc = np.zeros((12, 12))
        Qc[0:3, 0:3]     = (sigma_g**2)  * np.eye(3)   # gyro white
        Qc[3:6, 3:6]     = (sigma_a**2)  * np.eye(3)   # accel white
        Qc[6:9, 6:9]     = (sigma_bg**2) * np.eye(3)   # gyro bias RW
        Qc[9:12, 9:12]   = (sigma_ba**2) * np.eye(3)   # accel bias RW

        return Gmat @ Qc @ Gmat.T * dt_i

    def state_propagate(
        p: np.ndarray, v: np.ndarray, q: R,
        bg: np.ndarray, ba: np.ndarray,
        w_m: np.ndarray, a_m: np.ndarray,
        dt_i: float
    ):
        """
        Nominal strapdown propagation (discrete Euler):
          - Remove biases from measurements
          - Integrate attitude with gyro
          - Rotate specific force to world and add gravity
          - Integrate velocity and position

        Returns updated (p,v,q) and also the *de-biased* signals (w_b, f_b) used
        for linearization/noise mapping.
        """
        # De-biased measurements
        w_b = w_m - bg                # body angular rate
        f_b = a_m - ba                # specific force in body

        # Attitude integration (small-angle exponential map via rotvec)
        dq = R.from_rotvec(w_b * dt_i)
        q_new = q * dq
        Rbw = q_new.as_matrix()

        # Specific force -> world linear acceleration
        a_w = Rbw @ f_b + G * e3

        # Integrate velocity and position
        v_new = v + a_w * dt_i
        p_new = p + v * dt_i + 0.5 * a_w * (dt_i**2)
        return p_new, v_new, q_new, w_b, f_b

    def linearize_F(Rbw: np.ndarray, f_b: np.ndarray, w_b: np.ndarray, dt_i: float) -> np.ndarray:
        """
        Build the discrete-time state transition Φ ≈ I + F dt for the 15D error state.

        Key blocks (standard ESKF):
          δṗ = δv
          δv̇ = -R_bw [f_b]_x δθ - R_bw δb_a
          δθ̇ = -[w_b]_x δθ - δb_g
        Bias error derivatives are zero-mean RWs (handled in Qd).
        """
        F = np.zeros((15, 15))

        # δṗ = δv
        F[0:3, 3:6] = np.eye(3)

        # δv̇ = -R_bw [f_b]_x δθ - R_bw δb_a
        F[3:6, 6:9]   = -Rbw @ skew(f_b)
        F[3:6, 12:15] = -Rbw

        # δθ̇ = -[w_b]_x δθ - δb_g
        F[6:9, 6:9] = -skew(w_b)
        F[6:9, 9:12] = -np.eye(3)

        # First-order discretization
        Phi = I15 + F * dt_i
        return Phi

    # Measurement models / Jacobians
    def H_gravity(q: R):
        """
        Gravity-direction observation model:

          z = â_hat   (accelerometer direction unit vector)
          h(x) = ĝ_b_hat = normalize(R_bw^T g_w)

        The residual y = z - h is interpreted on the 3D unit-vector manifold
        (small-angle approx). Jacobian w.r.t. δθ is -[ĝ_b_hat]_x, and zero
        for other error-state components.
        """
        Rbw = q.as_matrix()
        g_b = Rbw.T @ (G * e3)
        g_b_hat = g_b / (np.linalg.norm(g_b) + 1e-12)

        H = np.zeros((3, 15))
        H[0:3, 6:9] = -skew(g_b_hat)  # only attitude error affects direction
        return H, g_b_hat

    # ZUPT velocity measurement: h(x) = v
    H_zupt = np.zeros((3, 15))
    H_zupt[0:3, 3:6] = np.eye(3)

    # Measurement covariances
    R_grav = (sigma_grav**2) * np.eye(3)
    R_zupt = (sigma_zupt**2) * np.eye(3)

    # --- Main filter loop ---
    p = np.zeros(3)
    v = np.zeros(3)

    for i in range(n):
        dt_i = dt[i]

        # 1) Propagate nominal state
        p, v, q, w_b, f_b = state_propagate(p, v, q, bg, ba, w_meas[i], a_meas[i], dt_i)
        Rbw = q.as_matrix()

        # 2) Propagate covariance
        Phi = linearize_F(Rbw, f_b, w_b, dt_i)
        Qd  = process_Q(Rbw, dt_i, f_b, w_b)
        P   = Phi @ P @ Phi.T + Qd

        # 3) Gravity-direction aiding (optionally gated)
        do_grav = gravity_gate(a_meas[i], w_meas[i]) or use_gravity_when_dynamic
        if do_grav:
            H, g_b_hat = H_gravity(q)

            # Measurement: accelerometer *direction* (unit vector)
            z = a_meas[i] / (np.linalg.norm(a_meas[i]) + 1e-12)
            h = g_b_hat

            # Innovation, Kalman gain, state correction
            y = z - h
            S = H @ P @ H.T + R_grav
            K = P @ H.T @ np.linalg.inv(S)
            dx = K @ y
            P = (np.eye(15) - K @ H) @ P

            # Inject error-state into nominal
            dp      = dx[0:3]
            dv      = dx[3:6]
            dtheta  = dx[6:9]
            dbg     = dx[9:12]
            dba     = dx[12:15]

            p += dp
            v += dv
            q = q * R.from_rotvec(dtheta)            # right-multiplicative attitude update
            q = R.from_quat(q.as_quat() / np.linalg.norm(q.as_quat()))  # renorm
            bg += dbg
            ba += dba

        # 4) Zero-velocity update (when confidently stationary)
        do_zupt = zupt_gate(a_meas[i], w_meas[i])
        if do_zupt:

            z = np.zeros(3)  # measured velocity = 0
            h = v # current velocity
            y = z - h # innovation

            S = H_zupt @ P @ H_zupt.T + R_zupt
            K = P @ H_zupt.T @ np.linalg.inv(S)
            dx = K @ y
            P  = (np.eye(15) - K @ H_zupt) @ P

            dp      = dx[0:3]
            dv      = dx[3:6]
            dtheta  = dx[6:9]
            dbg     = dx[9:12]
            dba     = dx[12:15]

            p += dp
            v += dv
            q = q * R.from_rotvec(dtheta)
            q = R.from_quat(q.as_quat() / np.linalg.norm(q.as_quat()))
            bg += dbg
            ba += dba

        # 5) Save trajectory sample
        pos[i] = p
        vel[i] = v
        q_xyzw = q.as_quat()
        quat_wxyz[i] = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])

        zupt_flags[i] = do_zupt
        grav_flags[i] = do_grav

    # --- Roll/Pitch/Yaw and their rates for plotting ---
    r_obj   = R.from_quat(quat_wxyz[:, [1, 2, 3, 0]])  # convert (wxyz) -> (xyzw)
    rpy_deg = r_obj.as_euler("xyz", degrees=True)      # roll, pitch, yaw in degrees
    rpy_rate_dps = np.vstack([
        np.gradient(rpy_deg[:, 0], t),
        np.gradient(rpy_deg[:, 1], t),
        np.gradient(rpy_deg[:, 2], t)
    ]).T

    return {
        "time": t,
        "position_m": pos,
        "velocity_mps": vel,
        "attitude_quat_wxyz": quat_wxyz,
        "rpy_deg": rpy_deg,
        "rpy_rate_dps": rpy_rate_dps,
        "zupt_flags": zupt_flags,
        "grav_flags": grav_flags,
    }

# ---------- CLI ----------
def main():
    ap = argparse.ArgumentParser(description="ESKF inertial odometry with gravity-direction aiding + ZUPT")
    ap.add_argument("pkl", type=str, help="Path to IMU data pickle")
    ap.add_argument("--dt-max", type=float, default=0.05, help="Clamp unusually large dt (s)")
    ap.add_argument("--use-gravity-when-dynamic", action="store_true",
                    help="Apply gravity aiding even during dynamics (downweighted by R)")
    ap.add_argument("--save", type=str, default="", help="Optional path to save results (pickle)")
    ap.add_argument("--plot", action="store_true", help="Show 2x2 plots")
    args = ap.parse_args()

    t, a, w = load_imu(args.pkl)

    out = eskf_inertial_odometry(
        t, a, w,
        dt_max=args.dt_max,
        use_gravity_when_dynamic=args.use_gravity_when_dynamic
    )

    if args.save:
        with open(args.save, "wb") as f:
            pickle.dump(out, f)
        print(f"Saved odometry to {args.save}")

    if args.plot:
        try:
            import matplotlib.pyplot as plt
            pos = out["position_m"]; vel = out["velocity_mps"]
            rpy = out["rpy_deg"]; rdot = out["rpy_rate_dps"]

            fig, axs = plt.subplots(2, 2, figsize=(12, 8))

            axs[0, 0].plot(t, pos[:, 0], label="x")
            axs[0, 0].plot(t, pos[:, 1], label="y")
            axs[0, 0].plot(t, pos[:, 2], label="z")
            axs[0, 0].set_title("Position (m)")
            axs[0, 0].legend(); axs[0, 0].grid(True)

            axs[1, 0].plot(t, vel[:, 0], label="vx")
            axs[1, 0].plot(t, vel[:, 1], label="vy")
            axs[1, 0].plot(t, vel[:, 2], label="vz")
            axs[1, 0].set_title("Velocity (m/s)")
            axs[1, 0].legend(); axs[1, 0].grid(True)

            axs[0, 1].plot(t, rpy[:, 0], label="Roll")
            axs[0, 1].plot(t, rpy[:, 1], label="Pitch")
            axs[0, 1].plot(t, rpy[:, 2], label="Yaw")
            axs[0, 1].set_title("Orientation (deg)")
            axs[0, 1].legend(); axs[0, 1].grid(True)

            axs[1, 1].plot(t, rdot[:, 0], label="Roll rate")
            axs[1, 1].plot(t, rdot[:, 1], label="Pitch rate")
            axs[1, 1].plot(t, rdot[:, 2], label="Yaw rate")
            axs[1, 1].set_title("Orientation rates (deg/s)")
            axs[1, 1].legend(); axs[1, 1].grid(True)

            def get_flag_clusters(binary):
                diffs = np.diff(binary.astype(int))
                starts = np.where(diffs == 1)[0] + 1  # transition 0→1
                ends   = np.where(diffs == -1)[0] + 1 # transition 1→0

                # Handle edge cases: if starts/ends at 1
                if binary[0] == 1:
                    starts = np.r_[0, starts]
                if binary[-1] == 1:
                    ends = np.r_[ends, len(binary)]

                # Combine into list of (start, end) index ranges
                clusters = list(zip(starts, ends))

                return clusters

            #### Commented out code to try to visualize when ZUPT and gravity aiding were applied
            # on all of the axes, draw boxes around the times when ZUPT and gravity aiding were applied
            # both_flags = out["zupt_flags"] & out["grav_flags"]
            # both_boxes = get_flag_clusters(both_flags)
            # both_box_times = [(t[start], t[end-1]) for start, end in both_boxes if end > start]

            # # drop any grav flags that appear in both_flags
            # grav_flags = out["grav_flags"] & ~both_flags
            # grav_boxes = get_flag_clusters(grav_flags)
            # grav_box_times = [(t[start], t[end-1]) for start, end in grav_boxes if end > start]

            # for ax in [axs[0,0]]:
            #     for start, end in grav_box_times:
            #         ax.axvspan(start, end, color='orange', alpha=0.3, label='Gravity aiding')
            #     for start, end in both_box_times:
            #         ax.axvspan(start, end, color='purple', alpha=0.3, label='Both ZUPT + Gravity')
            #### End of commented out code

            fig.tight_layout()
            plt.show()
        except Exception as e:
            print(f"Plotting skipped: {e}")

if __name__ == "__main__":
    main()
