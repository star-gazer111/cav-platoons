#!/usr/bin/env python3
"""
Dynamic CRPF snapshots for Part 2 (SV vs moving OV, constant speed).

- Uses trajectory_part2.csv if present; otherwise runs a quick sim.
- For selected times, plots:
    * CRPF heatmap at that instant (OV at its current position)
    * CRPF equipotential contours
    * Lane boundaries
    * SV path up to that time + current SV dot
    * OV path up to that time + current OV dot + collision disc

Usage:
    python plot_dynamic_crpf_snapshots_part2.py
    python plot_dynamic_crpf_snapshots_part2.py --times 2.0 6.0 10.0
"""

from pathlib import Path
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from math import cos, sin

# ----------------- tunables (must match your Part 2) -----------------
OUT = Path("outputs_part2")
SNAP_OUT = OUT / "dynamic_crpf"
SNAP_OUT.mkdir(parents=True, exist_ok=True)

# Lane geometry (keep consistent with your sim)
LANE_CENTER_X = 0.0
LANE_HALF_WIDTH = 4.0
ROAD_LEFT  = LANE_CENTER_X + LANE_HALF_WIDTH
ROAD_RIGHT = LANE_CENTER_X - LANE_HALF_WIDTH

# Vehicle discs
R_S = 1.0
R_O = 1.0
R_SUM = R_S + R_O

# CRPF model params (should match Part 2)
CRPF_G     = 1.0
CRPF_ZETA  = 2.2
PSEUDO_EPS = 1.2
PSEUDO_RHO = 0.02

# VO horizon (only used in fallback sim)
DT = 0.1
T_END = 22.0

# Figure extent
GRID_X = (-12, 12)
GRID_Y = (-50,  22)
GRID_NX = 121
GRID_NY = 145
# --------------------------------------------------------------------

def pseudo_distance(xs, ys, xo, yo, v_obs, eps=PSEUDO_EPS, rho=PSEUDO_RHO):
    dx, dy = xs - xo, ys - yo
    termx = np.sign(dx) * (abs(dx)**eps) * np.exp(-rho * max(v_obs, 0.0))
    termy = np.sign(dy) * (abs(dy)**eps)
    return float(np.hypot(termx, termy))

def accel_factor(ax, ay, xs, ys, xo, yo, K=1.0):
    a  = np.array([ax, ay], dtype=float)
    rd = np.array([xs - xo, ys - yo], dtype=float)
    na, nr = np.linalg.norm(a), np.linalg.norm(rd)
    if na < 1e-9 or nr < 1e-9: return 1.0
    cos_t = float(np.dot(a/na, rd/nr))
    denom = K - na*cos_t
    denom = float(np.clip(denom, 0.2, 10.0))
    return K/denom

def virtual_mass(m, v):
    return float(m * (1.566e-14 * (max(v,0.0) ** 6.687) + 0.3354))

def type_factor(size, kappa, size_star=4.0, kappa_star=1.0, w1=1.0, w2=1.0):
    return float((size / max(size_star,1e-6))*(kappa / max(kappa_star,1e-6))*w1*w2)

def CRPF(xs, ys, xo, yo, v_obs, ax_obs, ay_obs, m_obs, size_obs, kappa_obs,
         G=CRPF_G, zeta=CRPF_ZETA):
    rd = pseudo_distance(xs, ys, xo, yo, v_obs)
    if rd < 1e-6: rd = 1e-6
    rd_vec = np.array([xs - xo, ys - yo], dtype=float)
    nrd = np.linalg.norm(rd_vec)
    direction = rd_vec/nrd if nrd > 1e-9 else np.array([0.0,1.0])
    M    = virtual_mass(m_obs, v_obs)
    Tfac = type_factor(size_obs, kappa_obs)
    phi  = accel_factor(ax_obs, ay_obs, xs, ys, xo, yo)
    mag  = float(G * M * Tfac * 1.0 * phi / (rd ** zeta))
    return mag, mag*direction

# Fallback sim (minimal; close to your Part 2)
def quick_sim_if_needed():
    traj_path = OUT / "trajectory_part2.csv"
    if traj_path.exists():
        return pd.read_csv(traj_path)

    # If missing, run a simple forward sim that mirrors Part 2 defaults
    print("[info] trajectory_part2.csv not found — generating a quick baseline run...")
    # SV init
    sv_x, sv_y = 0.0, -40.0
    sv_v       = 12.0
    sv_theta   = np.deg2rad(90.0)
    # OV motion (constant)
    ov_x0, ov_y0 = 0.0, 0.0
    ov_speed     = 7.0
    ov_heading   = np.deg2rad(90.0)
    ov_vx, ov_vy = ov_speed*cos(ov_heading), ov_speed*sin(ov_heading)

    # Minimal “go-straight” log (no planning) just to have OV/SV for plotting
    N = int(T_END/DT)
    log = {"t":[], "sv_x":[], "sv_y":[], "sv_v":[], "sv_theta":[], "ov_x":[], "ov_y":[]}
    for k in range(N):
        t = k*DT
        ov_x = ov_x0 + ov_vx*t
        ov_y = ov_y0 + ov_vy*t
        sv_x += sv_v*cos(sv_theta)*DT
        sv_y += sv_v*sin(sv_theta)*DT
        log["t"].append(t); log["sv_x"].append(sv_x); log["sv_y"].append(sv_y)
        log["sv_v"].append(sv_v); log["sv_theta"].append(sv_theta)
        log["ov_x"].append(ov_x); log["ov_y"].append(ov_y)
    df = pd.DataFrame(log)
    df.to_csv(traj_path, index=False)
    return df

def pick_informative_times(df, k_max=6):
    """
    Pick moments when the SV turns most (large |Δθ|/Δt) + the closest approach.
    Returns a sorted list of snapshot times (seconds).
    """
    t = df["t"].to_numpy()
    th = df["sv_theta"].to_numpy()

    if t.size < 3:  # not enough samples to compute rates well
        return [float(t[-1])] if t.size else []

    # unwrap once to avoid double unwrap and shape mistakes
    th_u = np.unwrap(th)

    # finite differences (length N-1)
    dth = np.abs(th_u[1:] - th_u[:-1])
    dt  = np.diff(t)

    # avoid divide-by-zero
    dt[dt == 0] = 1e-9
    turn_rate = dth / dt  # length N-1

    # index of closest approach
    dist = np.hypot(df["sv_x"] - df["ov_x"], df["sv_y"] - df["ov_y"])
    i_min = int(np.nanargmin(dist.values))

    # top-k indices by turning rate; +1 to map from diff index to sample index
    idx_turn_sorted = np.argsort(turn_rate)[::-1]
    picked = []
    for idx in idx_turn_sorted[:max(k_max, 1)]:
        j = int(idx + 1)  # event occurs at the *later* sample
        if 0 <= j < len(t):
            picked.append(j)

    # include closest-approach sample
    if i_min not in picked:
        picked.append(i_min)

    # make times unique & sorted, and keep within [first,last]
    picked = sorted(set(picked))
    return [float(t[j]) for j in picked if 0 <= j < len(t)]

def plot_snapshot(df, t_snap,
                  ov_mass=1600.0, ov_size=4.6, ov_kappa=1.0,
                  ov_ax=0.0, ov_ay=0.0):
    """Plot one snapshot at time t_snap."""
    # find nearest index
    i = int(np.argmin(np.abs(df["t"].values - t_snap)))
    t_i = float(df["t"].iloc[i])

    # positions (up to i)
    sv_x_hist = df["sv_x"].iloc[:i+1].values
    sv_y_hist = df["sv_y"].iloc[:i+1].values
    ov_x_hist = df["ov_x"].iloc[:i+1].values
    ov_y_hist = df["ov_y"].iloc[:i+1].values

    # current positions
    sv_x, sv_y = sv_x_hist[-1], sv_y_hist[-1]
    ov_x, ov_y = ov_x_hist[-1], ov_y_hist[-1]

    # estimate OV speed at t_i from path (in case we loaded from CSV)
    if i >= 2:
        dt = float(df["t"].iloc[i] - df["t"].iloc[i-1]) or 1e-6
        ov_vx = (df["ov_x"].iloc[i] - df["ov_x"].iloc[i-1]) / dt
        ov_vy = (df["ov_y"].iloc[i] - df["ov_y"].iloc[i-1]) / dt
        ov_speed = float(np.hypot(ov_vx, ov_vy))
    else:
        ov_speed = 0.0

    # compute CRPF grid for OV at its current position & speed
    gx = np.linspace(GRID_X[0], GRID_X[1], GRID_NX)
    gy = np.linspace(GRID_Y[0], GRID_Y[1], GRID_NY)
    CR = np.zeros((len(gy), len(gx)))
    for j, yy in enumerate(gy):
        for kx, xx in enumerate(gx):
            cr, _ = CRPF(xx, yy, ov_x, ov_y, ov_speed, ov_ax, ov_ay,
                         ov_mass, ov_size, ov_kappa)
            CR[j, kx] = cr

    # ---- plot ----
    plt.figure(figsize=(7.4, 6.2))
    plt.title(f"Dynamic CRPF at t={t_i:.1f}s (OV shown) + SV Trajectory to now")
    extent = [gx.min(), gx.max(), gy.min(), gy.max()]
    im = plt.imshow(np.log10(CR + 1e-9), origin="lower", extent=extent, aspect="auto")
    plt.colorbar(im, label="log10(CRPF + 1e-9)")

    # contours
    levels = np.linspace(np.nanmin(np.log10(CR+1e-9)), np.nanmax(np.log10(CR+1e-9)), 10)
    plt.contour(gx, gy, np.log10(CR + 1e-9), levels=levels, colors="w", linewidths=0.6, alpha=0.7)

    # lane lines
    plt.axvline(ROAD_LEFT,  color="k", linestyle="--", linewidth=1)
    plt.axvline(ROAD_RIGHT, color="k", linestyle="--", linewidth=1)

    # OV path (so far) + current OV
    plt.plot(ov_x_hist, ov_y_hist, '--', color='orange', linewidth=1.7, label="OV path")
    plt.scatter([ov_x], [ov_y], c='orange', s=40, label="OV (current)")
    circ = plt.Circle((ov_x, ov_y), R_SUM, color='gray', fill=False, linestyle='--', linewidth=2)
    plt.gca().add_patch(circ)

    # SV path (so far) + current SV
    plt.plot(sv_x_hist, sv_y_hist, color='C0', linewidth=2.0, label="SV path")
    plt.scatter([sv_x], [sv_y], c='C0', s=40, label="SV (current)")

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.grid(True)
    plt.legend(loc="best")
    plt.tight_layout()

    fname = SNAP_OUT / f"step_{int(10*t_i):04d}.png"
    plt.savefig(fname, dpi=170)
    plt.close()
    print(f"[saved] {fname}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--times", type=float, nargs="*", default=None,
                        help="Snapshot times in seconds (e.g., --times 2 6 10)")
    args = parser.parse_args()

    traj_path = OUT / "trajectory_part2.csv"
    if traj_path.exists():
        df = pd.read_csv(traj_path)
    else:
        df = quick_sim_if_needed()

    # if no custom times, pick informative ones automatically
    if not args.times:
        times = pick_informative_times(df, k_max=6)
        print("[info] auto-picked snapshot times:", ", ".join(f"{x:.1f}s" for x in times))
    else:
        times = args.times

    for ts in times:
        plot_snapshot(df, ts)

if __name__ == "__main__":
    main()
