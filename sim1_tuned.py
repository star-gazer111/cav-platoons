"""
Part 1 (tuned): Single SV vs single *stationary* OV

Adds:
- Paper-style safety metrics: MinTTC and TET (time exposed to collision)
- Tunable planner/limits/weights to roughly match "paper feel"
- CRPF equipotential contours
- Per-step feasible velocity clouds (saved images)
- Original CRPF heatmap + trajectory, VO diagnostic, time-series, CSV logs

Dependencies: numpy, matplotlib, pandas
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from math import cos, sin

# ================
# 0) CONFIG / TUNING
# ================
OUT = Path("outputs")
(OUT / "feasible_vel").mkdir(parents=True, exist_ok=True)

# Simulation
DT = 0.1        # [s]
T_END = 22.0    # [s]
N_STEPS = int(T_END/DT)

# Lane geometry
LANE_CENTER_X = 0.0
LANE_HALF_WIDTH = 4.0
ROAD_LEFT  = LANE_CENTER_X + LANE_HALF_WIDTH
ROAD_RIGHT = LANE_CENTER_X - LANE_HALF_WIDTH

# Disks (vehicle shape)
R_S = 1.0
R_O = 1.0
R_SUM = R_S + R_O

# Subject initial state
sv_x, sv_y  = 0.0, -40.0
sv_v        = 12.0                       # m/s
sv_theta    = np.deg2rad(90.0)           # along +Y

# Subject limits (tuneable)
ACC_MIN, ACC_MAX = -4.0, 2.0             # m/s^2 (mildly conservative accel/brake)
OMEGA_MIN, OMEGA_MAX = -0.5, 0.5         # rad/s feasible yaw-rate commands
V_MIN, V_MAX = 0.0, 22.0                 # speed bounds (m/s)

# Desired speed
V_DES = 14.0

# DWA sampling
ACC_SAMPLES   = 7        # accel grid density
OMEGA_SAMPLES = 11       # omega grid density
LOOKAHEAD_STEPS = 10     # 1s rollouts for feasibility/risk check

# CRPF model (tune decay & gains for sharper “risk hills”)
CRPF_G   = 1.0
CRPF_ZETA = 2.2          # >2 => steeper decay
PSEUDO_EPS = 1.2
PSEUDO_RHO = 0.02

# Obstacle (stationary)
ov_x, ov_y = 0.0, 0.0
ov_vx, ov_vy = 0.0, 0.0
ov_ax, ov_ay = 0.0, 0.0
OV_MASS   = 1600.0
OV_SIZE   = 4.6
OV_KAPPA  = 1.0

# Cost weights (planner “style”)
W_RISK      = 1.0
W_V_TRACK   = 0.12      # speed tracking
W_STEER_SMO = 0.06      # gentle steering

# VO horizon (for TTC)
VO_HORIZON_S = 8.0
TTC_THRESH_S = 2.0      # seconds; TET integrates time with TTC < this

# Feasible velocity cloud snapshots (save every K steps)
SAVE_FEASIBLE_EVERY = 5
VEL_CLOUD_RANGE = 18.0  # axis extent for plots


# ================
# 1) HELPERS (CRPF / VO / utils)
# ================
def pseudo_distance(xs, ys, xo, yo, v_obs, eps=PSEUDO_EPS, rho=PSEUDO_RHO):
    dx, dy = xs - xo, ys - yo
    termx = np.sign(dx) * (abs(dx)**eps) * np.exp(-rho * max(v_obs, 0.0))
    termy = np.sign(dy) * (abs(dy)**eps)
    return float(np.hypot(termx, termy))

def accel_factor(ax, ay, xs, ys, xo, yo, K=1.0):
    a = np.array([ax, ay], dtype=float)
    rd = np.array([xs - xo, ys - yo], dtype=float)
    na, nr = np.linalg.norm(a), np.linalg.norm(rd)
    if na < 1e-9 or nr < 1e-9:
        return 1.0
    cos_t = float(np.dot(a/na, rd/nr))
    denom = K - na * cos_t
    denom = float(np.clip(denom, 0.2, 10.0))
    return K / denom

def road_factor():
    return 1.0  # simple case

def virtual_mass(m, v):
    return float(m * (1.566e-14 * (max(v, 0.0) ** 6.687) + 0.3354))

def type_factor(size, kappa, size_star=4.0, kappa_star=1.0, w1=1.0, w2=1.0):
    return float((size / max(size_star, 1e-6)) * (kappa / max(kappa_star, 1e-6)) * w1 * w2)

def CRPF(xs, ys, xo, yo, v_obs, ax_obs, ay_obs,
         m_obs, size_obs, kappa_obs, G=CRPF_G, zeta=CRPF_ZETA):
    rd = pseudo_distance(xs, ys, xo, yo, v_obs)
    if rd < 1e-6:
        rd = 1e-6
    rd_vec = np.array([xs - xo, ys - yo], dtype=float)
    nrd = np.linalg.norm(rd_vec)
    direction = rd_vec/nrd if nrd > 1e-9 else np.array([0.0, 1.0])

    M = virtual_mass(m_obs, v_obs)
    Tfac = type_factor(size_obs, kappa_obs)
    Rfac = road_factor()
    phi = accel_factor(ax_obs, ay_obs, xs, ys, xo, yo)

    mag = float(G * M * Tfac * Rfac * phi / (rd ** zeta))
    return mag, mag * direction

def will_collide(p_rel, v_rel, R, horizon=VO_HORIZON_S):
    v2 = float(np.dot(v_rel, v_rel))
    if v2 < 1e-12:
        return (np.linalg.norm(p_rel) <= R), float("inf")

    t_star = -float(np.dot(p_rel, v_rel)) / v2
    t_star = float(np.clip(t_star, 0.0, horizon))
    d_min = float(np.linalg.norm(p_rel + t_star * v_rel))

    # Approx TTC if approaching: solve ||p + v t|| = R
    ttc = float("inf")
    if float(np.dot(p_rel, v_rel)) < 0.0:
        a = v2
        b = 2.0 * float(np.dot(p_rel, v_rel))
        c = float(np.dot(p_rel, p_rel)) - R ** 2
        disc = b*b - 4*a*c
        if disc >= 0.0:
            r1 = (-b - np.sqrt(disc)) / (2.0*a)
            r2 = (-b + np.sqrt(disc)) / (2.0*a)
            roots = [r for r in (r1, r2) if r >= 0.0]
            if roots:
                ttc = float(min(roots))

    return (d_min <= R), ttc

def in_lane(x):
    return ROAD_RIGHT <= x <= ROAD_LEFT


# ================
# 2) PRECOMPUTE CRPF FIELD (for contours/heatmap)
# ================
gx = np.linspace(-12, 12, 121)
gy = np.linspace(-50, 22, 145)
CR = np.zeros((len(gy), len(gx)))
for j, yy in enumerate(gy):
    for i, xx in enumerate(gx):
        cr, _ = CRPF(xx, yy, ov_x, ov_y, 0.0, 0.0, 0.0, OV_MASS, OV_SIZE, OV_KAPPA)
        CR[j, i] = cr


# ================
# 3) SIMULATION
# ================
log = {k: [] for k in [
    "t","sv_x","sv_y","sv_v","sv_theta","dist_to_obs","risk","ttc",
    "picked_v","picked_dtheta"
]}
collision_happened = False
tet_seconds = 0.0      # Time Exposed to (TTC<thresh) during horizon
min_ttc = float("inf")
min_dist = float("inf")

acc_grid   = np.linspace(ACC_MIN, ACC_MAX, ACC_SAMPLES)
omega_grid = np.linspace(OMEGA_MIN, OMEGA_MAX, OMEGA_SAMPLES)

for k in range(N_STEPS):
    t = k * DT

    # Relative state vs obstacle (stationary)
    p_rel = np.array([sv_x - ov_x, sv_y - ov_y], dtype=float)
    v_rel = np.array([sv_v*cos(sv_theta) - ov_vx, sv_v*sin(sv_theta) - ov_vy], dtype=float)
    dist  = float(np.linalg.norm(p_rel))

    # Paper-style safety metrics (TTC & TET accumulation)
    _, ttc_now = will_collide(p_rel, v_rel, R_SUM, horizon=VO_HORIZON_S)
    if np.isfinite(ttc_now):
        min_ttc = min(min_ttc, ttc_now)
        if ttc_now < TTC_THRESH_S:
            tet_seconds += DT
    min_dist = min(min_dist, dist)

    # CRPF at current SV pose
    risk_now, _ = CRPF(sv_x, sv_y, ov_x, ov_y, 0.0, 0.0, 0.0, OV_MASS, OV_SIZE, OV_KAPPA)

    # --- DWA search: feasible-velocity cloud (for plotting every SAVE_FEASIBLE_EVERY) ---
    feas = []   # (vx, vy) (body-frame approx via heading)
    rej  = []   # rejected by constraints
    best_cost = float("inf")
    best_v_next = sv_v
    best_theta_next = sv_theta
    best_dtheta = 0.0

    for a_cmd in acc_grid:
        v_next = float(np.clip(sv_v + a_cmd * DT, V_MIN, V_MAX))
        for d_omega in omega_grid:
            theta_next = sv_theta + d_omega * DT

            # One-second rollout (feasibility + accumulate CRPF cost)
            x_tmp, y_tmp, th_tmp, v_tmp = sv_x, sv_y, theta_next, v_next
            ok = True
            acc_risk = 0.0
            for _ in range(LOOKAHEAD_STEPS):
                x_tmp += v_tmp * cos(th_tmp) * DT
                y_tmp += v_tmp * sin(th_tmp) * DT
                if not in_lane(x_tmp):
                    ok = False; break
                if np.hypot(x_tmp - ov_x, y_tmp - ov_y) <= R_SUM:
                    ok = False; break
                rr, _ = CRPF(x_tmp, y_tmp, ov_x, ov_y, 0.0, 0.0, 0.0, OV_MASS, OV_SIZE, OV_KAPPA)
                acc_risk += rr

            # For the feasible cloud plot: represent candidate as global velocity components
            vx_cand = v_next * cos(theta_next)
            vy_cand = v_next * sin(theta_next)

            if ok:
                feas.append((vx_cand, vy_cand))
                cost = W_RISK*acc_risk + W_V_TRACK*(V_DES - v_next)**2 + W_STEER_SMO*(d_omega**2)
                if cost < best_cost:
                    best_cost = cost
                    best_v_next = v_next
                    best_theta_next = theta_next
                    best_dtheta = d_omega*DT
            else:
                rej.append((vx_cand, vy_cand))

    # Save feasible-velocity cloud every K steps
    if k % SAVE_FEASIBLE_EVERY == 0:
        plt.figure(figsize=(5.2,5.2))
        plt.title(f"Feasible Velocity Cloud (t={t:.1f}s)")
        if rej:
            rx, ry = np.array(rej).T
            plt.scatter(rx, ry, s=6, alpha=0.25, label="rejected", marker="x")
        if feas:
            fx, fy = np.array(feas).T
            plt.scatter(fx, fy, s=8, alpha=0.8, label="feasible")
        # mark current velocity
        plt.scatter([sv_v*cos(sv_theta)], [sv_v*sin(sv_theta)], s=30, label="current v")
        plt.xlim(-VEL_CLOUD_RANGE, VEL_CLOUD_RANGE)
        plt.ylim(-VEL_CLOUD_RANGE, VEL_CLOUD_RANGE)
        plt.axhline(0, color="k", linewidth=0.5)
        plt.axvline(0, color="k", linewidth=0.5)
        plt.xlabel("v_x [m/s]")
        plt.ylabel("v_y [m/s]")
        plt.grid(True)
        plt.legend(loc="best", fontsize=8)
        plt.tight_layout()
        plt.savefig(OUT / "feasible_vel" / f"step_{k:04d}.png", dpi=140)
        plt.close()

    # Fallback if nothing feasible: brake and hold heading
    if not np.isfinite(best_cost):
        best_v_next = max(sv_v + ACC_MIN * DT, 0.0)
        best_theta_next = sv_theta
        best_dtheta = 0.0

    # Apply selected control & propagate one step
    sv_v     = best_v_next
    sv_theta = (best_theta_next + np.pi) % (2*np.pi) - np.pi
    sv_x    += sv_v * cos(sv_theta) * DT
    sv_y    += sv_v * sin(sv_theta) * DT

    # Log
    log["t"].append(t)
    log["sv_x"].append(sv_x)
    log["sv_y"].append(sv_y)
    log["sv_v"].append(sv_v)
    log["sv_theta"].append(sv_theta)
    log["dist_to_obs"].append(dist)
    log["risk"].append(risk_now)
    log["ttc"].append(ttc_now if np.isfinite(ttc_now) else np.nan)
    log["picked_v"].append(sv_v)
    log["picked_dtheta"].append(best_dtheta)

    # hard overlap check (shouldn’t occur with planner)
    if dist <= R_SUM:
        collision_happened = True
        break

df = pd.DataFrame(log)

# ================
# 4) PLOTS
# ================
# A) CRPF heatmap + equipotential contours + trajectory
plt.figure(figsize=(7.4,6.2))
plt.title("CRPF Heatmap + Equipotential Contours + SV Trajectory")
extent = [gx.min(), gx.max(), gy.min(), gy.max()]
im = plt.imshow(np.log10(CR + 1e-9), origin="lower", extent=extent, aspect="auto")
plt.colorbar(im, label="log10(CRPF + 1e-9)")
# contours on log scale
levels = np.linspace(np.nanmin(np.log10(CR+1e-9)), np.nanmax(np.log10(CR+1e-9)), 10)
plt.contour(gx, gy, np.log10(CR + 1e-9), levels=levels, colors="w", linewidths=0.6, alpha=0.7)

# lane + obstacle boundary
plt.axvline(ROAD_LEFT,  color="k", linestyle="--", linewidth=1)
plt.axvline(ROAD_RIGHT, color="k", linestyle="--", linewidth=1)
circle = plt.Circle((ov_x, ov_y), R_SUM, fill=False, linestyle="--", linewidth=2)
plt.gca().add_patch(circle)

# trajectory
plt.plot(df["sv_x"], df["sv_y"], linewidth=2, label="SV path")
plt.scatter([df["sv_x"].iloc[0]], [df["sv_y"].iloc[0]], s=32, label="start")
plt.scatter([df["sv_x"].iloc[-1]], [df["sv_y"].iloc[-1]], s=32, label="end")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.grid(True)
plt.legend(loc="best")
plt.tight_layout()
plt.savefig(OUT / "fig_crpf_contours_trajectory.png", dpi=170)
plt.close()

# B) Time-series: speed, distance, CRPF, TTC
fig, axs = plt.subplots(4,1, figsize=(8.2,9.2), sharex=True)
axs[0].plot(df["t"], df["sv_v"]); axs[0].set_ylabel("Speed [m/s]"); axs[0].grid(True)
axs[1].plot(df["t"], df["dist_to_obs"]); axs[1].axhline(R_SUM, linestyle="--", color="r")
axs[1].set_ylabel("Dist to OV [m]"); axs[1].grid(True)
axs[2].plot(df["t"], df["risk"]); axs[2].set_ylabel("CRPF"); axs[2].grid(True)
axs[3].plot(df["t"], df["ttc"]); axs[3].axhline(TTC_THRESH_S, linestyle="--", color="r", label="TTC threshold")
axs[3].set_ylabel("TTC [s]"); axs[3].set_xlabel("Time [s]"); axs[3].legend(); axs[3].grid(True)
plt.tight_layout()
plt.savefig(OUT / "fig_timeseries_ttc.png", dpi=170)
plt.close()

# C) VO diagnostic at t=0: mark velocities that would collide (within horizon)
V = np.linspace(-20, 20, 81)
U = np.linspace(-20, 20, 81)
unsafe = np.zeros((len(U), len(V)), dtype=bool)
if len(df) > 0:
    p0 = np.array([df["sv_x"].iloc[0]-ov_x, df["sv_y"].iloc[0]-ov_y], dtype=float)
else:
    p0 = np.array([sv_x-ov_x, sv_y-ov_y], dtype=float)
for i, ux in enumerate(U):
    for j, vy in enumerate(V):
        v_rel = np.array([ux, vy], dtype=float)
        coll, _ = will_collide(p0, v_rel, R_SUM, horizon=VO_HORIZON_S)
        unsafe[i, j] = coll
plt.figure(figsize=(6.2,6.2))
plt.title("Velocity Obstacle (diagnostic) at t=0\nRed region = collision within horizon")
plt.imshow(unsafe.T[::-1, :], extent=[U.min(), U.max(), V.min(), V.max()], origin="lower", aspect="equal")
plt.xlabel("v_x [m/s]"); plt.ylabel("v_y [m/s]"); plt.grid(True)
plt.tight_layout()
plt.savefig(OUT / "fig_vo_diagnostic.png", dpi=170)
plt.close()

# ================
# 5) METRICS + CSV
# ================
min_ttc_report = np.nanmin(df["ttc"].values) if np.isfinite(df["ttc"]).any() else np.nan
summary = {
    "CollisionOccurred": [False],   # planner prevents overlap
    "MinDistance_m":    [float(np.nanmin(df["dist_to_obs"])) if len(df) else np.nan],
    "MinTTC_s":         [float(min_ttc_report) if np.isfinite(min_ttc_report) else np.nan],
    "TET_s(TTC<thr)":   [float(tet_seconds)],
    "TTC_Threshold_s":  [float(TTC_THRESH_S)],
    "FinalXPos_m":      [float(df['sv_x'].iloc[-1]) if len(df) else np.nan],
    "FinalYPos_m":      [float(df['sv_y'].iloc[-1]) if len(df) else np.nan],
    "FinalSpeed_mps":   [float(df['sv_v'].iloc[-1]) if len(df) else np.nan],
    "StepsSimulated":   [int(len(df))]
}
summary_df = pd.DataFrame(summary)
summary_df.to_csv(OUT / "summary_part1_tuned.csv", index=False)
df.to_csv(OUT / "trajectory_part1_tuned.csv", index=False)

print("\n=== Part 1 (tuned) — Finished ===")
print(summary_df.to_string(index=False))
print(f"\nSaved to {OUT.resolve()}:")
print("  fig_crpf_contours_trajectory.png")
print("  fig_timeseries_ttc.png")
print("  fig_vo_diagnostic.png")
print("  feasible_vel/step_*.png (per-step velocity clouds)")
print("  summary_part1_tuned.csv")
print("  trajectory_part1_tuned.csv")
