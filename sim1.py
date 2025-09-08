"""
Part 1: Single Subject Vehicle (SV) vs Single Stationary Obstacle Vehicle (OV)

Implements:
- CRPF (Collision Risk Potential Field)
- VO (Velocity Obstacle) collision test
- DWA-style feasible velocity search (reachable + in-lane + no overlap + low risk)
- Simulation loop with simple kinematics
- Plots + CSV outputs

Dependencies: numpy, matplotlib, pandas
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from math import cos, sin
from pathlib import Path

# -----------------------------
# 0) Reproducibility & output dir
# -----------------------------
np.random.seed(0)
OUT = Path(".")
OUT.mkdir(parents=True, exist_ok=True)

# -----------------------------
# 1) Scenario & vehicle params
# -----------------------------
dt = 0.1                 # time step [s]
T  = 20.0                # total sim time [s]
N  = int(T/dt)

# Road/lane (straight lane along +Y, width 8 m, centered at X=0)
lane_center_x = 0.0
lane_half_width = 4.0
road_left  = lane_center_x + lane_half_width
road_right = lane_center_x - lane_half_width

# Vehicle radii (disc approximation)
r_S = 1.0  # subject "radius" [m]
r_O = 1.0  # obstacle "radius" [m]
R_sum = r_S + r_O

# Subject initial pose/state (start below obstacle, pointing north)
sv_x, sv_y = 0.0, -40.0
sv_v = 12.0                         # initial speed [m/s]
sv_theta = np.deg2rad(90.0)         # heading +Y
sv_omega = 0.0                      # yaw rate (unused; we apply dtheta directly)

# Subject limits
a_min, a_max = -4.0, 2.5            # accel limits [m/s^2]
omega_min, omega_max = -0.6, 0.6    # yaw-rate command bounds [rad/s]
v_min, v_max = 0.0, 25.0            # speed bounds [m/s]

# Desired speed
v_des = 15.0

# Obstacle (stationary) at origin
ov_x, ov_y = 0.0, 0.0
ov_vx, ov_vy = 0.0, 0.0
ov_ax, ov_ay = 0.0, 0.0
m_O = 1500.0   # kg
size_O = 4.5   # ~length surrogate
kappa_O = 1.0  # type weight (1 car, >1 truck)

# -----------------------------
# 2) Helper functions (CRPF, VO, etc.)
# -----------------------------
def pseudo_distance(xs, ys, xo, yo, v_obs, eps=1.2, rho=0.02):
    """Eq. (8) pseudo-distance: lane/velocity-aware effective distance."""
    dx = xs - xo
    dy = ys - yo
    # signed-power to keep direction, then magnitude for metric
    termx = np.sign(dx) * (abs(dx)**eps) * np.exp(-rho * max(v_obs, 0.0))
    termy = np.sign(dy) * (abs(dy)**eps)
    return float(np.sqrt(termx**2 + termy**2))

def accel_factor(ax, ay, xs, ys, xo, yo, K=1.0):
    """Eq. (9) acceleration impact: increases risk if obstacle accelerates toward subject."""
    a = np.array([ax, ay], dtype=float)
    rd_vec = np.array([xs - xo, ys - yo], dtype=float)
    na = np.linalg.norm(a)
    nr = np.linalg.norm(rd_vec)
    if na < 1e-9 or nr < 1e-9:
        return 1.0
    cos_t = float(np.dot(a/na, rd_vec/nr))
    denom = K - na * cos_t
    denom = float(np.clip(denom, 0.2, 10.0))  # guard
    return K / denom

def road_factor(visibility=1.0, mu=1.0, curvature=0.0, slope=0.0,
                vis_star=1.0, mu_star=1.0, curv_star=0.0, slope_star=0.0,
                g1=1.0, g2=1.0, g3=1.0, g4=1.0):
    """Eq. (10) road condition factor (defaults -> 1.0)."""
    term1 = (visibility / vis_star) ** g1 if vis_star > 0 else 1.0
    term2 = (mu / mu_star) ** g2 if mu_star > 0 else 1.0
    term3 = np.exp((curvature - curv_star) ** g3 + (slope - slope_star) ** g4) if (g3 or g4) else 1.0
    return float(term1 * term2 * term3)

def virtual_mass(m, v):
    """Eq. (6) M = m*(a*v^b + c)."""
    return float(m * (1.566e-14 * (max(v, 0.0) ** 6.687) + 0.3354))

def type_factor(size, kappa, size_star=4.0, kappa_star=1.0, w1=1.0, w2=1.0):
    """Eq. (7) T = (s/s*)(kappa/kappa*) * w1*w2."""
    return float((size / max(size_star, 1e-6)) * (kappa / max(kappa_star, 1e-6)) * w1 * w2)

def CRPF(xs, ys, xo, yo, v_obs, ax_obs, ay_obs,
         m_obs, size_obs, kappa_obs,
         G=1.0, zeta=2.0):
    """Eq. (11) risk magnitude & vector (direction from obstacle -> subject)."""
    rd = pseudo_distance(xs, ys, xo, yo, v_obs)
    rd_vec = np.array([xs - xo, ys - yo], dtype=float)
    if rd < 1e-6:
        rd = 1e-6
    M = virtual_mass(m_obs, v_obs)
    Tfac = type_factor(size_obs, kappa_obs)
    Rfac = road_factor()  # defaults
    phi = accel_factor(ax_obs, ay_obs, xs, ys, xo, yo)
    mag = float(G * M * Tfac * Rfac * phi / (rd ** zeta))
    nrd = np.linalg.norm(rd_vec)
    direction = rd_vec / nrd if nrd > 1e-9 else np.array([0.0, 1.0], dtype=float)
    vec = mag * direction
    return mag, vec

def will_collide(p_rel, v_rel, R, horizon=8.0):
    """
    VO-style check: if min distance within [0,horizon] <= R (two discs).
    Returns (collides_boolean, TTC_seconds or inf).
    """
    v2 = float(np.dot(v_rel, v_rel))
    if v2 < 1e-12:
        return (np.linalg.norm(p_rel) <= R), float("inf")
    t_star = -float(np.dot(p_rel, v_rel)) / v2
    t_star = float(np.clip(t_star, 0.0, horizon))
    d_min = float(np.linalg.norm(p_rel + t_star * v_rel))
    # approximate TTC if approaching: solve ||p + vt|| = R
    ttc = float("inf")
    if float(np.dot(p_rel, v_rel)) < 0.0:
        a = v2
        b = 2.0 * float(np.dot(p_rel, v_rel))
        c = float(np.dot(p_rel, p_rel)) - R ** 2
        disc = b * b - 4.0 * a * c
        if disc >= 0.0:
            r1 = (-b - np.sqrt(disc)) / (2.0 * a)
            r2 = (-b + np.sqrt(disc)) / (2.0 * a)
            roots = [r for r in (r1, r2) if r >= 0.0]
            if roots:
                ttc = float(min(roots))
    return (d_min <= R), ttc

def in_lane(x):
    return (road_right <= x <= road_left)

# -----------------------------
# 3) Precompute CRPF heatmap (for visualization)
# -----------------------------
grid_x = np.linspace(-12, 12, 121)
grid_y = np.linspace(-50, 20, 141)
CR = np.zeros((len(grid_y), len(grid_x)))
for j, yy in enumerate(grid_y):
    for i, xx in enumerate(grid_x):
        cr, _ = CRPF(xx, yy, ov_x, ov_y, 0.0, 0.0, 0.0, m_O, size_O, kappa_O, G=1.0, zeta=2.0)
        CR[j, i] = cr

# -----------------------------
# 4) Simulation loop
# -----------------------------
log = {
    "t": [], "sv_x": [], "sv_y": [], "sv_v": [], "sv_theta": [],
    "dist_to_obs": [], "risk": [], "ttc": [], "picked_v": [], "picked_dtheta": []
}

collision_happened = False
min_ttc = float("inf")
min_dist = float("inf")

for k in range(N):
    t = k * dt

    # Relative state vs obstacle (stationary)
    p_rel = np.array([sv_x - ov_x, sv_y - ov_y], dtype=float)
    v_rel = np.array([sv_v * cos(sv_theta) - ov_vx, sv_v * sin(sv_theta) - ov_vy], dtype=float)
    dist = float(np.linalg.norm(p_rel))

    # Risk at current pose
    risk, _ = CRPF(sv_x, sv_y, ov_x, ov_y, 0.0, 0.0, 0.0, m_O, size_O, kappa_O, G=1.0, zeta=2.0)

    # VO diagnostic at current velocity (not used for control here; we use a rollout-based filter)
    coll_pred, ttc = will_collide(p_rel, v_rel, R_sum, horizon=8.0)
    min_ttc = min(min_ttc, ttc if np.isfinite(ttc) else float("inf"))
    min_dist = min(min_dist, dist)

    # -------- DWA-style candidate search (one-step) --------
    a_samples = np.linspace(a_min, a_max, 7)         # accel options
    omega_samples = np.linspace(omega_min, omega_max, 9)  # yaw-rate options (as dtheta/dt)

    best_cost = float("inf")
    best_v_next = sv_v
    best_theta_next = sv_theta
    best_dtheta = 0.0

    for a_cmd in a_samples:
        v_next = float(np.clip(sv_v + a_cmd * dt, v_min, v_max))
        for d_omega in omega_samples:
            theta_next = sv_theta + d_omega * dt

            # Short rollout (1s) to check lane, overlap, and accumulate CRPF
            x_tmp, y_tmp, th_tmp, v_tmp = sv_x, sv_y, theta_next, v_next
            ok = True
            acc_risk = 0.0
            horizon_steps = 10  # 1.0 s lookahead

            for _ in range(horizon_steps):
                x_tmp += v_tmp * cos(th_tmp) * dt
                y_tmp += v_tmp * sin(th_tmp) * dt

                if not in_lane(x_tmp):
                    ok = False
                    break

                if np.hypot(x_tmp - ov_x, y_tmp - ov_y) <= R_sum:  # disc overlap check
                    ok = False
                    break

                rr, _ = CRPF(x_tmp, y_tmp, ov_x, ov_y, 0.0, 0.0, 0.0, m_O, size_O, kappa_O, G=1.0, zeta=2.0)
                acc_risk += rr

            if not ok:
                continue

            # Cost = integrated CRPF + (speed tracking) + (smooth steering)
            cost = 1.0 * acc_risk + 0.1 * (v_des - v_next) ** 2 + 0.05 * (d_omega ** 2)

            if cost < best_cost:
                best_cost = cost
                best_v_next = v_next
                best_theta_next = theta_next
                best_dtheta = d_omega * dt

    # Fallback: if no feasible candidate, hard-brake and hold heading
    if not np.isfinite(best_cost):
        best_v_next = max(sv_v + a_min * dt, 0.0)
        best_theta_next = sv_theta
        best_dtheta = 0.0

    # Apply chosen control (wrap angle to [-pi, pi])
    sv_v = best_v_next
    sv_theta = (best_theta_next + np.pi) % (2 * np.pi) - np.pi
    sv_x += sv_v * cos(sv_theta) * dt
    sv_y += sv_v * sin(sv_theta) * dt

    # Log
    log["t"].append(t)
    log["sv_x"].append(sv_x)
    log["sv_y"].append(sv_y)
    log["sv_v"].append(sv_v)
    log["sv_theta"].append(sv_theta)
    log["dist_to_obs"].append(dist)
    log["risk"].append(risk)
    log["ttc"].append(ttc if np.isfinite(ttc) else np.nan)
    log["picked_v"].append(sv_v)
    log["picked_dtheta"].append(best_dtheta)

    # Collision check
    if dist <= R_sum:
        collision_happened = True
        break

# Convert to DataFrame
df = pd.DataFrame(log)

# -----------------------------
# 5) Plots
# -----------------------------
# A) CRPF heatmap + trajectory
plt.figure(figsize=(7, 6))
plt.title("CRPF Heatmap (Stationary OV) + SV Trajectory")
extent = [grid_x.min(), grid_x.max(), grid_y.min(), grid_y.max()]
plt.imshow(np.log10(CR + 1e-6), origin="lower", extent=extent, aspect="auto")
plt.colorbar(label="log10(CRPF + 1e-6)")

# Obstacle collision boundary (sum of radii)
circ = plt.Circle((ov_x, ov_y), R_sum, fill=False, linestyle="--", linewidth=2)
plt.gca().add_patch(circ)

# Lane lines
plt.axvline(road_left, color="k", linestyle="--", linewidth=1)
plt.axvline(road_right, color="k", linestyle="--", linewidth=1)

# Trajectory
plt.plot(df["sv_x"], df["sv_y"], linewidth=2)
plt.scatter([df["sv_x"].iloc[0]], [df["sv_y"].iloc[0]], s=40, label="Start")
plt.scatter([df["sv_x"].iloc[-1]], [df["sv_y"].iloc[-1]], s=40, label="End")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.grid(True)
plt.legend(loc="best")
plt.tight_layout()
plt.savefig(OUT / "fig_crpf_trajectory.png", dpi=160)

# B) Time series (speed, distance, risk)
fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
axs[0].plot(df["t"], df["sv_v"])
axs[0].set_ylabel("Speed [m/s]")
axs[0].grid(True)

axs[1].plot(df["t"], df["dist_to_obs"])
axs[1].axhline(R_sum, linestyle="--", color="r", label="collision radius")
axs[1].set_ylabel("Dist to OV [m]")
axs[1].legend()
axs[1].grid(True)

axs[2].plot(df["t"], df["risk"])
axs[2].set_ylabel("CRPF Risk (scalar)")
axs[2].set_xlabel("Time [s]")
axs[2].grid(True)

plt.tight_layout()
plt.savefig(OUT / "fig_timeseries.png", dpi=160)

# C) VO diagnostic at t=0 (grid of hypothetical relative velocities)
V = np.linspace(-20, 20, 81)
U = np.linspace(-20, 20, 81)
unsafe = np.zeros((len(U), len(V)), dtype=bool)
if len(df) > 0:
    p0 = np.array([df["sv_x"].iloc[0] - ov_x, df["sv_y"].iloc[0] - ov_y], dtype=float)
else:
    p0 = np.array([sv_x - ov_x, sv_y - ov_y], dtype=float)

for i, ux in enumerate(U):
    for j, vy in enumerate(V):
        v_rel = np.array([ux, vy], dtype=float)
        coll, _ = will_collide(p0, v_rel, R_sum, horizon=8.0)
        unsafe[i, j] = coll

plt.figure(figsize=(6, 6))
plt.title("Velocity Obstacle (diagnostic) at t=0\nRed = collision within horizon")
plt.imshow(unsafe.T[::-1, :], extent=[U.min(), U.max(), V.min(), V.max()],
           origin="lower", aspect="equal")
plt.xlabel("v_x [m/s]")
plt.ylabel("v_y [m/s]")
plt.grid(True)
plt.tight_layout()
plt.savefig(OUT / "fig_vo_diagnostic.png", dpi=160)

# -----------------------------
# 6) Summary metrics + CSVs
# -----------------------------
min_ttc_report = np.nanmin(df["ttc"].values) if np.isfinite(df["ttc"]).any() else np.nan
summary = {
    "CollisionOccurred": [bool(collision_happened)],
    "MinDistance_m":    [float(np.nanmin(df["dist_to_obs"])) if len(df) else np.nan],
    "MinTTC_s":         [float(min_ttc_report) if np.isfinite(min_ttc_report) else np.nan],
    "FinalYPos_m":      [float(df["sv_y"].iloc[-1]) if len(df) else np.nan],
    "FinalXPos_m":      [float(df["sv_x"].iloc[-1]) if len(df) else np.nan],
    "FinalSpeed_mps":   [float(df["sv_v"].iloc[-1]) if len(df) else np.nan],
    "StepsSimulated":   [int(len(df))]
}
summary_df = pd.DataFrame(summary)
summary_df.to_csv(OUT / "summary_part1.csv", index=False)
df.to_csv(OUT / "trajectory_part1.csv", index=False)

print("\n=== Part 1: SV vs Stationary OV — Finished ===")
print(summary_df.to_string(index=False))
print(f"\nSaved:\n  {OUT/'fig_crpf_trajectory.png'}"
      f"\n  {OUT/'fig_timeseries.png'}"
      f"\n  {OUT/'fig_vo_diagnostic.png'}"
      f"\n  {OUT/'summary_part1.csv'}"
      f"\n  {OUT/'trajectory_part1.csv'}")
