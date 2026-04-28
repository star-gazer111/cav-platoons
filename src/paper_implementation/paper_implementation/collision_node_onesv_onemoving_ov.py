                      
"""
CRPF + DWA avoidance for one TurtleBot3 Burger and one moving obstacle.

The main Burger drives along its initial odom path. When its LiDAR sees a
Burger-sized obstacle in the forward cone, the obstacle pose and velocity are
tracked in odom and a CRPF-gated Dynamic Window Approach chooses velocity
commands that pass around it, then return to the original path after clearing
it. Right-side passing remains the default, with left-side fallback when the
right side is infeasible.

Topics:
  subscribes: scan with sensor-data QoS, odom
  publishes:  cmd_vel
"""

import math
from dataclasses import dataclass

import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


                                                              
                           
                                                              

DT = 0.10

                                                                             
                                                                  
OBSTACLE_LOOKAHEAD = 0.7     

                                             
BURGER_RADIUS = 0.18
PASS_CLEARANCE = 0.20
R_ROBOT = BURGER_RADIUS
R_OBS = BURGER_RADIUS
R_SUM = R_ROBOT + R_OBS + 0.06
PASS_OFFSET = 2.0 * BURGER_RADIUS + PASS_CLEARANCE
RIGHT_PASS_OFFSET = -PASS_OFFSET
LEFT_PASS_OFFSET = PASS_OFFSET
PASS_RIGHT = -1
PASS_LEFT = 1
PASS_NONE = 0

                                                    
FRONT_FOV_DEG = 60.0
FRONT_FOV_RAD = math.radians(FRONT_FOV_DEG)
MIN_SCAN_RANGE = 0.10
MAX_FRONT_LATERAL = 0.55
MIN_CLUSTER_POINTS = 3
CLUSTER_RANGE_GAP = 0.16

                                 
V_MIN = 0.02
V_MAX = 0.15
V_DES = 0.10
V_CRUISE_MIN = 0.045
V_AVOID_MIN = 0.040
V_AVOID_MAX = 0.075
W_MAX = 0.75
A_MIN = -0.45
A_MAX = 0.45
STEPS = 30
W_CMD_RATE = 0.70
V_CMD_RATE = 0.14

                                                             
LANE_HALF_WIDTH = 0.95
LANE_LEFT_LIMIT = LANE_HALF_WIDTH - BURGER_RADIUS
LANE_RIGHT_LIMIT = -LANE_HALF_WIDTH + BURGER_RADIUS
AVOID_START_AHEAD = 1.20
AVOID_FULL_AHEAD = 0.60
AVOID_RETURN_START_AHEAD = -0.25
X_CLEAR_AHEAD = 1.00
OBS_HOLD_SEC = 1.60
SCAN_STALE_SEC = 0.50
TRACK_SMOOTHING = 0.45
MAX_TRACK_DT = 0.80
MAX_OBS_SPEED = 0.35
TRACK_MAX_JUMP = 0.75

                                                                
CRPF_G = 0.40
CRPF_ZETA = 1.20
PSEUDO_EPS = 1.00
PSEUDO_RHO = 0.02

OBS_MASS = 10.0
OBS_SIZE = 0.30
OBS_KAPPA = 1.0
SIZE_STAR = 0.30
KAPPA_STAR = 1.0

CR_YELLOW_LOW = 0.006
CR_YELLOW_HIGH = 0.012

                   
W_RISK = 1.00
W_LAT = 1.05
W_HEAD = 0.24
W_END = 1.15
W_SPEED = 0.10
W_TURN = 0.24
W_TURN_CHANGE = 0.35
W_SIDE = 1.35
W_PASS_PREFERENCE = 0.18
W_SIDE_SWITCH = 0.30

                                                                      
K_FREE_LAT = 0.85
K_FREE_HEAD = 1.15

                        
SCAN_DEBUG_THROTTLE_SEC = 2.00
STATE_DEBUG_THROTTLE_SEC = 0.35
NO_SCAN_WARN_THROTTLE_SEC = 1.00


                                                              
              
                                                              

def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def smoothstep(edge0: float, edge1: float, value: float) -> float:
    if abs(edge1 - edge0) < 1e-9:
        return 1.0 if value >= edge1 else 0.0

    t = clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def finite_min(values: np.ndarray) -> float:
    valid = values[np.isfinite(values)]
    if valid.size == 0:
        return math.inf
    return float(np.min(valid))


def fmt_range(value: float) -> str:
    if not math.isfinite(value):
        return "inf"
    return f"{value:.2f}"


def bool_param(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def parse_pass_side(value: str) -> int:
    side = value.strip().lower()
    if side == "left":
        return PASS_LEFT
    if side == "none":
        return PASS_NONE
    return PASS_RIGHT


def pass_side_name(side: int) -> str:
    if side == PASS_LEFT:
        return "left"
    if side == PASS_RIGHT:
        return "right"
    return "none"


def pass_side_offset(side: int) -> float:
    if side == PASS_LEFT:
        return LEFT_PASS_OFFSET
    if side == PASS_RIGHT:
        return RIGHT_PASS_OFFSET
    return 0.0


                                                              
              
                                                              

def _pseudo_distance(xs, ys, xo, yo, v_obs=0.0):
    dx = xs - xo
    dy = ys - yo
    tx = PSEUDO_EPS * dx * math.exp(-PSEUDO_RHO * max(v_obs, 0.0))
    ty = PSEUDO_EPS * dy
    return math.hypot(tx, ty)


def _virtual_mass(m, v):
    return m * (1.566e-14 * max(v, 0.0) ** 6.687 + 0.3354)


def _type_factor(size, kappa):
    return (size / max(SIZE_STAR, 1e-6)) * (kappa / max(KAPPA_STAR, 1e-6))


def _accel_factor(ax, ay, xs, ys, xo, yo):
    k = 5.0
    a = np.array([ax, ay])
    na = np.linalg.norm(a)
    rd = np.array([xs - xo, ys - yo])
    nr = np.linalg.norm(rd)
    if na < 1e-9 or nr < 1e-9:
        return 1.0
    cos_t = float(np.dot(a / na, rd / nr))
    return k / min(max(k - na * cos_t, 0.2), 10.0)


def crpf_value(xs, ys, xo, yo, v_obs=0.0, ax=0.0, ay=0.0):
    rd = max(_pseudo_distance(xs, ys, xo, yo, v_obs), 0.25)
    mass = _virtual_mass(OBS_MASS, v_obs)
    type_factor = _type_factor(OBS_SIZE, OBS_KAPPA)
    phi = _accel_factor(ax, ay, xs, ys, xo, yo)
    return CRPF_G * mass * type_factor * phi / rd ** CRPF_ZETA


def gate_alpha(risk_now: float) -> float:
    if risk_now <= CR_YELLOW_LOW:
        return 0.0
    if risk_now >= CR_YELLOW_HIGH:
        return 1.0

    tau = (risk_now - CR_YELLOW_LOW) / (CR_YELLOW_HIGH - CR_YELLOW_LOW)
    return tau * tau * (3.0 - 2.0 * tau)


                                                              
                    
                                                              

@dataclass
class PathFrame:
    x0: float
    y0: float
    yaw0: float

    def along_lateral(self, x: float, y: float) -> tuple[float, float]:
        dx = x - self.x0
        dy = y - self.y0

        forward_x = math.cos(self.yaw0)
        forward_y = math.sin(self.yaw0)
        left_x = -math.sin(self.yaw0)
        left_y = math.cos(self.yaw0)

        along = dx * forward_x + dy * forward_y
        lateral = dx * left_x + dy * left_y
        return along, lateral

    def heading_error(self, yaw: float) -> float:
        return wrap_angle(yaw - self.yaw0)


                                                              
            
                                                              

class CRPFDWABurgerNode(Node):
    def __init__(self):
        super().__init__("crpf_dwa_burger_avoidance")

        self.scan_topic = str(self.declare_parameter("scan_topic", "scan").value)
        self.odom_topic = str(self.declare_parameter("odom_topic", "odom").value)
        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "cmd_vel").value)
        self.obstacle_lookahead = float(
            self.declare_parameter("obstacle_lookahead", OBSTACLE_LOOKAHEAD).value
        )
        self.front_fov_deg = float(self.declare_parameter("front_fov_deg", FRONT_FOV_DEG).value)
        self.front_fov_rad = math.radians(self.front_fov_deg)
        self.max_front_lateral = float(
            self.declare_parameter("max_front_lateral", MAX_FRONT_LATERAL).value
        )
        self.obs_hold_sec = float(self.declare_parameter("obs_hold_sec", OBS_HOLD_SEC).value)
        self.preferred_pass_side = parse_pass_side(
            str(self.declare_parameter("preferred_pass_side", "right").value)
        )
        self.allow_left_fallback = bool_param(
            self.declare_parameter("allow_left_fallback", True).value
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        self.path: PathFrame | None = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.have_odom = False

        self.have_scan = False
        self.scan_count = 0
        self.last_scan_time = None
        self.last_scan_stats = {}
        self.closest_front = math.inf

        self.obs_detected = False
        self.obs_x = math.inf
        self.obs_y = math.inf
        self.obs_vx = 0.0
        self.obs_vy = 0.0
        self.obs_last_seen = None
        self.obs_source = "none"
        self.obs_seen_count = 0

        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.last_target_lat = 0.0
        self.last_risk = 0.0
        self.last_alpha = 0.0
        self.last_best_cost = math.inf
        self.last_pass_side = self.preferred_pass_side

        self.timer = self.create_timer(DT, self.control_loop)

        self.get_logger().info(
            "CRPF+DWA Burger avoidance ready | "
            f"topics: scan={self.scan_topic}(BEST_EFFORT) "
            f"odom={self.odom_topic} cmd={self.cmd_vel_topic} | "
            f"lookahead={self.obstacle_lookahead:.2f} m | "
            f"front_fov={self.front_fov_deg:.0f} deg | "
            f"preferred_pass={pass_side_name(self.preferred_pass_side)} | "
            f"rollout={STEPS * DT:.1f} s"
        )

                                                              
               
                                                              

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_odom(msg)
        self.speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )
        self.have_odom = True

        if self.path is None:
            self.path = PathFrame(self.x, self.y, self.yaw)
            self.get_logger().info(
                f"Initial path locked at ({self.x:.2f}, {self.y:.2f}), "
                f"yaw={math.degrees(self.yaw):.1f} deg"
            )

    def scan_cb(self, msg: LaserScan):
        found, closest, lx, ly = self.detect_front_obstacle(msg)

        self.have_scan = True
        self.scan_count += 1
        now = self.get_clock().now()
        self.last_scan_time = now
        self.closest_front = closest

        if found:
            hit_range = max(math.hypot(lx, ly), 1e-6)
            center_lx = lx + R_OBS * (lx / hit_range)
            center_ly = ly + R_OBS * (ly / hit_range)

            obs_x = self.x + center_lx * math.cos(self.yaw) - center_ly * math.sin(self.yaw)
            obs_y = self.y + center_lx * math.sin(self.yaw) + center_ly * math.cos(self.yaw)
            self.update_obstacle_track(obs_x, obs_y, now)

            self.get_logger().warn(
                f"OV detected: range={closest:.2f} m local=({lx:.2f},{ly:.2f}) "
                f"center_local=({center_lx:.2f},{center_ly:.2f}) "
                f"center_odom=({self.obs_x:.2f},{self.obs_y:.2f}) "
                f"vel=({self.obs_vx:.2f},{self.obs_vy:.2f})",
                throttle_duration_sec=0.5,
            )
        else:
            self.expire_obstacle_if_needed()

        self.log_scan_debug(msg)

                                                              
               
                                                              

    def detect_front_obstacle(self, msg: LaserScan) -> tuple[bool, float, float, float]:
        ranges = np.asarray(msg.ranges, dtype=float)
        if ranges.size == 0:
            self.last_scan_stats = {
                "rays": 0,
                "finite_count": 0,
                "selected_count": 0,
                "selected_min": math.inf,
            }
            return False, math.inf, 0.0, 0.0

        angles = msg.angle_min + np.arange(ranges.size, dtype=float) * msg.angle_increment
        wrapped = (angles + math.pi) % (2.0 * math.pi) - math.pi

        finite = np.isfinite(ranges)
        positive = ranges > MIN_SCAN_RANGE
        within_lookahead = ranges <= self.obstacle_lookahead
        in_front_cone = np.abs(wrapped) <= self.front_fov_rad

        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)
        centered_front = (lx > 0.0) & (np.abs(ly) <= self.max_front_lateral)

        mask = finite & positive & within_lookahead & in_front_cone & centered_front
        front_cone = finite & positive & in_front_cone
        within_lookahead_anywhere = finite & positive & within_lookahead

        selected_idx = np.where(mask)[0]
        clusters = []
        if selected_idx.size > 0:
            split_at = np.where(np.diff(selected_idx) > 1)[0] + 1
            for cluster in np.split(selected_idx, split_at):
                if cluster.size >= MIN_CLUSTER_POINTS:
                    cluster_ranges = ranges[cluster]
                    if float(np.max(cluster_ranges) - np.min(cluster_ranges)) <= CLUSTER_RANGE_GAP:
                        clusters.append(cluster)
            if not clusters and selected_idx.size >= MIN_CLUSTER_POINTS:
                clusters.append(selected_idx)

        self.last_scan_stats = {
            "rays": int(ranges.size),
            "finite_count": int(np.sum(finite)),
            "positive_count": int(np.sum(finite & positive)),
            "front_cone_count": int(np.sum(front_cone)),
            "within_lookahead_count": int(np.sum(within_lookahead_anywhere)),
            "centered_front_count": int(np.sum(finite & positive & centered_front)),
            "selected_count": int(np.sum(mask)),
            "cluster_count": len(clusters),
            "min_all": finite_min(ranges[finite & positive]),
            "min_front_cone": finite_min(ranges[front_cone]),
            "min_within_lookahead": finite_min(ranges[within_lookahead_anywhere]),
            "selected_min": finite_min(ranges[mask]),
            "angle_min_deg": math.degrees(msg.angle_min),
            "angle_max_deg": math.degrees(msg.angle_max),
            "range_min": msg.range_min,
            "range_max": msg.range_max,
        }

        if not clusters:
            return False, math.inf, 0.0, 0.0

        best_cluster = min(clusters, key=lambda cluster: float(np.median(ranges[cluster])))
        self.last_scan_stats["best_cluster_points"] = int(best_cluster.size)
        closest = float(np.min(ranges[best_cluster]))
        return (
            True,
            closest,
            float(np.median(lx[best_cluster])),
            float(np.median(ly[best_cluster])),
        )

                                                              
                       
                                                              

    def update_obstacle_track(self, measured_x: float, measured_y: float, now):
        previous_track = (
            self.obs_detected
            and math.isfinite(self.obs_x)
            and math.isfinite(self.obs_y)
            and self.obs_last_seen is not None
        )

        if not previous_track:
            self.obs_x = measured_x
            self.obs_y = measured_y
            self.obs_vx = 0.0
            self.obs_vy = 0.0
            self.obs_detected = True
            self.obs_last_seen = now
            self.obs_source = "scan"
            self.obs_seen_count = 1
            return

        dt = (now - self.obs_last_seen).nanoseconds / 1e9
        if dt <= 1e-3 or dt > MAX_TRACK_DT:
            self.obs_x = measured_x
            self.obs_y = measured_y
            self.obs_vx = 0.0
            self.obs_vy = 0.0
            self.obs_last_seen = now
            self.obs_source = "scan-reset"
            self.obs_seen_count += 1
            return

        predicted_x = self.obs_x + self.obs_vx * dt
        predicted_y = self.obs_y + self.obs_vy * dt
        jump = math.hypot(measured_x - predicted_x, measured_y - predicted_y)
        if jump > TRACK_MAX_JUMP:
            self.obs_x = measured_x
            self.obs_y = measured_y
            self.obs_vx = 0.0
            self.obs_vy = 0.0
            self.obs_last_seen = now
            self.obs_source = "scan-reset"
            self.obs_seen_count += 1
            return

        raw_vx = (measured_x - self.obs_x) / dt
        raw_vy = (measured_y - self.obs_y) / dt
        raw_speed = math.hypot(raw_vx, raw_vy)
        if raw_speed > MAX_OBS_SPEED:
            scale = MAX_OBS_SPEED / raw_speed
            raw_vx *= scale
            raw_vy *= scale

        beta = TRACK_SMOOTHING
        self.obs_x = (1.0 - beta) * predicted_x + beta * measured_x
        self.obs_y = (1.0 - beta) * predicted_y + beta * measured_y
        self.obs_vx = (1.0 - beta) * self.obs_vx + beta * raw_vx
        self.obs_vy = (1.0 - beta) * self.obs_vy + beta * raw_vy
        self.obs_detected = True
        self.obs_last_seen = now
        self.obs_source = "scan-track"
        self.obs_seen_count += 1

    def predicted_obstacle_pose(self, horizon: float = 0.0) -> tuple[float, float]:
        if not self.obs_detected or not math.isfinite(self.obs_x) or self.obs_last_seen is None:
            return math.inf, math.inf

        age = (self.get_clock().now() - self.obs_last_seen).nanoseconds / 1e9
        dt = max(0.0, age + horizon)
        return self.obs_x + self.obs_vx * dt, self.obs_y + self.obs_vy * dt

    def clear_obstacle(self, reason: str):
        self.get_logger().info(
            f"Clearing obstacle track: {reason}",
            throttle_duration_sec=0.5,
        )
        self.obs_detected = False
        self.obs_x = math.inf
        self.obs_y = math.inf
        self.obs_vx = 0.0
        self.obs_vy = 0.0
        self.obs_last_seen = None
        self.obs_source = "none"
        self.obs_seen_count = 0

                                                              
                      
                                                              

    def control_loop(self):
        if not self.have_odom or self.path is None:
            self.get_logger().warn(
                "Waiting for /odom before driving.",
                throttle_duration_sec=NO_SCAN_WARN_THROTTLE_SEC,
            )
            self.publish_cmd(0.0, 0.0)
            return

        if not self.have_scan:
            self.get_logger().warn(
                "No /scan messages received yet. The node will not drive blind.",
                throttle_duration_sec=NO_SCAN_WARN_THROTTLE_SEC,
            )
            self.publish_cmd(0.0, 0.0)
            return

        scan_age = 0.0
        if self.last_scan_time is not None:
            scan_age = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if scan_age > SCAN_STALE_SEC:
            self.get_logger().warn(
                f"Latest /scan is stale: age={scan_age:.2f}s. Stopping.",
                throttle_duration_sec=NO_SCAN_WARN_THROTTLE_SEC,
            )
            self.publish_cmd(0.0, 0.0)
            return

        self.expire_obstacle_if_needed()

        if not self.obs_detected:
            v, w = self.free_space_command()
            self.last_risk = 0.0
            self.last_alpha = 0.0
            self.last_best_cost = 0.0
            self.publish_cmd(v, w)
            self.log_state_debug()
            return

        v, w, cost = self.choose_dwa_command()
        self.last_best_cost = cost
        self.publish_cmd(v, w)
        self.log_state_debug()

    def free_space_command(self) -> tuple[float, float]:
        _, lateral = self.path.along_lateral(self.x, self.y)
        heading_error = self.path.heading_error(self.yaw)
        angular = -K_FREE_LAT * lateral - K_FREE_HEAD * heading_error
        return V_DES, clamp(angular, -W_MAX, W_MAX)

    def choose_dwa_command(self) -> tuple[float, float, float]:
        if self.obs_detected:
            ox, oy = self.predicted_obstacle_pose(0.0)
            obs_speed = math.hypot(self.obs_vx, self.obs_vy)
            risk_now = crpf_value(self.x, self.y, ox, oy, obs_speed)
        else:
            ox = math.inf
            oy = math.inf
            obs_speed = 0.0
            risk_now = 0.0

        alpha = gate_alpha(risk_now)
        self.last_risk = risk_now
        self.last_alpha = alpha

        v_samples = self.velocity_samples()
        w_samples = np.linspace(-W_MAX, W_MAX, 21)
        pass_sides = self.pass_side_candidates(ox, oy)

        best_cost = math.inf
        best_v = 0.0
        best_w = 0.0
        best_side = self.last_pass_side

        for side in pass_sides:
            for v_next in v_samples:
                for w in w_samples:
                    cost = self.rollout_cost(
                        float(v_next),
                        float(w),
                        ox,
                        oy,
                        self.obs_vx,
                        self.obs_vy,
                        obs_speed,
                        alpha,
                        side,
                    )
                    if side != self.preferred_pass_side and side != PASS_NONE:
                        cost += W_PASS_PREFERENCE
                    if (
                        side != self.last_pass_side
                        and self.last_pass_side != PASS_NONE
                        and side != PASS_NONE
                    ):
                        cost += W_SIDE_SWITCH
                    if cost < best_cost:
                        best_cost = cost
                        best_v = float(v_next)
                        best_w = float(w)
                        best_side = side

        if not math.isfinite(best_cost):
            self.get_logger().warn(
                "All DWA candidates infeasible; stopping instead of creeping.",
                throttle_duration_sec=0.5,
            )
            self.last_pass_side = PASS_NONE
            return 0.0, 0.0, math.inf

        self.last_pass_side = best_side
        return best_v, best_w, best_cost

    def pass_side_candidates(self, ox: float, oy: float) -> list[int]:
        if not self.obs_detected or not math.isfinite(ox) or not math.isfinite(oy):
            return [PASS_NONE]

        _, obs_lat = self.path.along_lateral(ox, oy)
        preferred = (
            self.last_pass_side
            if self.last_pass_side in (PASS_LEFT, PASS_RIGHT)
            else self.preferred_pass_side
        )
        fallback = PASS_LEFT if preferred == PASS_RIGHT else PASS_RIGHT
        sides = [preferred]
        if self.preferred_pass_side not in sides:
            sides.append(self.preferred_pass_side)
        if self.allow_left_fallback and fallback not in sides:
            sides.append(fallback)

        feasible = []
        for side in sides:
            target = obs_lat + pass_side_offset(side)
            if LANE_RIGHT_LIMIT <= target <= LANE_LEFT_LIMIT:
                feasible.append(side)

        return feasible if feasible else sides

    def velocity_samples(self) -> np.ndarray:
        """DWA velocity candidates.

        From rest, purely acceleration-limited samples are too small for Gazebo
        to visibly move the Burger. Keep acceleration-window samples, but add
        practical cruise/avoidance speeds so free-space behavior is forward.
        """
        v_cur = clamp(self.speed, V_MIN, V_MAX)
        dyn_low = clamp(v_cur + A_MIN * DT, V_MIN, V_MAX)
        dyn_high = clamp(v_cur + A_MAX * DT, V_MIN, V_MAX)
        dynamic = np.linspace(dyn_low, dyn_high, 5)

        if self.obs_detected:
            practical = np.linspace(V_AVOID_MIN, V_AVOID_MAX, 6)
        else:
            practical = np.linspace(V_CRUISE_MIN, V_MAX, 6)

        samples = np.unique(np.round(np.concatenate((dynamic, practical)), 4))
        min_allowed = V_AVOID_MIN if self.obs_detected else V_MIN
        return samples[(samples >= min_allowed) & (samples <= V_MAX)]

    def rollout_cost(
        self,
        v: float,
        w: float,
        ox: float,
        oy: float,
        ovx: float,
        ovy: float,
        obs_speed: float,
        alpha: float,
        pass_side: int,
    ) -> float:
        x_t = self.x
        y_t = self.y
        yaw_t = self.yaw

        acc_risk = 0.0
        acc_lat = 0.0
        acc_head = 0.0
        acc_side = 0.0
        end_cte = 0.0

        for step in range(STEPS):
            yaw_t = wrap_angle(yaw_t + w * DT)
            x_t += v * math.cos(yaw_t) * DT
            y_t += v * math.sin(yaw_t) * DT

            along_t, lat_t = self.path.along_lateral(x_t, y_t)
            if not (LANE_RIGHT_LIMIT <= lat_t <= LANE_LEFT_LIMIT):
                return math.inf

            horizon = (step + 1) * DT
            ox_t = ox + ovx * horizon
            oy_t = oy + ovy * horizon

            target_lat = self.target_lateral(x_t, y_t, pass_side, ox_t, oy_t)
            cte = lat_t - target_lat
            head_err = self.path.heading_error(yaw_t)

            acc_lat += abs(cte)
            acc_head += abs(head_err)
            end_cte = cte

            if self.obs_detected:
                dist_to_obs = math.hypot(x_t - ox_t, y_t - oy_t)
                if dist_to_obs <= R_SUM:
                    return math.inf

                acc_risk += crpf_value(x_t, y_t, ox_t, oy_t, obs_speed)
                obs_along, obs_lat = self.path.along_lateral(ox_t, oy_t)
                dx_ahead = obs_along - along_t
                avoid_weight = self.avoidance_lateral_weight(dx_ahead)
                if avoid_weight > 0.0:
                    if pass_side == PASS_RIGHT:
                        safe_lat = obs_lat - R_SUM
                        acc_side += avoid_weight * max(0.0, lat_t - safe_lat)
                    elif pass_side == PASS_LEFT:
                        safe_lat = obs_lat + R_SUM
                        acc_side += avoid_weight * max(0.0, safe_lat - lat_t)

        target_speed = V_DES if not self.obs_detected else min(V_DES, 0.07)
        return (
            W_RISK * alpha * acc_risk / STEPS
            + W_LAT * acc_lat / STEPS
            + W_HEAD * acc_head / STEPS
            + W_END * abs(end_cte)
            + W_SIDE * acc_side / STEPS
            + W_SPEED * (target_speed - v) ** 2
            + W_TURN * w ** 2
            + W_TURN_CHANGE * (w - self.last_cmd_angular) ** 2
        )

    def target_lateral(
        self,
        x: float,
        y: float,
        pass_side: int | None = None,
        ox: float | None = None,
        oy: float | None = None,
    ) -> float:
        if not self.obs_detected:
            return 0.0
        if pass_side is None:
            pass_side = self.last_pass_side
        if pass_side == PASS_NONE:
            return 0.0

        robot_along, _ = self.path.along_lateral(x, y)
        if ox is None or oy is None:
            ox, oy = self.predicted_obstacle_pose(0.0)
        obs_along, obs_lat = self.path.along_lateral(ox, oy)
        dx_ahead = obs_along - robot_along

        if abs(obs_lat) > LANE_HALF_WIDTH:
            return 0.0

        weight = self.avoidance_lateral_weight(dx_ahead)
        if weight <= 0.0:
            return 0.0

        target = clamp(obs_lat + pass_side_offset(pass_side), LANE_RIGHT_LIMIT, LANE_LEFT_LIMIT)
        return weight * target

    def avoidance_lateral_weight(self, dx_ahead: float) -> float:
        if dx_ahead > AVOID_START_AHEAD or dx_ahead < -X_CLEAR_AHEAD:
            return 0.0
        if dx_ahead >= AVOID_FULL_AHEAD:
            return 1.0 - smoothstep(AVOID_FULL_AHEAD, AVOID_START_AHEAD, dx_ahead)
        if dx_ahead >= AVOID_RETURN_START_AHEAD:
            return 1.0
        return smoothstep(-X_CLEAR_AHEAD, AVOID_RETURN_START_AHEAD, dx_ahead)

    def expire_obstacle_if_needed(self):
        if not self.obs_detected or self.obs_last_seen is None:
            return

        age = (self.get_clock().now() - self.obs_last_seen).nanoseconds / 1e9
        if age > self.obs_hold_sec:
            self.clear_obstacle(f"not seen for {age:.2f}s")
            return

        if self.path is not None:
            robot_along, _ = self.path.along_lateral(self.x, self.y)
            obs_x, obs_y = self.predicted_obstacle_pose(0.0)
            obs_along, _ = self.path.along_lateral(obs_x, obs_y)
            if obs_along - robot_along < -X_CLEAR_AHEAD:
                self.clear_obstacle("passed")

                                                              
                          
                                                              

    def publish_cmd(self, linear: float, angular: float):
        if abs(linear) < 1e-6 and abs(angular) < 1e-6:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.last_cmd_linear + clamp(
                linear - self.last_cmd_linear,
                -V_CMD_RATE * DT,
                V_CMD_RATE * DT,
            )
            angular = self.last_cmd_angular + clamp(
                angular - self.last_cmd_angular,
                -W_CMD_RATE * DT,
                W_CMD_RATE * DT,
            )

        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.last_cmd_linear = cmd.linear.x
        self.last_cmd_angular = cmd.angular.z
        self.cmd_pub.publish(cmd)

    def log_scan_debug(self, msg: LaserScan):
        stats = self.last_scan_stats
        self.get_logger().info(
            "[SCAN_DEBUG] "
            f"msg#{self.scan_count} rays={stats.get('rays', 0)} "
            f"finite={stats.get('finite_count', 0)} "
            f"positive={stats.get('positive_count', 0)} | "
            f"angle=[{stats.get('angle_min_deg', 0.0):.1f},"
            f"{stats.get('angle_max_deg', 0.0):.1f}] deg "
            f"range=[{stats.get('range_min', msg.range_min):.2f},"
            f"{stats.get('range_max', msg.range_max):.2f}] | "
            f"min_all={fmt_range(stats.get('min_all', math.inf))} "
            f"min_front_cone={fmt_range(stats.get('min_front_cone', math.inf))} "
            f"min_lookahead={fmt_range(stats.get('min_within_lookahead', math.inf))} | "
            f"counts front_cone={stats.get('front_cone_count', 0)} "
            f"within_lookahead={stats.get('within_lookahead_count', 0)} "
            f"centered_front={stats.get('centered_front_count', 0)} "
            f"selected={stats.get('selected_count', 0)} "
            f"clusters={stats.get('cluster_count', 0)} "
            f"best_cluster={stats.get('best_cluster_points', 0)} | "
            f"closest_selected={fmt_range(self.closest_front)}",
            throttle_duration_sec=SCAN_DEBUG_THROTTLE_SEC,
        )

    def log_state_debug(self):
        along, lateral = self.path.along_lateral(self.x, self.y)
        heading_error = self.path.heading_error(self.yaw)
        self.last_target_lat = self.target_lateral(self.x, self.y)

        if self.obs_detected:
            obs_x, obs_y = self.predicted_obstacle_pose(0.0)
            obs_along, obs_lat = self.path.along_lateral(obs_x, obs_y)
            obs_text = (
                f"{self.obs_source}@({obs_x:.2f},{obs_y:.2f}) "
                f"dx={obs_along - along:.2f} lat={obs_lat:.2f} "
                f"v=({self.obs_vx:.2f},{self.obs_vy:.2f}) "
                f"side={pass_side_name(self.last_pass_side)}"
            )
        else:
            obs_text = "none"

        self.get_logger().info(
            f"[DWA] pos=({self.x:.2f},{self.y:.2f}) "
            f"along={along:.2f} lat={lateral:.2f} "
            f"target_lat={self.last_target_lat:.2f} "
            f"heading_err={math.degrees(heading_error):.1f} deg | "
            f"obs={obs_text} | risk={self.last_risk:.4f} alpha={self.last_alpha:.2f} "
            f"cost={self.last_best_cost:.3f} | "
            f"cmd=({self.last_cmd_linear:.2f}, {self.last_cmd_angular:.2f})",
            throttle_duration_sec=STATE_DEBUG_THROTTLE_SEC,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CRPFDWABurgerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
