#!/usr/bin/env python3
"""
Two moving SVs with one moving OV.

`tb3_0` is the leader and `tb3_1` is the follower. A third Burger acts as a
slow moving obstacle that drives straight in the main lane.

This file is intentionally self-contained for the moving-OV scenario: all
parameters and detection constants are defined here rather than imported from
the stationary-OV case.
"""

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from paper_implementation.collision_node_onesv_onemoving_ov import (
    PathFrame,
    crpf_value,
    gate_alpha,
)


DT = 0.10

LEADER_ID = 0
PLATOON_SIZE = 2

LANE_MAIN = 0.0
LANE_LEFT = 0.45
LANE_RIGHT = -0.45

V_MAX = 0.24
LEADER_CRUISE_SPEED = 0.18
HEADWAY_DES = 1.0
HEADWAY_KP = 1.1
MAX_ACCEL = 0.30
MAX_DECEL = 0.40

LOOKAHEAD_X = 0.55
STEER_KP = 2.60
MAX_TURN_RATE = 1.50
FOLLOWER_PATH_PREVIEW = 0.30

LANE_CHANGE_DURATION = 2.2
AVOID_TRIGGER_DIST = 1.4
CLEAR_DIST_BEHIND = 0.90

FRONT_FOV_DEG = 70.0
FRONT_FOV_RAD = math.radians(FRONT_FOV_DEG)
MIN_SCAN_RANGE = 0.12
MAX_SCAN_RANGE = 3.50
PLATOON_FILTER_RADIUS = 0.32
OBSTACLE_MAIN_LANE_TOL = 0.20
AVOID_RETURN_START_AHEAD = -0.25
LEADER_PATH_SAMPLE_DIST = 0.02
LEADER_PATH_MAXLEN = 4000

DEFAULT_OV_ODOM_TOPIC = "/tb3_2/odom"
DEFAULT_OV_LINEAR_SPEED = 0.08
DEFAULT_OBS_SPEED_MARGIN = 0.05
DEFAULT_FOLLOWER_OBS_BUFFER = 0.35
DEFAULT_OV_PREDICTION_TIME = 1.20
DEFAULT_TAIL_CLEAR_MARGIN = 0.25
DEFAULT_LEADER_CLEAR_MARGIN = 0.45
DEFAULT_MIN_REL_SPEED = 0.01
DEFAULT_STARTUP_GAP_TOL = 0.08
DEFAULT_STARTUP_CREEP_SPEED = 0.08
DEFAULT_SCAN_SPEED_CLAMP = 0.20
DEFAULT_SCAN_SPEED_SMOOTHING = 0.40


BURGER_RADIUS = 0.14
R_SUM = (2.0 * BURGER_RADIUS) + 0.04
PATH_LATERAL_LIMIT = 0.80

V_MIN = 0.2
V_CRUISE_MIN = 0.2
V_AVOID_MIN = 0.04
V_AVOID_MAX = 0.22

DWA_STEPS = 20
DWA_W_MAX = 1.40
DWA_W_SAMPLES = 21
V_CMD_RATE = 0.22
W_CMD_RATE = 1.60

W_RISK = 1.00
W_LAT = 1.05
W_HEAD = 0.22
W_END = 1.10
W_SPEED = 0.16
W_TURN = 0.18
W_TURN_CHANGE = 0.20
W_SIDE = 1.10
W_GAP = 0.70

FOLLOWER_LAT_MULT = 1.45
FOLLOWER_HEAD_MULT = 1.25
FOLLOWER_GAP_MULT = 1.90
FOLLOWER_SPEED_MULT = 0.85

MIN_FOLLOW_GAP = 0.24


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def smoothstep_quintic(u: float) -> float:
    u = clamp(u, 0.0, 1.0)
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def smoothstep(edge0: float, edge1: float, value: float) -> float:
    if abs(edge1 - edge0) < 1e-9:
        return 1.0 if value >= edge1 else 0.0
    return smoothstep_quintic((value - edge0) / (edge1 - edge0))


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rate_limit(current: float, target: float, rise_step: float, fall_step: float) -> float:
    if target > current:
        return min(current + rise_step, target)
    return max(current - fall_step, target)


class TwoSVMovingOVNode(Node):
    def __init__(self):
        super().__init__("two_sv_one_moving_ov")

        self.ns = self.get_namespace().strip("/")
        inferred_bot_id = int(self.ns.split("_")[-1]) if self.ns.startswith("tb3_") else 0
        self.bot_id = int(self.declare_parameter("bot_id", inferred_bot_id).value)
        self.is_leader = self.bot_id == LEADER_ID
        self.preced_id = self.bot_id - 1 if self.bot_id > LEADER_ID else -1

        self.scan_topic = str(self.declare_parameter("scan_topic", "scan").value)
        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "cmd_vel").value)
        self.max_scan_range = float(
            self.declare_parameter("max_scan_range", MAX_SCAN_RANGE).value
        )
        self.odom_topics = {
            0: str(self.declare_parameter("sv0_odom_topic", "/tb3_0/odom").value),
            1: str(self.declare_parameter("sv1_odom_topic", "/tb3_1/odom").value),
        }
        self.ov_odom_topic = str(
            self.declare_parameter("ov_odom_topic", DEFAULT_OV_ODOM_TOPIC).value
        )

        self.use_initial_pose_offsets = bool(
            self.declare_parameter("use_initial_pose_offsets", False).value
        )
        self.initial_positions = {
            0: (
                float(self.declare_parameter("sv0_initial_x", 0.0).value),
                float(self.declare_parameter("sv0_initial_y", 0.0).value),
            ),
            1: (
                float(self.declare_parameter("sv1_initial_x", -0.5).value),
                float(self.declare_parameter("sv1_initial_y", 0.0).value),
            ),
        }
        self.ov_initial_x = float(self.declare_parameter("ov_initial_x", 3.0).value)
        self.ov_initial_y = float(self.declare_parameter("ov_initial_y", 0.0).value)

        self.enable_debug_logs = bool(
            self.declare_parameter("enable_debug_logs", True).value
        )
        self.debug_interval_sec = float(
            self.declare_parameter("debug_interval_sec", 1.0).value
        )
        self.wait_warn_interval_sec = float(
            self.declare_parameter("wait_warn_interval_sec", 2.0).value
        )

        self.leader_speed = float(
            self.declare_parameter("leader_speed", LEADER_CRUISE_SPEED).value
        )
        self.desired_headway = float(
            self.declare_parameter("desired_headway", HEADWAY_DES).value
        )
        self.headway_gain = float(
            self.declare_parameter("headway_gain", HEADWAY_KP).value
        )
        self.follower_path_preview = float(
            self.declare_parameter(
                "follower_path_preview", FOLLOWER_PATH_PREVIEW
            ).value
        )
        self.lane_change_duration = float(
            self.declare_parameter(
                "lane_change_duration", LANE_CHANGE_DURATION
            ).value
        )
        self.avoid_trigger_distance = float(
            self.declare_parameter(
                "avoid_trigger_distance", AVOID_TRIGGER_DIST
            ).value
        )
        self.clear_distance_behind = float(
            self.declare_parameter("clear_distance_behind", CLEAR_DIST_BEHIND).value
        )
        self.avoid_full_ahead = float(
            self.declare_parameter(
                "avoid_full_ahead",
                min(0.9, 0.5 * self.avoid_trigger_distance),
            ).value
        )
        self.avoid_return_start_ahead = float(
            self.declare_parameter(
                "avoid_return_start_ahead", AVOID_RETURN_START_AHEAD
            ).value
        )

        self.nominal_obstacle_speed = float(
            self.declare_parameter("nominal_obstacle_speed", DEFAULT_OV_LINEAR_SPEED).value
        )
        self.obstacle_speed_margin = float(
            self.declare_parameter("obstacle_speed_margin", DEFAULT_OBS_SPEED_MARGIN).value
        )
        self.follower_obstacle_buffer = float(
            self.declare_parameter(
                "follower_obstacle_buffer", DEFAULT_FOLLOWER_OBS_BUFFER
            ).value
        )
        self.ov_prediction_time = float(
            self.declare_parameter(
                "ov_prediction_time", DEFAULT_OV_PREDICTION_TIME
            ).value
        )
        self.return_prediction_time = float(
            self.declare_parameter(
                "return_prediction_time", self.ov_prediction_time
            ).value
        )
        self.tail_clear_margin = float(
            self.declare_parameter("tail_clear_margin", DEFAULT_TAIL_CLEAR_MARGIN).value
        )
        self.leader_clear_margin = float(
            self.declare_parameter(
                "leader_clear_margin", DEFAULT_LEADER_CLEAR_MARGIN
            ).value
        )
        self.min_relative_speed = float(
            self.declare_parameter("min_relative_speed", DEFAULT_MIN_REL_SPEED).value
        )
        self.startup_gap_tolerance = float(
            self.declare_parameter(
                "startup_gap_tolerance", DEFAULT_STARTUP_GAP_TOL
            ).value
        )
        self.startup_creep_speed = float(
            self.declare_parameter(
                "startup_creep_speed", DEFAULT_STARTUP_CREEP_SPEED
            ).value
        )
        self.scan_speed_clamp = float(
            self.declare_parameter("scan_speed_clamp", DEFAULT_SCAN_SPEED_CLAMP).value
        )
        self.scan_speed_smoothing = float(
            self.declare_parameter(
                "scan_speed_smoothing", DEFAULT_SCAN_SPEED_SMOOTHING
            ).value
        )

        preferred_side = str(
            self.declare_parameter("preferred_pass_side", "right").value
        ).strip().lower()
        self.preferred_pass_lane = "LEFT" if preferred_side == "left" else "RIGHT"

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.odom_subs = []
        self.SVs = {idx: None for idx in range(PLATOON_SIZE)}
        self.odom_origins = {idx: None for idx in range(PLATOON_SIZE)}
        self.odom_counts = {idx: 0 for idx in range(PLATOON_SIZE)}
        for idx in range(PLATOON_SIZE):
            sub = self.create_subscription(
                Odometry,
                self.odom_topics[idx],
                lambda msg, bot_idx=idx: self.odom_callback(msg, bot_idx),
                10,
            )
            self.odom_subs.append(sub)

        self.scan_sub = None
        if self.is_leader:
            self.scan_sub = self.create_subscription(
                LaserScan,
                self.scan_topic,
                self.scan_callback,
                qos_profile_sensor_data,
            )

        self.ov_odom_sub = self.create_subscription(
            Odometry,
            self.ov_odom_topic,
            self.ov_odom_callback,
            10,
        )

        self.v_cmd = 0.0

        self.lane_cur = "MAIN"
        self.lane_from = "MAIN"
        self.lane_to = "MAIN"
        self.lc_active = False
        self.lc_t = 0.0

        self.ov_x = self.ov_initial_x
        self.ov_y = self.ov_initial_y
        self.ov_origin = None
        self.ov_odom_count = 0
        self.ov_state = None

        self.obstacle_seen = False
        self.obstacle_visible_in_scan = False
        self.last_detected_range = math.inf
        self.active_avoidance = False
        self.active_obstacle_x = None
        self.active_obstacle_y = None
        self.avoidance_mode = "CRUISE"
        self.scan_count = 0

        self.scan_obs_x = math.inf
        self.scan_obs_y = math.inf
        self.scan_obs_vx = 0.0
        self.scan_obs_vy = 0.0
        self.last_scan_obs_time_sec = None

        self.last_log_times = {}
        self.path_frame = None
        self.leader_path_history = deque(maxlen=LEADER_PATH_MAXLEN)
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.last_best_cost = 0.0
        self.last_risk = 0.0
        self.last_alpha = 0.0

        role = "leader" if self.is_leader else "follower"
        self.get_logger().info(
            f"{self.ns or 'tb3_0'} ready as {role} | "
            f"bot_id={self.bot_id} | "
            f"headway={self.desired_headway:.2f} m | "
            f"pass_side={self.preferred_pass_lane.lower()} | "
            f"scan={self.scan_topic}<= {self.max_scan_range:.2f} m "
            f"cmd={self.cmd_vel_topic} | "
            f"sv0_odom={self.odom_topics[0]} sv1_odom={self.odom_topics[1]} "
            f"ov_odom={self.ov_odom_topic}"
        )

        self.timer = self.create_timer(DT, self.control_loop)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def should_log(self, key: str, interval_sec: float) -> bool:
        if not self.enable_debug_logs:
            return False

        now = self.now_sec()
        last = self.last_log_times.get(key)
        if last is None or (now - last) >= interval_sec:
            self.last_log_times[key] = now
            return True
        return False

    def odom_callback(self, msg: Odometry, bot_idx: int):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        self.odom_counts[bot_idx] += 1

        if self.use_initial_pose_offsets:
            if self.odom_origins[bot_idx] is None:
                self.odom_origins[bot_idx] = (raw_x, raw_y)
                self.get_logger().info(
                    f"Captured odom origin for sv{bot_idx}: "
                    f"({raw_x:.2f}, {raw_y:.2f}) -> "
                    f"global seed {self.initial_positions[bot_idx]}"
                )

            origin_x, origin_y = self.odom_origins[bot_idx]
            seed_x, seed_y = self.initial_positions[bot_idx]
            px = seed_x + (raw_x - origin_x)
            py = seed_y + (raw_y - origin_y)
        else:
            px = raw_x
            py = raw_y

        self.SVs[bot_idx] = {
            "x": px,
            "y": py,
            "v": msg.twist.twist.linear.x,
            "yaw": yaw_from_odom(msg),
        }

        if bot_idx == LEADER_ID:
            if self.path_frame is None:
                self.path_frame = PathFrame(px, py, yaw_from_odom(msg))
                self.get_logger().info(
                    f"Locked shared path frame at ({px:.2f}, {py:.2f}) "
                    f"yaw={math.degrees(self.path_frame.yaw0):.1f} deg"
                )
            self.record_leader_path(px, py)

        if self.enable_debug_logs and self.odom_counts[bot_idx] == 1:
            self.get_logger().info(
                f"First odom for sv{bot_idx} on {self.odom_topics[bot_idx]} | "
                f"raw=({raw_x:.2f}, {raw_y:.2f}) mapped=({px:.2f}, {py:.2f})"
            )

    def record_leader_path(self, x: float, y: float):
        if self.path_frame is None:
            return

        along, _ = self.path_frame.along_lateral(x, y)
        if self.leader_path_history:
            last_along, _, _ = self.leader_path_history[-1]
            if abs(along - last_along) < LEADER_PATH_SAMPLE_DIST:
                return
        self.leader_path_history.append((along, x, y))

    def leader_path_target_point(self, query_along: float) -> tuple[float, float]:
        if not self.leader_path_history:
            return query_along, LANE_MAIN

        first_along, first_x, first_y = self.leader_path_history[0]
        if query_along <= first_along:
            return first_x, first_y

        last_along, last_x, last_y = self.leader_path_history[-1]
        if query_along >= last_along:
            return last_x, last_y

        prev_along, prev_x, prev_y = self.leader_path_history[0]
        for curr_along, curr_x, curr_y in self.leader_path_history:
            if curr_along >= query_along:
                dalong = curr_along - prev_along
                if abs(dalong) < 1e-6:
                    return curr_x, curr_y
                ratio = clamp((query_along - prev_along) / dalong, 0.0, 1.0)
                interp_x = ((1.0 - ratio) * prev_x) + (ratio * curr_x)
                interp_y = ((1.0 - ratio) * prev_y) + (ratio * curr_y)
                return interp_x, interp_y
            prev_along, prev_x, prev_y = curr_along, curr_x, curr_y

        return last_x, last_y

    def is_platoon_point(self, px: float, py: float) -> bool:
        for idx, sv in self.SVs.items():
            if idx == self.bot_id or sv is None:
                continue
            if math.hypot(px - sv["x"], py - sv["y"]) < PLATOON_FILTER_RADIUS:
                return True
        return False

    def ov_odom_callback(self, msg: Odometry):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        self.ov_odom_count += 1

        if self.use_initial_pose_offsets:
            if self.ov_origin is None:
                self.ov_origin = (raw_x, raw_y)
                self.get_logger().info(
                    "Captured odom origin for moving OV: "
                    f"({raw_x:.2f}, {raw_y:.2f}) -> "
                    f"global seed ({self.ov_initial_x:.2f}, {self.ov_initial_y:.2f})"
                )

            origin_x, origin_y = self.ov_origin
            px = self.ov_initial_x + (raw_x - origin_x)
            py = self.ov_initial_y + (raw_y - origin_y)
        else:
            px = raw_x
            py = raw_y

        self.ov_state = {
            "x": px,
            "y": py,
            "v": msg.twist.twist.linear.x,
            "yaw": yaw_from_odom(msg),
        }
        self.obstacle_seen = True

        if self.enable_debug_logs and self.ov_odom_count == 1:
            self.get_logger().info(
                f"First moving-OV odom on {self.ov_odom_topic} | "
                f"raw=({raw_x:.2f}, {raw_y:.2f}) mapped=({px:.2f}, {py:.2f}) "
                f"v={self.ov_state['v']:.2f}"
            )

    def scan_callback(self, msg: LaserScan):
        if self.SVs[self.bot_id] is None:
            return

        self.scan_count += 1

        bot = self.SVs[self.bot_id]
        rx = bot["x"]
        ry = bot["y"]
        ryaw = bot["yaw"]

        best_range = float("inf")
        best_x = self.ov_x
        best_y = self.ov_y

        for idx, rng in enumerate(msg.ranges):
            if not math.isfinite(rng):
                continue
            if rng < MIN_SCAN_RANGE or rng > self.max_scan_range:
                continue

            angle = msg.angle_min + idx * msg.angle_increment
            if abs(wrap_angle(angle)) > (FRONT_FOV_RAD / 2.0):
                continue

            lx = rng * math.cos(angle)
            ly = rng * math.sin(angle)
            px = rx + (lx * math.cos(ryaw) - ly * math.sin(ryaw))
            py = ry + (lx * math.sin(ryaw) + ly * math.cos(ryaw))

            if self.is_platoon_point(px, py):
                continue

            if rng < best_range:
                best_range = rng
                best_x = px
                best_y = py

        if math.isfinite(best_range):
            prev_x = self.scan_obs_x
            prev_y = self.scan_obs_y
            prev_t = self.last_scan_obs_time_sec
            prev_visible = (
                self.obstacle_visible_in_scan
                and math.isfinite(prev_x)
                and math.isfinite(prev_y)
                and prev_t is not None
            )

            self.ov_x = best_x
            self.ov_y = best_y
            self.scan_obs_x = best_x
            self.scan_obs_y = best_y
            self.obstacle_seen = True
            self.obstacle_visible_in_scan = True
            self.last_detected_range = best_range

            now = self.now_sec()
            if prev_visible:
                dt = now - prev_t
                if dt > 1e-3:
                    raw_vx = (self.scan_obs_x - prev_x) / dt
                    raw_vy = (self.scan_obs_y - prev_y) / dt
                    raw_speed = math.hypot(raw_vx, raw_vy)
                    max_scan_speed = max(
                        self.scan_speed_clamp,
                        self.nominal_obstacle_speed + 0.10,
                    )
                    if raw_speed > max_scan_speed:
                        scale = max_scan_speed / raw_speed
                        raw_vx *= scale
                        raw_vy *= scale
                    beta = clamp(self.scan_speed_smoothing, 0.0, 1.0)
                    self.scan_obs_vx = (1.0 - beta) * self.scan_obs_vx + beta * raw_vx
                    self.scan_obs_vy = (1.0 - beta) * self.scan_obs_vy + beta * raw_vy

            self.last_scan_obs_time_sec = now

            if self.enable_debug_logs and (
                self.scan_count == 1 or self.should_log("obstacle_track", self.debug_interval_sec)
            ):
                self.get_logger().info(
                    f"Obstacle track | range={best_range:.2f} m "
                    f"odom=({self.ov_x:.2f}, {self.ov_y:.2f}) "
                    f"scan_v=({self.scan_obs_vx:.2f}, {self.scan_obs_vy:.2f})"
                )
        else:
            self.obstacle_visible_in_scan = False
            self.last_detected_range = math.inf
            if self.obstacle_seen and self.should_log(
                "obstacle_not_visible", self.wait_warn_interval_sec
            ):
                self.get_logger().warning(
                    "Obstacle not visible in the latest scan; keeping last known track."
                )

        if self.enable_debug_logs and self.scan_count == 1:
            self.get_logger().info(
                f"First scan received on {self.scan_topic} with {len(msg.ranges)} ranges"
            )

    def current_obstacle_pose(self) -> tuple[float | None, float | None, str]:
        if self.obstacle_visible_in_scan and math.isfinite(self.scan_obs_x) and math.isfinite(
            self.scan_obs_y
        ):
            return self.scan_obs_x, self.scan_obs_y, "scan"
        if self.ov_state is not None:
            return self.ov_state["x"], self.ov_state["y"], "odom"
        if self.active_obstacle_x is not None and self.active_obstacle_y is not None:
            return self.active_obstacle_x, self.active_obstacle_y, "latched"
        return None, None, "none"

    def project_along_lateral(self, x: float, y: float) -> tuple[float, float]:
        if self.path_frame is None:
            return x, y
        return self.path_frame.along_lateral(x, y)

    def path_heading_error(self, yaw: float) -> float:
        if self.path_frame is None:
            return wrap_angle(yaw)
        return self.path_frame.heading_error(yaw)

    def predict_pose(self, state, horizon_sec: float) -> tuple[float, float]:
        speed = max(0.0, state["v"])
        return (
            state["x"] + speed * math.cos(state["yaw"]) * horizon_sec,
            state["y"] + speed * math.sin(state["yaw"]) * horizon_sec,
        )

    def platoon_tail_along(self) -> float | None:
        alongs = [
            self.project_along_lateral(sv["x"], sv["y"])[0]
            for sv in self.SVs.values()
            if sv is not None
        ]
        if not alongs:
            return None
        return min(alongs)

    def obstacle_velocity(self) -> tuple[float, float]:
        odom_vx = 0.0
        odom_vy = 0.0
        if self.ov_state is not None:
            speed = max(0.0, self.ov_state["v"])
            odom_vx = speed * math.cos(self.ov_state["yaw"])
            odom_vy = speed * math.sin(self.ov_state["yaw"])

        if (
            self.obstacle_visible_in_scan
            and self.last_scan_obs_time_sec is not None
            and math.hypot(self.scan_obs_vx, self.scan_obs_vy) > 1e-3
        ):
            if math.hypot(self.scan_obs_vx, self.scan_obs_vy) >= math.hypot(odom_vx, odom_vy):
                return self.scan_obs_vx, self.scan_obs_vy

        if self.ov_state is not None:
            return (
                speed * math.cos(self.ov_state["yaw"]),
                speed * math.sin(self.ov_state["yaw"]),
            )
        heading = self.path_frame.yaw0 if self.path_frame is not None else 0.0
        return (
            self.nominal_obstacle_speed * math.cos(heading),
            self.nominal_obstacle_speed * math.sin(heading),
        )

    def obstacle_speed(self) -> float:
        obs_vx, obs_vy = self.obstacle_velocity()
        odom_speed = self.ov_state["v"] if self.ov_state is not None else 0.0
        return max(odom_speed, math.hypot(obs_vx, obs_vy), self.nominal_obstacle_speed)

    def predicted_obstacle_pose(self, horizon_sec: float = 0.0) -> tuple[float | None, float | None]:
        obs_x, obs_y, _ = self.current_obstacle_pose()
        if obs_x is None or obs_y is None:
            return None, None

        obs_vx, obs_vy = self.obstacle_velocity()
        return (
            obs_x + obs_vx * max(0.0, horizon_sec),
            obs_y + obs_vy * max(0.0, horizon_sec),
        )

    def predicted_ov_x(self, obs_x: float | None, horizon_sec: float | None = None) -> float | None:
        if obs_x is None:
            return None
        if horizon_sec is None:
            horizon_sec = self.ov_prediction_time
        obs_vx, _ = self.obstacle_velocity()
        return obs_x + obs_vx * max(0.0, horizon_sec)

    def lane_center_of(self, lane_label: str) -> float:
        if lane_label == "LEFT":
            return LANE_LEFT
        if lane_label == "RIGHT":
            return LANE_RIGHT
        return LANE_MAIN

    def avoidance_lateral_weight(self, dx_ahead: float) -> float:
        if dx_ahead > self.avoid_trigger_distance or dx_ahead < -self.clear_distance_behind:
            return 0.0
        if dx_ahead >= self.avoid_full_ahead:
            return 1.0 - smoothstep(
                self.avoid_full_ahead,
                self.avoid_trigger_distance,
                dx_ahead,
            )
        if dx_ahead >= self.avoid_return_start_ahead:
            return 1.0
        return smoothstep(
            -self.clear_distance_behind,
            self.avoid_return_start_ahead,
            dx_ahead,
        )

    def leader_target_y(self, my_state) -> float:
        obs_x, obs_y, obs_source = self.current_obstacle_pose()
        if obs_x is None or obs_y is None:
            self.lane_cur = "MAIN"
            self.lc_active = False
            return LANE_MAIN

        robot_along, robot_lat = self.project_along_lateral(my_state["x"], my_state["y"])
        obs_along, obs_lat = self.project_along_lateral(obs_x, obs_y)
        pred_obs_x, pred_obs_y = self.predicted_obstacle_pose(self.ov_prediction_time)
        if pred_obs_x is None or pred_obs_y is None:
            pred_obs_along = obs_along
        else:
            pred_obs_along, _ = self.project_along_lateral(pred_obs_x, pred_obs_y)

        dx_leader = pred_obs_along - robot_along
        rel_speed = my_state["v"] - self.obstacle_speed()
        obstacle_in_main_lane = abs(obs_lat - LANE_MAIN) < OBSTACLE_MAIN_LANE_TOL

        if (
            not self.active_avoidance
            and obs_source in {"scan", "odom"}
            and obstacle_in_main_lane
            and rel_speed > self.min_relative_speed
            and 0.0 < dx_leader < self.avoid_trigger_distance
        ):
            self.active_avoidance = True
            self.active_obstacle_x = obs_x
            self.active_obstacle_y = obs_y
            self.avoidance_mode = "OVERTAKE"
            self.get_logger().info(
                f"Leader locked moving obstacle at ({self.active_obstacle_x:.2f}, "
                f"{self.active_obstacle_y:.2f}) with v={self.obstacle_speed():.2f} "
                "and is starting avoidance."
            )

        if not self.active_avoidance:
            self.active_obstacle_x = None
            self.active_obstacle_y = None
            self.lane_cur = "MAIN"
            self.lc_active = False
            return LANE_MAIN

        self.active_obstacle_x = obs_x
        self.active_obstacle_y = obs_y

        reference_along = self.platoon_tail_along()
        if reference_along is None:
            reference_along = robot_along

        clear_obs_x, clear_obs_y = self.predicted_obstacle_pose(self.return_prediction_time)
        if clear_obs_x is None or clear_obs_y is None:
            clear_obs_along = obs_along
        else:
            clear_obs_along, _ = self.project_along_lateral(clear_obs_x, clear_obs_y)

        dx_leader_ahead = pred_obs_along - robot_along
        dx_tail_ahead = pred_obs_along - reference_along
        target_lane_y = self.lane_center_of(self.preferred_pass_lane)
        leader_clear = robot_along > (clear_obs_along + self.leader_clear_margin)
        tail_clear = reference_along > (clear_obs_along + self.tail_clear_margin)

        if not leader_clear:
            pass_weight = self.avoidance_lateral_weight(dx_leader_ahead)
        elif not tail_clear:
            pass_weight = 1.0
        else:
            pass_weight = self.avoidance_lateral_weight(dx_tail_ahead)

        target_y = pass_weight * target_lane_y

        if abs(target_y) > 0.02:
            self.lane_cur = self.preferred_pass_lane
            self.lc_active = True
        else:
            self.lane_cur = "MAIN"
            self.lc_active = False

        if (
            leader_clear
            and tail_clear
            and abs(target_y) < 0.02
            and abs(robot_lat - LANE_MAIN) < 0.05
        ):
            self.get_logger().info(
                "Leader returned to MAIN after overtaking moving obstacle at "
                f"along={obs_along:.2f}"
            )
            self.active_avoidance = False
            self.active_obstacle_x = None
            self.active_obstacle_y = None
            self.avoidance_mode = "CRUISE"
            return LANE_MAIN

        return target_y

    def leader_target_speed(self) -> float:
        if not self.active_avoidance:
            return clamp(self.leader_speed, 0.0, V_MAX)
        return clamp(
            max(self.leader_speed, self.obstacle_speed() + self.obstacle_speed_margin),
            V_AVOID_MIN,
            V_MAX,
        )

    def follower_target_point(self, my_state) -> tuple[float, float]:
        if len(self.leader_path_history) < 2:
            leader_state = self.SVs[self.preced_id]
            fallback_x = my_state["x"] + LOOKAHEAD_X
            fallback_y = leader_state["y"] if leader_state is not None else my_state["y"]
            return fallback_x, fallback_y

        my_along, _ = self.project_along_lateral(my_state["x"], my_state["y"])
        preview_along = my_along + self.follower_path_preview
        target_x, target_y = self.leader_path_target_point(preview_along)
        return (
            max(target_x, my_state["x"] + 0.05),
            target_y,
        )

    def follower_target_lateral(self, my_state) -> float:
        target_x, target_y = self.follower_target_point(my_state)
        _, target_lat = self.project_along_lateral(target_x, target_y)
        return clamp(target_lat, LANE_RIGHT, LANE_LEFT)

    def follower_target_speed(self, my_state, target_y: float) -> float:
        leader_state = self.SVs[self.preced_id]
        if leader_state is None:
            return 0.0

        gap = leader_state["x"] - my_state["x"]
        gap_error = gap - self.desired_headway
        target_speed = leader_state["v"] + self.headway_gain * gap_error

        if gap < 0.25:
            target_speed = 0.0
        elif gap < self.desired_headway:
            target_speed = min(target_speed, leader_state["v"])

        lateral_error = abs(target_y - my_state["y"])
        if lateral_error > 0.08:
            target_speed = min(target_speed, leader_state["v"] + 0.02)
        if lateral_error > 0.18:
            target_speed = min(target_speed, 0.10)

        if (
            leader_state["v"] > 0.03
            and gap > (self.desired_headway - self.startup_gap_tolerance)
        ):
            target_speed = max(
                target_speed,
                min(self.startup_creep_speed, leader_state["v"]),
            )

        obs_x, obs_y, _ = self.current_obstacle_pose()
        if obs_x is not None and obs_y is not None:
            dx_ov = obs_x - my_state["x"]
            obstacle_in_main_lane = abs(obs_y - LANE_MAIN) < OBSTACLE_MAIN_LANE_TOL
            bypassing_obstacle = (
                abs(target_y) > 0.08 or abs(my_state["y"] - LANE_MAIN) > 0.08
            )
            if obstacle_in_main_lane and dx_ov > 0.0 and not bypassing_obstacle:
                buffer_dist = max(0.60, self.desired_headway + self.follower_obstacle_buffer)
                if dx_ov < buffer_dist:
                    target_speed = min(
                        target_speed,
                        self.obstacle_speed() + self.obstacle_speed_margin,
                    )
                if dx_ov < 0.30:
                    target_speed = min(target_speed, self.obstacle_speed())

        return clamp(target_speed, 0.0, V_MAX)

    def velocity_samples(self, current_speed: float, target_speed: float, obstacle_active: bool) -> np.ndarray:
        dyn_low = clamp(current_speed - MAX_DECEL * DT, V_MIN, V_MAX)
        dyn_high = clamp(current_speed + MAX_ACCEL * DT, V_MIN, V_MAX)
        dynamic = np.linspace(dyn_low, dyn_high, 5)

        if obstacle_active:
            practical = np.linspace(V_AVOID_MIN, min(V_AVOID_MAX, V_MAX), 6)
        else:
            practical = np.linspace(V_CRUISE_MIN, V_MAX, 6)

        anchor = np.array([clamp(target_speed, V_MIN, V_MAX)], dtype=float)
        samples = np.unique(np.round(np.concatenate((dynamic, practical, anchor)), 4))
        return samples[(samples >= V_MIN) & (samples <= V_MAX)]

    def choose_dwa_command(
        self,
        my_state,
        role: str,
        target_lat: float,
        target_speed: float,
    ) -> tuple[float, float, float]:
        obs_x, obs_y = self.predicted_obstacle_pose(0.0)
        obs_vx, obs_vy = self.obstacle_velocity()
        obstacle_active = obs_x is not None and obs_y is not None
        obs_speed = math.hypot(obs_vx, obs_vy) if obstacle_active else 0.0

        if obstacle_active:
            risk_now = crpf_value(my_state["x"], my_state["y"], obs_x, obs_y, obs_speed)
            alpha = gate_alpha(risk_now)
        else:
            risk_now = 0.0
            alpha = 0.0

        self.last_risk = risk_now
        self.last_alpha = alpha

        v_samples = self.velocity_samples(my_state["v"], target_speed, obstacle_active)
        w_samples = np.linspace(-DWA_W_MAX, DWA_W_MAX, DWA_W_SAMPLES)

        leader_state = self.SVs[self.preced_id] if role == "follower" else None
        best_cost = math.inf
        best_v = 0.0
        best_w = 0.0

        for v_next in v_samples:
            for w_next in w_samples:
                cost = self.rollout_cost(
                    my_state,
                    float(v_next),
                    float(w_next),
                    target_lat,
                    target_speed,
                    obs_x,
                    obs_y,
                    obs_vx,
                    obs_vy,
                    obs_speed,
                    alpha,
                    leader_state,
                )
                if cost < best_cost:
                    best_cost = cost
                    best_v = float(v_next)
                    best_w = float(w_next)

        if not math.isfinite(best_cost):
            _, current_lat = self.project_along_lateral(my_state["x"], my_state["y"])
            lat_error = target_lat - current_lat
            if obstacle_active and abs(lat_error) > 0.04:
                escape_v = V_AVOID_MIN
                escape_w = clamp(2.5 * lat_error, -DWA_W_MAX, DWA_W_MAX)
                self.get_logger().warning(
                    f"{role} DWA deadlock near OV; using low-speed escape turn "
                    f"(lat_error={lat_error:.2f}, cmd=({escape_v:.2f}, {escape_w:.2f}))",
                    throttle_duration_sec=0.5,
                )
                return escape_v, escape_w, math.inf
            return 0.0, 0.0, math.inf

        return best_v, best_w, best_cost

    def rollout_cost(
        self,
        my_state,
        v: float,
        w: float,
        target_lat: float,
        target_speed: float,
        obs_x: float | None,
        obs_y: float | None,
        obs_vx: float,
        obs_vy: float,
        obs_speed: float,
        alpha: float,
        leader_state=None,
    ) -> float:
        x_t = my_state["x"]
        y_t = my_state["y"]
        yaw_t = my_state["yaw"]

        acc_risk = 0.0
        acc_lat = 0.0
        acc_head = 0.0
        acc_side = 0.0
        acc_gap = 0.0
        end_cte = 0.0

        for step in range(DWA_STEPS):
            yaw_t = wrap_angle(yaw_t + w * DT)
            x_t += v * math.cos(yaw_t) * DT
            y_t += v * math.sin(yaw_t) * DT

            along_t, lat_t = self.project_along_lateral(x_t, y_t)
            if abs(lat_t) > PATH_LATERAL_LIMIT:
                return math.inf

            cte = lat_t - target_lat
            head_err = self.path_heading_error(yaw_t)
            acc_lat += abs(cte)
            acc_head += abs(head_err)
            end_cte = cte

            horizon = (step + 1) * DT

            if obs_x is not None and obs_y is not None:
                ox_t = obs_x + obs_vx * horizon
                oy_t = obs_y + obs_vy * horizon
                if math.hypot(x_t - ox_t, y_t - oy_t) <= R_SUM:
                    return math.inf

                acc_risk += crpf_value(x_t, y_t, ox_t, oy_t, obs_speed)
                obs_along, obs_lat = self.project_along_lateral(ox_t, oy_t)
                dx_ahead = obs_along - along_t
                avoid_weight = self.avoidance_lateral_weight(dx_ahead)
                if avoid_weight > 0.0 and abs(target_lat) > 0.02:
                    if target_lat < 0.0:
                        safe_lat = obs_lat - R_SUM
                        acc_side += avoid_weight * max(0.0, lat_t - safe_lat)
                    else:
                        safe_lat = obs_lat + R_SUM
                        acc_side += avoid_weight * max(0.0, safe_lat - lat_t)

            if leader_state is not None:
                leader_x_t, leader_y_t = self.predict_pose(leader_state, horizon)
                leader_along, _ = self.project_along_lateral(leader_x_t, leader_y_t)
                gap = leader_along - along_t
                if gap <= MIN_FOLLOW_GAP:
                    return math.inf
                acc_gap += abs(gap - self.desired_headway)

        return (
            (W_RISK * alpha * acc_risk / DWA_STEPS)
            + (W_LAT * acc_lat / DWA_STEPS)
            + (W_HEAD * acc_head / DWA_STEPS)
            + (W_END * abs(end_cte))
            + (W_SIDE * acc_side / DWA_STEPS)
            + (W_GAP * acc_gap / DWA_STEPS)
            + (W_SPEED * (target_speed - v) ** 2)
            + (W_TURN * w**2)
            + (W_TURN_CHANGE * (w - self.last_cmd_angular) ** 2)
        )

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

    def control_loop(self):
        my_state = self.SVs.get(self.bot_id)
        if my_state is None:
            if self.should_log("wait_self_odom", self.wait_warn_interval_sec):
                self.get_logger().warning(
                    f"Waiting for own odom on {self.odom_topics[self.bot_id]}"
                )
            self.cmd_pub.publish(Twist())
            return

        if self.use_initial_pose_offsets and self.odom_origins[self.bot_id] is None:
            self.cmd_pub.publish(Twist())
            return

        if self.is_leader and self.scan_count == 0:
            if self.should_log("wait_scan", self.wait_warn_interval_sec):
                self.get_logger().warning(
                    f"No scans received yet on {self.scan_topic}; "
                    "continuing in cruise mode until leader scan is available."
                )

        if self.ov_state is None and not self.obstacle_visible_in_scan:
            if self.should_log("wait_ov_track", self.wait_warn_interval_sec):
                self.get_logger().warning(
                    f"Waiting for moving OV tracking via scan or odom "
                    f"({self.scan_topic} / {self.ov_odom_topic})."
                )

        if not self.is_leader and self.SVs[self.preced_id] is None:
            if self.should_log("wait_preced_odom", self.wait_warn_interval_sec):
                self.get_logger().warning(
                    f"Waiting for leader odom on {self.odom_topics[self.preced_id]}"
                )
            self.cmd_pub.publish(Twist())
            return

        if self.is_leader:
            target_y = self.leader_target_y(my_state)
            target_speed = self.leader_target_speed()
            cmd_v, cmd_w, best_cost = self.choose_dwa_command(
                my_state,
                "leader",
                target_y,
                target_speed,
            )
            target_x = my_state["x"] + LOOKAHEAD_X
        else:
            target_x, target_y = self.follower_target_point(my_state)
            target_speed = self.follower_target_speed(my_state, target_y)
            target_lat = self.follower_target_lateral(my_state)
            cmd_v, cmd_w, best_cost = self.choose_dwa_command(
                my_state,
                "follower",
                target_lat,
                target_speed,
            )

        self.last_best_cost = best_cost
        self.publish_cmd(cmd_v, cmd_w)

        if self.should_log("control_state", self.debug_interval_sec):
            obs_x, obs_y, obs_source = self.current_obstacle_pose()
            dx_ov = obs_x - my_state["x"] if obs_x is not None else math.inf
            if self.is_leader:
                if obs_x is None or obs_y is None:
                    ov_text = "pending"
                    ov_speed = 0.0
                else:
                    ov_text = f"{obs_source}@({obs_x:.2f}, {obs_y:.2f})"
                    ov_speed = self.obstacle_speed()
                _, my_lat = self.project_along_lateral(my_state["x"], my_state["y"])
                self.get_logger().info(
                    f"Leader dbg | pose=({my_state['x']:.2f}, {my_state['y']:.2f}) "
                    f"v={my_state['v']:.2f} cmd=({self.last_cmd_linear:.2f}, {self.last_cmd_angular:.2f}) "
                    f"lane={self.lane_cur} mode={self.avoidance_mode} lc_active={self.lc_active} "
                    f"lat={my_lat:.2f} target_y={target_y:.2f} ov={ov_text} ov_v={ov_speed:.2f} "
                    f"visible_now={self.obstacle_visible_in_scan} "
                    f"range={self.last_detected_range:.2f} dx_ov={dx_ov:.2f} "
                    f"risk={self.last_risk:.4f} alpha={self.last_alpha:.2f} "
                    f"cost={self.last_best_cost:.3f}"
                )
            else:
                leader_state = self.SVs[self.preced_id]
                gap = (
                    leader_state["x"] - my_state["x"]
                    if leader_state is not None
                    else math.inf
                )
                leader_v = leader_state["v"] if leader_state is not None else 0.0
                self.get_logger().info(
                    f"Follower dbg | pose=({my_state['x']:.2f}, {my_state['y']:.2f}) "
                    f"v={my_state['v']:.2f} cmd=({self.last_cmd_linear:.2f}, {self.last_cmd_angular:.2f}) "
                    f"gap={gap:.2f} leader_v={leader_v:.2f} "
                    f"target_speed={target_speed:.2f} target=({target_x:.2f}, {target_y:.2f}) "
                    f"dx_ov={dx_ov:.2f} leader_path_points={len(self.leader_path_history)} "
                    f"risk={self.last_risk:.4f} alpha={self.last_alpha:.2f} "
                    f"cost={self.last_best_cost:.3f}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = TwoSVMovingOVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
