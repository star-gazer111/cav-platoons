#!/usr/bin/env python3
"""
Two moving SVs with one stationary OV.

`tb3_0` is the leader. It cruises in the main lane, detects the stationary
obstacle with LiDAR, changes lane to avoid it, and returns to the main lane
after clearing the obstacle.

`tb3_1` is the follower. It maintains the desired platoon headway from the
leader and mirrors the leader's lateral motion so it follows the same bypass
path around the obstacle.
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


DT = 0.10

LEADER_ID = 0
PLATOON_SIZE = 2

LANE_MAIN = 0.0
LANE_LEFT = 0.45
LANE_RIGHT = -0.45

V_MAX = 0.20
LEADER_CRUISE_SPEED = 0.14
HEADWAY_DES = 0.7
HEADWAY_KP = 0.90
MAX_ACCEL = 0.25
MAX_DECEL = 0.40

LOOKAHEAD_X = 0.55
STEER_KP = 2.60
MAX_TURN_RATE = 1.50
FOLLOWER_PATH_PREVIEW = 0.35

LANE_CHANGE_DURATION = 2.2
AVOID_TRIGGER_DIST = 2.0
CLEAR_DIST_BEHIND = 0.90

FRONT_FOV_DEG = 70.0
FRONT_FOV_RAD = math.radians(FRONT_FOV_DEG)
MIN_SCAN_RANGE = 0.12
MAX_SCAN_RANGE = 3.00
PLATOON_FILTER_RADIUS = 0.32
OBSTACLE_MAIN_LANE_TOL = 0.20
AVOID_RETURN_START_AHEAD = -0.25
LEADER_PATH_SAMPLE_DIST = 0.02
LEADER_PATH_MAXLEN = 4000


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


class TwoSVStationaryOVNode(Node):
    def __init__(self):
        super().__init__("two_sv_one_stationary_ov")

        self.ns = self.get_namespace().strip("/")
        inferred_bot_id = int(self.ns.split("_")[-1]) if self.ns.startswith("tb3_") else 0
        self.bot_id = int(self.declare_parameter("bot_id", inferred_bot_id).value)
        self.is_leader = self.bot_id == LEADER_ID
        self.preced_id = self.bot_id - 1 if self.bot_id > LEADER_ID else -1
        self.scan_topic = str(self.declare_parameter("scan_topic", "scan").value)
        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "cmd_vel").value)
        self.odom_topics = {
            0: str(self.declare_parameter("sv0_odom_topic", "/tb3_0/odom").value),
            1: str(self.declare_parameter("sv1_odom_topic", "/tb3_1/odom").value),
        }
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
        self.obstacle_initial_x = float(
            self.declare_parameter("obstacle_initial_x", 3.0).value
        )
        self.obstacle_initial_y = float(
            self.declare_parameter("obstacle_initial_y", 0.0).value
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

        self.v_cmd = 0.0

        self.lane_cur = "MAIN"
        self.lane_from = "MAIN"
        self.lane_to = "MAIN"
        self.lc_active = False
        self.lc_t = 0.0

        self.ov_x = self.obstacle_initial_x
        self.ov_y = self.obstacle_initial_y
        self.obstacle_seen = False
        self.obstacle_visible_in_scan = False
        self.last_detected_range = math.inf
        self.active_obstacle_x = None
        self.active_obstacle_y = None
        self.avoidance_mode = "CRUISE"
        self.scan_count = 0
        self.last_log_times = {}
        self.leader_path_history = deque(maxlen=LEADER_PATH_MAXLEN)

        role = "leader" if self.is_leader else "follower"
        self.get_logger().info(
            f"{self.ns or 'tb3_0'} ready as {role} | "
            f"bot_id={self.bot_id} | "
            f"headway={self.desired_headway:.2f} m | "
            f"pass_side={self.preferred_pass_lane.lower()} | "
            f"scan={self.scan_topic} cmd={self.cmd_vel_topic} | "
            f"sv0_odom={self.odom_topics[0]} sv1_odom={self.odom_topics[1]}"
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
            self.record_leader_path(px, py)

        if self.enable_debug_logs and self.odom_counts[bot_idx] == 1:
            self.get_logger().info(
                f"First odom for sv{bot_idx} on {self.odom_topics[bot_idx]} | "
                f"raw=({raw_x:.2f}, {raw_y:.2f}) mapped=({px:.2f}, {py:.2f})"
            )

    def record_leader_path(self, x: float, y: float):
        if self.leader_path_history:
            last_x, last_y = self.leader_path_history[-1]
            if math.hypot(x - last_x, y - last_y) < LEADER_PATH_SAMPLE_DIST:
                return
        self.leader_path_history.append((x, y))

    def leader_path_target_point(self, query_x: float) -> tuple[float, float]:
        if not self.leader_path_history:
            return query_x, LANE_MAIN

        first_x, first_y = self.leader_path_history[0]
        if query_x <= first_x:
            return first_x, first_y

        last_x, last_y = self.leader_path_history[-1]
        if query_x >= last_x:
            return last_x, last_y

        prev_x, prev_y = self.leader_path_history[0]
        for curr_x, curr_y in self.leader_path_history:
            if curr_x >= query_x:
                dx = curr_x - prev_x
                if abs(dx) < 1e-6:
                    return curr_x, curr_y
                ratio = clamp((query_x - prev_x) / dx, 0.0, 1.0)
                interp_x = ((1.0 - ratio) * prev_x) + (ratio * curr_x)
                interp_y = ((1.0 - ratio) * prev_y) + (ratio * curr_y)
                return interp_x, interp_y
            prev_x, prev_y = curr_x, curr_y

        return last_x, last_y

    def is_platoon_point(self, px: float, py: float) -> bool:
        for idx, sv in self.SVs.items():
            if idx == self.bot_id or sv is None:
                continue
            if math.hypot(px - sv["x"], py - sv["y"]) < PLATOON_FILTER_RADIUS:
                return True
        return False

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
            if rng < MIN_SCAN_RANGE or rng > MAX_SCAN_RANGE:
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
            self.ov_x = best_x
            self.ov_y = best_y
            self.obstacle_seen = True
            self.obstacle_visible_in_scan = True
            self.last_detected_range = best_range
            if self.enable_debug_logs and (
                self.scan_count == 1 or self.should_log("obstacle_track", self.debug_interval_sec)
            ):
                self.get_logger().info(
                    f"Obstacle track | range={best_range:.2f} m "
                    f"odom=({self.ov_x:.2f}, {self.ov_y:.2f})"
                )
        else:
            self.obstacle_visible_in_scan = False
            self.last_detected_range = math.inf
            if self.obstacle_seen and self.should_log(
                "obstacle_not_visible", self.wait_warn_interval_sec
            ):
                self.get_logger().warning(
                    "Obstacle not visible in the latest scan; keeping last known position."
                )

        if self.enable_debug_logs and self.scan_count == 1:
            self.get_logger().info(
                f"First scan received on {self.scan_topic} with {len(msg.ranges)} ranges"
            )

    def lane_center_of(self, lane_label: str) -> float:
        if lane_label == "LEFT":
            return LANE_LEFT
        if lane_label == "RIGHT":
            return LANE_RIGHT
        return LANE_MAIN

    def start_lane_change(self, next_lane: str):
        if self.lc_active or next_lane == self.lane_cur:
            return

        self.lane_from = self.lane_cur
        self.lane_to = next_lane
        self.lc_t = 0.0
        self.lc_active = True
        self.get_logger().info(
            f"{self.ns or 'tb3_0'} lane change: {self.lane_from} -> {self.lane_to}"
        )

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
        if self.active_obstacle_x is None and self.obstacle_seen:
            dx_ov = self.ov_x - my_state["x"]
            obstacle_in_main_lane = abs(self.ov_y - LANE_MAIN) < OBSTACLE_MAIN_LANE_TOL
            if obstacle_in_main_lane and 0.0 < dx_ov < self.avoid_trigger_distance:
                self.active_obstacle_x = self.ov_x
                self.active_obstacle_y = self.ov_y
                self.avoidance_mode = "AVOID"
                self.get_logger().info(
                    f"Leader locked obstacle at ({self.active_obstacle_x:.2f}, "
                    f"{self.active_obstacle_y:.2f}) and is starting avoidance."
                )

        if self.active_obstacle_x is None:
            self.lane_cur = "MAIN"
            self.lc_active = False
            return LANE_MAIN

        dx_ahead = self.active_obstacle_x - my_state["x"]
        weight = self.avoidance_lateral_weight(dx_ahead)
        target_lane_y = self.lane_center_of(self.preferred_pass_lane)
        target_y = weight * target_lane_y

        if weight > 0.02:
            self.lane_cur = self.preferred_pass_lane
            self.lc_active = True
        else:
            self.lane_cur = "MAIN"
            self.lc_active = False

        if (
            dx_ahead < -self.clear_distance_behind
            and abs(target_y) < 0.02
            and abs(my_state["y"] - LANE_MAIN) < 0.05
        ):
            self.get_logger().info(
                f"Leader returned to MAIN after clearing obstacle at x={self.active_obstacle_x:.2f}"
            )
            self.active_obstacle_x = None
            self.active_obstacle_y = None
            self.avoidance_mode = "CRUISE"
            self.obstacle_seen = False
            return LANE_MAIN

        return target_y

    def follower_target_point(self, my_state) -> tuple[float, float]:
        if len(self.leader_path_history) < 2:
            leader_state = self.SVs[self.preced_id]
            fallback_x = my_state["x"] + LOOKAHEAD_X
            fallback_y = leader_state["y"] if leader_state is not None else LANE_MAIN
            return fallback_x, clamp(fallback_y, LANE_RIGHT, LANE_LEFT)

        preview_x = my_state["x"] + self.follower_path_preview
        target_x, target_y = self.leader_path_target_point(preview_x)
        return (
            max(target_x, my_state["x"] + 0.05),
            clamp(target_y, LANE_RIGHT, LANE_LEFT),
        )

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

        return clamp(target_speed, 0.0, V_MAX)

    def steering_command(self, my_state, target_y: float, target_x: float | None = None) -> float:
        error_y = target_y - my_state["y"]
        lookahead_x = LOOKAHEAD_X
        if target_x is not None:
            lookahead_x = max(0.05, target_x - my_state["x"])
        desired_yaw = math.atan2(error_y, lookahead_x)
        yaw_error = wrap_angle(desired_yaw - my_state["yaw"])
        return clamp(STEER_KP * yaw_error, -MAX_TURN_RATE, MAX_TURN_RATE)

    def control_loop(self):
        my_state = self.SVs.get(self.bot_id)
        if my_state is None:
            if self.should_log("wait_self_odom", self.wait_warn_interval_sec):
                self.get_logger().warning(
                    f"Waiting for own odom on {self.odom_topics[self.bot_id]}"
                )
            return

        if self.use_initial_pose_offsets and self.odom_origins[self.bot_id] is None:
            return

        if self.is_leader and self.scan_count == 0:
            if self.should_log("wait_scan", self.wait_warn_interval_sec):
                self.get_logger().warning(
                    f"No scans received yet on {self.scan_topic}; obstacle detection inactive."
                )
        elif not self.is_leader and self.SVs[self.preced_id] is None:
            if self.should_log("wait_preced_odom", self.wait_warn_interval_sec):
                self.get_logger().warning(
                    f"Waiting for leader odom on {self.odom_topics[self.preced_id]}"
                )

        if self.is_leader:
            target_y = self.leader_target_y(my_state)
            target_x = my_state["x"] + LOOKAHEAD_X
            target_speed = clamp(self.leader_speed, 0.0, V_MAX)
        else:
            target_x, target_y = self.follower_target_point(my_state)
            target_speed = self.follower_target_speed(my_state, target_y)

        self.v_cmd = rate_limit(
            self.v_cmd,
            target_speed,
            MAX_ACCEL * DT,
            MAX_DECEL * DT,
        )

        cmd = Twist()
        cmd.linear.x = float(self.v_cmd)
        cmd.angular.z = float(self.steering_command(my_state, target_y, target_x))
        self.cmd_pub.publish(cmd)

        if self.should_log("control_state", self.debug_interval_sec):
            if self.is_leader:
                dx_ov = self.ov_x - my_state["x"] if self.obstacle_seen else math.inf
                self.get_logger().info(
                    f"Leader dbg | pose=({my_state['x']:.2f}, {my_state['y']:.2f}) "
                    f"v={my_state['v']:.2f} cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f}) "
                    f"lane={self.lane_cur} mode={self.avoidance_mode} lc_active={self.lc_active} "
                    f"target_y={target_y:.2f} obstacle_seen={self.obstacle_seen} "
                    f"visible_now={self.obstacle_visible_in_scan} "
                    f"range={self.last_detected_range:.2f} dx_ov={dx_ov:.2f} "
                    f"active_obstacle_x={self.active_obstacle_x}"
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
                    f"v={my_state['v']:.2f} cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f}) "
                    f"gap={gap:.2f} leader_v={leader_v:.2f} "
                    f"target_speed={target_speed:.2f} target=({target_x:.2f}, {target_y:.2f}) "
                    f"leader_path_points={len(self.leader_path_history)}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = TwoSVStationaryOVNode()
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
