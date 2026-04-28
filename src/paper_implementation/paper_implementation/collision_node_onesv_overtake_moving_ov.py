                      
"""
CRPF + DWA avoidance for one moving TurtleBot3 overtaking a slower moving obstacle.

This case reuses the existing one-SV / one-moving-OV tracker, but adds an
explicit speed bias during avoidance so the SV preserves a small closing-speed
margin over a slower obstacle and completes the pass before returning to its
original path.
"""

import math

import rclpy

from paper_implementation.collision_node_onesv_onemoving_ov import (
    CRPFDWABurgerNode,
    PASS_NONE,
    V_AVOID_MIN,
    V_DES,
    V_MAX,
    W_SPEED,
    clamp,
)


DEFAULT_NOMINAL_OBSTACLE_SPEED = 0.005
DEFAULT_OVERTAKE_MARGIN = 0.09
DEFAULT_OVERTAKE_SPEED_CAP = V_MAX
DEFAULT_OVERTAKE_SPEED_WEIGHT = W_SPEED * 6.0


class CRPFDWAMovingSVOvtakeNode(CRPFDWABurgerNode):
    def __init__(self):
        super().__init__()

        self.nominal_obstacle_speed = float(
            self.declare_parameter(
                "nominal_obstacle_speed",
                DEFAULT_NOMINAL_OBSTACLE_SPEED,
            ).value
        )
        self.min_overtake_margin = float(
            self.declare_parameter("min_overtake_margin", DEFAULT_OVERTAKE_MARGIN).value
        )
        self.overtake_speed_cap = float(
            self.declare_parameter("overtake_speed_cap", DEFAULT_OVERTAKE_SPEED_CAP).value
        )
        self.overtake_speed_weight = float(
            self.declare_parameter(
                "overtake_speed_weight",
                DEFAULT_OVERTAKE_SPEED_WEIGHT,
            ).value
        )

        self.get_logger().info(
            "Moving-SV / moving-OV overtake case ready | "
            f"nominal_ov_speed={self.nominal_obstacle_speed:.2f} m/s | "
            f"margin={self.min_overtake_margin:.2f} m/s | "
            f"speed_cap={self.overtake_speed_cap:.2f} m/s"
        )

    def overtake_target_speed(self, obs_speed: float, pass_side: int, ox: float, oy: float) -> float:
        if pass_side == PASS_NONE or self.path is None:
            return V_DES

        if not self.obs_detected or not math.isfinite(ox) or not math.isfinite(oy):
            return V_DES

        robot_along, _ = self.path.along_lateral(self.x, self.y)
        obs_along, _ = self.path.along_lateral(ox, oy)

                                                                                            
        if obs_along - robot_along < -0.10:
            return V_DES

        tracked_speed = max(obs_speed, self.nominal_obstacle_speed)
        target = tracked_speed + self.min_overtake_margin
        return clamp(target, V_AVOID_MIN, self.overtake_speed_cap)

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
        base_cost = super().rollout_cost(v, w, ox, oy, ovx, ovy, obs_speed, alpha, pass_side)
        if not math.isfinite(base_cost):
            return base_cost

        target_speed = self.overtake_target_speed(obs_speed, pass_side, ox, oy)
        if v >= target_speed:
            return base_cost

        speed_error = target_speed - v
        return base_cost + self.overtake_speed_weight * speed_error * speed_error


def main(args=None):
    rclpy.init(args=args)
    node = CRPFDWAMovingSVOvtakeNode()
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
