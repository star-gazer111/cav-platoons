#!/usr/bin/env python3
"""
Constant-speed straight-line driver for the moving obstacle Burger.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


DT = 0.10


class MovingObstacleStraightDriver(Node):
    def __init__(self):
        super().__init__("moving_obstacle_straight_driver")

        self.cmd_vel_topic = str(
            self.declare_parameter("cmd_vel_topic", "/tb3_2/cmd_vel").value
        )
        self.linear_speed = float(self.declare_parameter("linear_speed", 0.04).value)
        self.angular_speed = float(self.declare_parameter("angular_speed", 0.0).value)
        self.enable_debug_logs = bool(
            self.declare_parameter("enable_debug_logs", True).value
        )
        self.log_interval_sec = float(
            self.declare_parameter("log_interval_sec", 2.0).value
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.last_log_sec = None
        self.timer = self.create_timer(DT, self.control_loop)

        self.get_logger().info(
            "Moving obstacle driver ready | "
            f"cmd={self.cmd_vel_topic} | "
            f"linear={self.linear_speed:.2f} m/s angular={self.angular_speed:.2f} rad/s"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def should_log(self) -> bool:
        if not self.enable_debug_logs:
            return False

        now = self.now_sec()
        if self.last_log_sec is None or (now - self.last_log_sec) >= self.log_interval_sec:
            self.last_log_sec = now
            return True
        return False

    def publish_cmd(self, linear: float, angular: float):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        self.publish_cmd(self.linear_speed, self.angular_speed)
        if self.should_log():
            self.get_logger().info(
                f"Moving OV cmd | linear={self.linear_speed:.2f} angular={self.angular_speed:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacleStraightDriver()
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
