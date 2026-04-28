                      
"""
Single-node moving-SV / moving-OV overtake case.

This node controls the SV with the CRPF+DWA overtake logic and also publishes a
constant-velocity command directly to the obstacle, so the entire scenario is
driven by one ROS node.
"""

import rclpy
from geometry_msgs.msg import Twist

from paper_implementation.collision_node_onesv_overtake_moving_ov import (
    CRPFDWAMovingSVOvtakeNode,
)


class CRPFDWASingleNodeMovingOVNode(CRPFDWAMovingSVOvtakeNode):
    def __init__(self):
        super().__init__()

        self.obstacle_cmd_topic = str(
            self.declare_parameter("obstacle_cmd_topic", "/obstacle/cmd_vel").value
        )
        self.obstacle_linear_speed = float(
            self.declare_parameter("obstacle_linear_speed", self.nominal_obstacle_speed).value
        )
        self.obstacle_angular_speed = float(
            self.declare_parameter("obstacle_angular_speed", 0.0).value
        )

                                                                                
                                                  
        self.nominal_obstacle_speed = self.obstacle_linear_speed
        self.obstacle_cmd_pub = self.create_publisher(Twist, self.obstacle_cmd_topic, 10)

        self.get_logger().info(
            "Single-node mode enabled | "
            f"obstacle_cmd_topic={self.obstacle_cmd_topic} | "
            f"obstacle_speed={self.obstacle_linear_speed:.2f} m/s"
        )

    def publish_obstacle_cmd(self, linear: float, angular: float = 0.0):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.obstacle_cmd_pub.publish(cmd)

    def control_loop(self):
        self.publish_obstacle_cmd(self.obstacle_linear_speed, self.obstacle_angular_speed)
        super().control_loop()


def main(args=None):
    rclpy.init(args=args)
    node = CRPFDWASingleNodeMovingOVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.publish_obstacle_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
