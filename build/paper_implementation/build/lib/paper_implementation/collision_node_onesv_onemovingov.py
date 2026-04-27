# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# import numpy as np
# import math

# # --- PARAMETERS ---
# V_MAX = 0.22      
# W_MAX = 1.0       
# DT = 0.1          
# PREDICT_TIME = 2.0  

# # --- TUNED WEIGHTS (Right-Side Bias) ---
# W_RISK = 80.0       
# W_LANE = 8.0        
# W_HEADING = 4.0     
# W_PROGRESS = 1.5    

# LANE_Y = 0.0      
# GOAL_X = 10.0     

# class CollisionAvoidanceNode(Node):
#     def __init__(self):
#         super().__init__('collision_avoidance_node')
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
#         self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
#         self.state = np.array([0.0, 0.0, 0.0]) # x, y, yaw
#         self.obs = {'x': 999.0, 'y': 999.0, 'detected': False}
        
#         self.timer = self.create_timer(DT, self.control_loop)
#         self.get_logger().info("CRPF Right-Side Avoidance Started.")

#     def odom_callback(self, msg):
#         self.state[0] = msg.pose.pose.position.x
#         self.state[1] = msg.pose.pose.position.y
#         q = msg.pose.pose.orientation
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         self.state[2] = math.atan2(siny_cosp, cosy_cosp)

#     def scan_callback(self, msg):
#         ranges = np.array(msg.ranges)
#         valid_idx = np.where((ranges > 0.22) & (ranges < 3.0))[0]
        
#         if len(valid_idx) > 0:
#             min_idx = valid_idx[np.argmin(ranges[valid_idx])]
#             r = ranges[min_idx]
#             angle = msg.angle_min + min_idx * msg.angle_increment
            
#             rob_x, rob_y, rob_yaw = self.state
#             obs_local_x = r * math.cos(angle)
#             obs_local_y = r * math.sin(angle)
            
#             self.obs['x'] = rob_x + (obs_local_x * math.cos(rob_yaw) - obs_local_y * math.sin(rob_yaw))
#             self.obs['y'] = rob_y + (obs_local_x * math.sin(rob_yaw) + obs_local_y * math.cos(rob_yaw))
#             self.obs['detected'] = True
#         else:
#             self.obs['detected'] = False

#     def calculate_crpf(self, px, py):
#         dx = px - self.obs['x']
#         dy = py - self.obs['y']
#         dist = math.sqrt(dx**2 + dy**2)
        
#         # Hard collision
#         if dist < 0.6: 
#             return 1000.0
        
#         # Soft risk
#         base_risk = 0.0
#         if dist < 1.2:
#             base_risk = 1.0 / (dist - 0.5) 
            
#         return base_risk

#     def control_loop(self):
#         if self.state[0] >= GOAL_X - 0.1:
#             self.cmd_pub.publish(Twist())
#             return

#         best_u = [0.0, 0.0]
#         min_cost = float('inf')

#         # Heading Correction (Lookahead)
#         lookahead = 1.0
#         desired_yaw = math.atan2(LANE_Y - self.state[1], lookahead)

#         v_samples = np.linspace(0.05, V_MAX, 5)
#         w_samples = np.linspace(-W_MAX, W_MAX, 21)

#         for v in v_samples:
#             for w in w_samples:
#                 # Rollout
#                 pred_x, pred_y, pred_yaw = self.state[0], self.state[1], self.state[2]
                
#                 for _ in range(int(PREDICT_TIME/DT)):
#                     pred_x += v * math.cos(pred_yaw) * DT
#                     pred_y += v * math.sin(pred_yaw) * DT
#                     pred_yaw += w * DT

#                 # --- RIGHT SIDE BIAS ---
#                 # We artificially add cost if the robot goes LEFT (positive Y)
#                 # This makes the Right side (negative Y) mathematically "cheaper"
#                 side_bias_cost = 0.0
#                 if pred_y > self.obs['y']: # If we are to the LEFT of the obstacle
#                      side_bias_cost = 20.0 # Huge penalty!
                
#                 # --- STANDARD COSTS ---
#                 risk_cost = 0.0
#                 if self.obs['detected']:
#                     risk_cost = self.calculate_crpf(pred_x, pred_y)

#                 lane_cost = abs(pred_y - LANE_Y)

#                 # Heading Logic (Same as before)
#                 if risk_cost > 1.0:
#                     heading_cost = 0.0 
#                 else:
#                     heading_cost = abs(pred_yaw - desired_yaw)

#                 progress_cost = (GOAL_X - pred_x)

#                 total_cost = (W_RISK * risk_cost) + \
#                              (W_LANE * lane_cost) + \
#                              (W_HEADING * heading_cost) + \
#                              (W_PROGRESS * progress_cost) + \
#                              side_bias_cost # Added Bias

#                 if total_cost < min_cost:
#                     min_cost = total_cost
#                     best_u = [v, w]

#         cmd = Twist()
#         cmd.linear.x = best_u[0]
#         cmd.angular.z = best_u[1]
#         self.cmd_pub.publish(cmd)

# def main(args=None):
#     rclpy.init(args=args)
#     node = CollisionAvoidanceNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.cmd_pub.publish(Twist())
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

# --- PARAMETERS ---
V_MAX = 0.22      
W_MAX = 1.0       
DT = 0.1          
PREDICT_TIME = 3.0  # Look ahead 3 seconds (Crucial for overtaking)
DT_PREDICT = 0.1    # Time step for prediction

# --- OBSTACLE PARAMETERS ---
# We assume we know the obstacle moves at constant velocity (from the paper/setup)
V_OBS_X = 0.08      
OBS_RADIUS = 0.15   # Actual size
ROBOT_RADIUS = 0.12 # Burger size

# --- TUNED WEIGHTS (Overtaking Logic) ---
W_RISK = 80.0       # Safety
W_LANE = 2.5        # LOW: Allow robot to leave the lane easily
W_HEADING = 2.0     # LOW: Allow turning away from goal
W_PROGRESS = 4.0    # HIGH: Reward high speed and forward distance
W_SIDE_BIAS = 30.0  # FORCE Right-side passing

LANE_Y = 0.0      
GOAL_X = 20.0     

class CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__('collision_avoidance_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.state = np.array([0.0, 0.0, 0.0])
        
        # Obstacle State
        self.obs = {
            'x': 999.0, 
            'y': 999.0, 
            'detected': False,
            'last_seen': 0.0
        }
        
        self.timer = self.create_timer(DT, self.control_loop)
        self.get_logger().info("CRPF + VO Overtaking Node Started.")

    def odom_callback(self, msg):
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.state[2] = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        # Filter: Ignore robot body (<0.22) and distant noise (>3.5)
        valid_idx = np.where((ranges > 0.22) & (ranges < 3.5))[0]
        
        if len(valid_idx) > 0:
            min_idx = valid_idx[np.argmin(ranges[valid_idx])]
            r = ranges[min_idx]
            angle = msg.angle_min + min_idx * msg.angle_increment
            
            rob_x, rob_y, rob_yaw = self.state
            obs_local_x = r * math.cos(angle)
            obs_local_y = r * math.sin(angle)
            
            self.obs['x'] = rob_x + (obs_local_x * math.cos(rob_yaw) - obs_local_y * math.sin(rob_yaw))
            self.obs['y'] = rob_y + (obs_local_x * math.sin(rob_yaw) + obs_local_y * math.cos(rob_yaw))
            self.obs['detected'] = True
            self.obs['last_seen'] = self.get_clock().now().nanoseconds / 1e9
        else:
            # Short term memory (1.0s) to prevent flickering
            now = self.get_clock().now().nanoseconds / 1e9
            if (now - self.obs['last_seen']) > 1.0:
                self.obs['detected'] = False

    def calculate_crpf_risk(self, rob_x, rob_y, obs_x, obs_y):
        dx = rob_x - obs_x
        dy = rob_y - obs_y
        dist = math.sqrt(dx**2 + dy**2)
        
        # Safe Distance = Obs Radius + Robot Radius + Buffer
        # 0.15 + 0.12 + 0.15 = ~0.42m
        safe_dist = 0.45
        
        if dist < safe_dist:
            return 1000.0 # Collision
            
        # Risk gradient extends to 1.0m
        if dist < 1.0:
            return 1.0 / (dist - 0.4)
            
        return 0.0

    def control_loop(self):
        # Stop at goal
        if self.state[0] >= GOAL_X - 0.1:
            self.cmd_pub.publish(Twist())
            return

        best_u = [0.0, 0.0]
        min_cost = float('inf')

        # Heading Strategy:
        # If we are safely past the obstacle (robot x > obs x), aim for Lane (0.0).
        # Otherwise, aim slightly ahead.
        lookahead = 1.0
        desired_yaw = math.atan2(LANE_Y - self.state[1], lookahead)

        # DWA Sampling
        v_samples = np.linspace(0.1, V_MAX, 4) # Don't sample very slow speeds (force overtaking)
        w_samples = np.linspace(-W_MAX, W_MAX, 20)

        for v in v_samples:
            for w in w_samples:
                
                # --- TRAJECTORY ROLLOUT (The VO Logic) ---
                pred_x, pred_y, pred_yaw = self.state[0], self.state[1], self.state[2]
                
                # Initialize predicted obstacle position
                # (We assume it continues moving forward at constant velocity)
                pred_obs_x = self.obs['x']
                pred_obs_y = self.obs['y']
                
                total_risk = 0.0
                detected_crash = False
                
                steps = int(PREDICT_TIME / DT_PREDICT)
                for _ in range(steps):
                    # 1. Move Robot
                    pred_x += v * math.cos(pred_yaw) * DT_PREDICT
                    pred_y += v * math.sin(pred_yaw) * DT_PREDICT
                    pred_yaw += w * DT_PREDICT
                    
                    # 2. Move Obstacle (Prediction)
                    # This tells the robot: "The obstacle will move away as you approach"
                    if self.obs['detected']:
                        pred_obs_x += V_OBS_X * DT_PREDICT
                        
                        # Calculate Risk at this future moment
                        step_risk = self.calculate_crpf_risk(pred_x, pred_y, pred_obs_x, pred_obs_y)
                        
                        if step_risk > 500.0:
                            detected_crash = True
                            break
                        total_risk = max(total_risk, step_risk)

                if detected_crash:
                    continue

                # --- COST FUNCTION ---
                
                # A. Right-Side Bias
                # If predicted Y is > Obstacle Y (Left side), penalized heavily
                bias_cost = 0.0
                if self.obs['detected'] and pred_y > pred_obs_y:
                    bias_cost = W_SIDE_BIAS

                # B. Lane Cost (Reduced weight to allow overtaking)
                lane_cost = abs(pred_y - LANE_Y)

                # C. Heading Cost
                # If actively avoiding (High Risk), ignore heading to allow swerve
                if total_risk > 0.5:
                    heading_cost = 0.0
                else:
                    heading_cost = abs(pred_yaw - desired_yaw)

                # D. Progress Cost
                # We want to maximize X (minimize Goal Distance)
                progress_cost = (GOAL_X - pred_x)

                # Final Sum
                total_cost = (W_RISK * total_risk) + \
                             (W_LANE * lane_cost) + \
                             (W_HEADING * heading_cost) + \
                             (W_PROGRESS * progress_cost) + \
                             bias_cost

                if total_cost < min_cost:
                    min_cost = total_cost
                    best_u = [v, w]

        cmd = Twist()
        cmd.linear.x = best_u[0]
        cmd.angular.z = best_u[1]
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()