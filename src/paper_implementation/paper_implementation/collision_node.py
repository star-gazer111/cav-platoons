import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

                                    
DT = 0.1          
LANE_MAIN = 0.0
LANE_RIGHT = -0.65 

V_DES = 0.20                             
V_MIN = 0.05                           
V_MAX = 0.26                                                 
A_MIN = -0.4       
A_MAX = 0.3        

D_HEADWAY = 0.5                                
MERGE_TOL = 0.1    
TAU_LC = 2.0                                                         

                               
CRPF_G = 0.5
CRPF_ZETA = 1.2
PSEUDO_EPS = 1.0
PSEUDO_RHO = 0.02
CR_YELLOW_LOW = 0.008
CR_YELLOW_HIGH = 0.012

W_RISK = 1.0
W_SPEED = 0.08

class PlatoonSplitMergeNode(Node):
    def __init__(self):
        super().__init__('platoon_split_merge')
        
                          
        self.ns = self.get_namespace().strip('/')
        self.bot_id = int(self.ns.split('_')[-1]) if 'tb3_' in self.ns else 0
        self.preced_id = self.bot_id - 1 if self.bot_id > 0 else -1
        
        self.get_logger().info(f"Started Node for {self.ns}. Preced: {self.preced_id}")

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
                               
        self.SVs = {0: None, 1: None, 2: None} 
        self.odom_subs = []
        for i in range(3):
            sub = self.create_subscription(Odometry, f'/tb3_{i}/odom', 
                                           lambda msg, idx=i: self.odom_v2v_callback(msg, idx), 10)
            self.odom_subs.append(sub)

                                
        self.v = V_DES
        self.lane_cur = "MAIN"
        self.lane_to = "MAIN"
        self.lane_from = "MAIN"
        self.lc_active = False
        self.lc_t = 0.0
        
                                       
        self.ov_x = 3.0                         
        self.ov_y = 0.0
        self.ov_vx = 0.1                                        
        self.ov_vy = 0.0
        self.ov_detected = False
        
        self.timer = self.create_timer(DT, self.control_loop)

    def odom_v2v_callback(self, msg, bot_idx):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
        self.SVs[bot_idx] = {'x': px, 'y': py, 'v': vx, 'yaw': yaw}

    def scan_callback(self, msg):
        """ V2V-Filtered Lidar Scan """
        if self.SVs[self.bot_id] is None: return

        bot = self.SVs[self.bot_id]
        rx, ry, ryaw = bot['x'], bot['y'], bot['yaw']

        best_dist = float('inf')
        found_obs = False
        bx, by = 999.0, 999.0

        for i, r in enumerate(msg.ranges):
            if 0.15 < r < 3.0:
                angle = msg.angle_min + i * msg.angle_increment
                norm_angle = math.atan2(math.sin(angle), math.cos(angle))
                
                if abs(norm_angle) <= (math.pi / 2.0):
                    lx = r * math.cos(angle)
                    ly = r * math.sin(angle)
                    
                    px = rx + (lx * math.cos(ryaw) - ly * math.sin(ryaw))
                    py = ry + (lx * math.sin(ryaw) + ly * math.cos(ryaw))
                    
                    is_platoon_member = False
                    for idx, sv in self.SVs.items():
                        if idx == self.bot_id or sv is None: continue
                        if math.hypot(px - sv['x'], py - sv['y']) < 0.35:
                            is_platoon_member = True
                            break
                    
                    if not is_platoon_member:
                        if r < best_dist:
                            best_dist = r
                            bx, by = px, py
                            found_obs = True

        if found_obs:
            self.ov_x = bx
            self.ov_y = by
            self.ov_detected = True
        else:
            self.ov_detected = False

    def lane_center_of(self, lbl):
        if lbl == "MAIN": return LANE_MAIN
        else: return LANE_RIGHT

    def smoothstep_quintic(self, u):
        u = max(0.0, min(1.0, u))
        return 10*u**3 - 15*u**4 + 6*u**5

    def start_lane_change(self, next_lane):
        self.lane_from = self.lane_cur
        self.lane_to = next_lane
        self.lc_t = 0.0
        self.lc_active = True

    def gate_alpha(self, risk_now, rlow, rhigh):
        if risk_now <= rlow: return 0.0
        elif risk_now >= rhigh: return 1.0
        else:
            tau = (risk_now - rlow) / max(1e-12, (rhigh - rlow))
            return tau**2 * (3 - 2*tau)

    def crpf(self, rx, ry, ox, oy, v_obs):
                                  
                                                                                 
                                                                              
        if abs(ry - oy) > 0.35:
            return 0.0

        eps, rho = PSEUDO_EPS, PSEUDO_RHO
        dx = rx - ox
        dy = ry - oy
        termx = eps * dx * math.exp(-rho * max(v_obs, 0.0))
        termy = eps * dy
        rd = math.hypot(termx, termy)
        rd = max(rd, 0.25)
        
        M = 1.0 * (1.566e-14 * (max(v_obs,0.0)**6.687) + 0.3354)
        Tfac = (0.15 / 0.4) * (1.0 / 1.0) 
        val = CRPF_G * M * Tfac * 1.0 / (rd**CRPF_ZETA)
        return val

    def control_loop(self):
        if self.SVs[self.bot_id] is None: return
        my_state = self.SVs[self.bot_id]

                                   
        if not self.ov_detected:
            self.ov_x += self.ov_vx * DT
            self.ov_y += self.ov_vy * DT

        dxOV = self.ov_x - my_state['x']

                                              
        if not self.lc_active:
            if self.lane_cur == "MAIN":
                if 0.0 < dxOV < 1.5:
                    self.start_lane_change("RIGHT")
                    self.get_logger().info('Avoiding: Changing to RIGHT lane.')
            
            elif self.lane_cur == "RIGHT":
                if dxOV < -1.0:
                    self.start_lane_change("MAIN")
                    self.get_logger().info('Overtook: Returning to MAIN lane.')

        target_y = self.lane_center_of(self.lane_cur)
        if self.lc_active:
            self.lc_t += DT
            s = self.smoothstep_quintic(min(self.lc_t / TAU_LC, 1.0))
            y_from = self.lane_center_of(self.lane_from)
            y_to = self.lane_center_of(self.lane_to)
            target_y = (1 - s) * y_from + s * y_to
            
            if self.lc_t >= TAU_LC:
                self.lc_active = False
                self.lane_cur = self.lane_to
                target_y = y_to

                                             
        v_pref = V_DES
        if self.preced_id >= 0 and self.SVs[self.preced_id] is not None:
            pred = self.SVs[self.preced_id]
            gap = pred['x'] - my_state['x']
            
            k_p = 0.8
            gap_error = gap - D_HEADWAY
            v_pref = V_DES + (k_p * gap_error)
            v_pref = min(max(v_pref, V_MIN), V_MAX)

                                                       
        a_samples = np.linspace(A_MIN, A_MAX, 7)
        risk_OV = self.crpf(my_state['x'], my_state['y'], self.ov_x, self.ov_y, self.ov_vx)
        alpha = self.gate_alpha(risk_OV, CR_YELLOW_LOW, CR_YELLOW_HIGH)
        
        best_cost = float('inf')
        best_v_next = self.v
        
        for a_cmd in a_samples:
            v_next = min(max(self.v + a_cmd * DT, V_MIN), V_MAX)
            
            x_tmp = my_state['x']
            acc_risk = 0.0
            for _ in range(8): 
                x_tmp += v_next * DT
                rr = self.crpf(x_tmp, my_state['y'], self.ov_x, self.ov_y, self.ov_vx)
                acc_risk += rr
            
            mean_risk = acc_risk / 8.0
            cost = ((W_RISK * alpha) * mean_risk) + (W_SPEED * (v_pref - v_next)**2)
            
            if cost < best_cost:
                best_cost = cost
                best_v_next = v_next

        self.v = best_v_next

                                                
        error_y = target_y - my_state['y']
        lookahead_x = 0.5 
        
        desired_yaw = math.atan2(error_y, lookahead_x)
        yaw_error = desired_yaw - my_state['yaw']
        
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        cmd = Twist()
        cmd.linear.x = float(self.v)
        cmd.angular.z = float(2.5 * yaw_error) 
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PlatoonSplitMergeNode()
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