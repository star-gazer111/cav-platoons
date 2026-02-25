
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

# --- SCALED PHYSICAL PARAMETERS (MATLAB to TurtleBot Scale) ---
# Axes mapped: MATLAB Y -> ROS X (Forward), MATLAB X -> ROS Y (Lateral)
DT = 0.1          
LANE_MAIN = 0.0
LANE_LEFT = 0.45   # Scaled down from 4.0
LANE_RIGHT = -0.45 # Scaled down from -4.0

V_DES = 0.20       # Scaled from 15.0
V_MIN = 0.05       # Scaled from 5.0
V_MAX = 0.22       # Scaled from 25.0
A_MIN = -0.4       # Scaled from -4.0
A_MAX = 0.25       # Scaled from +2.5

D_HEADWAY = 0.5    # Scaled from 18.0
MERGE_TOL = 0.1    # Scaled from 3.5
TAU_LC = 2.5       # Duration of lane change
Y_CLEAR_AHEAD = 1.0 # Scaled from 10.0

# --- CRPF / DWA PARAMETERS ---
CRPF_G = 0.5
CRPF_ZETA = 1.2
PSEUDO_EPS = 1.0
PSEUDO_RHO = 0.02
CR_YELLOW_LOW = 0.008
CR_YELLOW_HIGH = 0.012

W_RISK = 1.0
W_SPEED = 0.08

R_SUM = 0.25 # Robot Radius + Obs Radius (Scaled)

class PlatoonSplitMergeNode(Node):
    def __init__(self):
        super().__init__('platoon_split_merge')
        
        # Identify Role
        self.ns = self.get_namespace().strip('/')
        self.bot_id = int(self.ns.split('_')[-1]) if 'tb3_' in self.ns else 0
        self.preced_id = self.bot_id - 1 if self.bot_id > 0 else -1
        
        self.get_logger().info(f"Started Node for {self.ns}. Preced: {self.preced_id}")

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # V2V State Tracking (Dictionary of all bots)
        self.SVs = {0: None, 1: None, 2: None} 
        
        # Subscribe to all Odometry to replicate MATLAB global array SV(i)
        self.odom_subs = []
        for i in range(3):
            sub = self.create_subscription(Odometry, f'/tb3_{i}/odom', 
                                           lambda msg, idx=i: self.odom_v2v_callback(msg, idx), 10)
            self.odom_subs.append(sub)

        # Local FSM State (Replicating MATLAB struct)
        self.v = V_DES
        self.mode = "PLATOON"
        self.lane_cur = "MAIN"
        self.lane_target = "MAIN"
        self.lane_from = "MAIN"
        self.lane_to = "MAIN"
        self.lc_active = False
        self.lc_t = 0.0
        self.recenter = False
        self.lock_main = False
        self.ok_ctr = 0

        # Obstacle State
        self.ov_x = 999.0
        self.ov_y = 999.0
        self.ov_vx = 0.0
        self.ov_vy = 0.0
        self.ov_detected = False
        
        self.timer = self.create_timer(DT, self.control_loop)

    def odom_v2v_callback(self, msg, bot_idx):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
        self.SVs[bot_idx] = {'x': px, 'y': py, 'v': vx, 'yaw': yaw, 'lane_target': "MAIN" if bot_idx != self.bot_id else self.lane_target}

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        front_ranges = np.concatenate((ranges[:90], ranges[-90:])) # Front 180
        valid_mask = (front_ranges > 0.15) & (front_ranges < 3.0)
        
        if np.any(valid_mask) and self.SVs[self.bot_id] is not None:
            min_idx = np.argmin(front_ranges)
            r = front_ranges[min_idx]
            actual_idx = min_idx if min_idx < 90 else (270 + min_idx - 90)
            angle = msg.angle_min + actual_idx * msg.angle_increment
            
            bot = self.SVs[self.bot_id]
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            
            # Note: Mapping to ROS standard (X forward, Y lateral)
            self.ov_x = bot['x'] + (lx * math.cos(bot['yaw']) - ly * math.sin(bot['yaw']))
            self.ov_y = bot['y'] + (lx * math.sin(bot['yaw']) + ly * math.cos(bot['yaw']))
            self.ov_detected = True
        else:
            self.ov_detected = False

    # ==========================================
    # EXACT MATLAB HELPER FUNCTIONS TRANSLATED
    # ==========================================
    def lane_center_of(self, lbl):
        if lbl == "MAIN": return LANE_MAIN
        elif lbl == "LEFT": return LANE_LEFT
        else: return LANE_RIGHT

    def smoothstep_quintic(self, u):
        u = max(0.0, min(1.0, u))
        return 10*u**3 - 15*u**4 + 6*u**5

    def lane_window_free(self, lane_y, x0, x1):
        if self.ov_detected:
            if abs(self.ov_y - lane_y) < 0.2 and (self.ov_x >= x0 and self.ov_x <= x1):
                return False
        for idx, sv in self.SVs.items():
            if idx == self.bot_id or sv is None: continue
            if abs(sv['y'] - lane_y) < 0.2 and (sv['x'] >= x0 and sv['x'] <= x1):
                return False
        return True

    def start_lane_change(self, next_lane):
        self.lane_from = self.lane_cur
        self.lane_to = next_lane
        self.lc_t = 0.0
        self.lc_active = True
        self.lane_target = next_lane

    def gate_alpha(self, risk_now, rlow, rhigh):
        if risk_now <= rlow: return 0.0
        elif risk_now >= rhigh: return 1.0
        else:
            tau = (risk_now - rlow) / max(1e-12, (rhigh - rlow))
            return tau**2 * (3 - 2*tau)

    def crpf(self, rx, ry, ox, oy, v_obs):
        # NOTE: Formula mappings adapted for ROS (X forward, Y lateral)
        eps, rho = PSEUDO_EPS, PSEUDO_RHO
        dx = rx - ox
        dy = ry - oy
        termx = eps * dx * math.exp(-rho * max(v_obs, 0.0))
        termy = eps * dy
        rd = math.hypot(termx, termy)
        rd = max(rd, 0.25) # Clamped for TurtleBot scale
        
        M = 1.0 * (1.566e-14 * (max(v_obs,0.0)**6.687) + 0.3354)
        Tfac = (0.15 / 0.4) * (1.0 / 1.0) # size_star scaled to 0.4
        
        # Accel factor (phi) assumed 1.0 for stationary
        val = CRPF_G * M * Tfac * 1.0 / (rd**CRPF_ZETA)
        return val

    def will_collide(self, p_rel, v_rel, R, horizon):
        v2 = np.dot(v_rel, v_rel)
        if v2 < 1e-12:
            collide = np.linalg.norm(p_rel) <= R
            return collide, float('inf')
        
        tstar = -np.dot(p_rel, v_rel) / v2
        tstar = min(max(tstar, 0.0), horizon)
        dmin = np.linalg.norm(p_rel + tstar*v_rel)
        
        ttc = float('inf')
        if np.dot(p_rel, v_rel) < 0:
            a = v2
            b = 2 * np.dot(p_rel, v_rel)
            c = np.dot(p_rel, p_rel) - R**2
            disc = b**2 - 4*a*c
            if disc >= 0:
                r1 = (-b - math.sqrt(disc)) / (2*a)
                r2 = (-b + math.sqrt(disc)) / (2*a)
                roots = [rt for rt in [r1, r2] if rt >= 0]
                if roots: ttc = min(roots)
                
        collide = (dmin <= R)
        return collide, ttc

    # ==========================================
    # MAIN LOOP (MATLAB REPLICATION)
    # ==========================================
    def control_loop(self):
        if self.SVs[self.bot_id] is None: return
        my_state = self.SVs[self.bot_id]

        # 1. SIDE DECISION (Which Lane)
        if self.lc_active:
            self.lane_target = self.lane_to
        elif self.lock_main:
            self.lane_target = "MAIN"
        else:
            dxOV = self.ov_x - my_state['x']
            need_consider = (dxOV > 0) and (dxOV < 3.0) and self.ov_detected
            
            if need_consider:
                if self.lane_cur == "MAIN":
                    x0 = my_state['x'] - 0.5
                    x1 = my_state['x'] + 1.2
                    freeL = self.lane_window_free(LANE_LEFT, x0, x1)
                    freeR = self.lane_window_free(LANE_RIGHT, x0, x1)
                    
                    next_lane = "MAIN"
                    if freeL and not freeR: next_lane = "LEFT"
                    elif freeR and not freeL: next_lane = "RIGHT"
                    elif freeL and freeR:
                        # Tiebreaker: MATLAB logic favors Right if balanced
                        next_lane = "RIGHT" 
                        
                    if next_lane != "MAIN":
                        self.start_lane_change(next_lane)
                else:
                    if my_state['x'] > (self.ov_x + Y_CLEAR_AHEAD):
                        self.start_lane_change("MAIN")
                        self.recenter = True
                        self.lock_main = True
            else:
                if self.lane_cur != "MAIN" and (not self.ov_detected or my_state['x'] > (self.ov_x + Y_CLEAR_AHEAD)):
                    self.start_lane_change("MAIN")
                    self.recenter = True
                    self.lock_main = True

        # 2. LONGITUDINAL DWA
        p_rel_OV = np.array([my_state['x'] - self.ov_x, my_state['y'] - self.ov_y])
        v_rel_OV = np.array([self.v - self.ov_vx, 0.0 - self.ov_vy])
        risk_OV = self.crpf(my_state['x'], my_state['y'], self.ov_x, self.ov_y, self.ov_vx) if self.ov_detected else 0.0
        
        _, ttcOV = self.will_collide(p_rel_OV, v_rel_OV, R_SUM, 8.0)
        
        ttcP = float('inf')
        if self.preced_id >= 0 and self.SVs[self.preced_id] is not None:
            pred = self.SVs[self.preced_id]
            p_rel_P = np.array([my_state['x'] - pred['x'], 0.0])
            v_rel_P = np.array([self.v - pred['v'], 0.0])
            _, ttcP = self.will_collide(p_rel_P, v_rel_P, 2*R_SUM, 8.0)

        # Split condition
        need_split = (ttcOV < 2.2) or (ttcP < 1.5) or (risk_OV >= CR_YELLOW_HIGH)
        if self.mode == "PLATOON" and need_split:
            self.mode = "CRUISE"
            
        # Merge condition
        if self.mode == "CRUISE" and self.preced_id >= 0 and not self.lc_active and self.lane_target == "MAIN":
            pred = self.SVs[self.preced_id]
            gap = pred['x'] - my_state['x']
            can_merge = (risk_OV <= CR_YELLOW_LOW) and \
                        (gap > D_HEADWAY - MERGE_TOL) and \
                        (gap < D_HEADWAY + 2*MERGE_TOL) and \
                        (self.lane_cur == "MAIN")
            if can_merge:
                self.mode = "PLATOON"

        # Acceleration sampling (DWA)
        a_samples = np.linspace(A_MIN, A_MAX, 7)
        alpha = self.gate_alpha(risk_OV, CR_YELLOW_LOW, CR_YELLOW_HIGH)
        
        best_cost = float('inf')
        best_v_next = self.v
        
        for a_cmd in a_samples:
            v_next = min(max(self.v + a_cmd * DT, V_MIN), V_MAX)
            
            x_tmp = my_state['x']
            acc_risk = 0.0
            for _ in range(8): # steps=8
                x_tmp += v_next * DT
                rr = self.crpf(x_tmp, my_state['y'], self.ov_x, self.ov_y, self.ov_vx) if self.ov_detected else 0.0
                acc_risk += rr
            
            mean_risk = acc_risk / 8.0
            cost = ((W_RISK * alpha) * mean_risk) + (W_SPEED * (V_DES - v_next)**2)
            
            if cost < best_cost:
                best_cost = cost
                best_v_next = v_next

        # Apply Longitudinal
        self.v = best_v_next

        # 3. LATERAL FSM UPDATE
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

        if self.recenter and self.lane_cur == "MAIN":
            self.ok_ctr += 1
            if self.ok_ctr >= 5: # HOLD_TICKS
                self.recenter = False
                self.lock_main = False
                self.ok_ctr = 0
        else:
            self.ok_ctr = 0

        # 4. DIFFERENTIAL DRIVE STEERING (Map (X,Y) to Twist)
        # We need to steer the bot towards target_y while moving forward at self.v
        error_y = target_y - my_state['y']
        lookahead_x = 0.5 # Look 0.5m ahead
        
        desired_yaw = math.atan2(error_y, lookahead_x)
        yaw_error = desired_yaw - my_state['yaw']
        
        # Normalize yaw error
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        cmd = Twist()
        cmd.linear.x = float(self.v)
        # P-Controller for steering based on Lateral Error
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