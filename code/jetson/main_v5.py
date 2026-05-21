import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import math
from scipy.ndimage import binary_dilation

import a_star
import dwa


class RobotNavigation(Node):
    '''Navi custom node'''
    def __init__(self):
        super().__init__('robot_navigation_node')
        self.dwa_cfg = dwa.DWAConfig()

        # auto-explore flag
        self.is_exploring = True
        # store init map as unknown
        self.init_map_data = None

        # stuck detection
        self.last_position = None
        self.last_position_time = self.get_clock().now()
        self.stuck_timeout = 3.0 
        self.stuck_distance = 0.05 # maximum distance 
        self.replan_cooldown = 5.0 # replan after this time
        self.last_replan_time = None
        self.current_goal = None

        # tf2 - localisation in a space
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscribers
        self.map_sub  = self.create_subscription(OccupancyGrid, '/map',       self.map_callback,  10) # map
        self.scan_sub = self.create_subscription(LaserScan,     '/scan',      self.scan_callback, 10) # lidar
        self.goal_sub = self.create_subscription(PoseStamped,   '/goal_pose', self.goal_callback, 10) # pointing a goal point from rviz2

        # publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel',      10) # velocities
        self.path_pub = self.create_publisher(Path,  '/planned_path', 10) # planned path

        # robot and map state
        self.grid_map = None
        self.map_resolution = 0.05 # grid size [meters]
        self.map_origin = [0.0, 0.0] # left-bottom corner
        self.latest_scan = None

        self.path = []
        self.target_idx = 0
        self.near_target_pos = 0.3 # distance check - goal reached

        self.current_v = 0.0
        self.current_w = 0.0
        self._recovery_dir = 1.0 # turn left (1.0) or right (-1.0) when stucks

        self.timer = self.create_timer(0.1, self.control_loop) # callback control loop every 0.1 s
        self.get_logger().info("Waiting for map...")

    def map_callback(self, msg):
        '''Transform SLAM into map'''
        self.map_resolution = msg.info.resolution
        self.map_origin     = [msg.info.origin.position.x, msg.info.origin.position.y]
        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
        # -1 -> unknown, 0 -> free, >50 -> obstacle
        self.init_map_data = data 

        # binary mask (1 -> obstacle)
        raw = np.where(data > 50, 1, 0).astype(bool)
        # dilatation (bold)
        inflation_cells = max(1, int(self.dwa_cfg.inflation_radius / self.map_resolution))
        inflated = binary_dilation(raw, iterations=inflation_cells)
        self.grid_map = inflated.astype(np.uint8)

        # auto-drive mode, lack of goal and current pose is none -> explore
        if self.is_exploring and self.current_goal is None and self.get_robot_pose() is not None:
            self.trigger_exploration()

    def scan_callback(self, msg):
        '''Fcn for LiDAR subscriber'''
        self.latest_scan = msg

    def goal_callback(self, msg):
        '''Manual mode (rviz2 pointing goal point)'''
        self.is_exploring = False
        self.get_logger().info("Manual goal received - auto-drive mode disable.")
        
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.current_goal = (goal_x, goal_y)
        
        current_pose = self.get_robot_pose()
        if current_pose is not None:
            self.run_astar(current_pose[0], current_pose[1], goal_x, goal_y)


    def trigger_exploration(self):
        '''Auto-drive mode'''
        pose = self.get_robot_pose()
        if pose is None or self.init_map_data is None:
            return

        x, y, _ = pose
        self.get_logger().info("Searching for the farthest unknown point...")
        
        start_grid = self.world_to_grid(x, y)

        # set pixel as free or unknow
        free_space = (self.init_map_data == 0)
        unknown_space = (self.init_map_data == -1)

        # find free space (capable to reach) that touches unknown space
        unknown_dilated = binary_dilation(unknown_space, iterations=1)
        frontiers = free_space & unknown_dilated

        # remove frontiers inside inflated walls
        frontiers = frontiers & (self.grid_map == 0)

        frontier_coords = np.argwhere(frontiers)

        # if there are no more points to explore
        if len(frontier_coords) == 0:
            self.get_logger().info("Map complete")
            self.is_exploring = False
            return

        # Euclidean distance from robot to all frontiers
        dists = np.linalg.norm(frontier_coords - np.array(start_grid), axis=1)

        # sort indicies of points to reach (from the farthest)
        sorted_indices = np.argsort(dists)[::-1]

        # find the farthest one that A* can actually reach (only top 15)
        for idx in sorted_indices[:15]:
            target_grid = tuple(frontier_coords[idx])
            
            # use A* to verify reachability
            path = a_star.a_star(self.grid_map, start_grid, target_grid)
            
            if path:
                goal_x, goal_y = self.grid_to_world(*target_grid)
                self.current_goal = (goal_x, goal_y)
                
                self.path = [self.grid_to_world(p[0], p[1]) for p in path]
                self.target_idx = 0
                self.publish_path_for_rviz()
                
                self.get_logger().info(f"Auto-Exploring to farthest frontier at ({goal_x:.2f}, {goal_y:.2f})")
                return

        self.get_logger().warn("Could not find a reachable frontier.")


    def run_astar(self, start_x, start_y, goal_x, goal_y):
        '''Operate A* algorithm'''
        # convert meteres to matrix indicies
        raw_start = self.world_to_grid(start_x, start_y)
        raw_goal  = self.world_to_grid(goal_x,  goal_y)

        # if too close to obstacles, find the nearest free point
        start_grid = self.nearest_free_cell(*raw_start)
        goal_grid  = self.nearest_free_cell(*raw_goal)

        grid_path = a_star.a_star(self.grid_map, start_grid, goal_grid)

        if not grid_path:
            self.get_logger().error("A* found no path.")
            return False

         # convert matrix indicies to meters
        self.path       = [self.grid_to_world(p[0], p[1]) for p in grid_path]
        self.target_idx = 0
        self.publish_path_for_rviz()
        return True

    def nearest_free_cell(self, grid_row, grid_col, search_radius=8):
        '''Find nearest free cell'''
        rows, cols = self.grid_map.shape
        # if git return the current index
        if (0 <= grid_row < rows and 0 <= grid_col < cols and self.grid_map[grid_row, grid_col] == 0):
            return (grid_row, grid_col)

        # check neighborhood to find free cell
        for r in range(1, search_radius + 1):
            for dr in range(-r, r + 1):
                for dc in range(-r, r + 1):
                    if abs(dr) != r and abs(dc) != r: continue
                    nr, nc = grid_row + dr, grid_col + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        if self.grid_map[nr, nc] == 0:
                            return (nr, nc)
        return (grid_row, grid_col)


    def check_if_stuck(self, x, y):
        '''Operate stuck detection'''
        now = self.get_clock().now()
        if self.last_position is None:
            self.last_position = (x, y)
            self.last_position_time = now
            return
        
        # Euklidean distance of motion - distnance stuck detection
        dist_moved = math.hypot(x - self.last_position[0], y - self.last_position[1])
        if dist_moved > self.stuck_distance:
            self.last_position = (x, y)
            self.last_position_time = now
            return

        # time stuck detection
        elapsed = (now - self.last_position_time).nanoseconds / 1e9
        if elapsed < self.stuck_timeout: return

        # time from last replan
        if self.last_replan_time is not None:
            if (now - self.last_replan_time).nanoseconds / 1e9 < self.replan_cooldown: return

        self.get_logger().warn("Stuck - attempting to escape...")
        self.last_replan_time = now
        self.last_position = (x, y)
        self.last_position_time = now

        # stuck during exploration -> change goal point
        if self.is_exploring:
            self.trigger_exploration()
        elif self.current_goal is not None:
            if not self.run_astar(x, y, *self.current_goal):
                self.path = []


    def control_loop(self):
        '''Main control function'''
        if not self.path or self.latest_scan is None: return

        current_pose = self.get_robot_pose()
        if current_pose is None: return
        x, y, theta = current_pose

        self.check_if_stuck(x, y)
        if not self.path: return

        target_x, target_y = self.path[self.target_idx]
        # goal reached
        if math.hypot(target_x - x, target_y - y) < self.near_target_pos:
            self.target_idx += 1 # next point    

            if self.target_idx >= len(self.path):
                self.path = []
                self.cmd_vel_pub.publish(Twist())
                self.current_v, self.current_w = 0.0, 0.0
                
                # explore mode -> find new goal point
                if self.is_exploring:
                    self.trigger_exploration()
                else:
                    self.get_logger().info("Manual goal reached.")
                return
                
            target_x, target_y = self.path[self.target_idx]

        # avoid obstacles
        obstacles = self.scan_to_coords(x, y, theta, self.latest_scan)
        best_v, best_w = self.dwa_control(x, y, theta, self.current_v, self.current_w, target_x, target_y, obstacles)

        msg = Twist()
        msg.linear.x, msg.angular.z = best_v, best_w
        self.cmd_vel_pub.publish(msg)

        self.current_v, self.current_w = best_v, best_w


    def dwa_control(self, x, y, theta, v, w, goal_x, goal_y, obstacles):
        '''Realization DWA algorithm (local navi)'''
        cfg = self.dwa_cfg
        # velocity limits (rapid velocity changes are impossible)
        v_min = max(cfg.min_linear_vel,  v - cfg.max_linear_acc  * cfg.dt)
        v_max = min(cfg.max_linear_vel,  v + cfg.max_linear_acc  * cfg.dt)
        w_min = max(cfg.min_angular_vel, w - cfg.max_angular_acc * cfg.dt)
        w_max = min(cfg.max_angular_vel, w + cfg.max_angular_acc * cfg.dt)

        best_score = -math.inf
        best_v, best_w = 0.0, 0.0

        # velocity samples (change resolutions to speed up calculations)
        v_samples = np.arange(v_min, v_max + cfg.linear_resolution,  cfg.linear_resolution)
        w_samples = np.arange(w_min, w_max + cfg.angular_resolution, cfg.angular_resolution)
        obs_arr = np.array(obstacles) if obstacles else None

        # simulate trajectory for each velocities' sample
        for sv in v_samples:
            for sw in w_samples:
                traj = self.simulate_trajectory(x, y, theta, sv, sw)
                min_dist = self.min_obstacle_dist(traj, obs_arr)

                if min_dist < cfg.robo_radius: continue

                # scores base on weights 
                heading_score = self.score_heading(traj, goal_x, goal_y)
                clearance_score = min(min_dist / cfg.lidar_max_range, 1.0)
                score = (cfg.w_heading * heading_score + cfg.w_clearance * clearance_score + cfg.w_velocity * sv)

                if score > best_score:
                    best_score = score
                    best_v, best_w = sv, sw

        # TODO: CHANGE THIS CONDITION (have no idea how)
        # no best scores-> spin around and look for best score
        if best_score == -math.inf:
            best_v = 0.0
            best_w = cfg.max_angular_vel * 0.5 * self._recovery_dir
            self._recovery_dir *= -1.0

        return best_v, best_w


    def simulate_trajectory(self, x, y, theta, v, w):
        '''Simulate trajectory for DWA in given range time'''
        cfg = self.dwa_cfg
        return [(x + v*math.cos(theta + w*cfg.dt*i)*cfg.dt*i, y + v*math.sin(theta + w*cfg.dt*i)*cfg.dt*i) for i in range(1, int(cfg.predict_time / cfg.dt) + 1)]

    def min_obstacle_dist(self, traj, obs_arr):
        '''Check how close to obstacle durign simulation'''
        if obs_arr is None or len(obs_arr) == 0: return self.dwa_cfg.lidar_max_range
        return float(np.linalg.norm(np.array(traj)[:, np.newaxis, :] - obs_arr[np.newaxis, :, :], axis=2).min())

    def score_heading(self, traj, goal_x, goal_y):
        '''Check how adequate is the given trajectory'''
        end_x, end_y = traj[-1]
        goal_angle   = math.atan2(goal_y - end_y, goal_x - end_x)
        # robot orientation
        end_theta = math.atan2(traj[-1][1] - traj[-2][1], traj[-1][0] - traj[-2][0]) if len(traj) >= 2 else 0.0

        # atan2(sin/cos) - angle normalization
        angle_diff = abs(math.atan2(math.sin(goal_angle - end_theta), math.cos(goal_angle - end_theta)))

        # provide heading toward target point
        w_head = 0.7
        return w_head * ((math.pi - angle_diff) / math.pi) + 0.3 * (1.0 / (1.0 + math.hypot(goal_x - end_x, goal_y - end_y)))

    def scan_to_coords(self, rx, ry, th, scan):
        '''Transfer LiDAR data into xy coordinates'''
        return [(rx + r*math.cos(th + scan.angle_min + i*scan.angle_increment), ry + r*math.sin(th + scan.angle_min + i*scan.angle_increment)) 
                for i, r in enumerate(scan.ranges) if scan.range_min < r < min(scan.range_max, self.dwa_cfg.lidar_max_range)]

    def get_robot_pose(self):
        '''Get robot pose'''
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y, math.atan2(2*(t.transform.rotation.w*t.transform.rotation.z + t.transform.rotation.x*t.transform.rotation.y), 1 - 2*(t.transform.rotation.y**2 + t.transform.rotation.z**2))
        except: return None

    def world_to_grid(self, x, y):
        '''Transfer meteres into matrix indices'''
        return int((y - self.map_origin[1]) / self.map_resolution), int((x - self.map_origin[0]) / self.map_resolution)
    
    def grid_to_world(self, r, c):
        '''Transfer matrix indices into mteres'''
        return c * self.map_resolution + self.map_origin[0], r * self.map_resolution + self.map_origin[1]
    
    def publish_path_for_rviz(self):
        '''Visualize planned path'''
        msg = Path(); msg.header.frame_id = 'map'; msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.path:
            pose = PoseStamped(); pose.pose.position.x = p[0]; pose.pose.position.y = p[1]; msg.poses.append(pose)
        self.path_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigation()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info("Stopping...")
    finally: node.cmd_vel_pub.publish(Twist()); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
