'''

Hybrid-SLAM-PF-OGM-DWA-DStarLite

author: "Guan-Ying Chen"

'''

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import math
import heapq
from scipy.ndimage import grey_dilation

np.random.seed(42)

dt = 0.1

sigma_v_squared = 1.0**2
sigma_gamma_squared = (1.0 * np.pi / 180)**2
sigma_r_squared = 0.5**2
sigma_phi_squared = (1.0 * np.pi / 180)**2

"PF DEFINE"
NP = 500
NTh = NP / 5.0

'''MAP DEFINE'''
MAP_SIZE = 500
RESOLUTION = 1.0
LOG_ODDS_FREE = np.log(0.4 / 0.6)
LOG_ODDS_OCC = np.log(0.9 / 0.1)
MAX_RANGE = 100.0
NUM_RAYS = 10

OCCUPANCY_THRESHOLD = 0.65

distance_threshold = 5.0

'''DWA'''
max_accel = 100.0
max_omega_accel = 100.0 * math.pi / 180.0
v_resolution = 4.0
max_speed = 300.0 
min_speed = 0.0  # MODIFIED: Changed from -0.5 to 0.0 to simplify and prevent backward movement.
to_goal_cost_gain = 0.15  # MODIFIED: Increased significantly to prioritize reaching the goal.
omega_resolution = 1.5 * math.pi / 180.0
max_omega = 140.0 * math.pi / 180.0
speed_cost_gain = 0.1 # MODIFIED: Reduced to lessen the penalty for slow speeds, allowing turns.
obstacle_cost_gain = 50.0
path_cost_gain = 1.5
predict_time = 3.0
robot_radius = 8.0

planning_safety_margin = 0.0


def create_wall_as_circles(start_pos, end_pos, radius):
    circles = []
    x1, y1 = start_pos
    x2, y2 = end_pos

    length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    if length == 0:
        return []

    step = radius * 2.0
    num_circles = int(length / step) + 1

    for i in range(num_circles + 1):
        ratio = i / num_circles
        x = x1 + ratio * (x2 - x1)
        y = y1 + ratio * (y2 - y1)
        circles.append((x, y, radius))
    return circles

maze_walls_lines = [
    ((0, 500), (500, 500)),   
    ((0, 0), (500, 0)),       
    ((0, 500), (0, 0)),      
    ((500, 500), (500, 0)),   

    ((50, 450), (100, 450)),
    ((150, 450), (300, 450)),
    ((350, 450), (400, 450)),

    ((0, 400), (50, 400)),
    ((100, 400), (150, 400)),
    ((400, 400), (500, 400)),

    ((150, 350), (200, 350)),
    ((250, 350), (350, 350)),
    ((450, 350), (500, 350)),

    ((0, 300), (50, 300)),
    ((200, 300), (250, 300)),
    ((400, 300), (450, 300)),

    ((50, 250), (100, 250)),
    ((300, 250), (400, 250)),

    ((0, 200), (50, 200)),
    ((100, 200), (200, 200)),
    ((250, 200), (300, 200)),
    ((350, 200), (400, 200)),

    ((50, 150), (100, 150)),
    ((200, 150), (250, 150)),

    ((0, 100), (50, 100)),
    ((150, 100), (200, 100)),
    ((300, 100), (350, 100)),
    ((400, 100), (450, 100)),

    ((100, 50), (150, 50)),
    ((200, 50), (300, 50)),
    ((350, 50), (400, 50)),

    ((50, 400), (50, 300)),
    ((50, 250), (50, 200)),
    ((50, 150), (50, 100)),

    ((100, 450), (100, 400)),
    ((100, 300), (100, 250)),
    ((100, 200), (100, 150)),

    ((150, 450), (150, 400)),
    ((150, 350), (150, 300)),
    ((150, 100), (150, 50)),

    ((200, 350), (200, 250)),
    ((200, 200), (200, 150)),
    ((200, 100), (200, 50)),

    ((250, 450), (250, 350)),
    ((250, 300), (250, 200)),

    ((300, 450), (300, 350)),
    ((300, 250), (300, 200)),
    ((300, 100), (300, 50)),

    ((350, 200), (350, 100)),

    ((400, 450), (400, 400)),
    ((400, 300), (400, 250)),
    ((400, 200), (400, 150)),
    ((400, 100), (400, 50)),

    ((450, 400), (450, 350)),
    ((450, 300), (450, 150)),
]


simulated_obstacles = []
wall_thickness_radius = 5.0  
for start, end in maze_walls_lines:
    simulated_obstacles.extend(create_wall_as_circles(start, end, wall_thickness_radius))


def on_close(event):
    global running
    print("Closing window...")
    running = False

def on_click(event):
    global goal, simulated_obstacles, global_path, last_inflated_map

    if event.inaxes != ax_slam:
        return

    x, y = event.xdata, event.ydata

    if event.button == 1: 
        mx, my = world_to_map(x, y)
        if last_inflated_map is not None and last_inflated_map[my, mx]:
            print("Cannot set goal on an obstacle!")
            return
        
        print(f"New goal set: [{x:.1f}, {y:.1f}]")
        goal = [x, y]
        global_path = None
    elif event.button == 3:  
        obs_radius = 10.0
        print(f"Obstacle added at: ({x:.1f}, {y:.1f}) with radius {obs_radius}")
        simulated_obstacles.append((x, y, obs_radius))

class GlobalPlannerDStarLite:
    class Node:
        def __init__(self, pos):
            self.pos = pos 
            self.g = float('inf')
            self.rhs = float('inf')
            self.is_obstacle = False
            self.parent = None

        def __lt__(self, other):
            return self.pos < other.pos

    def __init__(self, map_size, resolution):
        self.map_size = map_size
        self.resolution = resolution
        self.node_dict = {(x, y): self.Node((x, y)) for x in range(map_size) for y in range(map_size)}
        self.open_list = []
        self.open_set = set()
        self.start_node = None
        self.goal_node = None

    def _world_to_map(self, wx, wy):
        mx = int(wx / self.resolution)
        my = int(wy / self.resolution)
        return mx, my

    def _map_to_world(self, mx, my):
        wx = mx * self.resolution + self.resolution / 2
        wy = my * self.resolution + self.resolution / 2
        return wx, wy

    def _h(self, a, b):
        return abs(a.pos[0] - b.pos[0]) + abs(a.pos[1] - b.pos[1])

    def _cost(self, u, v):
        if v.is_obstacle or u.is_obstacle:
            return float('inf')
        return math.hypot(u.pos[0] - v.pos[0], u.pos[1] - v.pos[1]) 

    def _key(self, s):
        k1 = min(s.g, s.rhs) + self._h(self.start_node, s)
        k2 = min(s.g, s.rhs)
        return (k1, k2)

    def _push_to_open_list(self, node):
        if node not in self.open_set:
            heapq.heappush(self.open_list, (self._key(node), node))
            self.open_set.add(node)

    def _pop_from_open_list(self):
        while self.open_list:
            _, node = heapq.heappop(self.open_list)
            if node in self.open_set:
                self.open_set.remove(node)
                return node
        return None

    def _neighbor(self, u):
        dirs = [[0, 1], [1, 0], [-1, 0], [0, -1], [1, 1], [-1, 1], [1, -1], [-1, -1]] 
        ns = []
        for dx, dy in dirs:
            nx, ny = u.pos[0] + dx, u.pos[1] + dy
            if 0 <= nx < self.map_size and 0 <= ny < self.map_size:
                ns.append(self.node_dict[(nx, ny)])
        return ns

    def _update_vertex(self, u):
        if u != self.goal_node:
            min_rhs = float('inf')
            for s in self._neighbor(u):
                cost = self._cost(u, s) + s.g
                if cost < min_rhs:
                    min_rhs = cost
            u.rhs = min_rhs

        if u in self.open_set:
            self.open_set.remove(u) 

        if u.g != u.rhs:
            self._push_to_open_list(u)

    def _compute_shortest_path(self):
        while self.open_list:

            if self.start_node.rhs == self.start_node.g and self.open_list[0][0] >= self._key(self.start_node):
                break
            k_pop, u = heapq.heappop(self.open_list)
            
            if u not in self.open_set:
                continue
            
            self.open_set.remove(u)

            k_new = self._key(u)

            if k_pop < k_new:
                heapq.heappush(self.open_list, (k_new, u))
                self.open_set.add(u)
                continue

            if u.g > u.rhs:
                u.g = u.rhs
                for s in self._neighbor(u):
                    self._update_vertex(s)
            else:
                u.g = float('inf')
                self._update_vertex(u)
                for s in self._neighbor(u):
                    self._update_vertex(s)

        return self.start_node.g != float('inf')
    
    def _inflate_map(self, obstacle_map, radius):
        grid_radius = int(radius / self.resolution)
        size = grid_radius * 2 + 1
        

        y, x = np.ogrid[-size:size+1, -size:size+1]
        struct = x**2 + y**2 <= size**2
        
        inflated_map = grey_dilation(obstacle_map.astype(np.uint8), footprint=struct)
        
        return inflated_map

    def plan(self, start_pos_world, goal_pos_world, log_odds_map, threshold, robot_radius):
        print("global path planner: start planning...")
        
        log_odds_threshold = np.log(threshold / (1.0 - threshold))
        obstacle_map = log_odds_map > log_odds_threshold

        print("global path planner: inflate the map for safety path planning...")
        inflated_obstacle_map = self._inflate_map(obstacle_map, robot_radius + planning_safety_margin)
        print("global path planner: done inflating the map")

        changed_nodes = []
        for my in range(self.map_size):
            for mx in range(self.map_size):
                node = self.node_dict[(mx, my)]
                is_obs = inflated_obstacle_map[my, mx]
                if node.is_obstacle != is_obs:
                    node.is_obstacle = is_obs
                    changed_nodes.append(node)

        start_mx, start_my = self._world_to_map(start_pos_world[0], start_pos_world[1])
        goal_mx, goal_my = self._world_to_map(goal_pos_world[0], goal_pos_world[1])

        if inflated_obstacle_map[start_my, start_mx]:
            print(f"global path planner: WARN - original start point ({start_mx}, {start_my}) is in the inflated map。")
            q = [(start_mx, start_my)]
            visited = set(q)
            found_new_start = False
            while q:
                cx, cy = q.pop(0)
                if not inflated_obstacle_map[cy, cx]:
                    print(f"global path planner: Found valid start point ({cx}, {cy})")
                    start_mx, start_my = cx, cy
                    found_new_start = True
                    break
                for dx, dy in [[0, 1], [1, 0], [-1, 0], [0, -1], [1, 1], [-1, 1], [1, -1], [-1, -1]]:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.map_size and 0 <= ny < self.map_size and (nx, ny) not in visited:
                        q.append((nx, ny))
                        visited.add((nx, ny))
            if not found_new_start:
                print("global path planner: Error - Can not find a valid start point. planning failed")
                return None, inflated_obstacle_map

        if inflated_obstacle_map[goal_my, goal_my]:
            print(f"global path planner: WARN - goal node ({goal_mx}, {goal_my}) is in the inflated map. Force to set it valid。")
            inflated_obstacle_map[goal_my, goal_my] = False

        self.start_node = self.node_dict[(start_mx, start_my)]
        self.goal_node = self.node_dict[(goal_mx, goal_my)]

        self.open_list = []
        self.open_set = set()
        for node in self.node_dict.values():
            node.g = float('inf')
            node.rhs = float('inf')
        self.goal_node.rhs = 0
        self._push_to_open_list(self.goal_node)

        if changed_nodes:
            print(f"global path planner: {len(changed_nodes)} nodes of the map changed. Update vertex...")
            for u in changed_nodes:
                self._update_vertex(u)

        print("global path planner: calculating the shortest path...")
        path_found = self._compute_shortest_path()
        if not path_found:
            print("global path planner: can not find a path！")
            return None, inflated_obstacle_map

        path = []
        current = self.start_node
        path_length_limit = self.map_size * self.map_size
        while current != self.goal_node and len(path) < path_length_limit:
            path.append(self._map_to_world(current.pos[0], current.pos[1]))
            neighbors = self._neighbor(current)
            if not neighbors:
                print("global path planner: Failed rebuilding the path（no neighbor）")
                return None, inflated_obstacle_map

            best_next_node = min(neighbors, key=lambda s: self._cost(current, s) + s.g)
            if best_next_node.g == float('inf'):
                print("global path planner: Failed rebuilding the path（path interruption）")
                return None, inflated_obstacle_map
            current = best_next_node

        if len(path) >= path_length_limit:
            print("global path planner: Path reconstruction failed (too long).")
            return None, inflated_obstacle_map

        path.append(self._map_to_world(self.goal_node.pos[0], self.goal_node.pos[1]))
        print(f"global path planner: Find a path containing {len(path)} path points。")
        return path, inflated_obstacle_map

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.omega = omega

def world_to_map(x, y):
    mx = int(x / RESOLUTION)
    my = int(y / RESOLUTION)
    return mx, my

def bresenham(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    err = dx - dy

    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return points

def update_map(log_odds_map, robot_pose, ray_endpoints, hit_info):
    x, y, theta = robot_pose
    mx0, my0 = world_to_map(x, y)

    for i, (rx, ry) in enumerate(ray_endpoints):
        mx1, my1 = world_to_map(rx, ry)

        free_cells = bresenham(mx0, my0, mx1, my1)
        for cx, cy in free_cells[:-1]:
            if 0 <= cx < MAP_SIZE and 0 <= cy < MAP_SIZE:
                log_odds_map[cy, cx] += LOG_ODDS_FREE

        if hit_info[i]:
            if 0 <= mx1 < MAP_SIZE and 0 <= my1 < MAP_SIZE:
                log_odds_map[my1, mx1] += LOG_ODDS_OCC

    return log_odds_map

def motion_model(x, u):
    x_new = np.zeros(3)
    x_new[0] = x[0] + u[0] * math.cos(x[2]) * dt
    x_new[1] = x[1] + u[0] * math.sin(x[2]) * dt
    x_new[2] = x[2] + u[1] * dt 

    x_new[2] = (x_new[2] + np.pi) % (2 * np.pi) - np.pi

    return x_new

def get_observations(robot_pos, robot_theta, max_range, obstacles):
    observations = []
    ray_endpoints = []
    hit_info = []
    ray_angles = np.linspace(-np.pi/2, np.pi/2, NUM_RAYS) + robot_theta

    for angle in ray_angles:
        min_dist = max_range
        is_hit = False

        nearby_obstacles = []
        for ox, oy, r in obstacles:
            if abs(robot_pos[0] - ox) < max_range + r and abs(robot_pos[1] - oy) < max_range + r:
                 nearby_obstacles.append((ox, oy, r))

        for ox, oy, r in nearby_obstacles:
            a = 1
            b = 2 * ((robot_pos[0] - ox) * np.cos(angle) + (robot_pos[1] - oy) * np.sin(angle))
            c = (robot_pos[0] - ox)**2 + (robot_pos[1] - oy)**2 - r**2

            delta = b**2 - 4*a*c
            if delta >= 0:
                d1 = (-b - np.sqrt(delta)) / (2*a)
                if 0 < d1 < min_dist:
                    min_dist = d1
                    is_hit = True

        hit_info.append(is_hit)

        end_x = robot_pos[0] + min_dist * np.cos(angle)
        end_y = robot_pos[1] + min_dist * np.sin(angle)
        ray_endpoints.append((end_x, end_y))

        noisy_range = min_dist + np.random.normal(0, np.sqrt(sigma_r_squared))
        bearing = angle - robot_theta
        noisy_bearing = bearing + np.random.normal(0, np.sqrt(sigma_phi_squared))
        noisy_bearing = (noisy_bearing + np.pi) % (2 * np.pi) - np.pi
        observations.append([noisy_range, noisy_bearing])

    return np.array(observations), ray_endpoints, hit_info

def calculate_weights(px, pw, z, log_odds_map):
    for i in range(NP):
        particle_pose = px[:, i]
        weight = 1.0

        for obs in z:
            obs_range, obs_bearing = obs[0], obs[1]
            obs_x = particle_pose[0] + obs_range * math.cos(particle_pose[2] + obs_bearing)
            obs_y = particle_pose[1] + obs_range * math.sin(particle_pose[2] + obs_bearing)
            mx, my = world_to_map(obs_x, obs_y)

            if 0 <= mx < MAP_SIZE and 0 <= my < MAP_SIZE:
                prob_occ = 1.0 - 1.0 / (1.0 + np.exp(log_odds_map[my, mx]))
                weight *= (prob_occ + 1e-10)
            else:
                weight *= 1e-10

        pw[0, i] = weight

    if pw.sum() < 1e-100:
        pw = np.full((1, NP), 1.0 / NP)
    else:
        pw = pw / pw.sum()
    return pw

def re_sampling(px, pw):
    w_cum = np.cumsum(pw)
    base = np.arange(0.0, 1.0, 1.0/NP)
    re_sample_id = base + np.random.uniform(0, 1.0/NP)

    indexes = []
    ind = 0
    for ip in range(NP):
        while re_sample_id[ip] > w_cum[ind]:
            ind += 1
        indexes.append(ind)

    px = px[:, indexes]
    pw = np.full((1, NP), 1.0 / NP)
    return px, pw

def extract_obstacles_smart_filtering(log_odds_map, resolution, threshold, robot_pos, max_distance=40.0):
    obstacles = []
    height, width = log_odds_map.shape

    log_odds_threshold = np.log(threshold / (1.0 - threshold))

    robot_mx, robot_my = int(robot_pos[0] / resolution), int(robot_pos[1] / resolution)

    search_radius = int(max_distance / resolution)

    min_x = max(0, robot_mx - search_radius)
    max_x = min(width, robot_mx + search_radius)
    min_y = max(0, robot_my - search_radius)
    max_y = min(height, robot_my + search_radius)

    downsample_factor = 1

    for my in range(min_y, max_y, downsample_factor):
        for mx in range(min_x, max_x, downsample_factor):
            if log_odds_map[my, mx] > log_odds_threshold:
                world_x = mx * resolution + resolution / 2.0
                world_y = my * resolution + resolution / 2.0

                dist_to_robot = math.sqrt((world_x - robot_pos[0])**2 + (world_y - robot_pos[1])**2)
                if dist_to_robot <= max_distance:
                    obstacles.append((world_x, world_y))

    return obstacles

def dwa_control(state, goal, obstacles, global_path, ax):
    dw = {
        "v_min": max(min_speed, state.v - max_accel * dt),
        "v_max": min(max_speed, state.v + max_accel * dt),
        "omega_min": max(-max_omega, state.omega - max_omega_accel * dt),
        "omega_max": min(max_omega, state.omega + max_omega_accel * dt),
    }
    best_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([[state.x, state.y]])

    v_samples = set(np.arange(dw["v_min"], dw["v_max"], v_resolution))
    v_samples.add(0.0)

    for v in v_samples:
        for omega in np.arange(dw["omega_min"], dw["omega_max"], omega_resolution):
            trajectory = predict_trajectory(state, v, omega)
            to_goal_cost = to_goal_cost_gain * calc_to_goal_cost_optimized(trajectory, goal)
            speed_cost = speed_cost_gain * (max_speed - trajectory[-1, 3])
            obstacle_cost = obstacle_cost_gain * calc_obstacle_cost(trajectory, obstacles)

            path_deviation_cost = 0.0
            if global_path: 
                path_deviation_cost = path_cost_gain * calc_path_cost(trajectory, global_path)

            final_cost = to_goal_cost + speed_cost + obstacle_cost + path_deviation_cost

            if final_cost < best_cost:
                best_cost = final_cost
                best_u = [v, omega]
                best_trajectory = trajectory

    return best_u, best_trajectory

def predict_trajectory(state, v, omega):
    current_state = np.array([state.x, state.y, state.yaw, state.v, state.omega])
    trajectory = np.array(current_state)
    time = 0
    while time <= predict_time:
        current_state[2] += omega * dt
        current_state[0] += v * math.cos(current_state[2]) * dt
        current_state[1] += v * math.sin(current_state[2]) * dt
        current_state[3] = v
        current_state[4] = omega
        trajectory = np.vstack((trajectory, current_state))
        time += dt
    return trajectory

def calc_obstacle_cost(trajectory, obstacles):
    min_dist = float("inf")
    for i in range(len(trajectory)):
        for ox, oy in obstacles:
            dist = math.sqrt((trajectory[i, 0] - ox)**2 + (trajectory[i, 1] - oy)**2)

            if dist <= robot_radius:
                return float("inf")

            min_dist = min(min_dist, dist)

    if min_dist == float("inf"):
        return 0.0

    return 1.0 / min_dist

def calc_to_goal_cost_optimized(trajectory, goal):
    HEADING_WEIGHT = 6.0 
    DISTANCE_WEIGHT = 1.0

    final_pos = trajectory[-1, 0:2]
    final_theta = trajectory[-1, 2]

    dist_to_goal = math.hypot(goal[0] - final_pos[0], goal[1] - final_pos[1])
    distance_cost = DISTANCE_WEIGHT * dist_to_goal

    angle_to_goal = math.atan2(goal[1] - final_pos[1], goal[0] - final_pos[0])
    
    angle_diff = final_theta - angle_to_goal
    
    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
    
    heading_cost = HEADING_WEIGHT * abs(angle_diff)

    return distance_cost + heading_cost

def find_closest_path_segment(global_path, point):
    min_dist_sq = float('inf')
    closest_segment_index = 0

    path_np = np.array(global_path)
    point_np = np.array(point)

    for i in range(len(path_np) - 1):
        p1 = path_np[i]
        p2 = path_np[i+1]

        line_vec = p2 - p1
        pnt_vec = point_np - p1
        line_len_sq = np.dot(line_vec, line_vec)

        if line_len_sq == 0:
            dist_sq = np.dot(pnt_vec, pnt_vec)
        else:
            t = max(0, min(1, np.dot(pnt_vec, line_vec) / line_len_sq))
            closest_point_on_segment = p1 + t * line_vec
            dist_sq = np.dot(point_np - closest_point_on_segment, point_np - closest_point_on_segment)

        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            closest_segment_index = i

    return math.sqrt(min_dist_sq), closest_segment_index

def calc_path_cost(trajectory, global_path):
    final_pos = trajectory[-1, 0:2]

    dist, _ = find_closest_path_segment(global_path, final_pos)

    return dist

def get_dwa_target(global_path, robot_pose, lookahead_distance=30.0):
    if not global_path:
        return None

    robot_pos_np = np.array(robot_pose[:2])
    path_np = np.array(global_path)
    distances = np.linalg.norm(path_np - robot_pos_np, axis=1)
    closest_index = np.argmin(distances)

    target_index = closest_index
    total_dist = 0
    for i in range(closest_index, len(global_path) - 1):
        total_dist += np.linalg.norm(path_np[i+1] - path_np[i])
        if total_dist >= lookahead_distance:
            target_index = i + 1
            break
    else:
        target_index = len(global_path) - 1

    return global_path[target_index]

def is_path_obstructed(path, inflated_obstacle_map, map_size):
    if not path or inflated_obstacle_map is None:
        return False
    
    for wx, wy in path:
        mx, my = world_to_map(wx, wy)

        if not (0 <= mx < map_size and 0 <= my < map_size):
            continue

        
        if inflated_obstacle_map[my, mx]:
            print(f"Path obstructed on INFLATED map at cell ({mx}, {my}).")
            return True

    return False

car_pos = np.array([50.0, 50.0])
car_theta = np.deg2rad(45.0)

px = np.zeros((3, NP))
px[0, :] = car_pos[0] + np.random.randn(NP) * 10.0
px[1, :] = car_pos[1] + np.random.randn(NP) * 10.0
px[2, :] = car_theta + np.random.randn(NP) * np.deg2rad(5.0)
pw = np.full((1, NP), 1.0 / NP)

x_est = np.mean(px, axis=1)

trajectory_true = [car_pos.copy()]
trajectory_pf = [[x_est[0], x_est[1]]]

log_odds_map = np.zeros((MAP_SIZE, MAP_SIZE))

plt.ion()
fig = plt.figure(figsize=(16, 8))
fig.canvas.mpl_connect('close_event', on_close)
fig.canvas.mpl_connect('button_press_event', on_click)
ax_slam = fig.add_subplot(121)
ax_ogm = fig.add_subplot(122)

ax_slam.set_title('Interactive PF-SLAM with DWA (Planning from Map)')
ax_slam.set_xlim(0, MAP_SIZE); ax_slam.set_ylim(0, MAP_SIZE)
ax_slam.grid(True); ax_slam.set_aspect('equal', adjustable='box')

sim_obstacle_patches = []
for ox, oy, r in simulated_obstacles:
    patch = Circle((ox, oy), radius=r, color='gray', fill=True, alpha=0.3)
    ax_slam.add_patch(patch)
    sim_obstacle_patches.append(patch)

prob_map_display = 1 - 1 / (1 + np.exp(log_odds_map))
ogm_img = ax_ogm.imshow(prob_map_display, cmap='gray_r', origin='lower', vmin=0, vmax=1)
robot_dot_ogm, = ax_ogm.plot([], [], 'go', markersize=4, label="Est. Robot")
map_obstacles_plot, = ax_slam.plot([], [], 's', color='orange', markersize=2, alpha=0.5, label='Map Obstacles')


goal = None
u = [0.0, 0.0]

running = True

dynamic_artists = []

global_planner = GlobalPlannerDStarLite(MAP_SIZE, RESOLUTION)
global_path = None  
dwa_target = None   
replan_threshold = 15.0 
last_inflated_map = None 

CHECK_PATH_INTERVAL = 10
check_path_counter = 0

frame_count = 0
plot_update_interval = 10

if 'global_path_line' not in locals():
    global_path_line, = ax_slam.plot([], [], 'c-', linewidth=2, label='Global Path')
    dwa_target_marker, = ax_slam.plot([], [], 'm*', markersize=10, label='DWA Target')

while running:
    if goal is None:
        plt.pause(0.1)
        continue

    needs_replan = False
    if global_path:
        robot_pos_np = np.array(x_est[:2])
        path_np = np.array(global_path)
        dist_to_path, _ = find_closest_path_segment(global_path, robot_pos_np)
        if dist_to_path > replan_threshold:
            print(f"Deviated from path by {dist_to_path:.2f} > {replan_threshold}. Triggering replan.")
            needs_replan = True

        check_path_counter += 1
        if not needs_replan and check_path_counter >= CHECK_PATH_INTERVAL:
            check_path_counter = 0 
            print("Performing periodic path obstruction check...")
            last_inflated_map = global_planner._inflate_map(log_odds_map > np.log(OCCUPANCY_THRESHOLD / (1.0 - OCCUPANCY_THRESHOLD)), robot_radius + planning_safety_margin)
            if not needs_replan and is_path_obstructed(global_path, last_inflated_map, MAP_SIZE):
                print("Path is now obstructed by new obstacles. Triggering replan.")
                needs_replan = True

    if global_path is None or needs_replan:
        total_inflation_radius = robot_radius + planning_safety_margin
        print(f"Global planner: Using total inflation radius of {total_inflation_radius:.1f}")

        new_path, inflated_map = global_planner.plan(x_est, goal, log_odds_map, OCCUPANCY_THRESHOLD, total_inflation_radius)
        last_inflated_map = inflated_map 

        if new_path:
            global_path = new_path
        else:
            print("Could not find a global path. Robot will stop.")
            goal = None 
            global_path = None
            u = [0.0, 0.0]
            continue

    dwa_target = get_dwa_target(global_path, x_est)
    if dwa_target is None:
        print("Could not get DWA target, might be at the end of path.")
        dwa_target = goal

    dist_to_final_goal = math.sqrt((x_est[0] - goal[0])**2 + (x_est[1] - goal[1])**2)
    if dist_to_final_goal < distance_threshold:
        print("Final Goal reached!")
        goal = None
        global_path = None
        last_inflated_map = None
        u = [0.0, 0.0]  
        continue

    map_obstacles = extract_obstacles_smart_filtering(log_odds_map, RESOLUTION, OCCUPANCY_THRESHOLD, x_est)

    current_state = State(x=x_est[0], y=x_est[1], yaw=x_est[2], v=u[0], omega=u[1])
    u, best_trajectory = dwa_control(current_state, dwa_target, map_obstacles, global_path, None)

    car_pos[0] += u[0] * math.cos(car_theta) * dt
    car_pos[1] += u[0] * math.sin(car_theta) * dt
    car_theta += u[1] * dt
    car_theta = (car_theta + np.pi) % (2 * np.pi) - np.pi
    trajectory_true.append(car_pos.copy())

    observations, ray_endpoints, hit_info = get_observations(car_pos, car_theta, MAX_RANGE, simulated_obstacles)

    for i in range(NP):
        v_p = u[0] + np.random.normal(0, np.sqrt(sigma_v_squared))
        omega_p = u[1] + np.random.normal(0, np.sqrt(sigma_gamma_squared))
        px[:, i] = motion_model(px[:, i], np.array([v_p, omega_p]))

    pw = calculate_weights(px, pw, observations, log_odds_map)
    x_est = (px @ pw.T).flatten()
    x_est[2] = math.atan2(np.sum(pw * np.sin(px[2, :])), np.sum(pw * np.cos(px[2, :])))
    trajectory_pf.append([x_est[0], x_est[1]])

    log_odds_map = update_map(log_odds_map, x_est, ray_endpoints, hit_info)

    if 1.0 / (pw @ pw.T)[0, 0] < NTh:
        px, pw = re_sampling(px, pw)

    frame_count += 1
    if frame_count % plot_update_interval == 0:

        if 'true_traj_line' not in locals():
            ax_slam.legend(loc='upper left', fontsize='small')
            true_traj_line, = ax_slam.plot([], [], c='red', linestyle='-', label='True Trajectory')
            pf_traj_line, = ax_slam.plot([], [], c='blue', linestyle='--', label='PF Trajectory')
            pf_est_marker, = ax_slam.plot([], [], 'gx', markersize=10, mew=2, label='PF Estimate')
            particles_scatter = ax_slam.scatter([], [], c='orange', marker='o', s=6, alpha=0.6, label='Particles')
            robot_body_patch = Circle((0, 0), radius=robot_radius, color='blue', fill=False, alpha=0.8, linewidth=1.5)
            ax_slam.add_patch(robot_body_patch)
            best_traj_line, = ax_slam.plot([], [], color="lime", linewidth=2.0, label='DWA Best Traj.')
            goal_marker, = ax_slam.plot([], [], 'gx', markersize=15, mew=3, label='Current Goal')

        if global_path:
            path_x, path_y = zip(*global_path)
            global_path_line.set_data(path_x, path_y)
        else:
            global_path_line.set_data([], [])

        if dwa_target:
            dwa_target_marker.set_data([dwa_target[0]], [dwa_target[1]])
            dwa_target_marker.set_visible(True)
        else:
            dwa_target_marker.set_visible(False)

        true_traj_x, true_traj_y = zip(*trajectory_true) if trajectory_true else ([], [])
        true_traj_line.set_data(true_traj_x, true_traj_y)

        pf_traj_x, pf_traj_y = zip(*trajectory_pf) if trajectory_pf else ([], [])
        pf_traj_line.set_data(pf_traj_x, pf_traj_y)

        pf_est_marker.set_data([x_est[0]], [x_est[1]])
        particles_scatter.set_offsets(px[:2, :].T)

        robot_body_patch.set_center((x_est[0], x_est[1]))
        if best_trajectory is not None and len(best_trajectory) > 0:
            best_traj_line.set_data(best_trajectory[:, 0], best_trajectory[:, 1])
        else:
            best_traj_line.set_data([], [])

        if len(sim_obstacle_patches) < len(simulated_obstacles):
            newly_added_obstacles = simulated_obstacles[len(sim_obstacle_patches):]
            for ox, oy, r in newly_added_obstacles:
                patch = Circle((ox, oy), radius=r, color='darkred', fill=True, alpha=0.3)
                ax_slam.add_patch(patch)
                sim_obstacle_patches.append(patch)

        if map_obstacles:
            map_obs_x, map_obs_y = zip(*map_obstacles)
            map_obstacles_plot.set_data(map_obs_x, map_obs_y)
        else:
            map_obstacles_plot.set_data([], [])

        if goal:
            goal_marker.set_data([goal[0]], [goal[1]])
            goal_marker.set_visible(True)
        else:
            goal_marker.set_visible(False)

        prob_map_display = 1 - 1 / (1 + np.exp(log_odds_map))
        ogm_img.set_data(prob_map_display)
        rx_map, ry_map = world_to_map(x_est[0], x_est[1])
        robot_dot_ogm.set_data([rx_map], [ry_map])

        ax_slam.legend(loc='upper left', fontsize='small')

        fig.canvas.draw(); fig.canvas.flush_events()
    plt.pause(0.01)

print("Simulation finished.")
plt.ioff()
plt.show()