import numpy as np
import random
import heapq
import matplotlib.pyplot as plt

# --- Constants ---
FREE = 0
OBSTACLE = 1
START = 2
GOAL = 3
PATH = 4
UNKNOWN = -1

# --- World Generation ---
def generate_world(rows, cols, obstacle_prob=0.3):
    while True:
        world = np.random.choice([FREE, OBSTACLE], size=(rows, cols), p=[1 - obstacle_prob, obstacle_prob])
        start = (0, 0)
        goal = (rows - 1, cols - 1)
        world[start] = START
        world[goal] = GOAL
        if is_reachable(world, start, goal):
            return world, start, goal

def is_reachable(grid, start, goal):
    stack = [start]
    visited = set()
    rows, cols = grid.shape
    while stack:
        r, c = stack.pop()
        if (r, c) == goal:
            return True
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] != OBSTACLE and (nr, nc) not in visited:
                visited.add((nr, nc))
                stack.append((nr, nc))
    return False

# --- Sensor Model ---
def sensor_scan(true_map, robot_pos, sensor_range):
    visible = []
    rows, cols = true_map.shape
    r0, c0 = robot_pos
    for dr in range(-sensor_range, sensor_range + 1):
        for dc in range(-sensor_range, sensor_range + 1):
            rr, cc = r0 + dr, c0 + dc
            if 0 <= rr < rows and 0 <= cc < cols:
                visible.append(((rr, cc), true_map[rr, cc]))
    return visible

# --- Map Updating ---
def update_internal_map(internal_map, sensor_data):
    for (r, c), val in sensor_data:
        if internal_map[r, c] == UNKNOWN:
            internal_map[r, c] = val

# --- A* Planning ---
def a_star_on_known_map(internal_map, start, goal):
    rows, cols = internal_map.shape
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start, []))
    visited = set()

    while open_set:
        f, cost, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        path = path + [current]

        if current == goal:
            return path

        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = current[0] + dr, current[1] + dc
            if (
                0 <= nr < rows and 0 <= nc < cols and 
                internal_map[nr, nc] in [FREE, START, GOAL] and 
                (nr, nc) not in visited
            ):
                heapq.heappush(open_set, (cost + 1 + heuristic((nr, nc), goal), cost + 1, (nr, nc), path))
    
    return []  # No path found

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# --- SLAM Loop ---
def slam_loop(true_map, internal_map, start, goal, sensor_range, max_steps=1000):
    robot_pos = start
    path_history = [robot_pos]

    for step in range(max_steps):
        # 1. Sense
        sensor_data = sensor_scan(true_map, robot_pos, sensor_range)

        # 2. Update map
        update_internal_map(internal_map, sensor_data)

        # 3. Plan
        path = a_star_on_known_map(internal_map, robot_pos, goal)
        if not path:
            continue  # Still exploring

        # 4. Move
        if len(path) > 1:
            next_pos = path[1]
        else:
            next_pos = path[0]
        robot_pos = next_pos
        path_history.append(robot_pos)

        if robot_pos == goal:
            print(f"🎯 Reached goal in {len(path_history)} steps!")
            break

    return path_history

import matplotlib.colors as mcolors

def plot_grid(grid, title="Grid"):
    # Map cell values to numeric color indices
    color_map = {
        UNKNOWN: 0,
        FREE: 1,
        OBSTACLE: 2,
        START: 3,
        GOAL: 4,
        PATH: 5
    }

    # Define a list of colors matching the indices
    color_list = ["gray", "white", "black", "green", "red", "blue"]
    cmap = mcolors.ListedColormap(color_list)

    # Create a numeric grid for imshow
    numeric_grid = np.vectorize(color_map.get)(grid)

    plt.figure(figsize=(6, 6))
    plt.title(title)
    plt.imshow(numeric_grid, cmap=cmap)
    plt.axis("off")
    plt.show()


# --- Run Everything ---
if __name__ == "__main__":
    rows, cols = 20, 20
    world, start, goal = generate_world(rows, cols, obstacle_prob=0.25)
    internal_map = np.full_like(world, UNKNOWN)

    path_taken = slam_loop(world, internal_map, start, goal, sensor_range=2)

    # Visualize final internal map with path
    for r, c in path_taken:
        if internal_map[r, c] == FREE:
            internal_map[r, c] = PATH

    internal_map[start] = START
    internal_map[goal] = GOAL

    plot_grid(internal_map, "Robot's Final Map with Path")
