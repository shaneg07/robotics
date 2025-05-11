import numpy as np
import matplotlib.pyplot as plt
import random
import scipy.ndimage


MAP_SIZE = 50 
OCCUPANCY_GRID = np.zeros((MAP_SIZE, MAP_SIZE))

num_obstacles = int(MAP_SIZE * MAP_SIZE * 0.5) 

for _ in range(num_obstacles):
    x, y = random.randint(0, MAP_SIZE - 1), random.randint(0, MAP_SIZE - 1)
    #is this condition not happening?????
    if ((x == y) and ((x == 0) or (x == MAP_SIZE - 1))):
        break
    OCCUPANCY_GRID[x, y] = 1 

start, goal = (0, 0), (MAP_SIZE - 1, MAP_SIZE - 1)




plt.figure(figsize=(6, 6))
plt.imshow(OCCUPANCY_GRID, cmap="gray_r") 
plt.scatter(*start[::-1], c='green', label="Start") 
plt.scatter(*goal[::-1], c='red', label="Goal")  
plt.show()