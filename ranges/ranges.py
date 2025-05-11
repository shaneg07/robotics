import numpy as np
import matplotlib.pyplot as plt
import glob

def load_coordinates_from_files(directory):

    file_paths = sorted(glob.glob(f"{directory}/*.npy"))
    points = [np.load(file_path) for file_path in file_paths]  # Only take x, y
    return np.array(points)

def plot_trajectory(points, distances_list):

    fig, ax = plt.subplots()
    
    for (x, y), distances in zip(points, distances_list):
        angles = np.linspace(0, 360, 1081)  # 1081 points, each 0.333 degrees apart
        angles_rad = np.radians(angles)
        
        wall_x = x + distances * np.cos(angles_rad)
        wall_y = y + distances * np.sin(angles_rad)
        
        ax.scatter(wall_x, wall_y, color='r', s=1, alpha=0.5)  # Plot walls
    
    x, y = points[:, 0], points[:, 1]
    ax.plot(x, y, linestyle='-', color='b', label='Robot Path')
    
    # Labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('2D Robot Trajectory with Walls')
    ax.legend()
    
    plt.axis('equal')  # Ensure equal scaling
    plt.show()

# Example usage:
directory = "C:/Users/shane/Desktop/robotics/test_10/"
data_points = load_coordinates_from_files(directory)
print(data_points[0])
print("Len of one array is = " + str(len(data_points[0])))
# distance_files = sorted(glob.glob(f"C:/Users/shane/Desktop/robotics/test_10/d_*.npy"))
# distances_list = [np.load(file) for file in distance_files]  # Load corresponding distances
# plot_trajectory(data_points, distances_list)
