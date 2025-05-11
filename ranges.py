import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob

def load_coordinates_from_files(directory):
    """
    Loads 3D coordinates from multiple .npy files in a given directory.
    :param directory: Path to directory containing .npy files.
    :return: List of (x, y, z) tuples.
    """
    file_paths = sorted(glob.glob(f"{directory}/*.npy"))
    points = [np.load(file_path) for file_path in file_paths]
    return np.array(points)

def plot_trajectory(points, distances_list):
    """
    Plots a 3D trajectory with wall points based on distance readings.
    :param points: NumPy array of shape (N, 3)
    :param distances_list: List of NumPy arrays containing distances for each coordinate
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    for (x, y, z), distances in zip(points, distances_list):
        angles = np.linspace(0, 360, 1081) 
        angles_rad = np.radians(angles)
        
        wall_x = x + distances * np.cos(angles_rad)
        wall_y = y + distances * np.sin(angles_rad)
        wall_z = np.full_like(wall_x, z) 
        
        ax.scatter(wall_x, wall_y, wall_z, color='r', s=1, alpha=0.5)
    
    x, y, z = points[:, 0], points[:, 1], points[:, 2] * 0.5 
    ax.plot(x, y, z, linestyle='-', color='b', label='Robot Path')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Robot Trajectory with Walls')
    ax.legend()
    
    plt.show()

directory = "C:/Users/shane/Desktop/robotics/2024-04-11-14-37-14/positions"
data_points = load_coordinates_from_files(directory)
distance_files = sorted(glob.glob(f"C:/Users/shane/Desktop/robotics/2024-04-11-14-37-14/scans_lidar"))
distances_list = [np.load(file) for file in distance_files]  
plot_trajectory(data_points, distances_list)
