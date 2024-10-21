# lidar_module.py
import numpy as np
import open3d as o3d
import numpy as np
np.float = float


def print_saved_data(file_path):
    # Load the saved point cloud
    o3d_cloud = o3d.io.read_point_cloud(file_path)

    # Get points as numpy array
    points = np.asarray(o3d_cloud.points)

    # Print basic information about the point cloud
    print(f"Loaded {file_path}:")
    print(f"Number of points: {points.shape}")
    print(f"Points: \n{points}")


# Test function for Lidar
if __name__ == "__main__":
    print_saved_data("./1681353255_0000015281.pcd")
    print_saved_data("./test_lidar.pcd")
