# lidar_module.py
from pypcd import pypcd


class LidarSensor:
    def save_lidar_data(self, point_cloud_msg, file_path):
        lid_pc = pypcd.PointCloud.from_msg(point_cloud_msg)
        lid_pc.save_pcd(file_path, compression='binary_compressed')


# Test function for Lidar
if __name__ == "__main__":
    # Assuming point_cloud_msg is available for testing
    lidar_sensor = LidarSensor()
    # Replace `point_cloud_msg` with actual PointCloud2 data for testing
    test_point_cloud_msg = None  # Set this to valid PointCloud2 data for testing
    if test_point_cloud_msg is not None:
        lidar_sensor.save_lidar_data(test_point_cloud_msg, "test_lidar.pcd")
        print("Lidar point cloud data saved as test_lidar.pcd")
    else:
        print("No valid point cloud data for testing.")
