# lidar_module.py
from pypcd import pypcd


class LidarSensor:
    def save_lidar_data(self, point_cloud_msg, file_path):
        lid_pc = pypcd.PointCloud.from_msg(point_cloud_msg)
        lid_pc.save_pcd(file_path, compression='binary_compressed')


if __name__ == "__main__":
    # Initialize DataRetriever and start listening to topics
    data_retriever = DataRetriever()
    data_retriever.start_listening()
