import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
import open3d as o3d


class LidarSensor:
    def __init__(self, topic_name):
        self.topic_name = topic_name

    def save_lidar_data(self, point_cloud_msg, file_path):
        try:
            # Convert PointCloud2 message to numpy array using ros_numpy
            pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(point_cloud_msg)

            # Extract x, y, z points and filter out invalid data (NaNs or infinities)
            points = np.zeros((pc_data.shape[0], 3), dtype=np.float32)
            points[:, 0] = pc_data['x']
            points[:, 1] = pc_data['y']
            points[:, 2] = pc_data['z']
            points = points[~np.isnan(points).any(axis=1)]  # Remove NaN points

            # Create Open3D point cloud object
            o3d_cloud = o3d.geometry.PointCloud()
            o3d_cloud.points = o3d.utility.Vector3dVector(points)

            # Save point cloud as PCD file
            o3d.io.write_point_cloud(file_path, o3d_cloud, write_ascii=False)

            rospy.loginfo(f"Lidar data saved as {file_path}")

            # Print saved data
            self.print_saved_data(file_path)

        except Exception as e:
            rospy.logerr(f"Failed to save Lidar data: {str(e)}")

    def print_saved_data(self, file_path):
        # Load the saved point cloud
        o3d_cloud = o3d.io.read_point_cloud(file_path)

        # Get points as numpy array
        points = np.asarray(o3d_cloud.points)

        # Print basic information about the point cloud
        print(f"Loaded {file_path}:")
        print(f"Number of points: {points.shape[0]}")
        print(f"Points (first 5): \n{points}")

    def lidar_callback(self, msg):
        rospy.loginfo("Received Lidar point cloud data")
        self.save_lidar_data(msg, "test_lidar.pcd")

    def start_listener(self):
        rospy.init_node('lidar_listener_node', anonymous=True)
        rospy.Subscriber(self.topic_name, PointCloud2, self.lidar_callback)
        rospy.loginfo(
            f"Subscribed to {self.topic_name} topic. Waiting for data...")
        rospy.spin()


# Test function for Lidar with ROS
if __name__ == "__main__":
    topic_name = "/velodyne_points"  # Update this to match your Lidar point cloud topic
    lidar_sensor = LidarSensor(topic_name)
    try:
        lidar_sensor.start_listener()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
