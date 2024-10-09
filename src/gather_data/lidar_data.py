import rospy
from sensor_msgs.msg import PointCloud2
from pypcd import pypcd


class LidarSensor:
    def __init__(self, topic_name):
        self.topic_name = topic_name

    def save_lidar_data(self, point_cloud_msg, file_path):
        try:
            lid_pc = pypcd.PointCloud.from_msg(point_cloud_msg)
            lid_pc.save_pcd(file_path, compression='binary_compressed')
            rospy.loginfo(f"Lidar data saved as {file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save Lidar data: {str(e)}")

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
