# lidar_module.py
from pypcd import pypcd
import message_filters
from sensor_msgs.msg import PointCloud2
import rospy
import numpy as np
np.float = float


class LidarSensor:
    def __init__(self, topic_name):
        self.topic_name = topic_name

    def start_listener(self):
        lidar_sub = rospy.Subscriber(
            self.topic_name, PointCloud2)
        rospy.loginfo(
            f"Subscribed to {self.topic_name} topic. Waiting for data...")
        return lidar_sub

    def save_lidar_data(self, point_cloud_msg, file_path):
        try:
            lid_pc = pypcd.PointCloud.from_msg(point_cloud_msg)
            lid_pc.save_pcd(file_path, compression='binary_compressed')
            rospy.loginfo(f"Lidar data saved as {file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save Lidar data: {str(e)}")


# Test function for Lidar
rospy.init_node('lidar_listener_node', anonymous=True)
topic_name = "/velodyne_points"  # Update this to match your Lidar point cloud topic
lidar_sensor = LidarSensor(topic_name)


def callback(lidar_msg):
    lidar_sensor.save_lidar_data(lidar_msg, ".pcd")


try:
    lidar_sub = lidar_sensor.start_listener()
except rospy.ROSInterruptException:
    rospy.logerr("ROS node interrupted.")

# ROS message synchronizer
ts = message_filters.ApproximateTimeSynchronizer(
    [lidar_sub], 25, 0.25)
ts.registerCallback(callback)

rospy.spin()
