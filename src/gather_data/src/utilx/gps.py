# lidar_module.py
import message_filters
from sensor_msgs.msg import NavSatFix
import rospy
import numpy as np
np.float = float


class GPSSensor:
    def __init__(self, topic_name):
        self.topic_name = topic_name

    def start_listener(self):
        loc_sub = message_filters.Subscriber(self.topic_name, NavSatFix)
        rospy.loginfo(
            f"Subscribed to {self.topic_name} topic. Waiting for data...")
        return loc_sub


if __name__ == "__main__":
    print("halo")
    # Test function for GPS
    rospy.init_node('gps_listener_node', anonymous=True)
    topic_name = "latlon"  # Update this to match your gps topic
    lidar_sensor = GPSSensor(topic_name)

    try:
        loc_sub = lidar_sensor.start_listener()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

    # ROS message synchronizer
    ts = message_filters.ApproximateTimeSynchronizer(
        [loc_sub], 25, 0.25)

    rospy.spin()
