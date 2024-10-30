# lidar_module.py
import message_filters
from sensor_msgs.msg import NavSatFix
import rospy
import numpy as np
np.float = float


class GPSSensor:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.latest_gps_data = None
        rospy.Subscriber(self.topic_name, NavSatFix, self.gps_callback)

    def gps_callback(self, msg):
        # Update the latest Gps data when a new message is received
        self.latest_gps_data = msg

    def get_latest_gps_data(self):
        # Return the latest Gps data
        return self.latest_gps_data


if __name__ == "__main__":
    print("halo")
    # Test function for GPS
    rospy.init_node('gps_listener_node', anonymous=True)
    topic_name = "/latlon1"  # Update this to match your gps topic
    lidar_sensor = GPSSensor(topic_name)
    rospy.spin()
