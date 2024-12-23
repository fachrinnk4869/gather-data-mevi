# lidar_module.py
import message_filters
from sensor_msgs.msg import NavSatFix
import rospy
import numpy as np
np.float = float


class GPSSensor:
    def __init__(self, topic_name):
        self.topic_name = topic_name

        # Initialize placeholders for RGB, depth, and pose data
        self.location = None
        rospy.Subscriber(self.topic_name, NavSatFix,
                         self.callback)

    def start_listener(self):
        loc_sub = rospy.Subscriber(self.topic_name, NavSatFix)
        rospy.loginfo(
            f"Subscribed to {self.topic_name} topic. Waiting for data...")
        return loc_sub

    def get_location(self):
        # Retrieve the latest RGB and Depth images if available
        if self.rgb_image is not None:
            rospy.loginfo(f"Received GPS data: {self.location}")
            return self.location
        else:
            rospy.logwarn("RGB or Depth image not available yet.")
            return None, None

    def callback(self, locsub):
        self.location = locsub
        rospy.loginfo(f"Received GPS data: {self.location}")


if __name__ == "__main__":
    rospy.init_node('location_subscriber', anonymous=True)
    try:
        zed_subscriber = GPSSensor(
            topic_name='/latlon1')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
