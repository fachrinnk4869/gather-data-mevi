#!/usr/bin/env python
import pyzed.sl as sl
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
from cv_bridge import CvBridge

# ROS Subscriber class


class ZEDCameraSubscriber:
    def __init__(self, rgb_topic='/zed/rgb_image', pose_topic='/zed/pose'):
        self.rgb_topic = rgb_topic
        self.pose_topic = pose_topic
        self.bridge = CvBridge()

        # Subscribe to RGB and Pose topics
        rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)

        # Placeholder for received data
        self.rgb_image = None
        self.pose_position = None
        self.pose_orientation = None
    def get_pose(self):
        # Retrieve the latest pose data if available
        if self.pose_position is not None and self.pose_orientation is not None:
            return self.pose_position, self.pose_orientation
        else:
            rospy.logwarn("Pose data not available yet.")
            return None, None
    def get_frame(self):
        # Retrieve the latest RGB and Depth images if available
        if self.rgb_image is not None:
            return self.rgb_image
        else:
            rospy.logwarn("RGB or Depth image not available yet.")
            return None

    def rgb_callback(self, rgb_msg):
        # Convert RGB Image message to OpenCV format
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv2.imshow("RGB Image", self.rgb_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Failed to process RGB image: {e}")

    def pose_callback(self, pose_msg):
        # Store PoseStamped message's position and orientation
        self.pose_position = pose_msg.pose.position
        self.pose_orientation = pose_msg.pose.orientation
        # rospy.loginfo(
        #     f"Received Pose: Position={self.pose_position}, Orientation={self.pose_orientation}")


if __name__ == '__main__':
    rospy.init_node('zed_subscriber', anonymous=True)
    try:
        zed_subscriber = ZEDCameraSubscriber(
            rgb_topic='/zed/rgb_image', pose_topic='/zed/pose')
        zed_subscriber.run()
    except rospy.ROSInterruptException:
        pass
