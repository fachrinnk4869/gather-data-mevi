#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2


class ZEDCamera:
    def __init__(self, rgb_topic, depth_topic, pose_topic):
        # Initialize the ROS node

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Store topic names
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.pose_topic = pose_topic

        # Initialize placeholders for RGB, depth, and pose data
        self.rgb_image = None
        self.depth_image = None
        self.pose_position = None
        self.pose_orientation = None

        # Set up subscribers with individual callbacks
        rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback)
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)

    def rgb_callback(self, rgb_msg):
        # Convert RGB Image message to OpenCV format and store it
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            # rospy.loginfo("Received RGB image data.")
        except Exception as e:
            rospy.logerr(f"Failed to process RGB image: {e}")

    def depth_callback(self, depth_msg):
        # Convert Depth Image message to OpenCV format and store it
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            # rospy.loginfo("Received Depth image data.")
        except Exception as e:
            rospy.logerr(f"Failed to process Depth image: {e}")

    def pose_callback(self, pose_msg):
        # Store PoseStamped message's position and orientation
        self.pose_position = pose_msg.pose.position
        self.pose_orientation = pose_msg.pose.orientation
        # rospy.loginfo("Received Pose data.")

    def get_frame(self):
        # Retrieve the latest RGB and Depth images if available
        if self.rgb_image is not None and self.depth_image is not None:
            return self.rgb_image, self.depth_image
        else:
            rospy.logwarn("RGB or Depth image not available yet.")
            return None, None

    def get_pose(self):
        # Retrieve the latest pose data if available
        if self.pose_position is not None and self.pose_orientation is not None:
            return self.pose_position, self.pose_orientation
        else:
            rospy.logwarn("Pose data not available yet.")
            return None, None

    def run(self):
        # Spin to keep the node alive and allow callback processing
        rospy.loginfo("ZED Camera Subscriber is running. Waiting for data...")
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('zed_subscriber', anonymous=True)
    try:
        zed_subscriber = ZEDCamera(
            rgb_topic='/zed/rgb_image', depth_topic='/zed/depth_map', pose_topic='/zed/pose')
        zed_subscriber.run()
    except rospy.ROSInterruptException:
        pass
