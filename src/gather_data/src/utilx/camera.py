#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import message_filters
import cv2


class ZEDCamera:
    def __init__(self, topic_name_rgb, topic_name_depth, topic_name_pose):
        # Initialize the ROS node
        rospy.init_node('zed_subscriber', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Store topic names
        self.topic_name_rgb = topic_name_rgb
        self.topic_name_depth = topic_name_depth
        self.topic_name_pose = topic_name_pose

    def start_listener(self):
        # Create message filters for RGB image, depth image, and pose
        rgb_sub = message_filters.Subscriber(self.topic_name_rgb, Image)
        depth_sub = message_filters.Subscriber(self.topic_name_depth, Image)
        pose_sub = message_filters.Subscriber(
            self.topic_name_pose, Pose)

        rospy.loginfo(
            f"Subscribed to {self.topic_name_rgb}, {self.topic_name_depth}, and {self.topic_name_pose} topics. Waiting for data...")
        return rgb_sub, depth_sub, pose_sub

    def get_frame(self, rgb_msg, depth_msg):
        # Convert the ROS Image messages to OpenCV format
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        return rgb_image, depth_image

    def get_pose(self, pose_msg):
        return pose_msg.translation, pose_msg.orientation

    def run(self):
        self.start_listener()
        rospy.spin()


if __name__ == '__main__':
    try:
        zed_subscriber = ZEDCamera(
            rgb_topic='/zed/rgb_image', depth_topic='/zed/dept_map', pose_topic='/zed/pose')
        zed_subscriber.run()
    except rospy.ROSInterruptException:
        pass
