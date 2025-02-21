#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import message_filters
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

    def start_listener(self):
        # Create message filters for RGB image, depth image, and pose
        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        pose_sub = message_filters.Subscriber(
            self.pose_topic, PoseStamped)

        rospy.loginfo(
            f"Subscribed to {self.rgb_topic}, {self.depth_topic}, and {self.pose_topic} topics. Waiting for data...")
        return rgb_sub, depth_sub, pose_sub

    def get_frame(self, rgb_msg, depth_msg):
        # Convert the ROS Image messages to OpenCV format
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC4")
        return rgb_image, depth_image

    def get_pose(self, pose_msg):
        return pose_msg.pose.position, pose_msg.pose.orientation

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
