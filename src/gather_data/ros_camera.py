#!/usr/bin/env python
import pyzed.sl as sl
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2

# Base class for ZED camera operations without ROS


class ZEDCameraBase:
    def __init__(self, resolution=sl.RESOLUTION.VGA, fps=30, depth_mode=sl.DEPTH_MODE.ULTRA):
        # Initialize ZED Camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = resolution
        init_params.camera_fps = fps
        init_params.depth_mode = depth_mode
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_minimum_distance = 0.3
        init_params.depth_maximum_distance = 40

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Error opening ZED camera:", repr(err))
            self.zed.close()
            exit(1)

        self.runtime_params = sl.RuntimeParameters()
        self.frame_size = (1280, 720)
        self.pose = sl.Pose()
        tracking_parameters = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(tracking_parameters)

    def get_depth(self):
        # Retrieve depth data without ROS
        depth_map = sl.Mat(self.frame_size[0], self.frame_size[1])
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)
            depth_data = depth_map.get_data()
            return depth_data
        return None

    def get_pose(self):
        # Retrieve pose data
        self.zed.get_position(self.pose)
        translation = np.array(self.pose.get_translation(
            sl.Translation()).get()).tolist()
        orientation = np.array(self.pose.get_orientation(
            sl.Orientation()).get()).tolist()
        return translation, orientation


# ROS Publisher class
class ZEDCameraRGBROS(ZEDCameraBase):
    def __init__(self, rgb_topic='/zed/rgb_image', pose_topic='/zed/pose'):
        super().__init__()
        self.rgb_topic = rgb_topic
        self.pose_topic = pose_topic
        self.bridge = CvBridge()

        # ROS Publishers for RGB and Pose
        self.rgb_pub = rospy.Publisher(self.rgb_topic, Image, queue_size=10)
        self.pose_pub = rospy.Publisher(
            self.pose_topic, PoseStamped, queue_size=10)

    def get_rgb(self):
        # Retrieve RGB data and convert it to OpenCV format
        rgb_image = sl.Mat(self.frame_size[0], self.frame_size[1])
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(rgb_image, sl.VIEW.LEFT, sl.MEM.CPU)
            rgb_data = rgb_image.get_data()[:, :, :3]
            return rgb_data
        return None

    def publish_rgb_and_pose(self):
        # Publish RGB image and pose data to ROS
        rgb_data = self.get_rgb()
        if rgb_data is not None:
            # Convert RGB data to ROS Image message and publish
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_data, encoding="bgr8")
            self.rgb_pub.publish(rgb_msg)

        # Retrieve and publish pose data
        translation, orientation = self.get_pose()
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()  # Set the current time
        pose_msg.header.frame_id = "base_link"  # Set frame ID
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]
        pose_msg.pose.orientation.x = orientation[0]
        pose_msg.pose.orientation.y = orientation[1]
        pose_msg.pose.orientation.z = orientation[2]
        pose_msg.pose.orientation.w = orientation[3]
        self.pose_pub.publish(pose_msg)

# Depth retrieval class that shares the ZED camera instance


class ZEDDepthGetter(ZEDCameraBase):
    def get_depth(self):
        # Retrieve depth data without reinitializing the ZED camera
        depth_map = sl.Mat(self.frame_size[0], self.frame_size[1])
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_measure(depth_map, sl.MEASURE.XYZ)
            depth_data = depth_map.get_data()
            return depth_data
        return None


if __name__ == '__main__':
    rospy.init_node('zed_camera_node', anonymous=True)
    # Initialize publisher and subscriber objects
    zed_camera_publisher = ZEDCameraRGBROS()

    rate = rospy.Rate(10)  # Set publishing rate to 10 Hz

    while not rospy.is_shutdown():
        zed_camera_publisher.publish_rgb_and_pose()
        rate.sleep()
