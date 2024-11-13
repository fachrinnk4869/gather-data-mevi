import pyzed.sl as sl
import numpy as np
import cv2
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from std_msgs.msg import Header


class ZEDCamera:
    def __init__(self, resolution, fps, depth_mode):
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

        # ROS Publishers
        self.rgb_pub = rospy.Publisher('/zed/rgb_image', Image, queue_size=10)
        self.depth_pub = rospy.Publisher(
            '/zed/depth_map', Image, queue_size=10)
        self.pose_pub = rospy.Publisher(
            '/zed/pose', PoseStamped, queue_size=10)

        self.bridge = CvBridge()

    def get_frame(self):
        rgb_image = sl.Mat(self.frame_size[0], self.frame_size[1])
        depth_map = sl.Mat(self.frame_size[0], self.frame_size[1])

        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(rgb_image, sl.VIEW.LEFT, sl.MEM.CPU)
            self.zed.retrieve_measure(depth_map, sl.MEASURE.XYZ, sl.MEM.CPU)
            rgb_data = rgb_image.get_data()[:, :, :3]
            depth_data = depth_map.get_data()
            # print(depth_data.shape)
            return rgb_data, depth_data
        return None, None

    def get_pose(self):
        self.zed.get_position(self.pose)
        translation = np.array(self.pose.get_translation(
            sl.Translation()).get()).tolist()
        orientation = np.array(self.pose.get_orientation(
            sl.Orientation()).get()).tolist()
        return translation, orientation

    def publish_data(self):
        rgb_data, depth_data = self.get_frame()
        if rgb_data is not None and depth_data is not None:
            # Publish RGB image
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_data, encoding="bgr8")
            self.rgb_pub.publish(rgb_msg)

            # Publish depth map (convert depth to uint8 for visualization, if needed)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_data, encoding="32FC4")
            self.depth_pub.publish(depth_msg)
            

        # Publish Pose data
        translation, orientation = self.get_pose()
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()  # Set the current time
        pose_msg.header.frame_id = "base_link"  # Change to your desired frame_id
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]
        pose_msg.pose.orientation.x = orientation[0]
        pose_msg.pose.orientation.y = orientation[1]
        pose_msg.pose.orientation.z = orientation[2]
        pose_msg.pose.orientation.w = orientation[3]
        self.pose_pub.publish(pose_msg)


# Main loop to publish data
if __name__ == "__main__":
    rospy.init_node('zed_camera_node', anonymous=True)
    zed_camera = ZEDCamera(sl.RESOLUTION.VGA, 30, sl.DEPTH_MODE.ULTRA)
    rate = rospy.Rate(5)  # Set the publishing rate (10 Hz)

    while not rospy.is_shutdown():
        zed_camera.publish_data()
        rate.sleep()
