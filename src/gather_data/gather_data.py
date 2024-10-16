# main.py
import rospy
import message_filters
import yaml
import cv2
import pyzed.sl as sl
import numpy as np
from sensor_msgs.msg import NavSatFix, PointCloud2
from datetime import date
import os
from utilx.imu import IMUSensor
from utilx.camera import ZEDCamera
from utilx.lidar import LidarSensor
from utilx.gps import GPSSensor

# Create directories for storing data
datadir = "dataset/datasetx/"
prefix = str(date.today()) + "_route01"
dir_meta = datadir + prefix + "/meta/"
dir_front_cam = datadir + prefix + "/camera/front/"
dir_lidar = datadir + prefix + "/lidar/"
os.makedirs(dir_meta, exist_ok=True)
os.makedirs(dir_front_cam + "rgb", exist_ok=True)
os.makedirs(dir_front_cam + "depth/cld", exist_ok=True)
os.makedirs(dir_lidar, exist_ok=True)
# ROS initialization
rospy.init_node('data_retriever', anonymous=True)
# Initialize sensors
imu = IMUSensor(imu_usb="/dev/ttyUSB0", baudrate=9600)
camera = ZEDCamera(sl.RESOLUTION.HD720, 20,
                   sl.DEPTH_MODE.ULTRA)
gps_sensor = GPSSensor('ublox/fix')
lidar_sensor = LidarSensor('velodyne_points')

try:
    lidar_sub = lidar_sensor.start_listener()
    loc_sub = gps_sensor.start_listener()
except rospy.ROSInterruptException:
    rospy.logerr("ROS node interrupted.")


def callback(location, lidar_msg):
    # Get camera data
    rgb_data, depth_data = camera.get_frame()
    translation, orientation = camera.get_pose()
    rospy.loginfo(
            f"Received data...")
    # Get IMU data
    imu_data = imu.get_imu_data()

    # # Log metadata
    sec = str(location.header.stamp.secs).zfill(10)
    seq = str(location.header.seq).zfill(10)
    file_name = sec + "_" + seq

    meta_log = {
        'sec': sec,
        'seq': seq,
        'global_position_latlon': [location.latitude, location.longitude],
        'global_orientation_rpy': imu_data['orientation_rpy'],
        'local_position_xyz': translation,
        'local_orientation_xyzw': orientation,
        'acceleration_xyz': imu_data['acceleration'],
        'angular_speed_xyz': imu_data['angular_velocity'],
        'magnetic_xyz': imu_data['magnetic_vector']
    }

    # Save metadata and sensor data
    with open(dir_meta + file_name + ".yml", 'w') as file:
        yaml.dump(meta_log, file)
        rospy.loginfo(f"Meta data saved as {file_name}.yml")

    cv2.imwrite(dir_front_cam + "rgb/" + file_name + ".png", rgb_data)
    try:
        np.save(dir_front_cam + "depth/cld/" + file_name + ".npy", depth_data)
        rospy.loginfo(f"Camera depth data saved as {file_name}.npy")
    except Exception as e:
        rospy.logerr(f"Failed to save camera data: {str(e)}")
    lidar_sensor.save_lidar_data(lidar_msg, dir_lidar + file_name + ".pcd")

# ROS message synchronizer
ts = message_filters.ApproximateTimeSynchronizer(
    [loc_sub, lidar_sub], 25, 0.25)
ts.registerCallback(callback)

rospy.spin()
