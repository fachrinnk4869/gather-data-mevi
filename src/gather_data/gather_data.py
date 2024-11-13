import rospy
import message_filters
import yaml
import cv2
import pyzed.sl as sl
import numpy as np
from sensor_msgs.msg import NavSatFix, PointCloud2
from std_msgs.msg import String
from datetime import date
import os
import rospkg
from utilx.imu import IMUSensor
from utilx.camera import ZEDCameraSubscriber
from utilx.lidar import LidarSensor
from utilx.gps import GPSSensor
from utilx.low_level import LowLevelSensor
import time

# Create directories for storing data
rospack = rospkg.RosPack()
package_path = rospack.get_path('gather_data')

# Create directories for storing data
base_datadir = os.path.join(package_path, "dataset/")

# Ensure base directory exists
if not os.path.exists(base_datadir):
    os.makedirs(base_datadir)

# Find the latest dataset folder and increment it


def get_next_dataset_dir():
    # List all directories in the base data directory
    existing_dirs = [d for d in os.listdir(
        base_datadir) if os.path.isdir(os.path.join(base_datadir, d))]
    # Filter directories that match the dataset naming convention (e.g., dataset_1, dataset_2, ...)
    dataset_nums = [int(d.split('_')[1]) for d in existing_dirs if d.startswith(
        "dataset_") and d.split('_')[1].isdigit()]
    # Get the next dataset number
    next_num = max(dataset_nums, default=0) + 1
    return os.path.join(base_datadir, f"dataset_{next_num}/")


# Create the new directory
datadir = get_next_dataset_dir()
os.makedirs(datadir)
print(f"Created new dataset directory: {datadir}")
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
camera_rgb_sensor = ZEDCameraSubscriber('zed/rgb_image',  '/zed/pose')
gps_sensor = GPSSensor('/latlon1')
lidar_sensor = LidarSensor('/velodyne_points')
throttle_sensor = LowLevelSensor('gas')
vel_kiri_sensor = LowLevelSensor('/encoder1_value')
vel_kanan_sensor = LowLevelSensor('/encoder2_value')
steer_sensor = LowLevelSensor('curra')

# Create a Publisher for the file_name
file_name_pub = rospy.Publisher('/file_name_depth', String, queue_size=10)

try:
    lidar_sub = lidar_sensor.start_listener()
    loc_sub = gps_sensor.start_listener()
except rospy.ROSInterruptException:
    rospy.logerr("ROS node interrupted.")

print("--------WAIT CALIB ZEDCAM & WIT (3s)---------")
time.sleep(3)


def callback(location, lidar_msg):
    # Get camera data
    rgb_data = camera_rgb_sensor.get_frame()
    if rgb_data is None:
        rospy.logwarn("Camera frame is missing; skipping this callback")
        return
    translation, orientation = camera_rgb_sensor.get_pose()

    # Get IMU data
    imu_data = imu.get_imu_data()
    throttle = throttle_sensor.get_data()
    vel_kiri = vel_kiri_sensor.get_data()
    vel_kanan = vel_kanan_sensor.get_data()
    steer = steer_sensor.get_data()

    # Log metadata
    sec = str(int(time.time()))
    seq = str(location.header.seq).zfill(10)
    file_name = sec + "_" + seq

    # Create a metadata log
    meta_log = {
        'sec': sec,
        'seq': seq,
        'throttle': throttle,
        'vel_kiri': vel_kiri,
        'vel_kanan': vel_kanan,
        'steer': steer,
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

    # Save RGB and Depth images
    file_name_pub.publish(dir_front_cam + "depth/cld/" + file_name)
    try:
        cv2.imwrite(dir_front_cam + "rgb/" + file_name + ".png", rgb_data)
        rospy.loginfo(f"Camera depth data saved as {file_name}.npy")
    except Exception as e:
        rospy.logerr(f"Failed to save camera data: {str(e)}")

    # Save lidar data
    lidar_sensor.save_lidar_data(lidar_msg, dir_lidar + file_name + ".pcd")

    # Publish file_name to ROS topic
    rospy.loginfo(f"Published file name: {file_name}")


# ROS message synchronizer
ts = message_filters.ApproximateTimeSynchronizer(
    [loc_sub, lidar_sub], queue_size=500, slop=1e20)
ts.registerCallback(callback)

rospy.spin()
