#from https://github.com/stereolabs/zed-opencv/blob/master/python/zed-opencv.py

# import sys
import yaml
import os
import numpy as np
np.float = float
#import pyzed.sl as sl
import cv2
import time
from datetime import date
import rospy#, rosparam#, ros_numpy
from pypcd import pypcd #https://github.com/dimatura/pypcd/issues/7 #pip3 install --upgrade git+https://github.com/klintan/pypcd.git 
import message_filters # baca http://wiki.ros.org/message_filters#Example_.28Python.29-1
from sensor_msgs.msg import Joy, JointState #WHILL
from sensor_msgs.msg import NavSatFix #UBLOX
from nav_msgs.msg import Odometry #baca http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
from sensor_msgs.msg import PointCloud2 #Velodyne #http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html
from utilx.witmotion import IMU, protocol

q_sub = 25
t_sub = 0.25
baudratex = 115200 #9600 atau 115200
down_res_img = 1
imu_usb = "/dev/ttyUSB0"
# imu_usb = "/dev/ttyACM2"
datadir = "dataset/datasetx/"
ddir_tempo = "tempo/"
prefix = str(date.today()) + "_route02"



#buat folder
dir_meta = datadir+prefix+"/meta/"
dir_front_cam = datadir+prefix+"/camera/front/" #zed2i
dir_rear_cam = datadir+prefix+"/camera/rear/" #zed
dir_lidar = datadir+prefix+"/lidar/cld/"
dir_dvs = datadir+prefix+"/dvs/"
os.makedirs(dir_meta, exist_ok=True)
os.makedirs(dir_front_cam+"rgb", exist_ok=True)
# os.makedirs(dir_front_cam+"depth/img", exist_ok=True)
# os.makedirs(dir_front_cam+"depth/map", exist_ok=True)
os.makedirs(dir_front_cam+"depth/cld", exist_ok=True)
# os.makedirs(dir_rear_cam+"rgb", exist_ok=True)
# os.makedirs(dir_rear_cam+"depth/img", exist_ok=True)
# os.makedirs(dir_rear_cam+"depth/map", exist_ok=True)
# os.makedirs(dir_rear_cam+"depth/cld", exist_ok=True)
os.makedirs(dir_lidar, exist_ok=True)
os.makedirs(dir_dvs, exist_ok=True)




#SETUP ROS: UBLOX GNNS + VELODYNE LIDAR
rospy.init_node('data_retriever', anonymous=True)
#UBLOX
loc_sub = message_filters.Subscriber("/ublox/fix", NavSatFix)
#VELODYNE
lidar_sub = message_filters.Subscriber("/velodyne_points", PointCloud2) 


#SETTING HWT905----------------------------------------------------------------------
hwt905 = IMU(path=imu_usb, baudrate=baudratex)#cek status ch341-uart dengan dmesg | grep tty, baudrate 9600 atau 115200
#init setting baca: https://github.com/oskarnatan/witmotion/blob/master/witmotion/__init__.py https://witmotion.readthedocs.io/en/latest/api.html
hwt905.set_calibration_mode(protocol.CalibrationMode.magnetic) #2 magnetic, 1 gyro_acc, 0 none
hwt905.set_installation_direction(protocol.InstallationDirection.vertical) #1 vertical, 0 horizontal
hwt905.set_algorithm_dof(9) #9dof acc, gyro, magneto semua dipakai
hwt905.set_gyro_automatic_calibration(True)
hwt905.set_update_rate(200) #200hz rate pembacaan sensor paling tinggi
hwt905.save_configuration()



#SETTING ZED CAMERA------------------------------------------------------------------------------------

#MULTICAMERA: https://github.com/stereolabs/zed-multi-camera/blob/master/python/multi_camera.py
#https://www.stereolabs.com/docs/video/multi-camera/
#  cameras = sl.Camera.get_device_list()
#  print(cameras)
# [ZED 2i (0) /dev/video3 SN35828564 AVAILABLE, ZED (1) /dev/video5 SN14266 AVAILABLE]
#  cameras[0].serial_number
# 35828564
#  cameras[1].serial_number
# 14266
 


#BACA https://www.stereolabs.com/docs/video/using-video/
##BACA https://www.stereolabs.com/docs/depth-sensing/depth-settings/
#zed2i = sl.Camera()
# zed = sl.Camera()
#init_all = sl.InitParameters()
#init_all.coordinate_units = sl.UNIT.METER
#init_all.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD  #https://www.stereolabs.com/docs/positional-tracking/using-tracking/
#init_all.camera_resolution = sl.RESOLUTION.HD720 #HD720  #https://www.stereolabs.com/docs/video/camera-controls/   #HD720 HD1080 HD2K
#init_all.camera_fps = 20
#init_all.depth_mode = sl.DEPTH_MODE.ULTRA #PERFORMANCE ULTRA
#init_all.depth_minimum_distance = 0.3
#init_all.depth_maximum_distance = 40

#CEK KEDUA KAMERA
print("OPENING CAMERAS!")
init_all.set_from_serial_number(35828564) #ZED 2i
err2i = zed2i.open(init_all)
# init_all.set_from_serial_number(14266) #ZED
# err = zed.open(init_all)
if err2i != sl.ERROR_CODE.SUCCESS:# and err != sl.ERROR_CODE.SUCCESS:
    print(repr(err2i))
    zed2i.close()
    # zed.close()
    exit(1)

#parameter tambahan
runtime2i = sl.RuntimeParameters()
#runtime2i.sensing_mode = sl.SENSING_MODE.FILL  #STANDARD  FILL
tracking_parameters2i = sl.PositionalTrackingParameters()
track_err2i = zed2i.enable_positional_tracking(tracking_parameters2i)
# runtime = sl.RuntimeParameters()
# runtime.sensing_mode = sl.SENSING_MODE.FILL  #STANDARD  FILL

#siapkan matrix untuk menerima data
#frame_size = zed2i.get_camera_information().camera_resolution
class frame_size:
	height = 720/down_res_img
	width = 1280/down_res_img
zed2i_rgb = sl.Mat(frame_size.width, frame_size.height) #RGB #, sl.MAT_TYPE.U8_C4
# zed2i_depth_img = sl.Mat(frame_size.width, frame_size.height) #DEPTH img #, sl.MAT_TYPE.U8_C4
# zed2i_depth_map = sl.Mat(frame_size.width, frame_size.height) #DEPTH map
zed2i_depth_cld = sl.Mat(frame_size.width, frame_size.height)
zed2i_pose = sl.Pose()
# zed_rgb = sl.Mat(frame_size.width, frame_size.height) #RGB #, sl.MAT_TYPE.U8_C4
# zed_depth_img = sl.Mat(frame_size.width, frame_size.height) #DEPTH img #, sl.MAT_TYPE.U8_C4
# zed_depth_map = sl.Mat(frame_size.width, frame_size.height) #DEPTH map
# zed_depth_cld = sl.Mat(frame_size.width, frame_size.height)


print("--------WAIT CALIB ZEDCAM & WIT (3s)---------")
time.sleep(3)



#log penyimpanan
meta_log = {}
def callback0(location, lidar):
    #SEMUA BERDASARKAN TERBACA ATAU TIDAKNYA FRAME SEBAGAI SYARAT MUTLAK DATA LAIN DIPROSES
    
    if zed2i.grab(runtime2i) == sl.ERROR_CODE.SUCCESS:# and zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        #ZED CAMERA-------------------------------------------------------------------------------------------------
        #RETRIEVE RGBD ZED2I
        # MEM CPU or GPU, BACA!!! https://www.stereolabs.com/docs/depth-sensing/using-depth/
        zed2i.retrieve_image(zed2i_rgb, sl.VIEW.LEFT, sl.MEM.CPU, frame_size) #shape: HxWx4 (R,G,B,A)
        rgb_zed2i = zed2i_rgb.get_data()[:,:,:3] #AMBIL 3 CHANNEL PERTAMA, RGB
        # zed2i.retrieve_image(zed2i_depth_img, sl.VIEW.DEPTH, sl.MEM.CPU, frame_size) #shape: HxWx4 #normalized depth semua for display only!!!!!
        # depth_img_zed2i = zed2i_depth_img.get_data()[:,:,:1] #AMBIL SALAH SATU CHANNEL, DEPTH
        # zed2i.retrieve_measure(zed2i_depth_map, sl.MEASURE.DEPTH, sl.MEM.CPU, frame_size) # shape: HxW tiap pixel berisi informasi depth
        # depth_map_zed2i = zed2i_depth_map.get_data()
        zed2i.retrieve_measure(zed2i_depth_cld, sl.MEASURE.XYZ, sl.MEM.CPU, frame_size) # XYZRGBA --> HxWx4 (informasi channel: loc X, loc Y, loc Z, encoded 32b RGBA)
        depth_cld_zed2i = zed2i_depth_cld.get_data()#[:,:,:3]

        #RETRIEVE POSE ZED2I 
        state = zed2i.get_position(zed2i_pose)#, sl.REFERENCE_FRAME.FRAME_WORLD)
        py_translation = sl.Translation()
        # tx = zed2i_pose.get_translation(py_translation).get()[0]
        # ty = zed2i_pose.get_translation(py_translation).get()[1]
        # tz = zed2i_pose.get_translation(py_translation).get()[2]
        zed2i_pos_xyz = np.array(zed2i_pose.get_translation(py_translation).get()).tolist()
        py_orientation = sl.Orientation()
        zed2i_orien_xyzw = np.array(zed2i_pose.get_orientation(py_orientation).get()).tolist()
        # ox = zed2i_pose.get_orientation(py_orientation).get()[0]
        # oy = zed2i_pose.get_orientation(py_orientation).get()[1]
        # oz = zed2i_pose.get_orientation(py_orientation).get()[2]
        # ow = zed2i_pose.get_orientation(py_orientation).get()[3]
        # zed2i_pos = np.array([tx, ty, tz]).tolist()
        # zed2i_roll_x, zed2i_pitch_y, zed2i_yaw_z = euler_from_quaternion(w=ow, x=ox, y=oy, z=oz, rad=False)
        # zed2i_orien = np.array([zed2i_roll_x, zed2i_pitch_y, zed2i_yaw_z]).tolist()
        
        #RETRIEVE RGBD ZED
        # MEM CPU or GPU, BACA!!! https://www.stereolabs.com/docs/depth-sensing/using-depth/
        # zed.retrieve_image(zed_rgb, sl.VIEW.LEFT, sl.MEM.CPU, frame_size) #shape: HxWx4 (R,G,B,A)
        # rgb_zed = zed_rgb.get_data()[:,:,:3] #AMBIL 3 CHANNEL PERTAMA, RGB
        # zed.retrieve_image(zed_depth_img, sl.VIEW.DEPTH, sl.MEM.CPU, frame_size) #shape: HxWx4 #normalized depth semua for display only!!!!!
        # depth_img_zed = zed_depth_img.get_data()[:,:,:1] #AMBIL SALAH SATU CHANNEL, DEPTH
        # zed.retrieve_measure(zed_depth_map, sl.MEASURE.DEPTH, sl.MEM.CPU, frame_size) # shape: HxW tiap pixel berisi informasi depth
        # depth_map_zed = zed_depth_map.get_data()
        # zed.retrieve_measure(zed_depth_cld, sl.MEASURE.XYZ, sl.MEM.CPU, frame_size) # XYZRGBA --> HxWx4 (informasi channel: loc X, loc Y, loc Z, encoded 32b RGBA)
        # depth_cld_zed = zed_depth_cld.get_data()#[:,:,:3]


        #UBLOX GNSS-------------------------------------------------------------------------------------------------
        #sec = str(location.header.stamp.secs).zfill(10)
        #seq = str(location.header.seq).zfill(10)
        #file_namex = sec + "_" + seq
        ublox_pos = np.array([location.latitude, location.longitude]).tolist()


        #WITMOTION IMU-------------------------------------------------------------------------------------------------
        latest_acc = np.array([hwt905.get_acceleration()])[0].tolist()
        latest_gyr = np.array([hwt905.get_angular_velocity()])[0].tolist()
        latest_mag = np.array([hwt905.get_magnetic_vector()])[0].tolist()
        # hwt_roll_x, hwt_pitch_y, hwt_yaw_z = hwt905.get_angle() #dikomputasi oleh sensor, built_in
        # hwt_orien_rpy = np.array([hwt_roll_x, hwt_pitch_y, hwt_yaw_z]).tolist()
        hwt_orien_rpy = np.array([hwt905.get_angle()])[0].tolist()


        #CHECK YANG PENTING2 SAJA-------------------------------------------------------------------------------------------------
        print('File name:', file_namex)
        print("Position cov.: ", location.position_covariance)
        print("Vehicle Global Pos. Lat-Lon:", ublox_pos) 
        print("Vehicle Global Orien. R-P-Y: ", hwt_orien_rpy)
        #print("Vehicle Local Pos. X-Y-Z", zed2i_pos_xyz)
        #print("Vehicle Local Orien. X-Y-Z-W: ", zed2i_orien_xyzw)
        print("Point clouds: ", len(lidar.data))


        #SAVE DATA-------------------------------------------------------------------------------------------------
        meta_log['sec'] = sec
        meta_log['seq'] = seq
        meta_log['global_position_latlon'] = ublox_pos #ublox latitude-longitude, + utara, + timur
        meta_log['global_orientation_rpy'] = hwt_orien_rpy
        #meta_log['local_position_xyz'] = zed2i_pos_xyz
        # meta_log['local_orientation_rpy'] = zed2i_orien
        # meta_log['local_orientation_wxyz'] = np.array([ow, ox, oy, oz]).tolist()
        #meta_log['local_orientation_xyzw'] = zed2i_orien_xyzw
        meta_log['acceleration_xyz'] = latest_acc#[0].tolist()#[latest_acc[0][0], latest_acc[0][1], latest_acc[0][2]]
        meta_log['angular_speed_xyz'] = latest_gyr#[0].tolist()#[latest_gyr[0][0], latest_gyr[0][1], latest_gyr[0][2]]
        meta_log['magnetic_xyz'] = latest_mag#[0].tolist()#[latest_mag[0][0], latest_mag[0][1], latest_mag[0][2]]
        
        #meta
        with open(dir_meta+file_namex+".yml", 'w') as c:
            yaml.dump(meta_log, c)
        c.close()

        #rgbd
        #cv2.imwrite(dir_front_cam+"rgb/"+file_namex+".png", rgb_zed2i)
        # cv2.imwrite(dir_front_cam+"depth/img/"+file_namex+".png", depth_img_zed2i)
        # np.save(dir_front_cam+"depth/map/"+file_namex+".npy", depth_map_zed2i)
        #np.save(dir_front_cam+"depth/cld/"+file_namex+".npy", depth_cld_zed2i)
        # cv2.imwrite(dir_rear_cam+"rgb/"+file_namex+".png", rgb_zed)
        # cv2.imwrite(dir_rear_cam+"depth/img/"+file_namex+".png", depth_img_zed)
        # np.save(dir_rear_cam+"depth/map/"+file_namex+".npy", depth_map_zed)
        # np.save(dir_rear_cam+"depth/cld/"+file_namex+".npy", depth_cld_zed)

        #pointcloud2
        lid_pc = pypcd.PointCloud.from_msg(lidar) #baca: https://github.com/PRBonn/lidar-bonnetal/issues/78
        lid_pc.save_pcd(dir_lidar+file_namex+".pcd", compression='binary_compressed')

        # DVS
        #shutil.copyfile(ddir_tempo+'dvs_image.png', datadir+prefix+"/inivation/dvs/"+file_namex+".png")
        # shutil.copyfile(ddir_tempo+'rgb_image.png', datadir+prefix+"/inivation/rgb/"+file_namex+".png")
        dvs_davis = cv2.imread(ddir_tempo+"dvs_image0.png")
        if dvs_davis is None:
            dvs_davis = cv2.imread(ddir_tempo+"dvs_image1.png")
            if dvs_davis is None:
                dvs_davis = np.zeros((260, 346, 3))
        cv2.imwrite(dir_dvs+file_namex+".png", dvs_davis)
    



"""
def euler_from_quaternion(w, x, y, z, rad=True): #urutannya q0, q1, q2, q3
    #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    #https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/

    # Convert a quaternion into euler angles (roll, pitch, yaw)
    # roll is rotation around x in radians (counterclockwise)
    # pitch is rotation around y in radians (counterclockwise)
    # yaw is rotation around z in radians (counterclockwise)

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    
    if rad:
        return roll_x, pitch_y, yaw_z # in radians
    else:
        return np.degrees(roll_x), np.degrees(pitch_y), np.degrees(yaw_z)
"""



#ROS SPIN----------------------------------------------------------------------
ts0 = message_filters.ApproximateTimeSynchronizer([loc_sub, lidar_sub], q_sub, t_sub, allow_headerless=True)
ts0.registerCallback(callback0)

rospy.spin()




