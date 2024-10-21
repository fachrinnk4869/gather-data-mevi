#from https://github.com/stereolabs/zed-opencv/blob/master/python/zed-opencv.py

# import sys
import yaml
import os
import numpy as np
import pyzed.sl as sl
import cv2
import time
from datetime import date


q_sub = 25
t_sub = 0.25
baudratex = 115200 #9600 atau 115200
down_res_img = 1
imu_usb = "/dev/ttyUSB0"
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

#BACA https://www.stereolabs.com/docs/video/using-video/
##BACA https://www.stereolabs.com/docs/depth-sensing/depth-settings/
zed2i = sl.Camera()
# zed = sl.Camera()
init_all = sl.InitParameters()
init_all.coordinate_units = sl.UNIT.METER
init_all.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD  #https://www.stereolabs.com/docs/positional-tracking/using-tracking/
init_all.camera_resolution = sl.RESOLUTION.HD720 #HD720  #https://www.stereolabs.com/docs/video/camera-controls/   #HD720 HD1080 HD2K
init_all.camera_fps = 20
init_all.depth_mode = sl.DEPTH_MODE.ULTRA #PERFORMANCE ULTRA
init_all.depth_minimum_distance = 0.3
init_all.depth_maximum_distance = 40

#CEK KEDUA KAMERA
print("OPENING CAMERAS!")
err2i = zed2i.open(init_all)
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

# zed2i_depth_img = sl.Mat(frame_size.width, frame_size.height) #DEPTH img #, sl.MAT_TYPE.U8_C4
# zed2i_depth_map = sl.Mat(frame_size.width, frame_size.height) #DEPTH map
zed2i_pose = sl.Pose()
# zed_rgb = sl.Mat(frame_size.width, frame_size.height) #RGB #, sl.MAT_TYPE.U8_C4
# zed_depth_img = sl.Mat(frame_size.width, frame_size.height) #DEPTH img #, sl.MAT_TYPE.U8_C4
# zed_depth_map = sl.Mat(frame_size.width, frame_size.height) #DEPTH map
# zed_depth_cld = sl.Mat(frame_size.width, frame_size.height)


print("--------WAIT CALIB ZEDCAM & WIT (3s)---------")
time.sleep(3)



#log penyimpanan
meta_log = {}
def callback0():
    #SEMUA BERDASARKAN TERBACA ATAU TIDAKNYA FRAME SEBAGAI SYARAT MUTLAK DATA LAIN DIPROSES\

    index = 1
    while True:
        if zed2i.grab(runtime2i) == sl.ERROR_CODE.SUCCESS:# and zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:

            #RETRIEVE POSE ZED2I 
            state = zed2i.get_position(zed2i_pose)#, sl.REFERENCE_FRAME.FRAME_WORLD)
            py_translation = sl.Translation()
            zed2i_pos_xyz = np.array(zed2i_pose.get_translation(py_translation).get()).tolist()
            py_orientation = sl.Orientation()
            zed2i_orien_xyzw = np.array(zed2i_pose.get_orientation(py_orientation).get()).tolist()



            #SAVE DATA-------------------------------------------------------------------------------------------------
            
            meta_log['local_position_xyz'] = zed2i_pos_xyz
            meta_log['local_orientation_xyzw'] = zed2i_orien_xyzw
            
            #meta
            with open(dir_meta+"file_namex"+".yml", 'w') as c:
                yaml.dump(meta_log, c)
            c.close()

            # DVS
            #shutil.copyfile(ddir_tempo+'dvs_image.png', datadir+prefix+"/inivation/dvs/"+file_namex+".png")
            # shutil.copyfile(ddir_tempo+'rgb_image.png', datadir+prefix+"/inivation/rgb/"+file_namex+".png")
            # dvs_davis = cv2.imread(ddir_tempo+"dvs_image0.png")
            # if dvs_davis is None:
            #     dvs_davis = cv2.imread(ddir_tempo+"dvs_image1.png")
            #     if dvs_davis is None:
            #         dvs_davis = np.zeros((260, 346, 3))
            # cv2.imwrite(dir_dvs+file_namex+".png", dvs_davis)


callback0()