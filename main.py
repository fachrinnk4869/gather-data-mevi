import yaml
import os
import numpy as np
import pyzed.sl as sl
import cv2
import time
from datetime import date
# Create a ZED camera object
zed = sl.Camera()

# Initialize the ZED
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720  # Set resolution
init_params.camera_fps = 30  # Set FPS
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)
    
# zed2i_rgb = sl.Mat()
# zed2i_depth_cld = sl.Mat()
# runtime_parameters = sl.RuntimeParameters()
# if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS :
#   # A new image and depth is available if grab() returns SUCCESS
#   zed.retrieve_image(zed2i_rgb, sl.VIEW.LEFT) # Retrieve left image
#   zed.retrieve_measure(zed2i_depth_cld, sl.MEASURE.DEPTH) # Retrieve depth
# rgb_zed2i = zed2i_rgb.get_data()[:,:,:3] #AMBIL 3 CHANNEL PERTAMA, RGB
# depth_cld_zed2i = zed2i_depth_cld.get_data()#[:,:,:3]
# # save depth sensing
# zed2i_rgb.write("halo.png")
# zed2i_depth_cld.write("halo_depth.png")
# cv2.imwrite("rgb/"+"file"+".png", rgb_zed2i)
# cv2.imshow("halo", rgb_zed2i)
# np.save("depth"+"file"+".npy", depth_cld_zed2i)

# Create variables to store the image and runtime parameters
zed2i_rgb_left = sl.Mat()
zed2i_rgb_right = sl.Mat()
zed2i_depth_cld = sl.Mat()
runtime_parameters = sl.RuntimeParameters()
try:
    print("Press 's' to save the image and 'q' to quit...")
    index = 1
    while True:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left image
            # A new image and depth is available if grab() returns SUCCESS
            zed.retrieve_image(zed2i_rgb_left, sl.VIEW.LEFT) # Retrieve left image
            zed.retrieve_image(zed2i_rgb_right, sl.VIEW.RIGHT) # Retrieve left image
            zed.retrieve_measure(zed2i_depth_cld, sl.MEASURE.DEPTH) # Retrieve depth


            rgb_zed2i_left = zed2i_rgb_left.get_data()[:,:,:3] #AMBIL 3 CHANNEL PERTAMA, RGB
            rgb_zed2i_right = zed2i_rgb_right.get_data()[:,:,:3] #AMBIL 3 CHANNEL PERTAMA, RGB
            depth_cld_zed2i = zed2i_depth_cld.get_data()#[:,:,:3]
            # Display the frame in a window
            cv2.imshow("ZED Camera RGB left", zed2i_rgb_left.get_data())
            cv2.imshow("ZED Camera RGB right", zed2i_rgb_right.get_data())
            cv2.imshow("ZED Camera depth", depth_cld_zed2i)

            # save image and depth
            # cv2.imwrite(f"rgb/captured_image_{index}.png", zed2i_rgb_left.get_data())
            # np.save(f"depth/depth_{index}.npy", depth_cld_zed2i)
            
            # Wait for a key press
            key = cv2.waitKey(1) & 0xFF
            # Exit if 'q' is pressed
            if key == ord('q'):
                break
            index +=1

except KeyboardInterrupt:
    print("Recording stopped.")

# Release the camera and destroy all OpenCV windows
zed.close()
cv2.destroyAllWindows()