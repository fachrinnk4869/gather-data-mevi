# camera_module.py
import pyzed.sl as sl
import numpy as np
import cv2


class ZEDCamera:
    def __init__(self, resolution, fps, depth_mode):
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = resolution
        init_params.camera_fps = fps
        init_params.depth_mode = depth_mode
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD  #https://www.stereolabs.com/docs/positional-tracking/using-tracking/
        init_params.depth_minimum_distance = 0.3
        init_params.depth_maximum_distance = 40
        # init_params.set_from_serial_number(serial_number) # check serial number
        err = self.zed.open(init_params)

        if err != sl.ERROR_CODE.SUCCESS:
            print("Error opening ZED camera:", repr(err))
            self.zed.close()
            exit(1)

        self.runtime_params = sl.RuntimeParameters()
        self.frame_size = (1280, 720)
        self.pose = sl.Pose()
        tracking_parameters2i = sl.PositionalTrackingParameters()
        track_err2i = self.zed.enable_positional_tracking(tracking_parameters2i)

    def get_frame(self):
        rgb_image = sl.Mat(self.frame_size[0], self.frame_size[1])
        depth_map = sl.Mat(self.frame_size[0], self.frame_size[1])

        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(rgb_image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(depth_map, sl.MEASURE.XYZ)
            rgb_data = rgb_image.get_data()[:, :, :3]
            depth_data = depth_map.get_data()
            cv2.imshow("hehe", rgb_data)
            cv2.waitKey(1)
            return rgb_data, depth_data
        return None, None

    def get_pose(self):
        
        self.zed.get_position(self.pose)
        translation = np.array(self.pose.get_translation(sl.Translation()).get()).tolist()
        orientation = np.array(self.pose.get_orientation(sl.Orientation()).get()).tolist()
        return translation, orientation

# Test function for ZED Camera
if __name__ == "__main__":
    zed_camera = ZEDCamera(sl.RESOLUTION.HD720, 20,
                           sl.DEPTH_MODE.ULTRA)

    # Retrieve frame data
    rgb_data, depth_data = zed_camera.get_frame()
    if rgb_data is not None:
        print("Camera RGB Data Retrieved!")
        cv2.imwrite("test_rgb_image.png", rgb_data)

    # Retrieve pose data
    translation, orientation = zed_camera.get_pose()
    if translation is not None:
        print("Camera Pose - Translation:", translation)
        print("Camera Pose - Orientation:", orientation)
