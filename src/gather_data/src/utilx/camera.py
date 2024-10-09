# camera_module.py
import pyzed.sl as sl
import numpy as np
import cv2


class ZEDCamera:
    def __init__(self, resolution, fps, depth_mode, serial_number):
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = resolution
        init_params.camera_fps = fps
        init_params.depth_mode = depth_mode
        init_params.coordinate_units = sl.UNIT.METER
        init_params.set_from_serial_number(serial_number)
        err = self.zed.open(init_params)

        if err != sl.ERROR_CODE.SUCCESS:
            print("Error opening ZED camera:", repr(err))
            self.zed.close()
            exit(1)

        self.runtime_params = sl.RuntimeParameters()
        self.frame_size = (1280, 720)

    def get_frame_data(self):
        rgb_image = sl.Mat(self.frame_size[0], self.frame_size[1])
        depth_map = sl.Mat(self.frame_size[0], self.frame_size[1])

        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(rgb_image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(depth_map, sl.MEASURE.XYZ)
            rgb_data = rgb_image.get_data()[:, :, :3]
            depth_data = depth_map.get_data()

            return rgb_data, depth_data
        return None, None

    def get_pose(self):
        pose = sl.Pose()
        if self.zed.get_position(pose) == sl.POSITIONAL_TRACKING_STATE.OK:
            translation = pose.get_translation(sl.Translation()).get()
            orientation = pose.get_orientation(sl.Orientation()).get()
            return translation, orientation
        return None, None


if __name__ == "__main__":
    down_res_img = 1  # Resolution downscaling factor
    camera = ZEDCamera(down_res_img=down_res_img)

    # Open and initialize camera
    camera.open_camera()
    camera.enable_tracking()

    # Create frame size object
    frame_size = FrameSize(down_res_img)

    print("--------WAIT CALIB ZEDCAM & WIT (3s)---------")
    time.sleep(3)

    # Capture frames
    camera.capture_frame(frame_size)

    # Close the camera after capturing
    camera.close_camera()
