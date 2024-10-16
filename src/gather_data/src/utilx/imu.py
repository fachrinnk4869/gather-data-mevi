# imu_module.py
import numpy as np
from utilx.witmotion import IMU, protocol


class IMUSensor:
    def __init__(self, imu_usb, baudrate):
        self.hwt905 = IMU(path=imu_usb, baudrate=baudrate)
        self.hwt905.set_calibration_mode(protocol.CalibrationMode.magnetic)
        self.hwt905.set_installation_direction(
            protocol.InstallationDirection.vertical)
        self.hwt905.set_algorithm_dof(9)
        self.hwt905.set_gyro_automatic_calibration(True)
        self.hwt905.set_update_rate(200)
        self.hwt905.save_configuration()

    def get_imu_data(self):
        acceleration = np.array([self.hwt905.get_acceleration()])[0].tolist()
        angular_velocity = np.array(
            [self.hwt905.get_angular_velocity()])[0].tolist()
        magnetic_vector = np.array(
            [self.hwt905.get_magnetic_vector()])[0].tolist()
        orientation_rpy = np.array([self.hwt905.get_angle()])[0].tolist()

        return {
            "acceleration": acceleration,
            "angular_velocity": angular_velocity,
            "magnetic_vector": magnetic_vector,
            "orientation_rpy": orientation_rpy
        }


# Test function for IMU
if __name__ == "__main__":
    imu_sensor = IMUSensor(imu_usb="/dev/ttyUSB0", baudrate=9600)
    imu_data = imu_sensor.get_imu_data()
    print("IMU Data:", imu_data)
