#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from math import sin, cos
from witmotion import IMU

class IMUOdometryPublisher:
    def __init__(self):
        rospy.init_node('imu_odometri_publisher')
        self.imu = IMU('/dev/ttyUSB0', 9600)  # Sesuaikan dengan port dan baudrate yang diperlukan
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.last_time = rospy.Time.now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.rate = rospy.Rate(10)  # 10 Hz

    def publish_odom_data(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Ambil data dari IMU
        acc = self.imu.get_acceleration()
        ang_vel = self.imu.get_angular_velocity()
        orientation = self.imu.get_quaternion()

        if acc is not None and ang_vel is not None and orientation is not None:
            # Perbarui posisi berdasarkan kecepatan sudut dan percepatan
            self.theta += ang_vel[2] * dt
            delta_x = acc[0] * dt * dt / 2.0
            delta_y = acc[1] * dt * dt / 2.0
            self.x += delta_x * cos(self.theta) - delta_y * sin(self.theta)
            self.y += delta_x * sin(self.theta) + delta_y * cos(self.theta)

            # Buat pesan odometri
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = 'odom'

            # Set posisi
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = orientation[0]
            odom.pose.pose.orientation.y = orientation[1]
            odom.pose.pose.orientation.z = orientation[2]
            odom.pose.pose.orientation.w = orientation[3]

            # Set kecepatan
            odom.child_frame_id = 'base_link'
            odom.twist.twist.linear.x = acc[0] * dt
            odom.twist.twist.linear.y = acc[1] * dt
            odom.twist.twist.angular.z = ang_vel[2]

            # Terbitkan pesan odometri
            self.odom_pub.publish(odom)

            # Perbarui waktu terakhir
            self.last_time = current_time

    def run(self):
        rospy.loginfo("Node IMU Odometry Publisher dimulai.")
        while not rospy.is_shutdown():
            self.publish_odom_data()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        imu_odometry_publisher = IMUOdometryPublisher()
        imu_odometry_publisher.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception")
    except Exception as e:
        rospy.logerr("Terjadi kesalahan: {}".format(e))
