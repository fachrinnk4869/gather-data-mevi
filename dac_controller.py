#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Fungsi untuk mengonversi kecepatan linear ke nilai DAC
def velocity_to_dac(velocity):
    # Misal untuk kecepatan 5.56 m/s (20 km/jam), kita tetapkan DAC maksimum (4095)
    max_velocity = 5.56  # m/s
    max_dac_value = 4095  # nilai DAC maksimum

    # Konversi ke DAC
    dac_value = (velocity / max_velocity) * max_dac_value
    dac_value = max(0, min(dac_value, max_dac_value))  # Batasi nilai antara 0 dan 4095
    return int(dac_value)

# Callback saat menerima perintah kecepatan dari DWA (cmd_vel)
def cmd_vel_callback(cmd_vel_msg):
    linear_velocity = cmd_vel_msg.linear.x  # Kecepatan linear
    dac_value = velocity_to_dac(linear_velocity)  # Konversi ke nilai DAC
    dac_pub.publish(dac_value)  # Publikasikan nilai DAC

# Inisialisasi node ROS
rospy.init_node('dac_controller')

# Publisher untuk topik "dac_value"
dac_pub = rospy.Publisher('dac_value', Float32, queue_size=10)

# Subscriber untuk topik "cmd_vel"
rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

# ROS Spin untuk loop
rospy.spin()

