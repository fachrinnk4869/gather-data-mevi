# Import socket module
import socket
import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node('gps_publisher', anonymous=True)
    lat_pub = rospy.Publisher('/latitude', Float64, queue_size=10)
    lon_pub = rospy.Publisher('/longitude', Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    s = socket.socket()
    port = 9000
    ip_address_emlid_rover = '192.168.1.114' #'192.168.118.113'#
    s.connect((ip_address_emlid_rover, port))

    while not rospy.is_shutdown():
        data = s.recv(1024)
        if len(data) < 54:  # Check if data length is sufficient
            continue
        try:
            lat = float(data[26:39])
            lon = float(data[40:54])
        except ValueError:
            rospy.logwarn("Received invalid data: %s", data)
            continue
        
        rospy.loginfo("latitude: %f | longitude: %f" % (lat, lon))
        lat_pub.publish(lat)
        lon_pub.publish(lon)
        rate.sleep()

    s.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

