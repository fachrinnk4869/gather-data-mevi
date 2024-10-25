# Import socket module
import socket
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix

def main():
    rospy.init_node('gps_publisher', anonymous=True)
    gps_pub = rospy.Publisher('/latlon1', NavSatFix, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    s = socket.socket()
    port = 9000
    ip_address_emlid_rover = '192.168.1.114' #'192.168.118.113'#
    try:
        s.connect((ip_address_emlid_rover, port))
        print("succesfull conected")
    except socket.error as e:
        print("failed to connect:", e)

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
        
        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        # rospy.loginfo("latitude: %f | longitude: %f" % (lat, lon))
        gps_pub.publish(msg)
        rate.sleep()    

    s.close()

# if __name__ == '__main__':
    # try:
main()
    # except rospy.ROSInterruptException:
        # pass

