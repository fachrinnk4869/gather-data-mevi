import rospy
from sensor_msgs.msg import NavSatFix


class GPSPublisher:
    # class GPS publisher jangan dirubah
    def __init__(self, pub_topic_name):
        self.pub_topic_name = pub_topic_name
        self.publisher = rospy.Publisher(
            self.pub_topic_name, NavSatFix, queue_size=10)
    # jangan dirubah

    def publish_gps_data(self, lat, lon, altitude):
        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = altitude
        self.publisher.publish(msg)
        rospy.loginfo(
            f"Published GPS data: Lat: {lat}, Lon: {lon}, Alt: {altitude}")


if __name__ == "__main__":
    rospy.init_node('gps_publisher_node', anonymous=True)
    pub_topic_name = "/latlon"  # topik jangan dirubah
    gps_publisher = GPSPublisher(pub_topic_name)

    rate = rospy.Rate(1)  # Publish at 1 Hz
    while not rospy.is_shutdown():
        # Replace these values with actual GPS data or logic to retrieve it
        lat = 37.7749  # Example latitude
        lon = -122.4194  # Example longitude
        altitude = 10.0  # Example altitude

        gps_publisher.publish_gps_data(lat, lon, altitude)
        rate.sleep()
