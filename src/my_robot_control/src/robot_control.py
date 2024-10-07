#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def cmd_vel_publisher():
    # Initialize the ROS node
    rospy.init_node('cmd_vel_publisher', anonymous=True)

    # Create a publisher for the /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set the rate of publishing
    rate = rospy.Rate(10)  # 10 Hz

    # Create a Twist message
    vel_msg = Twist()

    # Set the desired linear and angular velocities
    vel_msg.linear.x = 5.56  # Set your desired speed (in m/s)
    vel_msg.angular.z = 0.0   # No rotation

    while not rospy.is_shutdown():
        # Publish the velocity command
        pub.publish(vel_msg)
        rospy.loginfo("Published velocity: %s", vel_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        cmd_vel_publisher()
    except rospy.ROSInterruptException:
        pass
