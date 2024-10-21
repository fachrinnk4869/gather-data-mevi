import message_filters
from std_msgs.msg import Float32
import rospy
import numpy as np
np.float = float  # To avoid the deprecation warning with numpy.float

class LowLevelSensor:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.data = None

    def callback(self, msg):
        # Process the incoming message and store it
        self.data = msg.data
        # rospy.loginfo(f"Received data from {self.topic_name}: {self.data}")

    def get_data(self):
        try:
            # Subscribe to the topic with the callback function
            rospy.Subscriber(self.topic_name, Float32, self.callback)
            rospy.loginfo(f"Subscribed to {self.topic_name} topic.")
        except rospy.ROSInterruptException as e:
            rospy.logerr(f"Failed to subscribe to {self.topic_name}: {str(e)}")
            return None

        # Return the latest data received
        return self.data
