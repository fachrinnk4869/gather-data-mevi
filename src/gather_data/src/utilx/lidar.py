# lidar_module.py
import rospy
import ros_numpy
import numpy as np
import message_filters
from sensor_msgs.msg import PointCloud2
import rospy
import numpy as np
import pypcd4 as pypcd
np.float = float


class LidarSensor:
    def __init__(self, topic_name):
        self.topic_name = topic_name

    def start_listener(self):
        lidar_sub = message_filters.Subscriber(
            self.topic_name, PointCloud2)
        # rospy.loginfo(f"Subscribed to {self.topic_name} topic. Waiting for data...")
        return lidar_sub

    def print_saved_data(self, file_path):
        lid_pc = pypcd.PointCloud.from_path(f"{file_path}.pcd")
        print(lid_pc.pc_data['x'])
        lid_x = lid_pc.pc_data['x']
        lid_y = lid_pc.pc_data['y']
        lid_z = lid_pc.pc_data['z']
        lid_intensity = lid_pc.pc_data['intensity']
        in_velodyne = np.zeros(
            lid_x.shape[0] + lid_y.shape[0] + lid_z.shape[0] + lid_intensity.shape[0], dtype=np.float32)
        in_velodyne[0::4] = lid_x
        in_velodyne[1::4] = lid_y
        in_velodyne[2::4] = lid_z
        in_velodyne[3::4] = lid_intensity
        in_velodyne = in_velodyne.astype('float32').reshape((-1, 4))

    def save_lidar_data(self, point_cloud_msg, file_path):
        try:
            # pointcloud2
            # baca: https://github.com/PRBonn/lidar-bonnetal/issues/78
            lid_pc = pypcd.PointCloud.from_msg(point_cloud_msg)
            lid_pc.save(file_path)
            print("save lidar data succesful")

        except Exception as e:
            rospy.logerr(f"Failed to save Lidar : {str(e)}")


# Test function for Lidar
if __name__ == "__main__":
    rospy.init_node('lidar_listener_node', anonymous=True)
    topic_name = "/velodyne_points"  # Update this to match your Lidar point cloud topic
    lidar_sensor = LidarSensor(topic_name)

    def callback(lidar_msg):
        lidar_sensor.save_lidar_data(lidar_msg, "./unittest/lidar.pcd")

    try:
        lidar_sub = lidar_sensor.start_listener()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

    # ROS message synchronizer
    ts = message_filters.ApproximateTimeSynchronizer(
        [lidar_sub], 25, 0.25)
    ts.registerCallback(callback)

    rospy.spin()
