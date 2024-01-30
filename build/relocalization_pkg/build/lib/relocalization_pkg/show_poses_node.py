import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')
        self.publisher_ = self.create_publisher(PoseArray, 'pose_array', 10)
        timer_period = 1  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Load the numpy array of poses
        self.poses = np.load('/home/simone/tesi_ws/src/create_dataset/create_dataset/dataset_processed_v2/filtered_dataset_v2.npy')

        # Create the PoseArray
        self.pose_array = PoseArray()
        self.publisher_.publish(self.pose_array)
        # Convert numpy array to PoseArray
        for pose in self.poses:
            p = Pose()
            self.pose_array.header.frame_id = 'map'
            p.position.x = pose[1]
            p.position.y = pose[2]
            p.orientation.z = pose[6]
            p.orientation.w = pose[7]
            self.pose_array.poses.append(p)
        self.publisher_.publish(self.pose_array)

    def timer_callback(self):
        self.publisher_.publish(self.pose_array)
        self.get_logger().info('Publishing poses')

def main(args=None):
    rclpy.init(args=args)

    pose_publisher_node = PosePublisherNode()

    rclpy.spin(pose_publisher_node)

if __name__ == '__main__':
    main()