import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np


FREQUENCY = 2 # Hz

class RelocCaptureNode(Node):
    def __init__(self):
        super().__init__('reloc_capture_node')
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/pi_camera/image_raw',
            self.image_callback,
            10)    
        self.bridge = CvBridge() # Convert between ROS Image messages and OpenCV images
        self.image_count = 0
        self.image_dir = '/home/simone/tesi_ws/src/create_dataset/create_dataset/dataset_relocalization/test_7' # Path to save the images 
        self.timer = self.create_timer(1.0/FREQUENCY, self.timer_callback)
        self.image = None
        
    def timer_callback(self):
        if self.image is None: 
            self.get_logger().info('No image received')   
            return
        # Save the image
        cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        cv2.imwrite(os.path.join(self.image_dir, 'image_reloc_{}.jpg'.format(self.image_count)), cv_image)
        self.image_count += 1
        
    def image_callback(self, msg):
        self.image = msg   
    

def main(args=None):
    rclpy.init(args=args)

    reloc_capture_node = RelocCaptureNode()

    rclpy.spin(reloc_capture_node)

    reloc_capture_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()