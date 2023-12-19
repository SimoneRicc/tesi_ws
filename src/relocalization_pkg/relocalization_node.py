import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2 as cv
import os
import numpy as np
from sklearn.cluster import DBSCAN
from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image as kimage


FREQUENCY = 1 # Hz

class RelocalizationNode(Node):
    def __init__(self):
        super().__init__('relocalization_node')
        
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/pi_camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge() # Convert between ROS Image messages and OpenCV images
        self.image_count = 0
        self.image_dir = '/home/simone/tesi_ws/src/relocalization_pkg/query_images_folder' # Path to save the query images
        self.image = None # Image message
        
        # Inizialize ResNet50 model
        self.model = ResNet50(weights='imagenet', include_top=False, input_shape=(224, 224, 3), pooling='avg')
        
        # Load the images from the database of the environment
        self.database_environment = np.load('/home/simone/tesi_ws/src/create_dataset/create_dataset/dataset_processed_v2/filtered_dataset_v2.npy')
        
        # Create a timer to save the images and the position and orientation
        self.timer = self.create_timer(1.0/FREQUENCY, self.relocalization_callback)

    def relocalization_callback(self):
        if self.image is None: 
            self.get_logger().info('No image received')   
            return
        
        # Visual Place Recognition with ResNet50
        

    def image_callback(self, msg):
        self.image = msg
        
    def destroy_node(self):
        np.save(os.path.join(self.image_dir, 'pose_array_v2'), self.pose_array)
        super().destroy_node()
  
def main(args=None):
    
    rclpy.init(args=args)
    
    image_capture = RelocalizationNode()
    
    try:
        rclpy.spin(image_capture)
    except KeyboardInterrupt:
        pass
    
    image_capture.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()