import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import tf2_ros

FREQUENCY = 3 # Hz

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/pi_camera/image_raw',
            self.image_callback,
            10)    
        self.bridge = CvBridge() # Convert between ROS Image messages and OpenCV images
        self.image_count = 0
        self.image_dir = '/home/simone/tesi_ws/src/create_dataset/create_dataset/dataset_not_processed' # Path to save the images
        
        # Subscribe to the position and orientation topic
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pose_array = np.array([])
        
        # Create a timer to save the images and the position and orientation
        self.image = None # Image message
        self.pose = None # Pose message
        self.timer = self.create_timer(1.0/FREQUENCY, self.timer_callback)


    def timer_callback(self):
        if self.image is None: 
            self.get_logger().info('No image received')   
            return
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.pose = transform.transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info('No transform available')
            return
            
        # Save the image
        cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        cv2.imwrite(os.path.join(self.image_dir, 'image_{}.jpg'.format(self.image_count)), cv_image)
        
        # Save position and orientation in numpy array
        pose_list = np.array(['image_{}'.format(self.image_count),
                              self.pose.translation.x,
                              self.pose.translation.y,
                              self.pose.translation.z,
                              self.pose.rotation.x,
                              self.pose.rotation.y,
                              self.pose.rotation.z,
                              self.pose.rotation.w])
        if self.pose_array.size == 0:
            self.pose_array = pose_list 
        else:
            self.pose_array = np.vstack((self.pose_array, pose_list))
        
        self.image_count += 1
        self.get_logger().info('Image {} and pose saved'.format(self.image_count))
             
    def image_callback(self, msg):
        self.image = msg
        
    def destroy_node(self):
        np.save(os.path.join(self.image_dir, 'pose_array'), self.pose_array)
        super().destroy_node()
  
def main(args=None):
    
    rclpy.init(args=args)
    
    image_capture = ImageCaptureNode()
    
    try:
        rclpy.spin(image_capture)
    except KeyboardInterrupt:
        pass
    
    image_capture.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()