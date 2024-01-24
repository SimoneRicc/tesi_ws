import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import os
import numpy as np
from sklearn.cluster import DBSCAN
from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image as kimage
from std_srvs.srv import SetBool
import threading


FREQUENCY = 1.5 # Hz

class RelocalizationNode(Node):
    def __init__(self):
        super().__init__('relocalization_node')
        
        # General variables
        self.trigger_relocalization = False # Set to True to enable relocalization task
        self.bridge = CvBridge() # Convert between ROS Image messages and OpenCV images
        self.image_count = 0
        self.image_dir = '/home/simone/tesi_ws/src/relocalization_pkg/query_images_folder' # Path to save the query images
        self.image = None # Image message
        
        # Subscribers
        self.pi_camera_sub = self.create_subscription(Image, '/pi_camera/image_raw', self.image_callback, 10)
        #self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Inizialize ResNet50 model
        self.image_test_dir = '/home/simone/tesi_ws/src/relocalization_pkg/reloc_test/test_1/' # Path for the images to be used to load the model
        self.images_test = [f for f in os.listdir(self.image_test_dir) if f.endswith('.jpg') or f.endswith('.png')]
        self.get_logger().info('Loading ResNet50 model...')
        self.model = ResNet50(weights='imagenet', include_top=False, input_shape=(224, 224, 3), pooling='avg')
        for count,image in enumerate(self.images_test[:5]):
            img = kimage.load_img(self.image_test_dir + image, target_size=(224, 224))
            img = preprocess_input(np.expand_dims(kimage.img_to_array(img), axis=0))
            features = self.model.predict(img)
            self.get_logger().info(f'Images {count} tested')
        self.get_logger().info('ResNet50 model loaded')

        # Service to trigger relocalization task
        self.trigger_relocalization_service = self.create_service(SetBool, 'trigger_relocalization', self.handle_trigger_relocalization)
        
        # Create callback for relocalization task
        self.check_trigger = self.create_timer(0.1, self.check_trigger)

    def check_trigger(self):
        if self.trigger_relocalization:
            self.get_logger().info('Relocalization pipeline triggered...')
            self.trigger_relocalization = False
            
            # Start the calculation function in a new thread
            threading.Thread(target=self.relocalization_pipeline).start()
            
    def relocalization_pipeline(self):
        # Task 1 - Acquisition of query images while moving
        twist = Twist()
        twist.angular.z = 1.0
        while True:
            self.cmd_vel_pub.publish(twist)
        
    
    def handle_trigger_relocalization(self, request, response):
        self.trigger_relocalization = request.data
        response.success = True
        response.message = 'Relocalization task triggered...'
        return response
    
    def image_callback(self, msg):
        self.image = msg
        
    def destroy_node(self):
        #np.save(os.path.join(self.image_dir, 'pose_array_v2'), self.pose_array)
        super().destroy_node()

def main(args=None):
    
    rclpy.init(args=args)
    
    relocalization_node = RelocalizationNode()
    
    try:
        rclpy.spin(relocalization_node)
    except KeyboardInterrupt:
        pass
    
    relocalization_node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()