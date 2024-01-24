import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import threading
from math import pi
from std_msgs.msg import Bool
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError

FREQUENCY_QUERY = 1.5 # Hz
TWIST_ANGULAR_Z = 1.0 # rad/s
SAVE_PATH = "/home/simone/tesi_ws/src/relocalization_pkg/reloc_test/test_17/"
 
class AcquireQueryNode(Node):
    def __init__(self):
        super().__init__('acquire_query_node')
        
        # General variables
        self.trigger_relocalization = False # Set to True to enable relocalization task
        self.image_count = 0
        self.image = None # Image message
        self.bridge = CvBridge()
        
        # Subscribers
        self.pi_camera_sub = self.create_subscription(Image, '/pi_camera/image_raw', self.image_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.query_image_pub = self.create_publisher(Image, '/query_image', 10)
        self.status_query_pub = self.create_publisher(Bool, '/status_acquisition_query',10)
        
        # Service to trigger relocalization task
        self.trigger_relocalization_service = self.create_service(SetBool, 'trigger_relocalization', self.handle_trigger_relocalization)
        
        # Create callback for relocalization task
        self.msg = Bool()
        self.msg.data = False
        self.status_query_pub.publish(self.msg)
        self.check_trigger = self.create_timer(0.1, self.check_trigger)

        self.get_logger().info('Waiting to trigger relocalization with the command ros2 service call /trigger_relocalization std_srvs/srv/SetBool "{data: true}" ...')
        
    def check_trigger(self):
        if self.trigger_relocalization:
            self.get_logger().info('Relocalization pipeline triggered...')
            self.trigger_relocalization = False
            
            # Start the calculation function in a new thread
            threading.Thread(target=self.publisher_query).start()
            
    def publisher_query(self):
        '''Robot starts to rotate about 360 degrees and publish query images at a certain frequency'''

        # Start rotation
        twist = Twist()
        twist.angular.z = TWIST_ANGULAR_Z
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Robot is rotating...')
        
        # Wait for the robot to rotate about 360 degrees
        start_time = self.get_clock().now()
        last_query_time = self.get_clock().now()
        while ((self.get_clock().now() - start_time).nanoseconds/1e9) < 2.6 * pi/ TWIST_ANGULAR_Z:
            self.msg.data = True
            self.status_query_pub.publish(self.msg)
            self.cmd_vel_pub.publish(twist)
            # Publish a query image every FREQUENCY_QUERY seconds
            if ((self.get_clock().now() - last_query_time).nanoseconds/1e9) >= 1.0 / FREQUENCY_QUERY:
                self.publish_query()
                last_query_time = self.get_clock().now()
        
        self.image_count = 0
        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.msg.data = False
        self.status_query_pub.publish(self.msg)
    
    def publish_query(self):
        '''Acquire an image from the camera publish it to the query_image topic'''
        if self.image is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
            except CvBridgeError as e:
                print(e)
            image_filename = os.path.join(SAVE_PATH, f"image_{self.image_count}.jpg")
            cv2.imwrite(image_filename, cv_image)
            self.query_image_pub.publish(self.image)
            self.get_logger().info(f'Image {self.image_count} acquired...')
            self.image_count += 1
    
    def handle_trigger_relocalization(self, request, response):
        '''Service to handle the trigger_relocalization flag'''
        self.trigger_relocalization = request.data
        response.success = True
        response.message = 'Relocalization task triggered...'
        return response
    
    def image_callback(self, msg):
        self.image = msg

def main(args=None):
    
    rclpy.init(args=args)
    node = AcquireQueryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()