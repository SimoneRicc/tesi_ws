import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from cv_bridge import CvBridge
import cv2 as cv
from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image as kimage
import os
from sklearn.cluster import DBSCAN

# PATHS
DATABASE_PATH = '/home/simone/tesi_ws/src/create_dataset/create_dataset/dataset_processed_v3/'
DEBUG_PATH = '/home/simone/tesi_ws/src/relocalization_pkg/reloc_test/test_debug/'

# DATABASES
DATABASE_FILE = 'filtered_dataset_v3.npy'
FEATURES_FILE = 'resnet.npy'

# PARAMETERS
NUM_PER_QUERY = 5
MIN_SAMPLES = 15
EPS = 1.5


class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')

        # ResNet50 Model initialization
        self.model = ResNet50(weights='imagenet', include_top=False, input_shape=(224, 224, 3), pooling='avg')
        
        # Load cuDNN
        self.image_test = [f for f in os.listdir('/home/simone/tesi_ws/src/relocalization_pkg/reloc_test/test_1') if f.endswith('.jpg') or f.endswith('.png')][:3]
        for img in self.image_test:
            test_image = kimage.load_img('/home/simone/tesi_ws/src/relocalization_pkg/reloc_test/test_1/' + img, target_size=(224, 224, 3))
            _ = self.model.predict(preprocess_input(np.expand_dims(kimage.img_to_array(test_image), axis=0)))
    
        # General variables
        self.query_buffer = []
        self.query_count = 0
        self.relocalization_status = False
        self.all_indexes_top_poses = []
        self.get_logger().info(f'{type(self.all_indexes_top_poses)}')
        
        
        # Databases
        self.filtered_database = np.load(os.path.join(DATABASE_PATH, DATABASE_FILE))
        self.features_database = np.load(os.path.join(DATABASE_PATH, FEATURES_FILE))
        
        # Subscribers
        self.query_image_sub = self.create_subscription(Image, '/query_image', self.query_callback, 10)
        self.relocalization_status_sub = self.create_subscription(Bool, '/status_acquisition_query', self.relocalization_status_callback, 10)  
        
        # Publishers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.status_initialpose_pub = self.create_publisher(Bool, '/status_initialpose', 10)
    
    # CALLBACKS
    def query_callback(self, msg):
        # Acquire query image
        bridge = CvBridge()
        query_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        query_image = cv.resize(query_image, (224, 224))
        self.query_buffer.append(query_image)
        
        # Save query image
        cv.imwrite(os.path.join(DEBUG_PATH, f'image_debug_{self.query_count}.png'), query_image)

    def relocalization_status_callback(self, msg):
        if self.relocalization_status:
            if self.query_buffer:
                self.process_query()
                
        if self.relocalization_status and not msg.data:
            self.process_poses()
            
        self.relocalization_status = msg.data
        
    # METHODS
    def process_query(self):
        query_image = self.query_buffer.pop(0)
        
        # Features extraction
        features = self.feature_extraction(query_image)
        
        # Similarity
        indexes_top_poses = self.similarity(features)
        
        # List of top poses
        self.all_indexes_top_poses.extend(indexes_top_poses)
        self.all_indexes_top_poses = list(set(self.all_indexes_top_poses))
        
        self.query_count += 1
        
    
    def feature_extraction(self, query_image):
        img_data = preprocess_input(np.expand_dims(kimage.img_to_array(query_image), axis=0))
        features = self.model.predict(img_data)
        return features
    
    def similarity(self, features): # Euclidean distance
        distances = np.array([np.linalg.norm(features - f) for f in self.features_database[:,8:]])
        return distances.argsort()[:NUM_PER_QUERY]
        
    def process_poses(self):
        # DBSCAN filtering
        self.all_indexes_top_poses = self.filter_dbscan()
        
        # Calculate initial pose with centroid
        initial_pose = self.centroid()
        
        # Publish initial pose
        self.publish_initial_pose(initial_pose)
        
        # Publish initial pose status for converge_to_pose_node
        status_initialpose_msg = Bool()
        status_initialpose_msg.data = True
        self.status_initialpose_pub.publish(status_initialpose_msg)
        
        # Reset variables
        self.query_buffer = []
        self.all_indexes_top_poses = []
        self.query_count = 0
        
        
    def filter_dbscan(self):
        minimum_samples = MIN_SAMPLES
        result = np.array([])
        while minimum_samples > 0 and len(result) == 0:
            clustering = DBSCAN(eps=EPS, min_samples=minimum_samples).fit(self.filtered_database[self.all_indexes_top_poses][:,1:3])
            result = np.array(self.all_indexes_top_poses)[clustering.labels_ != -1]
            minimum_samples -= 1
        return result
    
    def centroid(self):
        return np.mean(self.filtered_database[self.all_indexes_top_poses][:,1:8], axis=0)
        
    def publish_initial_pose(self, initial_pose):
        pose_msg = PoseWithCovarianceStamped()
        
        # Set the pose data
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = initial_pose[0]
        pose_msg.pose.pose.position.y = initial_pose[1]
        pose_msg.pose.pose.position.z = 0.01
        pose_msg.pose.pose.orientation.x = initial_pose[3]
        pose_msg.pose.pose.orientation.y = initial_pose[4]
        pose_msg.pose.pose.orientation.z = initial_pose[5]
        pose_msg.pose.pose.orientation.w = initial_pose[6]
        
        # Set the covariance matrix
        pose_msg.pose.covariance[0] = 0.4
        pose_msg.pose.covariance[7] = 0.4
        pose_msg.pose.covariance[28] = 0.4
        pose_msg.pose.covariance[35] = 0.4
        
        # Publish the pose
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info('Initial pose published')
                
def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()