import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import numpy as np
from cv_bridge import CvBridge
import cv2 as cv
from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image as kimage
import os
from sklearn.cluster import DBSCAN
import time


NUM_PER_QUERY = 5
DATABASE_PATH = '/home/simone/tesi_ws/src/create_dataset/create_dataset/dataset_processed_v2/'
SAVE_PATH = "/home/simone/tesi_ws/src/relocalization_pkg/reloc_test/test_debug/"
FILTER = True
EPS = 0.9
MIN_SAMPLES = 15
TWIST_ANGULAR_Z = 1.0 # rad/s


class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        
        # ResNet50 Model initialization
        self.get_logger().info('Inizialization of ResNet50...')
        self.image_test = [f for f in os.listdir('/home/simone/tesi_ws/src/relocalization_pkg/reloc_test/test_1') if f.endswith('.jpg') or f.endswith('.png')]
        self.model = ResNet50(weights='imagenet', include_top=False, input_shape=(224, 224, 3), pooling='avg')
        for img in self.image_test[:5]:
            test_image = kimage.load_img('/home/simone/tesi_ws/src/relocalization_pkg/reloc_test/test_1/' + img, target_size=(224, 224, 3))
            img_data = preprocess_input(np.expand_dims(kimage.img_to_array(test_image), axis=0))
            _ = self.model.predict(img_data)


        # General variables
        self.bridge = CvBridge()
        self.top_indices_all_queries = []
        self.status_query = False # 
        self.query_buffer_msg = []
        self.query_count = 0
        self.max_weight = 0.0
        
        # Databases
        self.filtered_database = np.load(os.path.join(DATABASE_PATH, 'filtered_dataset_v2.npy'))
        self.features_database = np.load(os.path.join(DATABASE_PATH,'resnet.npy'))
        
        # Subscribers
        self.query_image_sub = self.create_subscription(Image, '/query_image', self.acquire_query_callback, 10)
        self.status_query_sub = self.create_subscription(Bool, '/status_acquisition_query', self.status_query_callback,10)
        self.max_weight_particle_sub = self.create_subscription(Float64, '/max_particle_weight', self.max_weight_particle_callback, 10)
        
        # Publishers
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status_initialpose = self.create_publisher(Bool, '/status_initialpose', 10)

    # CALLBACKS 
    def acquire_query_callback(self, msg):
        if self.status_query:
            self.query_buffer_msg.append(msg)
            
    def max_weight_particle_callback(self, msg):
        self.max_weight = msg.data
       
            
    def status_query_callback(self, msg):
        '''
            I process the query image when the acquisition of the query is finished, so when self.status_query from True to False
        '''
        # Check if self.status_query from True to False and if there are images in the buffer---> quando finisce di ruotare
        if self.status_query and not msg.data and self.query_buffer_msg:
                self.get_logger().info('Start processing queries...')
                self.process_query()
                
                #self.process_query()
            
        self.status_query = msg.data
    
    def process_query(self):
        '''
        I process the query image when the acquisition of the query is finished, so when self.status_query from True to False
        '''
        features_matrix = np.array([])
        # Calculate the features of the query image
        for msg_query in self.query_buffer_msg:
            # Conversion of emssage in cv image
            cv_image = self.bridge.imgmsg_to_cv2(msg_query, desired_encoding='bgr8') # conversion to opencv image
            image_filename = os.path.join(SAVE_PATH, f"image_posenode_{self.query_count}.jpg")
            cv.imwrite(image_filename, cv_image)
            
            # Calculate the features of the query image
            self.get_logger().info(f'Calculating features of query image {self.query_count}...')
            features = self.feature_extraction(cv_image)
            self.get_logger().info(f'{features[:5]}')
            
            # Calculate the similarity of the query image
            self.get_logger().info(f'Calculating similarity of query image {self.query_count}...')
            top_indices = self.similarity(features)
            self.top_indices_all_queries.extend(top_indices)
            
            self.query_count += 1
        image_dir = "/home/simone/tesi_ws/src/relocalization_pkg/reloc_test/test_debug/"
        np.save(os.path.join(image_dir, 'top_poses'), self.top_indices_all_queries)
        np.save(os.path.join(image_dir, 'feature_matrix_debug'), features_matrix)
        # Process the poses and publish the initial pose
        self.process_poses()
        
        status_initialpose_msg = Bool()
        status_initialpose_msg.data = True
        self.pub_status_initialpose.publish(status_initialpose_msg)
        #self.converge_to_pose()
        self.max_weight = 0.0
        self.query_buffer_msg = []
        self.top_indices_all_queries = []
        self.query_count = 0
        
    
    # FUNCTION FOR PROCESSING
    def feature_extraction(self, cv_image): # ResNet50 features
        cv_image = cv.resize(cv_image, (224, 224))
        features = self.model.predict(preprocess_input(np.expand_dims(kimage.img_to_array(cv_image), axis=0))) # calculate features
        return features
    
    def similarity(self, features): # Euclidean distance
        similarities = np.array([np.linalg.norm(features - db_feature) for db_feature in self.features_database[:, 8:]])
        return similarities.argsort()[:NUM_PER_QUERY]
    
    def process_poses(self):
        # DBSCAN filtering
        if FILTER:
            self.get_logger().info('DBSCAN filtering...')
            self.top_indices_all_queries = self.filter_dbscan()
            
        # Compute centroid
        self.get_logger().info('Computing average of top poses...')
        best_match_pose = self.calculate_centroid()
             
        # Publish initial pose
        self.get_logger().info('Publishing on /initialpose...')
        self.publish_pose(best_match_pose)
    
    def filter_dbscan(self):
        minimum_samples = MIN_SAMPLES
        result = np.array([])
        while minimum_samples > 0 and len(result) == 0:
            clustering = DBSCAN(eps=EPS, min_samples=minimum_samples).fit(self.filtered_database[self.top_indices_all_queries][:, 1:3])
            result = np.array(self.top_indices_all_queries)[clustering.labels_ != -1]
            minimum_samples -= 1
        self.get_logger().info('DBSCAN completed')
        return result

    def calculate_centroid(self):
        centroid_pose = np.mean(self.filtered_database[self.top_indices_all_queries][:, 1:8], axis=0)
        self.get_logger().info(f'Centroid calculated: {centroid_pose}')
        return centroid_pose
        
    def publish_pose(self, pose):
        pose_msg = PoseWithCovarianceStamped()
        
        # Set the pose data
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = pose[0]
        pose_msg.pose.pose.position.y = pose[1]
        pose_msg.pose.pose.position.z = 0.01
        pose_msg.pose.pose.orientation.x = pose[3]
        pose_msg.pose.pose.orientation.y = pose[4]
        pose_msg.pose.pose.orientation.z = pose[5]
        pose_msg.pose.pose.orientation.w = pose[6]
        
        # Set the covariance matrix
        pose_msg.pose.covariance[0] = 0.4
        pose_msg.pose.covariance[7] = 0.4
        pose_msg.pose.covariance[28] = 0.4
        pose_msg.pose.covariance[35] = 0.4
        
        # Publish the pose
        self.pub_pose.publish(pose_msg)
        self.get_logger().info('Initial pose published')
    
    def converge_to_pose(self):
        self.max_weight = 0.0
        self.get_logger().info(f'Max weight: {self.max_weight}')
        self.get_logger().info('Converging to pose... Start rotating...')
        while self.max_weight < MAX_WEIGHT_THRESHOLD:
            self.get_logger().info(f'Max weight: {self.max_weight}')
            twist = Twist()
            twist.angular.z = TWIST_ANGULAR_Z
            self.pub_vel.publish(twist)
            time.sleep(0.3)
                
    
def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()