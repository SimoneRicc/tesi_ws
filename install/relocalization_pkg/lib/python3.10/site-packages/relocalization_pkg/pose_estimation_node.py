import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from cv_bridge import CvBridge
from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image as kimage
import os
from sklearn.cluster import DBSCAN


NUM_PER_QUERY = 5
DATABASE_PATH = '/home/simone/tesi_ws/src/create_dataset/create_dataset/dataset_processed_v2/'

FILTER = True
EPS = 0.9
MIN_SAMPLES = 15


class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        
        # General variables
        self.bridge = CvBridge()
        self.top_indices_all_query = []
        self.status_query = False # 
        self.query_buffer = []
        self.index_query = 0
        
        # ResNet50 Model initialization
        self.get_logger().info('Inizialization of ResNet50...')
        self.image_test = [f for f in os.listdir('/home/simone/tesi_ws/src/relocalization_pkg/reloc_test/test_1') if f.endswith('.jpg') or f.endswith('.png')]
        self.model = ResNet50(weights='imagenet', include_top=False, input_shape=(224, 224, 3), pooling='avg')
        for img in self.image_test[:5]:
            test_image = kimage.load_img('/home/simone/tesi_ws/src/relocalization_pkg/reloc_test/test_1/' + img, target_size=(224, 224))
            img_data = preprocess_input(np.expand_dims(kimage.img_to_array(test_image), axis=0))
            _ = self.model.predict(img_data)
        
        # Databases
        self.filtered_database = np.load(os.path.join(DATABASE_PATH, 'filtered_dataset_v2.npy'))
        self.features_database = np.load(os.path.join(DATABASE_PATH,'resnet.npy'))
        
        # Subscribers
        self.query_image_sub = self.create_subscription(Image, '/query_image', self.query_image_callback, 10)
        self.status_query_sub = self.create_subscription(Bool, '/status_acquisition_query', self.status_query_callback,10)
        
        # Publishers
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def query_image_callback(self, msg):
        # During rotation of the robot and acquisition of query images, for each one of them the feature extraction is performed
        if self.status_query:
            self.get_logger().info('Query image received')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.query_buffer.append(cv_image)
            features = self.feature_extraction(cv_image)
            self.get_logger().info('Feature extraction completed')
            top_indices = self.similarity(features)
            self.top_indices_all_query.append(top_indices)
        
        # When the robot stops rotating, process the poses and publish the initial pose
        elif self.top_indices_all_query:
            self.process_poses()
    
    def status_query_callback(self,msg):
        self.status_query = msg
        
    def feature_extraction(self, query_image):
        features = self.model.predict(preprocess_input(np.expand_dims(kimage.img_to_array(query_image), axis=0)))
        return features
    
    def similarity(self, feature_vector):
        similarities = np.array([np.linalg.norm(feature_vector - db_feature) for db_feature in self.features_database[:, 8:]])
        return similarities.argsort()[:NUM_PER_QUERY]
    
    def process_poses(self):
        if FILTER:
            self.top_indices_all_query = self.filter_dbscan(self.top_indices_all_query)
        
        # Compute the average of the top poses
        self.get_logger().info('Computing average of top poses...')
        best_match_pose = self.calculate_centroid()
        self.get_logger().info('Publishing on /initialpose...')
        self.publish_pose()
        self.top_indices_all_query = []
        self.query_buffer = []
        return
        
    def filter_dbscan(self, top_indices_all_query):
        minimum_samples = MIN_SAMPLES
        result = np.array([])
        while minimum_samples > 0 and len(result) == 0:
            clustering = DBSCAN(eps=EPS, min_samples=minimum_samples).fit(top_indices_all_query)
            result = np.array(self.top_indices_all_queries)[clustering.labels_ != -1]
            minimum_sample -= 1
        self.get_logger().info('DBSCAN completed')
        return result

    def calculate_centroid(self):
        centroid_pose = np.mean(self.filtered_database[self.top_indices_all_queries][:, 1:8], axis=0)
        self.get_logger().info('Centroid calculated')
        return centroid_pose
    
    def publish_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        
        # Set the pose data
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = self.best_match_pose[0]
        pose_msg.pose.pose.position.y = self.best_match_pose[1]
        pose_msg.pose.pose.position.z = self.best_match_pose[2]
        pose_msg.pose.pose.orientation.x = self.best_match_pose[3]
        pose_msg.pose.pose.orientation.y = self.best_match_pose[4]
        pose_msg.pose.pose.orientation.z = self.best_match_pose[5]
        pose_msg.pose.pose.orientation.w = self.best_match_pose[6]
        
        # Set the covariance matrix
        pose_msg.pose.covariance[0] = 0.4
        pose_msg.pose.covariance[7] = 0.4
        pose_msg.pose.covariance[28] = 0.4
        pose_msg.pose.covariance[35] = 0.4
        
        # Publish the pose
        self.pub_pose.publish(pose_msg)
        
        
def main(args=None):
    
    rclpy.init(args=args)
    node = PoseEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()  

if __name__ == '__main__':
    main()