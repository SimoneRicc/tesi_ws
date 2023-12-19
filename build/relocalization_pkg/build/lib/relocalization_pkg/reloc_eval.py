import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from nav2_msgs.msg import ParticleCloud 
from std_msgs.msg import Float64
import numpy as np
from time import time

class RelocEvalNode(Node):
    def __init__(self):
        super().__init__('max_particle_weight_publisher')
        self.publisher_v2 = self.create_publisher(Float64, '/max_particle_weight', 10)
        self.publisher_ = self.create_publisher(Float64, '/entropy', 10)
        self.publisher_v3 = self.create_publisher(Float64, '/entropy_change', 10)
        
        qos_profile = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            ParticleCloud,  
            '/particle_cloud',
            self.listener_callback,
            qos_profile) 
        
        self.subscription_v2 = self.create_subscription(
            ParticleCloud,  
            '/particlecloud',
            self.listener_callback_v2,
            qos_profile)
        
        self.previous_entropy = None
    def listener_callback(self, msg):
        weights = np.array([particle.weight for particle in msg.particles])
        weights = weights / np.sum(weights)
        entropy = -np.sum(weights * np.log2(weights))
        current_time = time()
        # Calculate and publish entropy change
        if self.previous_entropy is not None:
            entropy_change = entropy - self.previous_entropy
            msg_change = Float64()
            msg_change.data = entropy_change
            self.publisher_v3.publish(msg_change)
        
        self.previous_entropy = entropy  # Update the previous entropy value

        msg = Float64()
        msg.data = entropy
        self.publisher_.publish(msg)
    
    def listener_callback_v2(self, msg):
        max_weight = max([particle.weight for particle in msg.particles])
        msg = Float64()
        msg.data = max_weight
        self.publisher_v2.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    max_particle_weight_publisher = RelocEvalNode()
    rclpy.spin(max_particle_weight_publisher)

    max_particle_weight_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()