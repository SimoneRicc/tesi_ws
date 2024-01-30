import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from nav2_msgs.msg import ParticleCloud 
from std_msgs.msg import Float64
import numpy as np

class RelocEvalNode(Node):
    def __init__(self):
        super().__init__('reloc_eval_node')
        self.pub_max_weight_particle = self.create_publisher(Float64, '/max_particle_weight', 10)
        self.pub_entropy = self.create_publisher(Float64, '/entropy', 10)
        #self.pub = self.create_publisher(Float64, '/entropy_change', 10)
        
        qos_profile = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            ParticleCloud,  
            '/particle_cloud',
            self.max_weight_particle_callback,
            qos_profile) 
           
        self.subscription_v2 = self.create_subscription(
            ParticleCloud,  
            '/particlecloud',
            self.entropy_callback,
            qos_profile)
        
        self.previous_entropy = None
    def entropy_callback(self, msg):
        weights = np.array([particle.weight for particle in msg.particles])
        weights = weights / np.sum(weights)
        entropy = -np.sum(weights * np.log2(weights))
        msg = Float64()
        msg.data = entropy
        self.pub_entropy.publish(msg)
    
    def max_weight_particle_callback(self, msg):
        max_weight = max([particle.weight for particle in msg.particles])
        msg = Float64()
        msg.data = max_weight
        self.get_logger().info(f'Max weight: {max_weight}')
        self.pub_max_weight_particle.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    max_particle_weight_publisher = RelocEvalNode()
    rclpy.spin(max_particle_weight_publisher)

    max_particle_weight_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()