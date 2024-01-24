import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
import time

MAX_WEIGHT_THRESHOLD = 0.00095


class MyNode(Node):
    def __init__(self):
        super().__init__('converge_to_pose_node')
        
        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriptions
        self.subscription_max_particle_weight = self.create_subscription(
            Float64,
            '/max_particle_weight',
            self.max_particle_weight_callback,
            10)
        self.subscription_status_initialpose = self.create_subscription(
            Bool,
            '/status_initialpose',
            self.status_initialpose_callback,
            10)
        
        # General variables
        self.status_initialpose = False
        self.max_particle_weight = 0.0
        
        
    def max_particle_weight_callback(self, msg):
        self.max_particle_weight = msg.data
        
        if self.max_particle_weight < MAX_WEIGHT_THRESHOLD and self.status_initialpose:
            pub_msg = Twist()
            pub_msg.angular.z = 1.0
            self.pub_cmd_vel.publish(pub_msg)

        
    def status_initialpose_callback(self, msg):
        self.status_initialpose = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()