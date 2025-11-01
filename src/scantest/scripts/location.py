import numpy as np
if not hasattr(np, 'float'):
    np.float = float

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D
from tf_transformations import euler_from_quaternion



# Define the desired publishing rate (1.0 Hz = publish every 1.0 second)
PUBLISH_FREQUENCY_HZ = 1.0 

class PoseConverter(Node):
    def __init__(self):
        super().__init__('pose_converter')
        
        self.latest_pose_msg = None
        
        # 1. Create a Subscriber for the input topic. This runs whenever data arrives.
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10)
        
        # 2. Create a Publisher for the output topic.
        self.publisher_ = self.create_publisher(Pose2D, '/location', 10)
        
        # 3. Create a Timer to publish the data every 1 second (1.0 Hz).
        timer_period = 1.0 / PUBLISH_FREQUENCY_HZ  # 1.0 / 1.0 = 1.0 second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Pose Converter Node running. Publishing to /location at {PUBLISH_FREQUENCY_HZ:.1f} Hz.')

    def pose_callback(self, msg):
        # This function runs when a message is received on /pose.
        # We only store the message; we do NOT publish here.
        self.latest_pose_msg = msg

    def timer_callback(self):
        # This function runs every 1 second, regardless of how often /pose is updated.
        
        if self.latest_pose_msg is None:
            # Skip publishing if no message has been received yet
            self.get_logger().warn('Waiting for first message on /pose...')
            return
            
        # --- Extraction and Conversion Logic ---
        
        pose_data = self.latest_pose_msg.pose.pose.position
        orientation_q = self.latest_pose_msg.pose.pose.orientation
        
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w)
            
        # Get Yaw (theta)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        
        # Create and populate the new Pose2D message
        location_msg = Pose2D()
        location_msg.x = pose_data.x
        location_msg.y = pose_data.y
        location_msg.theta = math.degrees(yaw)
        
        # --- Publish ---
        self.publisher_.publish(location_msg)
        
        self.get_logger().debug(
            f"Timer published /location: x: {location_msg.x:.4f}, y: {location_msg.y:.4f}, theta: {location_msg.theta:.4f}"
        )

def main(args=None):
    rclpy.init(args=args)
    pose_converter = PoseConverter()
    rclpy.spin(pose_converter)
    pose_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()