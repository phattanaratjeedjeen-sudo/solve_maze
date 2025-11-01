import numpy as np
if not hasattr(np, 'float'):
    np.float = float

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from tf_transformations import euler_from_quaternion

# Define the desired publishing rate (1.0 Hz = publish every 1.0 second)
PUBLISH_FREQUENCY_HZ = 1.0

# Map and grid parameters
MAP_SIZE_M = 3.0   # 3m × 3m map
GRID_SIZE = 10     # 10×10 grid
CELL_SIZE = MAP_SIZE_M / GRID_SIZE  # 0.3m per cell

class PoseConverter(Node):
    def __init__(self):
        super().__init__('pose_converter')
        
        self.latest_pose_msg = None
        
        # Subscribe to /pose (PoseWithCovarianceStamped)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10)
        
        # Publishers
        self.publisher_location = self.create_publisher(Pose2D, '/location', 10)
        self.publisher_grid = self.create_publisher(Pose2D, '/grid_location', 10)
        
        timer_period = 1.0 / PUBLISH_FREQUENCY_HZ
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            f'Pose Converter Node running. Publishing to /location and /grid_location at {PUBLISH_FREQUENCY_HZ:.1f} Hz.'
        )

    def pose_callback(self, msg):
        self.latest_pose_msg = msg

    def timer_callback(self):
        if self.latest_pose_msg is None:
            self.get_logger().warn('Waiting for first message on /pose...')
            return
        
        pose_data = self.latest_pose_msg.pose.pose.position
        orientation_q = self.latest_pose_msg.pose.pose.orientation
        
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w)
            
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        
        # --- Continuous pose (meters, degrees) ---
        location_msg = Pose2D()
        location_msg.x = pose_data.x
        location_msg.y = pose_data.y
        location_msg.theta = math.degrees(yaw)
        self.publisher_location.publish(location_msg)
        
        # --- Discrete grid mapping (use absolute x, y) ---
        abs_x = abs(pose_data.x)
        abs_y = abs(pose_data.y)
        
        grid_x = int(abs_x / CELL_SIZE)
        grid_y = int(abs_y / CELL_SIZE)
        
        # Clamp values to grid boundaries
        grid_x = max(0, min(GRID_SIZE - 1, grid_x))
        grid_y = max(0, min(GRID_SIZE - 1, grid_y))
        
        # Cast to float for Pose2D
        grid_msg = Pose2D()
        grid_msg.x = float(grid_x)
        grid_msg.y = float(grid_y)
        grid_msg.theta = math.degrees(yaw)
        
        self.publisher_grid.publish(grid_msg)

        # Debug info
        self.get_logger().debug(
            f"/location: ({location_msg.x:.2f}, {location_msg.y:.2f}, {location_msg.theta:.1f}°) "
            f"=> /grid_location: ({grid_x}, {grid_y})"
        )

def main(args=None):
    rclpy.init(args=args)
    pose_converter = PoseConverter()
    rclpy.spin(pose_converter)
    pose_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
