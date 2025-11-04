#!/usr/bin/python3

import numpy as np
if not hasattr(np, 'float'):
    np.float = float

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from std_msgs.msg import Float64 # Import Float64 message type
from tf_transformations import euler_from_quaternion
from rcl_interfaces.msg import SetParametersResult

# Define the desired publishing rate (1.0 Hz = publish every 1.0 second)
PUBLISH_FREQUENCY_HZ = 1.0

# Map and grid parameters
MAP_SIZE_M = 3.0   # 3m × 3m map
GRID_SIZE = 10     # 10×10 grid
CELL_SIZE = MAP_SIZE_M / GRID_SIZE  # 0.3m per cell

class PoseConverter(Node):
    def __init__(self):
        super().__init__('pose_converter_node')
        
        self.latest_pose_msg = None
        # parameters
        self.declare_parameter('initial_grid_x', 0.0)
        self.declare_parameter('initial_grid_y', 0.0)
        self.initial_grid_x = self.get_parameter('initial_grid_x').get_parameter_value().double_value
        self.initial_grid_y = self.get_parameter('initial_grid_y').get_parameter_value().double_value
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Subscribe to /pose (PoseWithCovarianceStamped)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10)
        
        # Publishers
        self.publisher_location = self.create_publisher(Pose2D, '/location', 10)
        self.publisher_grid = self.create_publisher(Pose2D, '/grid_location', 10)
        
        # New Publishers for std_msgs.msg/Float64
        # self.publisher_grid_x_float = self.create_publisher(Float64, '/ros2mqtt/grid_x_float', 10)
        # self.publisher_grid_y_float = self.create_publisher(Float64, '/ros2mqtt/grid_y_float', 10)
        # self.publisher_grid_theta_float = self.create_publisher(Float64, '/ros2mqtt/grid_theta_float', 10)

        timer_period = 1.0 / PUBLISH_FREQUENCY_HZ
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            f'Pose Converter Node running. Publishing to /location, /grid_location, /grid_x_float, /grid_y_float, and /grid_theta_float at {PUBLISH_FREQUENCY_HZ:.1f} Hz.'
        )


    def parameter_callback(self, params):
        for param in params:
            if param.name == 'initial_grid_x':
                self.initial_grid_x = param.value
                self.get_logger().info(f'Initial grid X set to: {self.initial_grid_x}')
            elif param.name == 'initial_grid_y':
                self.initial_grid_y = param.value
                self.get_logger().info(f'Initial grid Y set to: {self.initial_grid_y}')
        return SetParametersResult(successful=True)

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
        
        # Convert yaw to degrees
        yaw_degrees = math.degrees(yaw)
        
        # --- Continuous pose (meters, degrees) ---
        location_msg = Pose2D()
        location_msg.x = pose_data.x
        location_msg.y = pose_data.y
        location_msg.theta = yaw_degrees
        self.publisher_location.publish(location_msg)

        grid_x = int(abs(1 + self.initial_grid_x + pose_data.x / CELL_SIZE))
        grid_y = int(abs(1 + self.initial_grid_y + pose_data.y / CELL_SIZE))
        
        # # --- Discrete grid mapping (use absolute x, y) ---
        # abs_x = abs(pose_data.x)
        # abs_y = abs(pose_data.y)
        
        # grid_x = int(abs_x / CELL_SIZE)
        # grid_y = int(abs_y / CELL_SIZE)
        
        # # Clamp values to grid boundaries
        # grid_x = max(0, min(GRID_SIZE - 1, grid_x))
        # grid_y = max(0, min(GRID_SIZE - 1, grid_y))
        
        # Cast to float for Pose2D
        grid_msg = Pose2D()
        grid_msg.x = float(grid_x)
        grid_msg.y = float(grid_y)
        grid_msg.theta = yaw_degrees
        
        self.publisher_grid.publish(grid_msg)


        # grid_x_float_msg = Float64()
        # grid_x_float_msg.data = float(grid_x)
        # self.publisher_grid_x_float.publish(grid_x_float_msg)

        # grid_y_float_msg = Float64()
        # grid_y_float_msg.data = float(grid_y)
        # self.publisher_grid_y_float.publish(grid_y_float_msg)

        # grid_theta_float_msg = Float64()
        # grid_theta_float_msg.data = yaw_degrees
        # self.publisher_grid_theta_float.publish(grid_theta_float_msg)
        
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