import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import numpy as np
if not hasattr(np, 'float'):
    np.float = float

PUBLISH_FREQUENCY_HZ = 1.0

MIN_GRID_COORD = 0
MAX_GRID_COORD = 5 

class GridPublisherMock(Node):
    def __init__(self):
        super().__init__('grid_publisher_mock')
        
        # Publisher for /grid_location (Pose2D)
        self.publisher_grid = self.create_publisher(Pose2D, '/grid_location', 10)
        
        # Timer to publish at the defined frequency
        timer_period = 1.0 / PUBLISH_FREQUENCY_HZ
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            f'Grid Publisher Mock Node running. Publishing random grid locations to /grid_location at {PUBLISH_FREQUENCY_HZ:.1f} Hz.'
        )

    def timer_callback(self):

        random_x = np.random.randint(MIN_GRID_COORD, MAX_GRID_COORD + 1)
        random_y = np.random.randint(MIN_GRID_COORD, MAX_GRID_COORD + 1)
        
        grid_msg = Pose2D()

        grid_msg.x = float(random_x)
        grid_msg.y = 0.0
        grid_msg.theta = 0.0
        
        self.publisher_grid.publish(grid_msg)

        self.get_logger().info(
            f"Published Mock Grid Location: ({grid_msg.x}, {grid_msg.y})"
        )

def main(args=None):
    rclpy.init(args=args)
    grid_publisher = GridPublisherMock()
    rclpy.spin(grid_publisher)
    grid_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()