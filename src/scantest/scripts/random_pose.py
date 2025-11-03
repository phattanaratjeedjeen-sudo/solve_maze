#!/usr/bin/python3

from scantest.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import random


class RandomPoseNode(Node):
    def __init__(self):
        super().__init__('random_pose_node')
        self.get_logger().info('Random Pose Node has started.')

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,'/pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position.x = random.uniform(0.0, 3.0)
        msg.pose.pose.position.y = random.uniform(0.0, 3.0)
        self.publish(msg)

    def publish(self, msg: PoseWithCovarianceStamped):
        self.pose_pub.publish(msg)
        self.get_logger().info(f'Published random pose: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
        

def main(args=None):
    rclpy.init(args=args)
    node = RandomPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
