#!/usr/bin/python3

from scantest.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D


class PrepareMqtt(Node):
    def __init__(self):
        super().__init__('prepare_mqtt_node')

        self.create_subscription(Pose2D, '/grid_location', self.pose_callback, 10)
        self.create_subscription(String, '/current_state', self.state_callback, 10)

        self.mqttmsg_pub = self.create_publisher(String, '/mqttmsg', 10)
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Prepare MQTT Node has started.')

        self.state_str = ""
        self.pose_str = ""

    def state_callback(self, msg: String):
        self.state_str = msg.data

    def pose_callback(self, msg: Pose2D):
        self.pose_str = f"xx/{int(msg.x)},yy/{int(msg.y)},theta/{int(msg.theta)}"

    def timer_callback(self):
        mqtt_msg = String()
        mqtt_msg.data = f"{self.state_str},{self.pose_str}"
        self.mqttmsg_pub.publish(mqtt_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PrepareMqtt()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
