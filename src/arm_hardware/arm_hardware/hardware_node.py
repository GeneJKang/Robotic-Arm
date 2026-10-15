import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HardwareNode(Node):

    def __init__(self):
        super().__init__('hardware_node')

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(JointState, '/joint_commands', self.command_callback, 10)

    def command_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = HardwareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()