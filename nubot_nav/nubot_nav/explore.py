"""Explore the navigation stack"""
import rclpy
from rclpy.node import Node

def explore_entry(args=None):
    """Entry point for the explore node"""
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


class Explore(Node):
    """Explore the navigation stack"""

    def __init__(self):
        """Initialize the node"""
        super().__init__('explore')
        self.get_logger().info('Exploring the navigation stack')


if __name__ == '__main__':
    import sys
    explore_entry(args=sys.argv)